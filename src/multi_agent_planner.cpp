#include "planning/multi_agent_planner.hpp"

namespace adore
{
namespace planner
{

void
MultiAgentPlanner::set_parameters( const std::map<std::string, double>& params )
{
  for( const auto& [name, value] : params )
  {
    if( name == "dt" && value > 0 ) // Ensure dt > 0
      dt = value;
    if( name == "horizon_steps" && value > 0 )
      horizon_steps = static_cast<size_t>( value );
    if( name == "lane_error" )
      weights.lane_error = value;
    if( name == "long_error" )
      weights.long_error = value;
    if( name == "speed_error" )
      weights.speed_error = value;
    if( name == "heading_error" )
      weights.heading_error = value;
    if( name == "steering_angle" )
      weights.steering_angle = value;
    if( name == "acceleration" )
      weights.acceleration = value;
    if( name == "max_iterations" )
      solver_params.max_iterations = value;
    if( name == "tolerance" )
      solver_params.tolerance = value;
    if( name == "max_ms" )
      solver_params.max_ms = value;
    if( name == "debug" )
      solver_params.debug = value;
    if( name == "max_lateral_acceleration" )
      max_lateral_acceleration = value;
    if( name == "idm_time_headway" )
      idm_time_headway = value;
    if( name == "ref_traj_length" && value > 0 )
      ref_traj_length = value;
  }
}

dynamics::TrafficParticipantSet
MultiAgentPlanner::plan_all_participants( const dynamics::TrafficParticipantSet& in_participants,
                                          const std::shared_ptr<map::Map>&       in_local_map )
{
  traffic_participants = in_participants;
  local_map            = in_local_map;

  solve_problem();
  extract_trajectories();
  return traffic_participants;
}

mas::MotionModel
MultiAgentPlanner::get_planning_model( const dynamics::PhysicalVehicleParameters& params )
{
  return [params]( const mas::State& x, const mas::Control& u ) -> mas::StateDerivative {
    mas::StateDerivative dxdt;
    dxdt.setZero( 5 );
    dxdt( 0 ) = x( 3 ) * std::cos( x( 2 ) );                    // x
    dxdt( 1 ) = x( 3 ) * std::sin( x( 2 ) );                    // y
    dxdt( 2 ) = x( 3 ) * std::tan( u( 0 ) ) / params.wheelbase; // yaw_angle
    dxdt( 3 ) = u( 1 );
    dxdt( 4 ) = x( 3 );
    return dxdt;
  };
}

mas::OCP
MultiAgentPlanner::create_single_ocp( size_t id )
{

  auto& participant = traffic_participants.participants.at( id );

  mas::OCP problem;
  problem.state_dim     = 5;
  problem.control_dim   = 2;
  problem.horizon_steps = horizon_steps;
  problem.dt            = dt;
  problem.initial_state = Eigen::VectorXd( problem.state_dim );
  problem.dynamics      = get_planning_model( participant.physical_parameters );

  // set bounds
  Eigen::VectorXd lower_bounds( problem.control_dim ), upper_bounds( problem.control_dim );
  lower_bounds << -participant.physical_parameters.steering_angle_max, participant.physical_parameters.acceleration_min;
  upper_bounds << participant.physical_parameters.steering_angle_max, participant.physical_parameters.acceleration_max;
  problem.input_lower_bounds = lower_bounds;
  problem.input_upper_bounds = upper_bounds;

  problem.stage_cost = [&, weights = weights, participant = participant, id = id]( const mas::State& x, const mas::State& u,
                                                                                   size_t time_idx ) -> double {
    double cost        = 0;
    double guide_speed = 2;
    // lane deviation cost
    if( participant.route )
    {
      double s               = x( 4 );
      auto   ref             = participant.route->get_pose_at_s( s );
      double curvature       = participant.route->get_curvature_at_s( s );
      auto   map_point       = participant.route->get_map_point_at_s( s );
      double max_curve_speed = max_speed;
      if( curvature > 0 )
        max_curve_speed = std::sqrt( max_lateral_acceleration / curvature );

      guide_speed = std::min( max_speed, max_curve_speed );
      if( map_point.max_speed )
        guide_speed = std::min( guide_speed, map_point.max_speed.value() );

      const double dx = x( 0 ) - ref.x;
      const double dy = x( 1 ) - ref.y;

      const double cref_yaw = math::fast_cos( ref.yaw );
      const double sref_yaw = math::fast_sin( ref.yaw );

      const double lon_err = dx * cref_yaw + dy * sref_yaw;
      const double lat_err = -dx * sref_yaw + dy * cref_yaw;

      cost += weights.lane_error * lat_err * lat_err;
      cost += pow( math::normalize_angle( ref.yaw - x( 2 ) ), 2 ) * weights.heading_error;
    }

    double       heading = x( 2 );
    const double cyaw    = math::fast_cos( heading );
    const double syaw    = math::fast_sin( heading );

    double max_dist  = 100.0;
    double speed_obj = 0.0;

    for( const auto& [other_id, other_ocp_ptr] : aggregator.agent_ocps )
    {
      if( other_id == id )
        continue;

      const auto& other_state = other_ocp_ptr->best_states.col( time_idx );

      const double rel_x = other_state( 0 ) - x( 0 );
      const double rel_y = other_state( 1 ) - x( 1 );

      const double long_dist = rel_x * cyaw + rel_y * syaw;
      const double lat_dist  = -rel_x * syaw + rel_y * cyaw;

      if( long_dist > 2 && std::fabs( lat_dist ) < 3 && long_dist < max_dist )
      {
        max_dist  = long_dist;
        speed_obj = other_state( 3 );
      }
    }

    double remaining_route_length = participant.route ? participant.route->get_length() - x( 4 ) : 100;

    double idm_acc = idm::calculate_idm_acc( remaining_route_length, max_dist, guide_speed, idm_time_headway, 8.0, x( 3 ),
                                             participant.physical_parameters.steering_angle_max, speed_obj );

    cost += weights.speed_error * std::pow( idm_acc - u( 1 ), 2 );
    // control effort
    cost += weights.steering_angle * u( 0 ) * u( 0 );


    return cost;
  };

  problem.terminal_cost = []( const mas::State& x ) -> double { return 0.0; };

  auto& state = participant.state;

  double s = participant.route ? participant.route->get_s( state ) : 0;

  problem.initial_state << state.x, state.y, state.yaw_angle, state.vx, s;
  problem.initialize_problem();
  problem.verify_problem();
  return problem;
}

void
MultiAgentPlanner::solve_problem()
{

  aggregator.reset();
  assign_routes();
  mas::SolverParams inner_params{
    { "max_iterations",   60 },
    {      "tolerance", 1e-5 },
    {         "max_ms",   80 },
    {          "debug",  1.0 }
  };
  const size_t max_outer = 1;

  // create ocp for each participant
  for( auto& [id, participant] : traffic_participants.participants )
  {
    auto single_ocp           = create_single_ocp( id );
    aggregator.agent_ocps[id] = std::make_shared<mas::OCP>( single_ocp );
  }
  aggregator.compute_offsets();

  aggregator.solve_decentralized_line_search<mas::OSQPCollocation>( max_outer, inner_params );
}

void
MultiAgentPlanner::extract_trajectories()
{
  for( auto& [id, participant] : traffic_participants.participants )
  {
    dynamics::Trajectory trajectory;
    auto                 problem = aggregator.agent_ocps[id];
    trajectory.states.reserve( problem->horizon_steps );
    for( size_t i = 0; i < problem->horizon_steps; ++i )
    {
      dynamics::VehicleStateDynamic state;
      auto                          x = problem->best_states.col( i );
      auto                          u = problem->best_controls.col( i );

      state.x              = x( 0 );
      state.y              = x( 1 );
      state.yaw_angle      = x( 2 );
      state.vx             = x( 3 );
      state.time           = participant.state.time + i * dt;
      state.steering_angle = u( 0 );
      state.ax             = u( 1 );

      trajectory.states.push_back( state );
    }
    participant.trajectory = trajectory;
  }
}

void
MultiAgentPlanner::assign_routes()
{
  const double max_route_length = 100.0;
  for( auto& [id, participant] : traffic_participants.participants )
  {
    if( !participant.route && participant.classification != dynamics::PEDESTRIAN )
    {
      participant.route = map::get_default_route( participant.state, max_route_length, local_map );
    }
  }
}

} // namespace planner
} // namespace adore
