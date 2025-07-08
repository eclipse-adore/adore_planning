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
    dxdt.setZero( 4 );
    dxdt( 0 ) = x( 3 ) * std::cos( x( 2 ) );                    // x
    dxdt( 1 ) = x( 3 ) * std::sin( x( 2 ) );                    // y
    dxdt( 2 ) = x( 3 ) * std::tan( u( 0 ) ) / params.wheelbase; // yaw_angle
    dxdt( 3 ) = u( 1 );                                         // v
    return dxdt;
  };
}

mas::OCP
MultiAgentPlanner::create_single_ocp( size_t id )
{

  auto& participant = traffic_participants.participants.at( id );

  mas::OCP problem;
  problem.state_dim     = 4;
  problem.control_dim   = 2;
  problem.horizon_steps = horizon_steps;
  problem.dt            = dt;
  problem.initial_state = Eigen::VectorXd( 4 );
  problem.dynamics      = get_planning_model( participant.physical_parameters );

  // set bounds
  Eigen::VectorXd lower_bounds( problem.control_dim ), upper_bounds( problem.control_dim );
  lower_bounds << -participant.physical_parameters.steering_angle_max, participant.physical_parameters.acceleration_min;
  upper_bounds << participant.physical_parameters.steering_angle_max, participant.physical_parameters.acceleration_max;
  problem.input_lower_bounds = lower_bounds;
  problem.input_upper_bounds = upper_bounds;

  problem.stage_cost = [&, weights = weights, participant = participant]( const mas::State& x, const mas::State& u,
                                                                          size_t time_idx ) -> double {
    double cost        = 0;
    double guide_speed = 5.0;
    // lane deviation cost
    if( participant.route )
    {
      math::Point2d p( x( 0 ), x( 1 ) );
      double        s = participant.route->get_s( p );
      // std::cerr << "!! s - " << s << std::endl;
      auto ref = participant.route->get_pose_at_s( s );

      const double dx = x( 0 ) - ref.x;
      const double dy = x( 1 ) - ref.y;

      const double cyaw = std::cos( ref.yaw );
      const double syaw = std::sin( ref.yaw );

      const double lon_err = dx * cyaw + dy * syaw;
      const double lat_err = -dx * syaw + dy * cyaw;


      cost += weights.lane_error * lat_err * lat_err;
      // cost += pow( ref_pose.yaw - x( 2 ), 2 );
    }

    // speed deviation cost
    cost += pow( x( 3 ) - guide_speed, 2 ) * weights.speed_error;

    // control effort
    cost += weights.steering_angle * u( 0 ) * u( 0 ) + weights.acceleration * u( 1 ) * u( 1 );


    // longitudinal interaction cost
    double          heading = x( 2 );
    Eigen::Vector2d heading_vec( std::cos( heading ), std::sin( heading ) );

    for( const auto& [other_id, other_ocp_ptr] : aggregator.agent_ocps )
    {
      if( other_id == id )
        continue;

      const auto& other_state = other_ocp_ptr->initial_states.col( time_idx );

      Eigen::Vector2d rel_pos           = other_state.head<2>() - x.head<2>();
      double          longitudinal_dist = rel_pos.dot( heading_vec );

      // Optional: only consider other agents in front
      if( longitudinal_dist > 2.0 && longitudinal_dist < 10.0 )
      {
        constexpr double gamma  = 0.5; // decay factor (tune as needed)
        cost                   += weights.proximity * std::exp( -gamma * longitudinal_dist * longitudinal_dist );
      }
    }

    return cost;
  };

  problem.terminal_cost = []( const mas::State& x ) -> double { return 0.0; };

  auto& s = participant.state;

  problem.initial_state << s.x, s.y, s.yaw_angle, s.vx;
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
    { "max_iterations",   20 },
    {      "tolerance", 1e-3 },
    {         "max_ms",   20 },
    {          "debug",  1.0 }
  };
  const size_t max_outer = 3;

  // create ocp for each participant
  for( auto& [id, participant] : traffic_participants.participants )
  {
    auto single_ocp           = create_single_ocp( id );
    aggregator.agent_ocps[id] = std::make_shared<mas::OCP>( single_ocp );
  }
  aggregator.compute_offsets();

  aggregator.solve_decentralized_trust_region<mas::OSQPCollocation>( max_outer, inner_params );
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
