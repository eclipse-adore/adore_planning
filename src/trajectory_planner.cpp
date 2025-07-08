#include "planning/trajectory_planner.hpp"

#include "controllers/iLQR.hpp"

namespace adore
{
namespace planner
{

void
TrajectoryPlanner::set_parameters( const std::map<std::string, double>& params )
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

void
TrajectoryPlanner::set_vehicle_parameters( const dynamics::PhysicalVehicleParameters& params )
{
  speed_profile.vehicle_params = params;
}

mas::MotionModel
TrajectoryPlanner::get_planning_model( const dynamics::PhysicalVehicleParameters& params )
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

mas::StageCostFunction
TrajectoryPlanner::make_trajectory_cost( const dynamics::Trajectory& ref_traj )
{
  return [ref_traj, weights = weights, dt = dt]( const mas::State& x, const mas::Control& u, std::size_t k ) -> double {
    const double t   = k * dt;
    const auto   ref = ref_traj.get_state_at_time( t );

    const double dx = x( 0 ) - ref.x;
    const double dy = x( 1 ) - ref.y;

    const double c = std::cos( ref.yaw_angle );
    const double s = std::sin( ref.yaw_angle );

    const double lon_err = dx * c + dy * s;
    const double lat_err = -dx * s + dy * c;
    const double hdg_err = math::normalize_angle( x( 2 ) - ref.yaw_angle );
    const double spd_err = x( 3 ) - ref.vx;

    return weights.lane_error * lat_err * lat_err + weights.long_error * lon_err * lon_err + weights.heading_error * hdg_err * hdg_err
         + weights.speed_error * spd_err * spd_err + weights.steering_angle * u( 0 ) * u( 0 ) + weights.acceleration * u( 1 ) * u( 1 );
  };
}

dynamics::Trajectory
TrajectoryPlanner::plan_route_trajectory( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                                          const dynamics::TrafficParticipantSet& traffic_participants )
{
  double initial_s = latest_route.get_s( current_state );
  speed_profile.generate_from_route_and_participants( latest_route, traffic_participants, current_state.vx, initial_s, current_state.time,
                                                      max_lateral_acceleration, idm_time_headway, ref_traj_length );
  return optimize_trajectory( current_state, generate_trajectory_from_speed_profile( speed_profile, latest_route, dt ) );
}

dynamics::Trajectory
TrajectoryPlanner::optimize_trajectory( const dynamics::VehicleStateDynamic& current_state, const dynamics::Trajectory& ref_traj )
{
  start_state          = current_state;
  reference_trajectory = ref_traj;
  setup_problem();
  solve_problem();
  return extract_trajectory();
}

void
TrajectoryPlanner::solve_problem() // THIS ONE WORKS
{
  mas::SolverParams params;
  params["max_iterations"] = solver_params.max_iterations;
  params["tolerance"]      = solver_params.tolerance;
  params["max_ms"]         = solver_params.max_ms;
  params["debug"]          = solver_params.debug;

  mas::OSQPCollocation osqp_collocation_solver;
  osqp_collocation_solver.set_params( params );
  mas::CGD cgd_solver;
  cgd_solver.set_params( params );

  auto solve_with = [&]( auto&& solver, double max_ms ) {
    params["max_ms"] = max_ms;
    solver.set_params( params );
    solver.solve( *problem );
  };

  // first pass with collocation
  solve_with( mas::OSQPCollocation{}, 50 );
  // then refine with CGD
  solve_with( mas::CGD{}, 10 );
}

dynamics::Trajectory
TrajectoryPlanner::extract_trajectory()
{
  dynamics::Trajectory trajectory;
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
    state.time           = start_state.time + i * dt;
    state.steering_angle = u( 0 );
    state.ax             = u( 1 );

    trajectory.states.push_back( state );
  }
  trajectory.adjust_start_time( start_state.time );

  return trajectory;
}

void
TrajectoryPlanner::setup_problem()
{
  problem = std::make_shared<mas::OCP>();

  problem->state_dim     = 4;
  problem->control_dim   = 2;
  problem->horizon_steps = horizon_steps;
  problem->dt            = dt;
  problem->initial_state = Eigen::VectorXd( 4 );
  problem->dynamics      = get_planning_model( speed_profile.vehicle_params );


  Eigen::VectorXd lower_bounds( problem->control_dim ), upper_bounds( problem->control_dim );
  lower_bounds << -speed_profile.vehicle_params.steering_angle_max, speed_profile.vehicle_params.acceleration_min;
  upper_bounds << speed_profile.vehicle_params.steering_angle_max, speed_profile.vehicle_params.acceleration_max;
  problem->input_lower_bounds = lower_bounds;
  problem->input_upper_bounds = upper_bounds;
  problem->stage_cost         = make_trajectory_cost( reference_trajectory );
  problem->terminal_cost      = []( const mas::State& x ) -> double { return 0.0; };

  problem->initial_state << start_state.x, start_state.y, start_state.yaw_angle, start_state.vx;
  problem->initialize_problem();
  problem->verify_problem();
}
} // namespace planner
} // namespace adore
