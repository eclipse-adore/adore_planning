#include "planning/osqp_planner.hpp"

namespace adore
{
namespace planner
{

dynamics::Trajectory
OSQPPlanner::plan_trajectory( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                              const map::Map& latest_map, const dynamics::TrafficParticipantSet& traffic_participants )
{

  double initial_s = latest_route.get_s( current_state );

  // Generate speed profile
  speed_profile.generate_from_route_and_participants( latest_route, traffic_participants, current_state.vx, initial_s, 1.0, 5.0, 100 );


  OCP problem;

  // Dimensions
  problem.state_dim     = 4;
  problem.control_dim   = 2;
  problem.horizon_steps = 30;
  problem.dt            = 0.1; // 0.1 s per step.

  dynamics::Trajectory initial_guess_trajectory = generate_trajectory_from_speed_profile( speed_profile, latest_route, problem.dt );

  // set best states and controls to the initial guess in opposite of end of function
  problem.initial_states   = Eigen::MatrixXd::Zero( problem.state_dim, problem.horizon_steps + 1 );
  problem.initial_controls = Eigen::MatrixXd::Zero( problem.control_dim, problem.horizon_steps );

  for( int i = 0; i < problem.horizon_steps - 1; ++i )
  {
    const auto& state = initial_guess_trajectory.states.at( i );
    problem.initial_states.col( i ) << state.x, state.y, state.yaw_angle, state.vx;
    problem.initial_controls.col( i ) << state.steering_angle, state.ax;
  }
  // Initial state: for example, X=1, Y=1, psi=1, vx=1
  problem.initial_state = Eigen::VectorXd::Zero( problem.state_dim );
  problem.initial_state << current_state.x, current_state.y, current_state.yaw_angle, current_state.vx;

  // Dynamics: use the dynamic_bicycle_model defined in your code.
  problem.dynamics = single_track_model;

  // Cost weights.
  const double w_lane  = 1.0;   // Penalize lateral deviation.
  const double w_speed = 1.0;   // Penalize speed error.
  const double w_delta = 0.001; // Penalize steering.
  const double w_acc   = 0.001; // Penalize acceleration.

  // Stage cost function.
  problem.stage_cost = [=]( const State& state, const Control& control ) -> double {
    double s = latest_route.get_s( adore::math::Point2d( state( 0 ), state( 1 ) ) );

    auto ref_pose  = latest_route.get_pose_at_s( s );
    auto ref_speed = speed_profile.get_speed_at_s( s + 0.5 );

    double vx = state( 3 );

    // Unpack control: steering delta and acceleration.
    double delta = control( 0 );
    double a_cmd = control( 1 );

    double dx            = state( 0 ) - ref_pose.x;
    double dy            = state( 1 ) - ref_pose.y;
    double heading_error = math::normalize_angle( state( 2 ) - ref_pose.yaw );

    // Rotate the position error into the reference frame.
    double lateral_error = dx * dx + dy * dy;
    // Speed error.
    double speed_error = vx - ref_speed;

    // Total cost.
    double cost  = 0.0;
    cost        += w_lane * lateral_error;
    cost        += w_speed * speed_error * speed_error;
    cost        += w_delta * delta * delta;
    cost        += w_acc * a_cmd * a_cmd;
    cost        += heading_error * heading_error; // Penalize heading error.
    return cost;
  };

  // Terminal cost (set to zero here, can be modified if needed).
  problem.terminal_cost = [=]( const State& state ) -> double { return 0.0; };
  // --- Add analytic derivatives for the cost function. ---

  problem.dynamics_state_jacobian = []( const MotionModel& /*dyn*/, const State& x, const Control& u ) -> Eigen::MatrixXd {
    return single_track_state_jacobian( x, u );
  };
  problem.dynamics_control_jacobian = []( const MotionModel& /*dyn*/, const State& x, const Control& u ) -> Eigen::MatrixXd {
    return single_track_control_jacobian( x, u );
  };


  Eigen::VectorXd lower_bounds( 2 ), upper_bounds( 2 );
  lower_bounds << -0.7, -1.0;
  upper_bounds << 0.7, 1.0;
  problem.input_lower_bounds = lower_bounds;
  problem.input_upper_bounds = upper_bounds;

  // Initialize and verify the problem.
  problem.initialize_problem();


  SolverParams params;
  params["max_iterations"] = 50;
  params["tolerance"]      = 1e-4;

  auto solver_start = std::chrono::steady_clock::now();

  ilqr_solver( problem, params );

  auto solver_end  = std::chrono::steady_clock::now();
  auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>( solver_end - solver_start ).count();

  std::cerr << "[Timing] osqp_solver took " << duration_ms << " ms" << std::endl;

  dynamics::Trajectory trajectory;
  double               start_time = current_state.time;
  for( size_t i = 0; i < problem.horizon_steps; i++ )
  {
    dynamics::VehicleStateDynamic state;
    auto                          opt_state   = problem.best_states.col( i );
    auto                          opt_control = problem.best_controls.col( i );
    state.x                                   = opt_state( 0 );
    state.y                                   = opt_state( 1 );
    state.yaw_angle                           = opt_state( 2 );
    state.vx                                  = opt_state( 3 );
    state.time                                = problem.dt * i + start_time;
    state.ax                                  = opt_control( 1 );
    state.steering_angle                      = opt_control( 0 );
    trajectory.states.push_back( state );
  }
  std::cerr << "best cost : " << problem.best_cost << std::endl;

  return trajectory;
}

} // namespace planner
} // namespace adore
