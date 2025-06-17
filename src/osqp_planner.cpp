#include "planning/osqp_planner.hpp"

#include "controllers/iLQR.hpp"

namespace adore
{
namespace planner
{

struct PlannerCostWeights
{
  double lane_error             = 50.0;
  double long_error             = 1.0;
  double speed_error            = 50.0;
  double heading_error          = 10.0;
  double steering_angle         = 0.01;
  double acceleration           = 0.01;
  double steering_rate          = 0.1;
  double jerk                   = 0.1;
  double terminal_heading_error = 50.0;
  double terminal_speed_error   = 10.0;
  double terminal_lateral_error = 100.0;
};

// inline Eigen::VectorXd
// analytic_cost_state_gradient( const State& x, const Control& u, size_t time_idx, const dynamics::Trajectory& ref_traj, double dt,
//                               const PlannerCostWeights& weights )
// {
//   Eigen::VectorXd grad = Eigen::VectorXd::Zero( 4 );

// double t   = time_idx * dt;
// auto   ref = ref_traj.get_state_at_time( t );

// double dx = x( 0 ) - ref.x;
// double dy = x( 1 ) - ref.y;

// double cos_yaw = std::cos( ref.yaw_angle );
// double sin_yaw = std::sin( ref.yaw_angle );

// double lateral_error      = -dx * sin_yaw + dy * cos_yaw;
// double longitudinal_error = dx * cos_yaw + dy * sin_yaw;
// double heading_error      = math::normalize_angle( x( 2 ) - ref.yaw_angle );
// double speed_error        = x( 3 ) - ref.vx;

// // d(cost)/d(x, y)
// grad( 0 ) += 2.0 * weights.lane_error * lateral_error * ( -sin_yaw );
// grad( 1 ) += 2.0 * weights.lane_error * lateral_error * cos_yaw;

// grad( 0 ) += weights.long_error * cos_yaw;
// grad( 1 ) += weights.long_error * sin_yaw;

// // d(cost)/d(psi)
// grad( 2 ) += 2.0 * weights.heading_error * heading_error;

// // d(cost)/d(vx)
// grad( 3 ) += 2.0 * weights.speed_error * speed_error;

// return grad;
// }

// inline Eigen::VectorXd
// analytic_cost_control_gradient( const State& x, const Control& u, const PlannerCostWeights& weights )
// {
//   Eigen::VectorXd grad = Eigen::VectorXd::Zero( 2 );

// grad( 0 ) = 2.0 * weights.steering_angle * u( 0 );
// grad( 1 ) = 2.0 * weights.acceleration * u( 1 );

// return grad;
// }

// inline Eigen::MatrixXd
// analytic_cost_state_hessian( const State& x, const Control& u, size_t time_idx, const dynamics::Trajectory& ref_traj, double dt,
//                              const PlannerCostWeights& weights )
// {
//   Eigen::MatrixXd H = Eigen::MatrixXd::Zero( 4, 4 );

// double t   = time_idx * dt;
// auto   ref = ref_traj.get_state_at_time( t );

// double dx = x( 0 ) - ref.x;
// double dy = x( 1 ) - ref.y;

// double cos_yaw = std::cos( ref.yaw_angle );
// double sin_yaw = std::sin( ref.yaw_angle );

// // Second derivatives of lateral error terms wrt x/y
// double d_lat_dx = -sin_yaw;
// double d_lat_dy = cos_yaw;

// H( 0, 0 ) += 2.0 * weights.lane_error * d_lat_dx * d_lat_dx;
// H( 1, 1 ) += 2.0 * weights.lane_error * d_lat_dy * d_lat_dy;
// H( 0, 1 ) += 2.0 * weights.lane_error * d_lat_dx * d_lat_dy;
// H( 1, 0 )  = H( 0, 1 ); // symmetric

// // Add heading and speed error curvature
// H( 2, 2 ) += 2.0 * weights.heading_error;
// H( 3, 3 ) += 2.0 * weights.speed_error;

// return H;
// }

// inline Eigen::MatrixXd
// analytic_cost_control_hessian( const State& x, const Control& u, const PlannerCostWeights& weights )
// {
//   Eigen::MatrixXd H = Eigen::MatrixXd::Zero( 2, 2 );
//   H( 0, 0 )         = 2.0 * weights.steering_angle;
//   H( 1, 1 )         = 2.0 * weights.acceleration;
//   return H;
// }

// inline Eigen::MatrixXd
// analytic_cost_cross_term( const State& x, const Control& u )
// {
//   return Eigen::MatrixXd::Zero( 2, 4 ); // Assuming decoupled state/control cost
// }

inline MotionModel
get_planning_model( const dynamics::PhysicalVehicleParameters& params )
{
  return [params]( const State& x, const Control& u ) -> StateDerivative {
    StateDerivative dxdt( 4 );
    dxdt( 0 ) = x( 3 ) * std::cos( x( 2 ) );
    dxdt( 1 ) = x( 3 ) * std::sin( x( 2 ) );
    dxdt( 2 ) = x( 3 ) * std::tan( u( 0 ) ) / params.wheelbase;
    dxdt( 3 ) = u( 1 );
    return dxdt;
  };
}

dynamics::Trajectory
OSQPPlanner::plan_trajectory( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                              const map::Map& latest_map, const dynamics::TrafficParticipantSet& traffic_participants )
{
  const double dt            = 0.1;
  const size_t horizon_steps = 30;

  double initial_s = latest_route.get_s( current_state );

  speed_profile.generate_from_route_and_participants( latest_route, traffic_participants, current_state.vx, initial_s, current_state.time,
                                                      1.0, 5.0, 100 );

  auto ref_traj = generate_trajectory_from_speed_profile( speed_profile, latest_route, dt );
  // ref_traj.adjust_start_time( current_state.time );

  PlannerCostWeights weights;
  OCP                problem;
  problem.state_dim     = 4;
  problem.control_dim   = 2;
  problem.horizon_steps = horizon_steps;
  problem.dt            = dt;
  problem.initial_state = Eigen::VectorXd( 4 );
  problem.initial_state << current_state.x, current_state.y, current_state.yaw_angle, current_state.vx;
  problem.dynamics = get_planning_model( speed_profile.vehicle_params );

  problem.stage_cost = [=]( const State& x, const Control& u, size_t time_idx ) -> double {
    double t         = time_idx * dt;
    auto   ref_state = ref_traj.get_state_at_time( t );

    double dx = x( 0 ) - ref_state.x;
    double dy = x( 1 ) - ref_state.y;

    double ref_cos            = std::cos( ref_state.yaw_angle );
    double ref_sin            = std::sin( ref_state.yaw_angle );
    double longitudinal_error = dx * ref_cos + dy * ref_sin;
    double lateral_error      = -dx * ref_sin + dy * ref_cos;
    double heading_error      = math::normalize_angle( x( 2 ) - ref_state.yaw_angle );
    double speed_error        = x( 3 ) - ref_state.vx;

    double cost  = 0.0;
    cost        += weights.lane_error * lateral_error * lateral_error;
    cost        += weights.speed_error * speed_error * speed_error;
    cost        += weights.heading_error * heading_error * heading_error;
    cost        += weights.steering_angle * u( 0 ) * u( 0 );
    cost        += weights.acceleration * u( 1 ) * u( 1 );
    cost        += weights.long_error * longitudinal_error * longitudinal_error;

    return cost;
  };
  problem.terminal_cost = []( const State& x ) -> double { return 0.0; };

  // problem.cost_state_gradient = [=]( const StageCostFunction&, const State& x, const Control& u, size_t t ) {
  //   return analytic_cost_state_gradient( x, u, t, ref_traj, dt, weights );
  // };
  // problem.cost_control_gradient = [=]( const StageCostFunction&, const State& x, const Control& u, size_t t ) {
  //   return analytic_cost_control_gradient( x, u, weights );
  // };
  // problem.cost_state_hessian = [=]( const StageCostFunction&, const State& x, const Control& u, size_t t ) {
  //   return analytic_cost_state_hessian( x, u, t, ref_traj, dt, weights );
  // };
  // problem.cost_control_hessian = [=]( const StageCostFunction&, const State& x, const Control& u, size_t t ) {
  //   return analytic_cost_control_hessian( x, u, weights );
  // };
  // problem.cost_cross_term = [=]( const StageCostFunction&, const State& x, const Control& u, size_t t ) {
  //   return analytic_cost_cross_term( x, u );
  // };


  Eigen::VectorXd lower_bounds( problem.control_dim ), upper_bounds( problem.control_dim );
  lower_bounds << -speed_profile.vehicle_params.steering_angle_max, speed_profile.vehicle_params.acceleration_min;
  upper_bounds << speed_profile.vehicle_params.steering_angle_max, speed_profile.vehicle_params.acceleration_max;
  problem.input_lower_bounds = lower_bounds;
  problem.input_upper_bounds = upper_bounds;

  if( last_problem )
  {
    problem.initial_controls = last_problem->best_controls;
    for( size_t i = 0; i < problem.horizon_steps - 1; ++i )
    {
      problem.initial_controls.col( i ) = last_problem->best_controls.col( i + 1 );
    }
    problem.initial_controls.col( problem.horizon_steps - 1 ) = problem.initial_controls.col( problem.horizon_steps - 2 );
  }

  problem.initialize_problem();
  problem.verify_problem();

  SolverParams params;
  params["max_iterations"] = 1000;
  params["tolerance"]      = 1e-8;
  params["max_ms"]         = 30;
  params["debug"]          = 1.0;


  // You can switch to ilqr_solver here if needed
  // osqp_solver( problem, params );
  ilqr_solver( problem, params );
  cgd_solver( problem, params );


  dynamics::Trajectory trajectory;
  for( size_t i = 0; i < problem.horizon_steps; ++i )
  {
    dynamics::VehicleStateDynamic state;
    auto                          x = problem.best_states.col( i );
    auto                          u = problem.best_controls.col( i );

    state.x              = x( 0 );
    state.y              = x( 1 );
    state.yaw_angle      = x( 2 );
    state.vx             = x( 3 );
    state.time           = current_state.time + i * dt;
    state.steering_angle = u( 0 );
    state.ax             = u( 1 );

    trajectory.states.push_back( state );
  }
  // last_problem = problem;
  trajectory.adjust_start_time( current_state.time );
  return trajectory;
}

} // namespace planner
} // namespace adore
