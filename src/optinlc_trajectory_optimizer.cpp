/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Sanath Himasekhar Konthala
 ********************************************************************************/
#include "planning/optinlc_trajectory_optimizer.hpp"

namespace adore
{
namespace planner
{
void
OptiNLCTrajectoryOptimizer::set_parameters( const std::map<std::string, double>& params )
{
  options.intermediateIntegration = 3;
  options.OptiNLC_ACC             = 1e-4;
  options.maxNumberOfIteration    = 500;
  options.OSQP_verbose            = false;
  options.OSQP_max_iter           = 500;
  options.OptiNLC_time_limit      = 0.09;
  options.perturbation            = 1e-6;
  options.timeStep                = sim_time / control_points;

  for( const auto& [name, value] : params )
  {
    if( name == "intermediate_integration" )
      options.intermediateIntegration = static_cast<int>( value );
    if( name == "max_iterations" )
      options.maxNumberOfIteration = static_cast<int>( value );
    if( name == "osqp_max_iterations" )
      options.OSQP_max_iter = static_cast<int>( value );
    if( name == "OptiNLC_ACC" )
      options.OptiNLC_ACC = value;
    if( name == "time_limit" )
      options.OptiNLC_time_limit = value;
  }
  options.OSQP_verbose = false;
  options.timeStep     = sim_time / control_points;
}

void
OptiNLCTrajectoryOptimizer::setup_constraints(
  OptiNLC_OCP<double, OptiNLCTrajectoryOptimizer::input_size, OptiNLCTrajectoryOptimizer::state_size, 0,
              OptiNLCTrajectoryOptimizer::control_points>& ocp )
{

  // Define a simple input update method
  ocp.setInputUpdate( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& input, double, void* ) {
    VECTOR<double, input_size> update_input = { input[DELTA], input[ACC] };
    return update_input;
  } );

  // State Constraints
  ocp.setUpdateStateLowerBounds( [&]( const VECTOR<double, OptiNLCTrajectoryOptimizer::state_size>&, const VECTOR<double, input_size>& ) {
    VECTOR<double, OptiNLCTrajectoryOptimizer::state_size> state_constraints;
    state_constraints.setConstant( -std::numeric_limits<double>::infinity() );
    return state_constraints;
  } );

  ocp.setUpdateStateUpperBounds( [&]( const VECTOR<double, OptiNLCTrajectoryOptimizer::state_size>&, const VECTOR<double, input_size>& ) {
    VECTOR<double, OptiNLCTrajectoryOptimizer::state_size> state_constraints;
    state_constraints.setConstant( std::numeric_limits<double>::infinity() );
    return state_constraints;
  } );

  // Input Constraints
  ocp.setUpdateInputLowerBounds( [&]( const VECTOR<double, OptiNLCTrajectoryOptimizer::state_size>&, const VECTOR<double, input_size>& ) {
    VECTOR<double, input_size> input_constraints;
    input_constraints[0] = -limits.max_steering_angle;
    input_constraints[1] = limits.min_acceleration;
    return input_constraints;
  } );

  ocp.setUpdateInputUpperBounds( [&]( const VECTOR<double, OptiNLCTrajectoryOptimizer::state_size>&, const VECTOR<double, input_size>& ) {
    VECTOR<double, input_size> input_constraints;
    input_constraints[0] = limits.max_steering_angle;
    input_constraints[1] = limits.max_acceleration;
    return input_constraints;
  } );

  // Define a functions constraints method
  ocp.setUpdateFunctionConstraints( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& ) {
    VECTOR<double, constraints_size> functions_constraint;
    functions_constraint.setConstant( 0.0 );
    return functions_constraint;
  } );

  ocp.setUpdateFunctionConstraintsLowerBounds( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& ) {
    VECTOR<double, constraints_size> functions_constraint;
    functions_constraint.setConstant( -std::numeric_limits<double>::infinity() );
    return functions_constraint;
  } );

  ocp.setUpdateFunctionConstraintsUpperBounds( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& ) {
    VECTOR<double, constraints_size> functions_constraint;
    functions_constraint.setConstant( std::numeric_limits<double>::infinity() );
    return functions_constraint;
  } );
}

void
OptiNLCTrajectoryOptimizer::setup_objective_function(
  OptiNLC_OCP<double, OptiNLCTrajectoryOptimizer::input_size, OptiNLCTrajectoryOptimizer::state_size, 0,
              OptiNLCTrajectoryOptimizer::control_points>& ocp )
{
  ocp.setObjectiveFunction(
    [&]( const VECTOR<double, OptiNLCTrajectoryOptimizer::state_size>& state, const VECTOR<double, input_size>&, double ) {
      return state[L]; // Minimize the cost function `L`
    } );
}

// Public method to get the next vehicle command based onOptiNLCTrajectoryOptimizer::::Trajectory
dynamics::Trajectory
OptiNLCTrajectoryOptimizer::plan_trajectory( const dynamics::Trajectory&          reference_trajectory,
                                             const dynamics::VehicleStateDynamic& current_state )
{
  auto start_time = std::chrono::high_resolution_clock::now();

  // Initial state and input
  VECTOR<double, OptiNLCTrajectoryOptimizer::input_size> initial_input = { current_state.steering_angle, 0.25 };
  VECTOR<double, OptiNLCTrajectoryOptimizer::state_size> initial_state = { current_state.x, current_state.y, current_state.yaw_angle,
                                                                           current_state.vx, 0.0 };

  // Create an MPC problem (OCP)
  OptiNLC_OCP<double, OptiNLCTrajectoryOptimizer::input_size, OptiNLCTrajectoryOptimizer::state_size, 0,
              OptiNLCTrajectoryOptimizer::control_points>
    ocp( &options );

  // Set up dynamic model, objective, and constraints
  setup_dynamic_model( ocp, reference_trajectory );

  setup_objective_function( ocp );

  setup_constraints( ocp );

  // Solve the MPC problem
  OptiNLC_Solver<double, input_size, state_size, 0, control_points> solver( ocp );

  solver.solve( current_state.time, initial_state, initial_input );

  auto opt_x = solver.get_optimal_states();

  dynamics::Trajectory planned_trajectory;
  for( int i = 0; i < control_points; i++ )
  {
    dynamics::VehicleStateDynamic state;
    state.x         = opt_x[i * state_size + X];
    state.y         = opt_x[i * state_size + Y];
    state.yaw_angle = opt_x[i * state_size + PSI];
    state.vx        = opt_x[i * state_size + V];
    state.time      = current_state.time + i * options.timeStep;
    planned_trajectory.states.push_back( state );
  }

  // Calculate time taken
  auto                          end_time        = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_seconds = end_time - start_time;

  return planned_trajectory;
}

void
OptiNLCTrajectoryOptimizer::setup_dynamic_model(
  OptiNLC_OCP<double, OptiNLCTrajectoryOptimizer::input_size, OptiNLCTrajectoryOptimizer::state_size, 0,
              OptiNLCTrajectoryOptimizer::control_points>& ocp,
  const dynamics::Trajectory&                              trajectory )
{
  ocp.setDynamicModel( [&]( const VECTOR<double, OptiNLCTrajectoryOptimizer::state_size>& state,
                            const VECTOR<double, OptiNLCTrajectoryOptimizer::input_size>& input,
                            VECTOR<double, OptiNLCTrajectoryOptimizer::state_size>& derivative, double current_time, void* ) {
    const double wheelbase = 2.69; // wheelbase, can be tuned based on your vehicle

    // Dynamic model equations
    derivative[X]   = state[V] * cos( state[PSI] );               // X derivative (velocity * cos(psi))
    derivative[Y]   = state[V] * sin( state[PSI] );               // Y derivative (velocity * sin(psi))
    derivative[PSI] = state[V] * tan( input[DELTA] ) / wheelbase; // PSI derivative (steering angle / wheelbase)
    derivative[V]   = input[ACC];                                 // Velocity derivative (acceleration)

    // Reference trajectory point at current time
    auto reference_point = trajectory.get_state_at_time( current_time );

    // Position error terms
    double dx = state[X] - reference_point.x;
    double dy = state[Y] - reference_point.y;
    double dv = state[V] - reference_point.vx;

    // Calculate longitudinal and lateral errors relative to the vehicle's heading
    double cos_yaw = cos( reference_point.yaw_angle );
    double sin_yaw = sin( reference_point.yaw_angle );

    double longitudinal_cost  = dx * cos_yaw + dy * sin_yaw;
    longitudinal_cost        *= longitudinal_cost * longitudinal_weight;

    double lateral_cost  = -dx * sin_yaw + dy * cos_yaw;
    lateral_cost        *= lateral_cost * lateral_weight;


    double velocity_cost = dv * dv * velocity_weight;

    // Heading error term
    double heading_cost  = adore::math::normalize_angle( reference_point.yaw_angle - state[PSI] );
    heading_cost        *= heading_cost * heading_weight;

    // Steering input cost
    double steering_cost = input[DELTA] * input[DELTA] * steering_weight;

    // Total cost derivative
    derivative[L] = longitudinal_cost + lateral_cost + heading_cost + steering_cost + velocity_cost;
  } );
}

OptiNLCTrajectoryOptimizer::OptiNLCTrajectoryOptimizer()
{
  options.setDefaults();
  set_parameters( {} );
}
} // namespace planner
} // namespace adore
