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
#include <stdexcept>
#include <iostream>

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
  ocp.setInputUpdate( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& input, double, void* ) {
    VECTOR<double, input_size> update_input = { input[DELTA], input[ACC] };
    return update_input;
  } );

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
      return state[L];
    } );
}

dynamics::Trajectory
OptiNLCTrajectoryOptimizer::plan_trajectory( const dynamics::Trajectory&          reference_trajectory,
                                             const dynamics::VehicleStateDynamic& current_state )
{
  // Validate reference trajectory
  if( reference_trajectory.states.empty() )
  {
    throw std::invalid_argument( "Reference trajectory is empty" );
  }

  // Check if trajectory covers the required time horizon
  double trajectory_duration = reference_trajectory.states.back().time - reference_trajectory.states.front().time;
  double required_duration = sim_time;
  
  if( trajectory_duration < required_duration )
  {
    std::cerr << "Warning: Reference trajectory duration (" << trajectory_duration 
              << "s) is shorter than required optimization horizon (" << required_duration << "s)" << std::endl;
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  VECTOR<double, OptiNLCTrajectoryOptimizer::input_size> initial_input = { current_state.steering_angle, 0.25 };
  VECTOR<double, OptiNLCTrajectoryOptimizer::state_size> initial_state = { current_state.x, current_state.y, current_state.yaw_angle,
                                                                           current_state.vx, 0.0 };

  OptiNLC_OCP<double, OptiNLCTrajectoryOptimizer::input_size, OptiNLCTrajectoryOptimizer::state_size, 0,
              OptiNLCTrajectoryOptimizer::control_points>
    ocp( &options );

  try
  {
    setup_dynamic_model( ocp, reference_trajectory );
    setup_objective_function( ocp );
    setup_constraints( ocp );

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

    auto                          end_time        = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;

    return planned_trajectory;
  }
  catch( const std::exception& e )
  {
    std::cerr << "Exception in trajectory optimization: " << e.what() << std::endl;
    
    // Return a fallback trajectory with just the current state
    dynamics::Trajectory fallback_trajectory;
    fallback_trajectory.states.push_back( current_state );
    return fallback_trajectory;
  }
}

void
OptiNLCTrajectoryOptimizer::setup_dynamic_model(
  OptiNLC_OCP<double, OptiNLCTrajectoryOptimizer::input_size, OptiNLCTrajectoryOptimizer::state_size, 0,
              OptiNLCTrajectoryOptimizer::control_points>& ocp,
  const dynamics::Trajectory&                              trajectory )
{
  // Store trajectory bounds for safe access
  double trajectory_start_time = trajectory.states.front().time;
  double trajectory_end_time = trajectory.states.back().time;
  
  ocp.setDynamicModel( [&, trajectory_start_time, trajectory_end_time]( 
                            const VECTOR<double, OptiNLCTrajectoryOptimizer::state_size>& state,
                            const VECTOR<double, OptiNLCTrajectoryOptimizer::input_size>& input,
                            VECTOR<double, OptiNLCTrajectoryOptimizer::state_size>& derivative, 
                            double current_time, void* ) {
    const double wheelbase = 2.69;

    derivative[X]   = state[V] * cos( state[PSI] );
    derivative[Y]   = state[V] * sin( state[PSI] );
    derivative[PSI] = state[V] * tan( input[DELTA] ) / wheelbase;
    derivative[V]   = input[ACC];

    // Safe trajectory access with bounds checking
    dynamics::VehicleStateDynamic reference_point;
    
    try
    {
      // Clamp current_time to valid trajectory range
      double clamped_time = std::max( trajectory_start_time, std::min( current_time, trajectory_end_time ) );
      reference_point = trajectory.get_state_at_time( clamped_time );
    }
    catch( const std::exception& e )
    {
      // Fallback: use the closest available state
      if( current_time <= trajectory_start_time )
      {
        reference_point = trajectory.states.front();
      }
      else
      {
        reference_point = trajectory.states.back();
      }
      
      std::cerr << "Warning: Failed to get reference state at time " << current_time 
                << ", using fallback. Error: " << e.what() << std::endl;
    }

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

    double heading_cost  = adore::math::normalize_angle( reference_point.yaw_angle - state[PSI] );
    heading_cost        *= heading_cost * heading_weight;

    double steering_cost = input[DELTA] * input[DELTA] * steering_weight;

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
