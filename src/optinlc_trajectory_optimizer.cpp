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
  options.intermediateIntegration = 2;
  options.OptiNLC_ACC             = 1e-3;
  options.maxNumberOfIteration    = 500;
  options.OSQP_verbose            = false;
  options.OSQP_max_iter           = 500;
  options.OptiNLC_time_limit      = 0.09;
  options.perturbation            = 1e-6;
  options.timeStep                = sim_time / control_points;
  options.debugPrint              = true;

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
OptiNLCTrajectoryOptimizer::setup_constraints( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp )
{

  // Define a simple input update method
  ocp.setInputUpdate( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& input, double, void* ) {
    VECTOR<double, input_size> update_input = { input[ddDELTA] };
    return update_input;
  } );

  // State Constraints
  ocp.setUpdateStateLowerBounds( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& ) {
    VECTOR<double, state_size> state_constraints;
    state_constraints.setConstant( -std::numeric_limits<double>::infinity() );
    state_constraints[V]      = max_reverse_speed;
    state_constraints[DELTA]  = -limits.max_steering_angle;
    state_constraints[dDELTA] = -max_steering_velocity;
    return state_constraints;
  } );

  ocp.setUpdateStateUpperBounds( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& ) {
    VECTOR<double, state_size> state_constraints;
    state_constraints.setConstant( std::numeric_limits<double>::infinity() );
    state_constraints[V]      = max_forward_speed;
    state_constraints[DELTA]  = limits.max_steering_angle;
    state_constraints[dDELTA] = max_steering_velocity;
    return state_constraints;
  } );

  // Input Constraints
  ocp.setUpdateInputLowerBounds( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& ) {
    VECTOR<double, input_size> input_constraints;
    input_constraints[ddDELTA] = -max_steering_acceleration;
    return input_constraints;
  } );

  ocp.setUpdateInputUpperBounds( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& ) {
    VECTOR<double, input_size> input_constraints;
    input_constraints[ddDELTA] = max_steering_acceleration;
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
OptiNLCTrajectoryOptimizer::setup_objective_function( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp )
{
  ocp.setObjectiveFunction( [&]( const VECTOR<double, state_size>& state, const VECTOR<double, input_size>&, double ) {
    return state[L]; // Minimize the cost function `L`
  } );
}

// Public method to get the next vehicle command based on OptiNLCTrajectoryOptimizer
dynamics::Trajectory
OptiNLCTrajectoryOptimizer::plan_trajectory( const dynamics::Trajectory&          latest_reference_trajectory,
                                             const dynamics::VehicleStateDynamic& current_state )
{
  reference_to_piecewise_polynomial reference_trajectory = setup_optimizer_parameters_using_reference( latest_reference_trajectory, current_state );
  auto                          start_time      = std::chrono::high_resolution_clock::now();

  // Initial state and input
  VECTOR<double, input_size> initial_input = { 0.0 };
  VECTOR<double, state_size> initial_state = {
    current_state.x, current_state.y, current_state.yaw_angle, current_state.vx, current_state.steering_angle, 0.0, 0.0, 0.0
  };

  // Create an MPC problem (OCP)
  OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points> ocp( &options );

  // Set up reference route
  setup_reference_trajectory( reference_trajectory );
  if( reference_trajectory_x.breaks.size() < 1 )
  {
    dynamics::Trajectory empty_trajectory;
    std::cerr << "end of route or invalid route received" << std::endl;
    return empty_trajectory;
  }
  reference_velocity = latest_reference_trajectory.states.back().vx;
  std::cerr << "reference velocity: " << reference_velocity << std::endl;

  // Set up dynamic model, objective, and constraints
  setup_dynamic_model( ocp );

  setup_objective_function( ocp );

  setup_constraints( ocp );

  // Solve the MPC problem
  OptiNLC_Solver<double, input_size, state_size, constraints_size, control_points> solver( ocp );

  solver.solve( current_state.time, initial_state, initial_input );

  auto   opt_x                   = solver.get_optimal_states();
  auto   time                    = solver.getTime();
  double last_objective_function = solver.get_final_objective_function();

  bad_condition = false;
  if( bad_counter > 4 )
  {
    bad_counter = 0;
  }
  for( size_t i = 0; i < control_points; i++ )
  {
    if( last_objective_function > threshold_bad_output || opt_x[i * state_size + V] > max_forward_speed
        || opt_x[i * state_size + V] < max_reverse_speed || std::abs( opt_x[i * state_size + dDELTA] ) > max_steering_velocity + 0.1 )
    {
      bad_condition  = true;
      bad_counter   += 1;
      break;
    }
  }

  dynamics::Trajectory planned_trajectory;
  for( size_t i = 0; i < control_points; i++ )
  {
    dynamics::VehicleStateDynamic state;
    state.x              = opt_x[i * state_size + X];
    state.y              = opt_x[i * state_size + Y];
    state.yaw_angle      = opt_x[i * state_size + PSI];
    state.vx             = opt_x[i * state_size + V];
    state.steering_angle = opt_x[i * state_size + DELTA];
    state.steering_rate  = opt_x[i * state_size + dDELTA];
    state.time           = time[i];
    if( i < control_points - 1 )
    {
      state.yaw_rate = ( opt_x[( i + 1 ) * state_size + PSI] - opt_x[i * state_size + PSI] ) / options.timeStep;
      state.ax       = ( opt_x[( i + 1 ) * state_size + V] - opt_x[i * state_size + V] ) / options.timeStep;
    }
    planned_trajectory.states.push_back( state );
  }
  planned_trajectory.states[control_points - 1].yaw_rate = planned_trajectory.states[control_points - 2].yaw_rate;
  planned_trajectory.states[control_points - 1].ax       = planned_trajectory.states[control_points - 2].ax;

  // Calculate time taken
  auto                          end_time        = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_seconds = end_time - start_time;

  // Log cost, time taken, and convergence status
  if( bad_condition == false && bad_counter < 5 )
  {
    previous_trajectory = planned_trajectory;
    bad_counter         = 0;
    return planned_trajectory;
  }
  else
  {
    return previous_trajectory;
  }
}

void
OptiNLCTrajectoryOptimizer::setup_dynamic_model( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp )
{
  ocp.setDynamicModel( [&]( const VECTOR<double, state_size>& state, const VECTOR<double, input_size>& input,
                            VECTOR<double, state_size>& derivative, double, void* ) {
    
    // Reference trajectory point at current progress
    int    index             = pp.findIndex( state[S], reference_trajectory_x );
    double reference_x       = pp.splineEvaluation( index, state[S], reference_trajectory_x );
    double reference_y       = pp.splineEvaluation( index, state[S], reference_trajectory_y );
    double reference_heading = pp.splineEvaluation( index, state[S], reference_trajectory_heading );
    if( reference_velocity - state[V] > 0 )
    {
      tau = 2.5; // Higher value for smooth acceleration
    }
    else
    {
      tau = 1.25; // Lower value for quick braking
    }

    // Dynamic model equations
    derivative[X]      = state[V] * cos( state[PSI] );                      // X derivative (velocity * cos(psi))
    derivative[Y]      = state[V] * sin( state[PSI] );                      // Y derivative (velocity * sin(psi))
    derivative[PSI]    = state[V] * tan( state[DELTA] ) / wheelbase;        // PSI derivative (steering angle / wheelbase)
    derivative[V]      = ( 1.0 / tau ) * ( reference_velocity - state[V] ); // Velocity derivative (first order)
    derivative[DELTA]  = state[dDELTA];                                     // Steering angle derivative
    derivative[dDELTA] = input[ddDELTA];                                    // Steering angle rate derivative
    derivative[S]      = state[V];                                          // Progress derivate (velocity)


    // Position error terms
    double dx = state[X] - reference_x;
    double dy = state[Y] - reference_y;

    // Calculate longitudinal and lateral errors relative to the vehicle's heading
    double cos_yaw = cos( reference_heading );
    double sin_yaw = sin( reference_heading );

    double lateral_cost  = -dx * sin_yaw + dy * cos_yaw;
    lateral_cost        *= lateral_cost * lateral_weight;

    // Heading error term
    double heading_cost  = atan2( -sin_yaw * cos( state[PSI] ) + cos_yaw * sin( state[PSI] ),
                                  cos_yaw * cos( state[PSI] ) + sin_yaw * sin( state[PSI] ) );
    heading_cost        *= heading_cost * heading_weight;

    // Steering input cost
    // double steering_cost = state[DELTA] * state[DELTA] * steering_weight;

    // Total cost derivative
    derivative[L] = lateral_cost + heading_cost;
  } );
}

OptiNLCTrajectoryOptimizer::OptiNLCTrajectoryOptimizer()
{
  options.setDefaults();
  set_parameters( {} );
}

void
OptiNLCTrajectoryOptimizer::setup_reference_trajectory( reference_to_piecewise_polynomial& reference_trajectory )
{
  reference_trajectory_x       = reference_trajectory.x;
  reference_trajectory_y       = reference_trajectory.y;
  reference_trajectory_v       = reference_trajectory.v;
  reference_trajectory_heading = reference_trajectory.heading;
}

reference_to_piecewise_polynomial
OptiNLCTrajectoryOptimizer::setup_optimizer_parameters_using_reference( const dynamics::Trajectory&          latest_reference_trajectory,
                                                                        const dynamics::VehicleStateDynamic& current_state )
{
  auto start_time = std::chrono::high_resolution_clock::now();

  reference_to_piecewise_polynomial reference;

  if( latest_reference_trajectory.states.size() < 2 )
  {
    return reference;
  }

  reference_to_follow.s.clear();
  reference_to_follow.x.clear();
  reference_to_follow.y.clear();
  reference_to_follow.psi.clear();

  std::vector<double> w;

  for (int i = 0; i < latest_reference_trajectory.states.size(); i++)
  {
    reference_to_follow.x.push_back( latest_reference_trajectory.states[i].x );
    reference_to_follow.y.push_back( latest_reference_trajectory.states[i].y );
    reference_to_follow.v.push_back( latest_reference_trajectory.states[i].vx );
    reference_to_follow.psi.push_back( latest_reference_trajectory.states[i].yaw_angle );
    w.push_back( 1.0 );
    if (i == 0)
    {
      reference_to_follow.s.push_back( 0.0 );
    }
    else
    {
      reference_to_follow.s.push_back( reference_to_follow.s[i-1] + sqrt( (reference_to_follow.x[i] - reference_to_follow.x[i-1]) * (reference_to_follow.x[i] - reference_to_follow.x[i-1])
                                      + (reference_to_follow.y[i] - reference_to_follow.y[i-1]) * (reference_to_follow.y[i] - reference_to_follow.y[i-1]) ) );
    }
  }

  if( reference_to_follow.s.size() < 3 ) // safety check for route
  {
    return reference;
  }
  reference_to_follow.s[0] = 0.0; // overwriting the first element to 0 (start from ego vehicle)
  reference.x              = pp.CubicSplineSmoother( reference_to_follow.s, reference_to_follow.x, w, 0.9 );
  reference.y              = pp.CubicSplineSmoother( reference_to_follow.s, reference_to_follow.y, w, 0.9 );
  reference.v              = pp.CubicSplineSmoother( reference_to_follow.s, reference_to_follow.v, w, 0.9 );
  reference.heading        = pp.CubicSplineSmoother( reference_to_follow.s, reference_to_follow.psi, w, 0.9 );

  // Calculate time taken
  auto                          end_time        = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_seconds = end_time - start_time;

  return reference;
}
} // namespace planner
} // namespace adore
