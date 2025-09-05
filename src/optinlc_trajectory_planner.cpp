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
#include "planning/optinlc_trajectory_planner.hpp"

namespace adore
{
namespace planner
{

void
OptiNLCTrajectoryPlanner::set_parameters( const std::map<std::string, double>& params )
{
  options.intermediateIntegration = 2;
  options.OptiNLC_ACC             = 1e-3;
  options.maxNumberOfIteration    = 500;
  options.OSQP_verbose            = false;
  options.OSQP_max_iter           = 500;
  options.OptiNLC_time_limit      = 0.09;
  options.perturbation            = 1e-6;
  options.timeStep                = sim_time / control_points;
  options.debugPrint              = false;

  for( const auto& [name, value] : params )
  {
    if( name == "wheel_base" )
      wheelbase = value;
    if( name == "lateral_weight" )
      lateral_weight = value;
    if( name == "heading_weight" )
      heading_weight = value;
  }
}

void
OptiNLCTrajectoryPlanner::setup_constraints( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp )
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
    state_constraints[S]      = 0.0;
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
OptiNLCTrajectoryPlanner::setup_objective_function( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp )
{
  ocp.setObjectiveFunction( [&]( const VECTOR<double, state_size>& state, const VECTOR<double, input_size>&, double ) {
    return state[L]; // Minimize the cost function `L`
  } );
}

// Public method to get the next vehicle command based on OptiNLCTrajectoryPlanner
dynamics::Trajectory
OptiNLCTrajectoryPlanner::plan_trajectory( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                                           const map::Map& latest_map, const dynamics::TrafficParticipantSet& traffic_participants )
{
  prediction_to_piecewise_polynomial  ego_prediction  = setup_optimizer_parameters_using_prediction( traffic_participants, current_state );
  auto                          start_time      = std::chrono::high_resolution_clock::now();
  
  // Initial state and input
  VECTOR<double, input_size> initial_input = { 0.0 };
  VECTOR<double, state_size> initial_state = {
    current_state.x, current_state.y, current_state.yaw_angle, current_state.vx, current_state.steering_angle, 0.0, 0.0, 0.0
  };
  
  // Create an MPC problem (OCP)
  OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points> ocp( &options );
  
  // Set up reference route or trajectory
  setup_reference_trajectory( ego_prediction );

  // Set up dynamic model, objective, and constraints
  setup_dynamic_model( ocp );

  setup_objective_function( ocp );

  max_steering_acceleration = 0.75;
  max_steering_velocity = 0.75;
  if ( current_state.vx > 10.0 )
  {
    max_steering_acceleration = 0.1;
    max_steering_velocity = 0.1;
  }
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
        || opt_x[i * state_size + V] < max_reverse_speed || std::abs( opt_x[i * state_size + dDELTA] ) > max_steering_velocity )
    {
      bad_condition  = true;
      bad_counter   += 1;
      break;
    }
  }

  double max_offset = 0.0;
  int valid_count = 0;
  double projection = max_offset;
  if( trajectory_to_follow.x.size() < 100 )
  {
    for ( size_t i = 1; i < 6; i++ )
    {
      // Tangent vector of line 1
      double dx = trajectory_to_follow.x[i] - trajectory_to_follow.x[i-1];
      double dy = trajectory_to_follow.y[i] - trajectory_to_follow.y[i-1];
      double norm = std::hypot(dx, dy);

      if ( norm == 0.0 )
        continue;

      // Unit tangent and normal vectors
      double tx = dx / norm;
      double ty = dy / norm;
      double nx = -ty;
      double ny = tx;

      // Vector from line1 to line2 at point i
      double delta_x = opt_x[(i-1) * 5 * state_size + X] - trajectory_to_follow.x[i];
      double delta_y = opt_x[(i-1) * 5 * state_size + Y] - trajectory_to_follow.y[i];

      // Project delta onto normal direction
      double lateral_offset = delta_x * nx + delta_y * ny;
      if ( std::abs( lateral_offset ) > max_offset )
        max_offset = std::abs(lateral_offset);
      valid_count++;
    }
    projection = max_offset;
  }

  if( trajectory_to_follow.x.size() > 100 )
  {
    for ( size_t i = 1; i < control_points; i++ )
    {
      // Tangent vector of line 1
      double dx = opt_x[i * state_size + X] - opt_x[(i-1) * state_size + X];
      double dy = opt_x[i * state_size + Y] - opt_x[(i-1) * state_size + Y];
      double norm = std::hypot(dx, dy);

      if ( norm == 0.0 )
        continue;

      // Unit tangent and normal vectors
      double tx = dx / norm;
      double ty = dy / norm;
      double nx = -ty;
      double ny = tx;

      // Vector from line1 to line2 at point i
      double delta_x = trajectory_to_follow.x[(i-1) * 2] - opt_x[i * state_size + X];
      double delta_y = trajectory_to_follow.y[(i-1) * 2] - opt_x[i * state_size + Y];

      // Project delta onto normal direction
      double lateral_offset = delta_x * nx + delta_y * ny;
      if ( std::abs( lateral_offset ) > max_offset )
        max_offset = std::abs(lateral_offset);
      valid_count++;
    }
    projection = max_offset;
  }
  
  // std::cerr << "projection: " << projection << std::endl;
  if ( projection > 0.75 && bad_condition == false )
  {
    bad_condition = true;
    bad_counter   += 1;
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
  if( bad_condition == false && bad_counter < 5 || iteration == 0 )
  {
    previous_trajectory = planned_trajectory;
    bad_counter         = 0;
    iteration           = 1;
    return planned_trajectory;
  }
  else
  {
    return previous_trajectory;
  }
}

void
OptiNLCTrajectoryPlanner::setup_dynamic_model( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp )
{
  ocp.setDynamicModel( [&]( const VECTOR<double, state_size>& state, const VECTOR<double, input_size>& input,
                            VECTOR<double, state_size>& derivative, double, void* ) {
   
    // Reference trajectory point at current progress
    int    index             = pp_prediction.findIndex( state[S], ego_prediction_x );
    double reference_x       = pp_prediction.splineEvaluation( index, state[S], ego_prediction_x );
    double reference_y       = pp_prediction.splineEvaluation( index, state[S], ego_prediction_y );
    double reference_ax      = pp_prediction.splineEvaluation( index, state[S], ego_prediction_ax );
    double reference_heading = pp_prediction.splineEvaluation( index, state[S], ego_prediction_heading );

    // Dynamic model equations
    derivative[X]      = state[V] * cos( state[PSI] );                      // X derivative (velocity * cos(psi))
    derivative[Y]      = state[V] * sin( state[PSI] );                      // Y derivative (velocity * sin(psi))
    derivative[PSI]    = state[V] * tan( state[DELTA] ) / wheelbase;        // PSI derivative (steering angle / wheelbase)
    // derivative[V]      = speed_profile.get_acc_at_s( state[S] );         // Velocity derivative (first order)
    derivative[V]      = reference_ax;                                      // Velocity derivative
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

void
OptiNLCTrajectoryPlanner::setup_reference_trajectory( prediction_to_piecewise_polynomial& ego_prediction )
{
  ego_prediction_x        = ego_prediction.x;
  ego_prediction_y        = ego_prediction.y;
  ego_prediction_ax       = ego_prediction.ax;
  ego_prediction_heading  = ego_prediction.heading;
}

OptiNLCTrajectoryPlanner::OptiNLCTrajectoryPlanner()
{
  options.setDefaults();
  set_parameters( {} );
}

prediction_to_piecewise_polynomial
OptiNLCTrajectoryPlanner::setup_optimizer_parameters_using_prediction( const dynamics::TrafficParticipantSet& traffic_participants,
                                                                      const dynamics::VehicleStateDynamic& current_state )
{
  auto start_time = std::chrono::high_resolution_clock::now();
  prediction_to_piecewise_polynomial ego_prediction;
  dynamics::TrafficParticipant ego_vehicle;
  auto it = traffic_participants.participants.find( 777 );
  if ( it != traffic_participants.participants.end() ) 
  {
    ego_vehicle = it->second;
  }

  if( !ego_vehicle.trajectory.has_value() )
    return ego_prediction;

  if( ego_vehicle.trajectory.value().states.empty() )
  {
    return ego_prediction;
  }

  trajectory_to_follow.s.clear();
  trajectory_to_follow.x.clear();
  trajectory_to_follow.y.clear();
  trajectory_to_follow.ax.clear();
  trajectory_to_follow.psi.clear();
  std::vector<double> progress( ego_vehicle.trajectory.value().states.size() );
  std::vector<double> w;

  for (int i = 0; i < ego_vehicle.trajectory.value().states.size(); i++)
  {
    trajectory_to_follow.x.push_back( ego_vehicle.trajectory.value().states[i].x );
    trajectory_to_follow.y.push_back( ego_vehicle.trajectory.value().states[i].y );
    trajectory_to_follow.ax.push_back( ego_vehicle.trajectory.value().states[i].ax );
    trajectory_to_follow.psi.push_back( ego_vehicle.trajectory.value().states[i].yaw_angle );
    w.push_back( 1.0 ); 
    if (i == 0)
    {
      //progress[i] = initial_s;
      trajectory_to_follow.s.push_back( 0.0 );
    }
    else
    {
      progress[i] = progress[i-1] +  
      sqrt(
        ( ego_vehicle.trajectory.value().states[i].x - ego_vehicle.trajectory.value().states[i-1].x ) *
        ( ego_vehicle.trajectory.value().states[i].x - ego_vehicle.trajectory.value().states[i-1].x ) +
        ( ego_vehicle.trajectory.value().states[i].y - ego_vehicle.trajectory.value().states[i-1].y ) * 
        ( ego_vehicle.trajectory.value().states[i].y - ego_vehicle.trajectory.value().states[i-1].y )
      ) + 1e-4;
      trajectory_to_follow.s.push_back( progress[i] );
    }
  }

  if( trajectory_to_follow.s.size() < 3 ) // safety check for route
  {
    return ego_prediction;
  }
  ego_prediction.x               = pp_prediction.CubicSplineSmoother( trajectory_to_follow.s, trajectory_to_follow.x, w, position_smoothing_factor );
  ego_prediction.y               = pp_prediction.CubicSplineSmoother( trajectory_to_follow.s, trajectory_to_follow.y, w, position_smoothing_factor );
  ego_prediction.ax              = pp_prediction.CubicSplineSmoother( trajectory_to_follow.s, trajectory_to_follow.ax, w, position_smoothing_factor );
  ego_prediction.heading         = pp_prediction.CubicSplineSmoother( trajectory_to_follow.s, trajectory_to_follow.psi, w, heading_smoothing_factor );

  // Calculate time taken
  auto                          end_time        = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_seconds = end_time - start_time;

  return ego_prediction;
}

} // namespace planner
} // namespace adore
