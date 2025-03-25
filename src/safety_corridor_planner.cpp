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
#include "planning/safety_corridor_planner.hpp"

namespace adore
{
namespace planner
{

void
SafetyCorridorPlanner::set_parameters( const std::map<std::string, double>& params )
{
  options.intermediateIntegration = 3;
  options.OptiNLC_ACC             = 1e-4;
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
}

void
SafetyCorridorPlanner::setup_constraints( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp )
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
SafetyCorridorPlanner::setup_objective_function( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp )
{
  ocp.setObjectiveFunction( [&]( const VECTOR<double, state_size>& state, const VECTOR<double, input_size>&, double ) {
    return state[L]; // Minimize the cost function `L`
  } );
}

// Public method to get the next vehicle command based on SafetyCorridorPlanner
dynamics::Trajectory
SafetyCorridorPlanner::plan_trajectory( std::vector<adore::math::Point2d> border, const dynamics::VehicleStateDynamic& current_state )
{
  adore::math::Point2d closestIntersection;
  double               minDistance  = std::numeric_limits<double>::max();
  int                  closestIndex = -1;
  std::string          relativePosition;
  // Check each segment of the polyline
  for( size_t i = 0; i < border.size() - 1; i++ )
  {
    adore::math::Point2d intersection;
    adore::math::Point2d p1;
    p1.x = border[i].x;
    p1.y = border[i].y;
    adore::math::Point2d p2;
    p2.x = border[i + 1].x;
    p2.y = border[i + 1].y;
    adore::math::Point2d carPosition;
    carPosition.x = current_state.x;
    carPosition.y = current_state.y;
    if( findIntersection( p1, p2, carPosition, current_state.yaw_angle, intersection ) )
    {
      double distance = std::hypot( intersection.x - carPosition.x, intersection.y - carPosition.y );
      if( distance < minDistance )
      {
        minDistance         = distance;
        closestIntersection = intersection;
        closestIndex        = i + 1;
        relativePosition    = getRelativePosition( p1, p2, carPosition );
      }
    }
  }

  if( relativePosition == "right" && minDistance > 0.5 )
  {
    reference_velocity = 0.0;
  }
  if( distance_moved == 0.0 && car_start == true )
  {
    car_previous_x = current_state.x;
    car_previous_y = current_state.y;
    car_start      = false;
  }
  distance_moved += std::sqrt( ( current_state.x - car_previous_x ) * ( current_state.x - car_previous_x )
                               + ( current_state.y - car_previous_y ) * ( current_state.y - car_previous_y ) );
  car_previous_x  = current_state.x;
  car_previous_y  = current_state.y;

  std::vector<double> border_x;
  std::vector<double> border_y;
  std::vector<double> border_heading;
  for( size_t i = 0; i < border.size() - closestIndex; i++ )
  {
    border_x.push_back( border[i + closestIndex].x );
    border_y.push_back( border[i + closestIndex].y );
  }
  border_x.insert( border_x.begin(), closestIntersection.x );
  border_y.insert( border_y.begin(), closestIntersection.y );

  std::vector<double> w;
  std::vector<double> progress;
  double              N = border_x.size();
  w.resize( N );
  progress.resize( N );
  for( int i = 0; i < N; i++ )
  {
    w[i] = 1.0;
    if( i == 0 )
    {
      progress[i] = 0.0;
    }
    else
    {
      progress[i] = progress[i - 1]
                  + sqrt( ( border_x[i] - border_x[i - 1] ) * ( border_x[i] - border_x[i - 1] )
                          + ( border_y[i] - border_y[i - 1] ) * ( border_y[i] - border_y[i - 1] ) );
    }
  }
  safety_corridor_x = pp.CubicSplineSmoother( progress, border_x, w, 0.9 );
  safety_corridor_y = pp.CubicSplineSmoother( progress, border_y, w, 0.9 );
  std::vector<double> x, dx;
  std::vector<double> y, dy;
  pp.CubicSplineEvaluation( x, dx, progress, safety_corridor_x );
  pp.CubicSplineEvaluation( y, dy, progress, safety_corridor_y );
  border_heading.resize( N );
  for( int i = 0; i < N - 1; i++ )
  {
    border_heading[i] = std::atan2( dy[i], dx[i] );
  }
  border_heading[N - 1]   = border_heading[N - 2];
  safety_corridor_heading = pp.CubicSplineSmoother( progress, border_heading, w, 0.75 );

  auto start_time = std::chrono::high_resolution_clock::now();

  // Initial state and input
  VECTOR<double, input_size> initial_input = { 0.0 };
  VECTOR<double, state_size> initial_state = {
    current_state.x, current_state.y, current_state.yaw_angle, current_state.vx, current_state.steering_angle, 0.0, 0.0, 0.0
  };

  // Create an MPC problem (OCP)
  OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points> ocp( &options );

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
  for( size_t i = 0; i < control_points / 2; i++ )
  {
    if( last_objective_function > 20.0 || opt_x[i * state_size + V] > max_forward_speed || opt_x[i * state_size + V] < max_reverse_speed
        || std::abs( opt_x[i * state_size + dDELTA] ) > max_steering_velocity )
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
SafetyCorridorPlanner::setup_dynamic_model( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp )
{
  ocp.setDynamicModel( [&]( const VECTOR<double, state_size>& state, const VECTOR<double, input_size>& input,
                            VECTOR<double, state_size>& derivative, double, void* ) {
    double tau = 2.5; // Higher value means slower acceleration

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

    // Reference trajectory point at current progress
    int    index             = pp.findIndex( state[S], safety_corridor_x );
    double reference_x       = pp.splineEvaluation( index, state[S], safety_corridor_x );
    double reference_y       = pp.splineEvaluation( index, state[S], safety_corridor_y );
    double reference_heading = pp.splineEvaluation( index, state[S], safety_corridor_heading );

    // Position error terms
    double dx = state[X] - reference_x;
    double dy = state[Y] - reference_y;

    // Calculate longitudinal and lateral errors relative to the vehicle's heading
    double cos_yaw = cos( reference_heading );
    double sin_yaw = sin( reference_heading );

    double lateral_cost  = -dx * sin_yaw + dy * cos_yaw;
    lateral_cost        *= lateral_cost * lateral_weight;

    // Heading error term
    double heading_cost = atan2( -sin_yaw * cos( state[PSI] ) + cos_yaw * sin( state[PSI] ),
                                 cos_yaw * cos( state[PSI] ) + sin_yaw * sin( state[PSI] ) );

    heading_cost *= heading_cost * heading_weight;

    // Steering input cost
    // double steering_cost = input[DELTA] * input[DELTA] * steering_weight;

    // Total cost derivative
    derivative[L] = lateral_cost + heading_cost;
  } );
}

SafetyCorridorPlanner::SafetyCorridorPlanner()
{
  options.setDefaults();
  set_parameters( {} );
}

bool
SafetyCorridorPlanner::findIntersection( const adore::math::Point2d& p1, const adore::math::Point2d& p2, const adore::math::Point2d& carPos,
                                         double heading, adore::math::Point2d& intersection )
{
  // Direction vector along the car's heading
  double cosTheta = std::cos( heading );
  double sinTheta = std::sin( heading );

  // Perpendicular direction vector to the right of the car's heading
  double perpX = sinTheta;
  double perpY = -cosTheta;

  // Vector along the line segment (p1, p2)
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;

  // Solving for t and u
  double denominator = perpX * dy - perpY * dx;
  if( std::abs( denominator ) < 1e-9 )
  {
    return false; // The line is parallel to the line segment
  }

  // double t = ( ( p1.x - carPos.x ) * dy - ( p1.y - carPos.y ) * dx ) / denominator;
  double u = ( ( p1.x - carPos.x ) * perpY - ( p1.y - carPos.y ) * perpX ) / denominator;

  if( u < 0 || u > 1 )
  {
    return false; // The intersection point is not on the segment
  }

  // Calculating the intersection point
  intersection.x = p1.x + u * dx;
  intersection.y = p1.y + u * dy;
  return true;
}

std::string
SafetyCorridorPlanner::getRelativePosition( const adore::math::Point2d& p1, const adore::math::Point2d& p2,
                                            const adore::math::Point2d& carPos )
{
  // Calculate the cross product
  double crossProduct = ( p2.x - p1.x ) * ( carPos.y - p1.y ) - ( p2.y - p1.y ) * ( carPos.x - p1.x );

  if( crossProduct > 0 )
  {
    return "left";
  }
  else if( crossProduct < 0 )
  {
    return "right";
  }
  else
  {
    return "on the line";
  }
}

} // namespace planner
} // namespace adore
