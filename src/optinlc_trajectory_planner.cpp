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
  options.intermediateIntegration = 3;
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
    if( name == "maximum_velocity" )
      maximum_velocity = value;
    if( name == "min_distance_to_vehicle_ahead" )
      min_distance_to_vehicle_ahead = value;
  }
}

void
OptiNLCTrajectoryPlanner::setup_constraints( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp )
{

  // Define a simple input update method
  ocp.setInputUpdate( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& input, double, void* ) {
    VECTOR<double, input_size> update_input = { input[dDELTA] };
    return update_input;
  } );

  // State Constraints
  ocp.setUpdateStateLowerBounds( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& ) {
    VECTOR<double, state_size> state_constraints;
    state_constraints.setConstant( -std::numeric_limits<double>::infinity() );
    state_constraints[V]      = max_reverse_speed;
    state_constraints[DELTA]  = -limits.max_steering_angle;
    //state_constraints[dDELTA] = -max_steering_velocity;
    return state_constraints;
  } );

  ocp.setUpdateStateUpperBounds( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& ) {
    VECTOR<double, state_size> state_constraints;
    state_constraints.setConstant( std::numeric_limits<double>::infinity() );
    state_constraints[V]      = max_forward_speed;
    state_constraints[DELTA]  = limits.max_steering_angle;
    //state_constraints[dDELTA] = max_steering_velocity;
    return state_constraints;
  } );

  // Input Constraints
  ocp.setUpdateInputLowerBounds( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& ) {
    VECTOR<double, input_size> input_constraints;
    input_constraints[dDELTA] = -max_steering_velocity;
    return input_constraints;
  } );

  ocp.setUpdateInputUpperBounds( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& ) {
    VECTOR<double, input_size> input_constraints;
    input_constraints[dDELTA] = max_steering_velocity;
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
  route_to_piecewise_polynomial reference_route = setup_optimizer_parameters_using_route( latest_route, current_state );
  auto                          start_time      = std::chrono::high_resolution_clock::now();

  // Initial state and input
  if (current_state.vx < 0.25)
  {
    steering_rate = 0.0;
  }
  VECTOR<double, input_size> initial_input = { current_state.steering_rate };
  VECTOR<double, state_size> initial_state = {
    current_state.x, current_state.y, current_state.yaw_angle, current_state.vx, current_state.steering_angle, 0.0, 0.0
  };

  // Create an MPC problem (OCP)
  OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points> ocp( &options );

  // Set up reference route
  setup_reference_route( reference_route );
  if( route_x.breaks.size() < 1 )
  {
    dynamics::Trajectory empty_trajectory;
    std::cerr << "end of route or invalid route received" << std::endl;
    return empty_trajectory;
  }

  // Set up reference velocity
  setup_reference_velocity( latest_route, current_state, latest_map, traffic_participants );

  // Set up dynamic model, objective, and constraints
  setup_dynamic_model( ocp );

  setup_objective_function( ocp );

  setup_constraints( ocp );

  // Solve the MPC problem
  OptiNLC_Solver<double, input_size, state_size, constraints_size, control_points> solver( ocp );

  solver.solve( current_state.time, initial_state, initial_input );

  auto   opt_x                   = solver.get_optimal_states();
  auto   opt_u                   = solver.get_optimal_inputs();
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
        || opt_x[i * state_size + V] < max_reverse_speed || std::abs( opt_u[i * input_size + dDELTA] ) > max_steering_velocity )
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
    state.steering_rate  = opt_u[i * input_size + dDELTA];
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
    iteration = 1;
    steering_rate = planned_trajectory.states[1].steering_rate;
    return planned_trajectory;
  }
  else
  {
    steering_rate = previous_trajectory.states[1].steering_rate;
    return previous_trajectory;
  }
}

void
OptiNLCTrajectoryPlanner::setup_dynamic_model( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp )
{
  ocp.setDynamicModel( [&]( const VECTOR<double, state_size>& state, const VECTOR<double, input_size>& input,
                            VECTOR<double, state_size>& derivative, double, void* ) {
    if( reference_velocity - state[V] > 0 )
    {
      tau = 2.5; // Higher value for smooth acceleration
    }
    else
    {
      tau = 1.25; // Lower value for quick braking
    }
    // Reference trajectory point at current progress
    int    index             = pp.findIndex( state[S], route_x );
    double reference_x       = pp.splineEvaluation( index, state[S], route_x );
    double reference_y       = pp.splineEvaluation( index, state[S], route_y );
    double reference_heading = pp.splineEvaluation( index, state[S], route_heading );

    // Dynamic model equations
    derivative[X]      = state[V] * cos( state[PSI] );                      // X derivative (velocity * cos(psi))
    derivative[Y]      = state[V] * sin( state[PSI] );                      // Y derivative (velocity * sin(psi))
    derivative[PSI]    = state[V] * tan( state[DELTA] ) / wheelbase;        // PSI derivative (steering angle / wheelbase)
    derivative[V]      = ( 1.0 / tau ) * ( reference_velocity - state[V] ); // Velocity derivative (first order)
    derivative[DELTA]  = input[dDELTA];                                     // Steering angle derivative
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
OptiNLCTrajectoryPlanner::setup_reference_route( route_to_piecewise_polynomial& reference_route )
{
  route_x       = reference_route.x;
  route_y       = reference_route.y;
  route_heading = reference_route.heading;
}

OptiNLCTrajectoryPlanner::OptiNLCTrajectoryPlanner()
{
  options.setDefaults();
  set_parameters( {} );
}

route_to_piecewise_polynomial
OptiNLCTrajectoryPlanner::setup_optimizer_parameters_using_route( const adore::map::Route&             latest_route,
                                                                  const dynamics::VehicleStateDynamic& current_state )
{
  auto start_time = std::chrono::high_resolution_clock::now();

  route_to_piecewise_polynomial route;

  double maximum_required_road_length = sim_time * max_forward_speed;

  if( maximum_required_road_length < min_distance_in_route )
  {
    return route;
  }

  if( latest_route.center_lane.empty() )
  {
    return route;
  }

  double state_s = latest_route.get_s( current_state );

  route_to_follow.s.clear();
  route_to_follow.x.clear();
  route_to_follow.y.clear();
  route_to_follow.psi.clear();

  std::vector<double> w;

  double previous_s = 0.0;
  for( const auto& [s, point] : latest_route.center_lane )
  {
    if( s < state_s )
      continue;
    if( s - state_s > maximum_required_road_length )
      break;
    double local_progress = s - state_s;
    if( local_progress - previous_s > 0.1 ) // adding points every 75 cm
    {
      route_to_follow.s.push_back( local_progress );
      route_to_follow.x.push_back( point.x );
      route_to_follow.y.push_back( point.y );
      w.push_back( 1.0 );
      previous_s = local_progress;
    }
  }

  if( route_to_follow.s.size() < 3 ) // safety check for route
  {
    return route;
  }
  route_to_follow.s[0] = 0.0; // overwriting the first element to 0 (start from ego vehicle)
  route.x              = pp.CubicSplineSmoother( route_to_follow.s, route_to_follow.x, w, position_smoothing_factor );
  route.y              = pp.CubicSplineSmoother( route_to_follow.s, route_to_follow.y, w, position_smoothing_factor );

  std::vector<double> x, dx;
  std::vector<double> y, dy;
  pp.CubicSplineEvaluation( x, dx, route_to_follow.s, route.x );
  pp.CubicSplineEvaluation( y, dy, route_to_follow.s, route.y );
  for( size_t i = 0; route_to_follow.s.size() > 0 && i < route_to_follow.s.size() - 1; i++ )
  {
    if( dx[i] == 0.0 || dx.size() < 1 || dy.size() < 1 )
    {
      return route;
    }
    route_to_follow.psi.push_back( std::atan2( dy[i], dx[i] ) );
  }
  route_to_follow.psi[route_to_follow.s.size() - 1] = route_to_follow.psi[route_to_follow.s.size() - 2];
  route.heading = pp.CubicSplineSmoother( route_to_follow.s, route_to_follow.psi, w, heading_smoothing_factor );

  // Calculate time taken
  auto                          end_time        = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_seconds = end_time - start_time;

  return route;
}

std::vector<double>
OptiNLCTrajectoryPlanner::compute_curvatures( const dynamics::VehicleStateDynamic& current_state )
{
  int n          = pp.findIndex( lookahead_time * 5.0, route_x );
  n              = std::max( n, safe_index );
  std::vector<double> curvatures( n, 0.0 );

  // Compute curvature using a larger window size for robustness against noise
  if( route_to_follow.s.size() > n )
  {
    for( size_t i = 2; i < n - 2; ++i ) // Adjust the window size here
    {
      // Average positions in a larger window to reduce noise sensitivity
      double x1 = ( route_to_follow.x[i - 2] + route_to_follow.x[i - 1] ) / 2.0;
      double y1 = ( route_to_follow.y[i - 2] + route_to_follow.y[i - 1] ) / 2.0;
      double x2 = route_to_follow.x[i];
      double y2 = route_to_follow.y[i];
      double x3 = ( route_to_follow.x[i + 1] + route_to_follow.x[i + 2] ) / 2.0;
      double y3 = ( route_to_follow.y[i + 1] + route_to_follow.y[i + 2] ) / 2.0;

      double kappa  = adore::math::compute_curvature( x1, y1, x2, y2, x3, y3 );
      curvatures[i] = std::abs( kappa );
    }
  }

  // Handle boundary points by copying neighboring values
  if( n > 4 )
  {
    curvatures[0]     = curvatures[2];
    curvatures[1]     = curvatures[2];
    curvatures[n - 1] = curvatures[n - 3];
    curvatures[n - 2] = curvatures[n - 3];
  }
  else
  {
    curvatures[0] = 0.0;
    if( n > 1 )
      curvatures[1] = 0.0;
  }

  // Apply a simple moving average filter to smooth the computed curvatures
  std::vector<double> smoothed_curvatures( n, 0.0 );
  const int           smoothing_window = 3; // Adjust window size for desired smoothing level
  for( size_t i = 0; i < n; ++i )
  {
    double sum   = 0.0;
    int    count = 0;
    for( int j = -smoothing_window; j <= smoothing_window; ++j )
    {
      int index = i + j;
      if( index >= 0 && static_cast<size_t>( index ) < n )
      {
        sum += curvatures[index];
        ++count;
      }
    }
    smoothed_curvatures[i] = sum / count;
  }

  return smoothed_curvatures;
}

void
OptiNLCTrajectoryPlanner::setup_reference_velocity( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                                                    const map::Map&                        latest_map,
                                                    const dynamics::TrafficParticipantSet& traffic_participants )
{
  reference_velocity = maximum_velocity;
  int index          = pp.findIndex( lookahead_time * current_state.vx, route_x );
  index              = std::max( index, safe_index );

  // Curvature calculation
  std::vector<double> curvature;
  if( route_to_follow.s.size() > index + 2 )
  {
    // for( int i = 0; i < index; i++ )
    // {
    //   double kappa = adore::math::compute_curvature( route_to_follow.x[i], route_to_follow.y[i], route_to_follow.x[i + 1],
    //                                                  route_to_follow.y[i + 1], route_to_follow.x[i + 2], route_to_follow.y[i + 2] );
    //   curvature.push_back( std::abs( kappa ) );
    // }
    curvature = compute_curvatures( current_state );
    distance_moved += current_state.vx * dt;
    if( distance_moved > distance_to_add_behind )
    {
      curvature_behind.push_back( curvature[0] );
      distance_to_add_behind += 1;
    }

    if( curvature_behind.size() > look_behind_for_curvature )
    {
      curvature_behind.erase( curvature_behind.begin() );
    }
    std::vector<double> total_curvature = curvature_behind;

    total_curvature.insert( total_curvature.end(), curvature.begin(), curvature.end() );
    double max_curvature      = *std::max_element( total_curvature.begin(), total_curvature.end() );
    double curvature_velocity = std::max( sqrt( lateral_acceleration / max_curvature ), minimum_velocity_in_curve );
    curvature_velocity = std::min(curvature_velocity, maximum_velocity);
    double a = (curvature_velocity - current_state.vx) / 2.0;
    curvature_velocity = std::max(current_state.vx + a, minimum_velocity_in_curve);
    reference_velocity        = std::min( reference_velocity, curvature_velocity );
  }

  double idm_velocity = calculate_idm_velocity( latest_route, current_state, latest_map, traffic_participants );
  reference_velocity  = std::min( reference_velocity, idm_velocity );

  double min_dist = std::numeric_limits<double>::max();
  auto   nearest  = latest_map.quadtree.get_nearest_point( current_state, min_dist );

  if( nearest )
  {
    double current_route_point_max_speed = latest_map.get_lane_speed_limit( nearest.value().parent_id );
    reference_velocity                   = std::min( reference_velocity, current_route_point_max_speed );
  }
  std::cerr << "reference velocity: " << reference_velocity << std::endl;
}

double
OptiNLCTrajectoryPlanner::calculate_idm_velocity( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                                                  const map::Map& latest_map, const dynamics::TrafficParticipantSet& traffic_participants )
{
  double distance_to_object_min     = std::numeric_limits<double>::max();
  double distance_to_maintain_ahead = min_distance_to_vehicle_ahead + wheelbase / 2;
  double idm_velocity               = maximum_velocity;
  double state_s                    = latest_route.get_s( current_state );

  for( const auto& [id, participant] : traffic_participants.participants )
  {
    math::Point2d object_position;
    object_position.x                        = participant.state.x;
    object_position.y                        = participant.state.y;
    math::Point2d object_position_predicted  = object_position;
    double        distance_to_object         = latest_route.get_s( object_position );
    double        offset                     = math::distance_2d( object_position, latest_route.get_pose_at_s( distance_to_object ) );
    auto          map_point                  = latest_route.get_map_point_at_s( distance_to_object );
    bool          within_lane                = offset < (latest_map.lanes.at( map_point.parent_id )->get_width( map_point.s ) / 2);
    distance_to_object                      -= state_s;
    if( !within_lane )
    {
      distance_to_object = std::numeric_limits<double>::max();
    }
    // for( int i = 0; i < prediction_horizon; i++ )
    // {
    //   object_position_predicted.x          = object_position_predicted.x + participant.state.vx * dt * cos( participant.state.yaw_angle );
    //   object_position_predicted.y          = object_position_predicted.y + participant.state.vx * dt * sin( participant.state.yaw_angle );
    //   double distance_to_object_predicted  = latest_route.get_s( object_position_predicted );
    //   double offset    = math::distance_2d( object_position_predicted, latest_route.get_pose_at_s( distance_to_object_predicted ) );
    //   auto   map_point = latest_route.get_map_point_at_s( distance_to_object_predicted );
    //   bool   within_lane_predicted = offset < (latest_map.lanes.at( map_point.parent_id )->get_width( map_point.s ) / 2);
    //   distance_to_object_predicted        -= state_s;
    //   if( within_lane_predicted && distance_to_object_predicted < distance_to_object )
    //   {
    //     //distance_to_object = distance_to_object_predicted;
    //     within_lane        = true;
    //   }
    // }
    if( within_lane && distance_to_object < distance_to_object_min && distance_to_object > 0 )
    {
      front_vehicle_velocity = participant.state.vx;
      distance_to_object_min = distance_to_object;
    }
  }
  std::cerr << "distance to object: " << distance_to_object_min << std::endl;

  distance_to_goal = latest_route.get_length() - state_s;
  std::cerr << "distance to goal: " << distance_to_goal << std::endl;

  double distance_for_idm = std::min( distance_to_object_min, distance_to_goal );

  if( distance_to_goal < distance_to_object_min )
  {
    distance_to_maintain_ahead = 5.0;
  }

  double s_star = distance_to_maintain_ahead + current_state.vx * desired_time_headway
                + current_state.vx * ( current_state.vx - front_vehicle_velocity ) / ( 2 * sqrt( max_acceleration * max_deceleration ) );
  double velocity_ratio = current_state.vx / maximum_velocity;
  idm_velocity          = current_state.vx
               + max_acceleration
                   * ( 1 - velocity_ratio * velocity_ratio * velocity_ratio * velocity_ratio
                       - ( s_star / distance_for_idm ) * ( s_star / distance_for_idm ) );

  idm_velocity = std::max( 0.0, std::min( idm_velocity, max_forward_speed ) ); // clamping idm velocity from 0 to max speed allowed

  return idm_velocity;
}

} // namespace planner
} // namespace adore