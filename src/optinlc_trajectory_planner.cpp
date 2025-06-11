// #include "planning/optinlc_trajectory_planner.hpp"

// namespace adore
// {
// namespace planner
// {

// OptiNLCTrajectoryPlanner::OptiNLCTrajectoryPlanner()
// {
//   options.setDefaults();
//   set_parameters( {} );
// }

// void
// OptiNLCTrajectoryPlanner::set_parameters( const std::map<std::string, double>& params )
// {
//   options.intermediateIntegration = 1;
//   options.OptiNLC_ACC             = 1e-4;
//   options.maxNumberOfIteration    = 500;
//   options.OSQP_verbose            = false;
//   options.OSQP_max_iter           = 500;
//   options.OptiNLC_time_limit      = 0.09;
//   options.perturbation            = 1e-6;
//   options.timeStep                = sim_time / control_points;
//   options.debugPrint              = true;

// for( const auto& [name, value] : params )
// {
//   if( name == "wheel_base" )
//     wheelbase = value;
// }
// }

// void
// OptiNLCTrajectoryPlanner::setup_constraints( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp )
// {
//   ocp.setInputUpdate( []( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& input, double, void* ) {
//     VECTOR<double, input_size> update_input = { input[dDELTA] };
//     return update_input;
//   } );

// ocp.setUpdateStateLowerBounds( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& ) {
//   VECTOR<double, state_size> bounds;
//   bounds.setConstant( -std::numeric_limits<double>::infinity() );
//   bounds[V]     = max_reverse_speed;
//   bounds[DELTA] = -limits.max_steering_angle;
//   // bounds[dDELTA] = -max_steering_velocity;
//   return bounds;
// } );

// ocp.setUpdateStateUpperBounds( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& ) {
//   VECTOR<double, state_size> bounds;
//   bounds.setConstant( std::numeric_limits<double>::infinity() );
//   bounds[V]     = max_forward_speed;
//   bounds[DELTA] = limits.max_steering_angle;
//   // bounds[dDELTA] = max_steering_velocity;
//   return bounds;
// } );

// ocp.setUpdateInputLowerBounds( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& ) {
//   VECTOR<double, input_size> input_bounds;
//   input_bounds[dDELTA] = -max_steering_velocity;
//   return input_bounds;
// } );

// ocp.setUpdateInputUpperBounds( [&]( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& ) {
//   VECTOR<double, input_size> input_bounds;
//   input_bounds[dDELTA] = max_steering_velocity;
//   return input_bounds;
// } );

// ocp.setUpdateFunctionConstraints( []( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& ) {
//   VECTOR<double, constraints_size> constraints;
//   constraints.setConstant( 0.0 );
//   return constraints;
// } );

// ocp.setUpdateFunctionConstraintsLowerBounds( []( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& ) {
//   VECTOR<double, constraints_size> constraints;
//   constraints.setConstant( -std::numeric_limits<double>::infinity() );
//   return constraints;
// } );

// ocp.setUpdateFunctionConstraintsUpperBounds( []( const VECTOR<double, state_size>&, const VECTOR<double, input_size>& ) {
//   VECTOR<double, constraints_size> constraints;
//   constraints.setConstant( std::numeric_limits<double>::infinity() );
//   return constraints;
// } );
// }

// void
// OptiNLCTrajectoryPlanner::setup_objective_function( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp )
// {
//   ocp.setObjectiveFunction( [&]( const VECTOR<double, state_size>& state, const VECTOR<double, input_size>& input, double ) {
//     return state[L] + 0.01 * input[dDELTA] * input[dDELTA];
//   } );
// }

// void
// OptiNLCTrajectoryPlanner::setup_dynamic_model( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp )
// {
//   ocp.setDynamicModel( [&]( const VECTOR<double, state_size>& state, const VECTOR<double, input_size>& input,
//                             VECTOR<double, state_size>& derivative, double, void* ) {
//     derivative[X]     = state[V] * cos( state[PSI] );
//     derivative[Y]     = state[V] * sin( state[PSI] );
//     derivative[PSI]   = state[V] * tan( state[DELTA] ) / wheelbase;
//     derivative[V]     = speed_profile.get_acc_at_s( state[S] );
//     derivative[DELTA] = input[dDELTA];
//     derivative[S]     = state[V];

// auto ref_pose = route.get_pose_at_s( state[S] + 1.0 );

// double dx = state[X] - ref_pose.x;
// double dy = state[Y] - ref_pose.y;

// derivative[L] = dx * dx + dy * dy + input[dDELTA] * input[dDELTA] * 0.01;
// } );
// }

// dynamics::Trajectory
// OptiNLCTrajectoryPlanner::plan_trajectory( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
//                                            const map::Map& latest_map, const dynamics::TrafficParticipantSet& traffic_participants )
// {
//   dynamics::Trajectory empty_trajectory;
//   double               initial_s = latest_route.get_s( current_state );

// // Generate speed profile
// speed_profile.generate_from_route_and_participants( latest_route, traffic_participants, current_state.vx, initial_s,
//                                                     max_lateral_acceleration, desired_time_headway, planning_horizon_s );

// bool generate_without_optinlc = false;
// if( generate_without_optinlc )
// {
//   return generate_trajectory_from_speed_profile( speed_profile, latest_route, 0.1 );
// }


// VECTOR<double, input_size> initial_input = { 0.0 };
// VECTOR<double, state_size> initial_state = {
//   current_state.x, current_state.y, current_state.yaw_angle, current_state.vx, current_state.steering_angle, initial_s, 0.0
// };

// OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points> ocp( &options );
// route = latest_route;

// setup_dynamic_model( ocp );
// setup_objective_function( ocp );
// setup_constraints( ocp );

// OptiNLC_Solver<double, input_size, state_size, constraints_size, control_points> solver( ocp );
// solver.solve( current_state.time, initial_state, initial_input );

// auto   opt_x                   = solver.get_optimal_states();
// auto   opt_u                   = solver.get_optimal_inputs();
// auto   time                    = solver.getTime();
// double last_objective_function = solver.get_final_objective_function();

// dynamics::Trajectory planned_trajectory;
// for( size_t i = 0; i < control_points; i++ )
// {
//   dynamics::VehicleStateDynamic state;
//   state.x              = opt_x[i * state_size + X];
//   state.y              = opt_x[i * state_size + Y];
//   state.yaw_angle      = opt_x[i * state_size + PSI];
//   state.vx             = opt_x[i * state_size + V];
//   state.steering_angle = opt_x[i * state_size + DELTA];
//   state.steering_rate  = opt_u[i * input_size + dDELTA];
//   state.time           = time[i];
//   if( i < control_points - 1 )
//   {
//     state.yaw_rate = ( opt_x[( i + 1 ) * state_size + PSI] - opt_x[i * state_size + PSI] ) / options.timeStep;
//     state.ax       = ( opt_x[( i + 1 ) * state_size + V] - opt_x[i * state_size + V] ) / options.timeStep;
//   }
//   planned_trajectory.states.push_back( state );
// }
// if( !planned_trajectory.states.empty() )
// {
//   planned_trajectory.states.back().yaw_rate = planned_trajectory.states[planned_trajectory.states.size() - 2].yaw_rate;
//   planned_trajectory.states.back().ax       = planned_trajectory.states[planned_trajectory.states.size() - 2].ax;
// }
// return planned_trajectory;
// }

// } // namespace planner
// } // namespace adore
