/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include "planning/trajectory_planner.hpp"

#include "adore_math/fast_trig.h"

#include "controllers/iLQR.hpp"
#include "planning/speed_profile_qp.hpp"
#include <dynamics/comfort_settings.hpp>

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
    if( name == "ref_traj_length" && value > 0 )
      ref_traj_length = value;
  }
}

void
TrajectoryPlanner::set_comfort_settings( const dynamics::ComfortSettings& settings )
{
  comfort_settings = settings;
  comfort_settings.clamp( vehicle_params );
}

dynamics::PhysicalVehicleParameters
TrajectoryPlanner::get_physical_vehicle_parameters()
{
  return vehicle_params;
}

void
TrajectoryPlanner::set_vehicle_parameters( const dynamics::PhysicalVehicleParameters& params )
{
  vehicle_params = params;
}

mas::MotionModel
TrajectoryPlanner::get_planning_model( const dynamics::PhysicalVehicleParameters& params )
{
  return [params]( const mas::State& x, const mas::Control& u ) -> mas::StateDerivative {
    mas::StateDerivative dxdt;
    dxdt.setZero( 5 );
    dxdt( 0 ) = x( 3 ) * std::cos( x( 2 ) );                    // x
    dxdt( 1 ) = x( 3 ) * std::sin( x( 2 ) );                    // y
    dxdt( 2 ) = x( 3 ) * std::tan( x( 4 ) ) / params.wheelbase; // yaw_angle
    dxdt( 3 ) = u( 1 );                                         // v
    dxdt( 4 ) = u( 0 );                                         // steering angle
    return dxdt;
  };
}

mas::StageCostFunction
TrajectoryPlanner::make_trajectory_cost( const dynamics::Trajectory& ref_traj )
{
  return [start_state = start_state, ref_traj = ref_traj, weights = weights, max_allowed_speed = max_allowed_speed, dt = dt]( const mas::State& x, const mas::Control& u,
                                                                                       std::size_t k ) -> double {
    double cost = 0.0;

    const double t   = k * dt;
    const auto   ref = ref_traj.get_state_at_time( t );

    const double dx = x( 0 ) - ref.x;
    const double dy = x( 1 ) - ref.y;

    const double c = math::fast_cos( ref.yaw_angle );
    const double s = math::fast_sin( ref.yaw_angle );

    const double lon_err = dx * c + dy * s;
    const double lat_err = -dx * s + dy * c;
    const double hdg_err = math::normalize_angle( x( 2 ) - ref.yaw_angle );
    const double spd_err = x( 3 ) - std::min( ref.vx, max_allowed_speed );

    cost += weights.lane_error * lat_err * lat_err;
    cost += weights.long_error * lon_err * lon_err;
    cost += weights.heading_error * hdg_err * hdg_err;
    cost += weights.speed_error * spd_err * spd_err;
    cost += weights.steering_angle * u( 0 ) * u( 0 ) + weights.acceleration * u( 1 ) * u( 1 );
    return cost;
  };
}

dynamics::Trajectory
TrajectoryPlanner::plan_route_trajectory( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                                          const dynamics::TrafficParticipantSet& traffic_participants,
                                          const std::optional<double>& max_speed,
                                          const dynamics::TrafficSignalSet&      traffic_signals )
{
  if( max_speed.has_value() )
  {
    max_allowed_speed = std::max( max_speed.value(), current_state.vx - 0.5 );
  }
  else
  {
    max_allowed_speed = comfort_settings.max_speed;
  }
  std::cerr << "objects received in planner: " << traffic_participants.participants.size() << std::endl;
  return plan_route_trajectory_with_custom_comfort_settings( latest_route, current_state, traffic_participants, comfort_settings,
                                                             traffic_signals );
}

dynamics::Trajectory
TrajectoryPlanner::plan_route_trajectory_with_custom_comfort_settings( const map::Route&                      latest_route,
                                                                       const dynamics::VehicleStateDynamic&   current_state,
                                                                       const dynamics::TrafficParticipantSet& traffic_participants,
                                                                       const dynamics::ComfortSettings        custom_comfort_settings,
                                                                       const dynamics::TrafficSignalSet&      traffic_signals )
{
  double     initial_s          = latest_route.get_s( current_state );
  map::Route route_with_signals = compute_traffic_light_behavior( current_state, latest_route, traffic_signals );
  return plan_route_trajectory_impl( route_with_signals, current_state, traffic_participants, initial_s, custom_comfort_settings );
}


dynamics::Trajectory
TrajectoryPlanner::plan_route_trajectory_from_s( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                                                 const dynamics::TrafficParticipantSet& traffic_participants,
                                                 double initial_s )
{
  return plan_route_trajectory_impl( latest_route, current_state, traffic_participants, initial_s, comfort_settings );
}


dynamics::Trajectory
TrajectoryPlanner::plan_route_trajectory_with_custom_comfort_settings_from_s( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                                                 const dynamics::TrafficParticipantSet& traffic_participants,
                                                 const dynamics::ComfortSettings custom_comfort_settings,
                                                 double initial_s )
{
  return plan_route_trajectory_impl( latest_route, current_state, traffic_participants, initial_s, custom_comfort_settings );
}


dynamics::Trajectory
TrajectoryPlanner::plan_route_trajectory_impl( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                                               const dynamics::TrafficParticipantSet& traffic_participants,
                                               double initial_s, const dynamics::ComfortSettings& custom_comfort_settings )
{
  SpeedProfile speed_profile;
  speed_profile.set_vehicle_parameters( vehicle_params );
  speed_profile.set_comfort_settings( custom_comfort_settings );

  speed_profile.generate_from_route_and_participants( latest_route, traffic_participants, current_state.vx, initial_s,
                                                      current_state.time, ref_traj_length );

  auto ref_traj = generate_trajectory_from_speed_profile( speed_profile, latest_route, current_state, dt );

  if( ref_traj.states.size() < 1 )
  {
    std::cerr << "reached here danger" << std::endl;
    dynamics::VehicleStateDynamic empty_state;
    empty_state.x  = current_state.x;
    empty_state.y  = current_state.y;
    empty_state.vx = 0.0;
    ref_traj.states.assign( 3, empty_state );
  }


  // PID-based initial guess
  controllers::PurePursuit pid;
  pid.model        = dynamics::PhysicalVehicleModel();
  pid.model.params = vehicle_params;

  auto guess_state = current_state;
  guess_state.time = 0.0;

  dynamics::Trajectory initial_guess;

  for( size_t i = 0; i < horizon_steps; ++i )
  {
    auto command               = pid.get_next_vehicle_command( ref_traj, guess_state );
    guess_state.ax             = command.acceleration;
    guess_state.steering_angle = command.steering_angle;
    initial_guess.states.push_back( guess_state );
    guess_state = dynamics::integrate_rk4( guess_state, command, dt, pid.model.motion_model );
  }

  return optimize_trajectory( current_state, ref_traj, initial_guess );
}

dynamics::Trajectory
TrajectoryPlanner::optimize_trajectory( const dynamics::VehicleStateDynamic& current_state, const dynamics::Trajectory& ref_traj,
                                        const dynamics::Trajectory& initial_guess )
{
  start_state          = current_state;
  reference_trajectory = ref_traj;
  guess_trajectory     = initial_guess;
  setup_problem();
  solve_problem();
  auto out_trajectory = extract_trajectory();
  return out_trajectory;
}

void
TrajectoryPlanner::solve_problem()
{
  mas::SolverParams params;
  params["max_iterations"] = solver_params.max_iterations;
  params["tolerance"]      = solver_params.tolerance;
  params["max_ms"]         = solver_params.max_ms;
  params["debug"]          = solver_params.debug;

  auto solve_with = [&]( auto&& solver, double max_ms ) {
    params["max_ms"] = max_ms;
    solver.set_params( params );
    solver.solve( *problem );
    problem->update_initial_with_best();
  };

  // first pass with collocation
  solve_with( mas::OSQPCollocation{}, 60 );
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
    state.yaw_angle      = math::normalize_angle( x( 2 ) );
    state.vx             = x( 3 );
    state.time           = start_state.time + i * dt;
    state.steering_angle = math::normalize_angle( x( 4 ) );
    state.ax             = u( 1 );

    trajectory.states.push_back( state );
  }

  return trajectory;
}

void
TrajectoryPlanner::setup_problem()
{
  problem = std::make_shared<mas::OCP>();

  problem->state_dim     = 5;
  problem->control_dim   = 2;
  problem->horizon_steps = horizon_steps;
  problem->dt            = dt;
  problem->initial_state = Eigen::VectorXd( 5 );
  problem->dynamics      = get_planning_model( vehicle_params );


  Eigen::VectorXd lower_bounds( problem->control_dim ), upper_bounds( problem->control_dim );
  lower_bounds << -vehicle_params.steering_angle_max, vehicle_params.acceleration_min;
  upper_bounds << vehicle_params.steering_angle_max, vehicle_params.acceleration_max;
  problem->input_lower_bounds = lower_bounds;
  problem->input_upper_bounds = upper_bounds;
  problem->stage_cost         = make_trajectory_cost( reference_trajectory );

  problem->initial_state << start_state.x, start_state.y, start_state.yaw_angle, start_state.vx, start_state.steering_angle;

  // initialize best guess controls from guess trajectory
  problem->initial_controls = mas::ControlTrajectory::Zero( problem->control_dim, problem->horizon_steps );
  for( size_t i = 0; i < problem->horizon_steps; ++i )
  {
    if( i >= guess_trajectory.states.size() )
      break;
    double t                          = i * dt;
    auto   ref                        = guess_trajectory.get_state_at_time( t );
    problem->initial_controls( 1, i ) = ref.ax; // steering
    problem->initial_controls( 0, i ) = 0.0;
  }

  problem->initialize_problem();
  problem->verify_problem();
}

map::Route
TrajectoryPlanner::compute_traffic_light_behavior( const dynamics::VehicleStateDynamic& current_state, const map::Route& latest_route,
                                                   const dynamics::TrafficSignalSet& traffic_signals )
{
  auto   route_with_signal = latest_route;
  double current_s         = latest_route.get_s( current_state );
  auto   closest_point     = latest_route.get_pose_at_s( current_s );

  bool found_light                   = false;
  bool did_not_stop_at_traffic_light = false;

  adore::math::Point2d first_light_point;
  int                  first_light_state = 2; // GREEN

  // Find first traffic light
  for( const auto& p : route_with_signal.reference_line )
  {
    if( p.first < current_s )
      continue;

    auto it = std::find_if( traffic_signals.signals.begin(), traffic_signals.signals.end(), [&]( const auto& kv ) {
      const auto& signal = kv.second;

      return adore::math::distance_2d( signal, p.second ) < 3.0;
    } );

    if( it != traffic_signals.signals.end() )
    {
      const auto& signal = it->second;

      first_light_point.x = signal.x;
      first_light_point.y = signal.y;
      first_light_state   = signal.state;
      found_light         = true;
      break;
    }
  }

  if( found_light )
  {
    double v = current_state.vx;

    adore::math::Point2d ego;
    ego.x = current_state.x;
    ego.y = current_state.y;

    double d_light = adore::math::distance_2d( ego, first_light_point );

    // ================= PARAMETERS =================
    const double a_comfort  = 1.5; // comfortable braking
    const double t_reaction = 0.5;
    const double buffer     = 3.0;

    double d_reaction = v * t_reaction;
    double d_brake    = v * v / ( 2.0 * a_comfort );

    double d_required = d_reaction + d_brake + buffer;

    bool should_stop = false;

    if( first_light_state == 0 ) // RED
    {
      should_stop = true;
    }
    else if( first_light_state == 1 ) // YELLOW
    {
      // Stop if comfortable, otherwise continue
      if( d_light > d_required || d_light / v > 2.5 )
        should_stop = true;
    }
    else // GREEN
    {
      should_stop = false;
    }

    static bool latched_stop = false;

    if( should_stop )
      latched_stop = true;

    if( first_light_state == 2 ) // GREEN resets
      latched_stop = false;

    should_stop = latched_stop;

    // prevent creeping through red
    if( d_light < 5.0 && v < 1.0 && first_light_state != 2 )
    {
      should_stop = true;
    }

    if( should_stop )
    {
      double distance_to_next_traffic_light = adore::math::distance_2d( current_state, first_light_point );
      if( distance_to_next_traffic_light - previous_distance > 0.0 )
        did_not_stop_at_traffic_light = true;
      previous_distance = distance_to_next_traffic_light;
      for( auto& p : route_with_signal.reference_line )
      {
        double d = adore::math::distance_2d( p.second, first_light_point ) - vehicle_params.body_length / 2;

        if( d < 2.0 )
        {
          p.second.max_speed = 0.0;
        }
      }
    }
  }
  return route_with_signal;
}
} // namespace planner
} // namespace adore
