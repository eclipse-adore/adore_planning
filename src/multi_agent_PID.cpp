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
 *    Giovanni Lucente
 *    Marko Mizdrak
 ********************************************************************************/

#include "planning/multi_agent_PID.hpp"

#include <cmath>

#include "adore_math/curvature.hpp"
#include "adore_math/point.h"
#include "adore_math/spline.h"

#include "dynamics/integration.hpp"
#include "dynamics/vehicle_state.hpp"

namespace adore
{
namespace planner
{

MultiAgentPID::MultiAgentPID() = default;

void
MultiAgentPID::set_parameters( const std::map<std::string, double>& params )
{
  for( const auto& [name, value] : params )
  {
    if( name == "max_speed" )
      max_speed = value;
    else if( name == "desired_acceleration" )
      desired_acceleration = value;
    else if( name == "desired_deceleration" )
      desired_deceleration = value;
    else if( name == "max_lateral_acceleration" )
      max_lateral_acceleration = value;
    else if( name == "number_of_integration_steps" )
      number_of_integration_steps = value;
    else if( name == "k_speed" )
      k_speed = value;
    else if( name == "k_yaw" )
      k_yaw = value;
    else if( name == "k_distance" )
      k_distance = value;
    else if( name == "k_goal_point" )
      k_goal_point = value;
    else if( name == "k_sigmoid" )
      k_sigmoid = value;
    else if( name == "dt" )
      dt = value;
    else if( name == "min_distance" )
      min_distance = value;
    else if( name == "time_headway" )
      time_headway = value;
    else if( name == "preview_distance" )
      preview_distance = value;
  }
}

dynamics::VehicleStateDynamic
MultiAgentPID::get_current_state( const dynamics::TrafficParticipant& participant )
{
  if( participant.trajectory && !participant.trajectory->states.empty() )
  {
    return participant.trajectory->states.back();
  }
  return participant.state;
}

void
MultiAgentPID::plan_trajectories( dynamics::TrafficParticipantSet& traffic_participant_set )
{
  for( auto& [id, participant] : traffic_participant_set.participants )
  {
    participant.trajectory = dynamics::Trajectory();
  }
  // Precompute motion model lambdas for each participant.
  std::map<int, MotionModel> motion_models;

  for( auto& [id, participant] : traffic_participant_set.participants )
  {
    if( participant.physical_parameters.wheelbase == 0 )
      participant.physical_parameters.wheelbase = 0.5;

    motion_models[id] = [params = participant.physical_parameters]( const dynamics::VehicleStateDynamic& state,
                                                                    const dynamics::VehicleCommand& cmd ) -> dynamics::VehicleStateDynamic {
      return dynamics::kinematic_bicycle_model( state, params, cmd );
    };
  }

  for( int i = 0; i < number_of_integration_steps; ++i )
  {
    for( auto& [id, participant] : traffic_participant_set.participants )
    {
      if( participant.physical_parameters.wheelbase == 0 )
        participant.physical_parameters.wheelbase = 0.5;
      dynamics::VehicleStateDynamic next_state;
      dynamics::VehicleStateDynamic current_state = get_current_state( participant );

      dynamics::VehicleCommand vehicle_command = dynamics::VehicleCommand( 0.0, 0.0 );

      if( participant.route && !participant.route->center_lane.empty() )
      {
        vehicle_command = compute_vehicle_command( current_state, traffic_participant_set, id );
      }

      next_state = dynamics::integrate_euler( current_state, vehicle_command, dt, motion_models[id] );

      next_state.ax             = vehicle_command.acceleration;
      next_state.steering_angle = vehicle_command.steering_angle;

      if( traffic_participant_set.validity_area )
      {
        if( !traffic_participant_set.validity_area->point_inside( next_state ) )
        {
          continue;
        }
      }


      participant.trajectory->states.push_back( next_state );
    }
  }
}

dynamics::VehicleCommand
MultiAgentPID::compute_vehicle_command( const dynamics::VehicleStateDynamic&   current_state,
                                        const dynamics::TrafficParticipantSet& traffic_participant_set, const int id )
{
  auto& participant = traffic_participant_set.participants.at( id );

  double state_s = participant.route->get_s( current_state );

  double goal_dist = participant.route->get_length() - state_s;

  // 1. Compute lane-following (center-line) errors
  auto [error_lateral, error_yaw] = compute_lane_following_errors( current_state, participant );

  // 2. Calculate the nearest obstacle distance & offset
  auto [closest_obstacle_distance, obstacle_speed, offset] = get_nearest_obstacle_distance_speed_offset( traffic_participant_set, id );

  // 3. Compute the “desired velocity” from IDM logic
  double idm_velocity = compute_idm_velocity( closest_obstacle_distance, goal_dist, obstacle_speed, current_state );

  // 4. Construct base vehicle command: lane-following
  dynamics::VehicleCommand vehicle_command;
  vehicle_command.steering_angle = k_yaw * error_yaw + k_distance * error_lateral;
  vehicle_command.acceleration   = -k_speed * ( current_state.vx - idm_velocity );

  // 9. Finally, clamp commands (accelerations, steering, etc.) to your vehicle limits
  vehicle_command.clamp_within_limits( limits );

  return vehicle_command;
}

std::pair<double, double>
MultiAgentPID::compute_lane_following_errors( const dynamics::VehicleStateDynamic& current_state,
                                              const dynamics::TrafficParticipant&  participant )
{
  double       current_trajectory_s = participant.route->get_s( current_state );
  double       target_distance      = current_trajectory_s + preview_distance + 0.1 * current_state.vx;
  math::Pose2d target_pose          = participant.route->get_pose_at_s( target_distance );

  double error_lateral = compute_error_lateral_distance( current_state, target_pose );
  double error_yaw     = compute_error_yaw( current_state.yaw_angle, target_pose.yaw );

  return { error_lateral, error_yaw };
}

double
MultiAgentPID::compute_error_lateral_distance( const dynamics::VehicleStateDynamic& current_state, const math::Pose2d& target_pose )
{
  double sin_yaw = std::sin( target_pose.yaw );
  double cos_yaw = std::cos( target_pose.yaw );

  double delta_x = current_state.x - target_pose.x;
  double delta_y = current_state.y - target_pose.y;

  return -( cos_yaw * delta_y - sin_yaw * delta_x );
}

double
MultiAgentPID::compute_idm_velocity( double obstacle_distance, double goal_distance, double obstacle_speed,
                                     const dynamics::VehicleStateDynamic& current_state )
{


  double effective_distance     = std::min( obstacle_distance, goal_distance );
  double effective_min_distance = ( goal_distance < obstacle_distance ) ? 0.0 : min_distance;

  double s_star = effective_min_distance + current_state.vx * time_headway
                + current_state.vx * ( current_state.vx - obstacle_speed )
                    / ( 2 * std::sqrt( desired_acceleration * desired_deceleration ) );

  return current_state.vx
       + desired_acceleration * ( 1 - std::pow( current_state.vx / max_speed, 4 ) - std::pow( s_star / effective_distance, 2 ) );
}

std::tuple<double, double, double>
MultiAgentPID::get_nearest_obstacle_distance_speed_offset( const dynamics::TrafficParticipantSet& traffic_participant_set, const int id )
{
  double closest_obstacle_distance = std::numeric_limits<double>::max();
  double obstacle_speed            = 0.0;
  double obstacle_offset           = 0.0;

  auto& participants = traffic_participant_set.participants;

  double own_distance_on_route = participants.at( id ).route->get_s( participants.at( id ).state );
  auto&  own_route             = participants.at( id ).route;
  auto&  own_state             = participants.at( id ).state;

  for( const auto& [other_id, other_participant] : participants )
  {
    if( id == other_id )
      continue;

    auto   other_state     = other_participant.state;
    double distance        = own_route->get_s( other_state ) - own_distance_on_route;
    auto   point_on_route  = own_route->get_pose_at_s( own_distance_on_route + distance );
    double obstacle_offset = math::distance_2d( other_state, point_on_route );

    if( obstacle_offset < lane_width / 2 && distance < closest_obstacle_distance )
    {
      closest_obstacle_distance = distance;
      obstacle_speed            = other_participant.state.vx;
    }
  }

  return { closest_obstacle_distance, obstacle_speed, obstacle_offset };
}

double
MultiAgentPID::compute_error_yaw( double current_yaw, double target_yaw )
{
  return math::normalize_angle( target_yaw - current_yaw );
}


} // namespace planner
} // namespace adore
