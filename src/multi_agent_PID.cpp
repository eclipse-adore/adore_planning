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
    participant.state.vx   = 0.0;
  }
  // Precompute motion model lambdas for each participant.
  std::map<int, MotionModel> motion_models;

  for( auto& [id, participant] : traffic_participant_set.participants )
  {
    if( participant.physical_parameters.wheelbase == 0 )
      participant.physical_parameters.wheelbase = 0.5;

    motion_models[id] = [params = participant.physical_parameters]( const dynamics::VehicleStateDynamic& state,
                                                                    const dynamics::VehicleCommand& cmd ) -> dynamics::VehicleStateDynamic
    {
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
        vehicle_command.clamp_within_limits( participant.physical_parameters );
      }

      next_state = dynamics::integrate_euler( current_state, vehicle_command, dt, motion_models[id] );

      next_state.ax             = vehicle_command.acceleration;
      next_state.steering_angle = vehicle_command.steering_angle;

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
  auto [closest_obstacle_distance, obstacle_speed, offset] = compute_distance_speed_offset_nearest_obstacle( traffic_participant_set, id );

  // 3. Compute the “desired velocity” from IDM logic
  double idm_velocity = compute_idm_velocity( closest_obstacle_distance, goal_dist, obstacle_speed, current_state );


  // 5. Construct base vehicle command: lane-following
  dynamics::VehicleCommand vehicle_command;
  vehicle_command.steering_angle = k_yaw * error_yaw + k_distance * error_lateral;
  vehicle_command.acceleration   = -k_speed * ( current_state.vx - idm_velocity );

  return vehicle_command;
}

std::pair<double, double>
MultiAgentPID::compute_lane_following_errors( const dynamics::VehicleStateDynamic& current_state,
                                              const dynamics::TrafficParticipant&  participant )
{
  double       current_trajectory_s = participant.route->get_s( current_state );
  double       target_distance      = current_trajectory_s + 0.5 + 0.1 * current_state.vx;
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

double
MultiAgentPID::compute_error_yaw( double current_yaw, double target_yaw )
{
  return math::normalize_angle( target_yaw - current_yaw );
}

std::tuple<double, double, double>
MultiAgentPID::compute_distance_speed_offset_nearest_obstacle( const dynamics::TrafficParticipantSet& traffic_participant_set,
                                                               int                                    vehicle_id )
{
  double closest_distance      = std::numeric_limits<double>::max();
  double offset_closest_object = std::numeric_limits<double>::max();
  double obstacle_speed        = 0.0;

  auto& ref_participant = traffic_participant_set.participants.at( vehicle_id );
  if( !ref_participant.route )
  {
    return { closest_distance, obstacle_speed, offset_closest_object };
  }

  auto&  route         = ref_participant.route.value();
  double ref_current_s = route.get_s( get_current_state( ref_participant ) );

  for( const auto& [id, other_participant] : traffic_participant_set.participants )
  {
    if( id == vehicle_id )
      continue;

    dynamics::VehicleStateDynamic object_state = get_current_state( other_participant );
    double                        object_s     = route.get_s( object_state );
    double                        distance     = object_s - ref_current_s;

    if( distance < 1.0 )
      continue;

    auto pose_at_distance = route.get_pose_at_s( object_s );

    // Compute signed lateral offset (negative = left, positive = right)
    double dx             = object_state.x - pose_at_distance.x;
    double dy             = object_state.y - pose_at_distance.y;
    double current_offset = -dx * std::sin( pose_at_distance.yaw ) + dy * std::cos( pose_at_distance.yaw );

    if( std::abs( current_offset ) > 0.5 * lane_width )
      continue;

    if( std::abs( current_offset ) > obstacle_avoidance_offset_threshold )
    {
      offset_closest_object = current_offset;
      continue;
    }

    if( distance < closest_distance )
    {
      closest_distance      = distance;
      obstacle_speed        = object_state.vx;
      offset_closest_object = current_offset;
    }
  }

  return { closest_distance, obstacle_speed, offset_closest_object };
}

std::pair<double, double>
MultiAgentPID::compute_obstacle_avoidance_speed_component_errors( const dynamics::VehicleStateDynamic&   current_state,
                                                                  const dynamics::TrafficParticipantSet& traffic_participant_set,
                                                                  int                                    vehicle_id )
{
  double lateral_speed_error      = 0.0;
  double longitudinal_speed_error = 0.0;
  auto&  ref_participant          = traffic_participant_set.participants.at( vehicle_id );
  auto&  ref_participant_route    = ref_participant.route.value();

  for( const auto& [id, other_participant] : traffic_participant_set.participants )
  {
    if( id == vehicle_id )
      continue;

    double distance_on_the_route = ref_participant_route.get_s( other_participant.state );
    auto   pose_at_distance      = ref_participant_route.get_pose_at_s( distance_on_the_route );
    double offset                = -( other_participant.state.x - pose_at_distance.x ) * std::sin( pose_at_distance.yaw )
                  + ( other_participant.state.y - pose_at_distance.y ) * std::cos( pose_at_distance.yaw );


    double current_s = ref_participant_route.get_s( current_state );


    if( std::abs( offset ) > 0.5 * lane_width || current_s > distance_on_the_route )
      continue;

    double distance_to_object                              = math::distance_2d( current_state, other_participant.state );
    double activation_weight                               = sigmoid_activation( distance_to_object, min_distance, k_sigmoid );
    auto [target_longitudinal_speed, target_lateral_speed] = compute_target_speed_components( current_state, other_participant.state,
                                                                                              ref_participant_route );

    double angle_diff = pose_at_distance.yaw - current_state.yaw_angle;

    // Standard 2D rotation
    double v_parallel = current_state.vx * std::cos( angle_diff );

    double v_perp = current_state.vx * std::sin( angle_diff );

    lateral_speed_error      += -activation_weight * ( 1 / ( offset ) );
    longitudinal_speed_error += activation_weight * ( target_longitudinal_speed - v_parallel );
  }

  return { 0.0, lateral_speed_error };
}

std::pair<double, double>
MultiAgentPID::compute_target_speed_components( const dynamics::VehicleStateDynamic& current_state,
                                                const dynamics::VehicleStateDynamic& other_participant_state, const map::Route& route )
{
  constexpr double object_radius = 2.5;
  double           U_speed       = current_state.vx;

  // Compute distance vector and its norm
  Eigen::Vector2d distance_vector( other_participant_state.x - current_state.x, other_participant_state.y - current_state.y );
  double          distance_to_object = distance_vector.norm();

  // Compute lane-aligned vectors
  double          s_object         = route.get_s( other_participant_state );
  auto            pose_center_lane = route.get_pose_at_s( s_object );
  Eigen::Vector2d center_lane_versor( std::cos( pose_center_lane.yaw ), std::sin( pose_center_lane.yaw ) );

  // Compute angle between lane direction and distance vector
  double theta           = std::atan2( distance_vector.y(), distance_vector.x() ) - pose_center_lane.yaw;
  double theta_2         = 2 * theta;
  double inv_distance_sq = 1.0 / ( distance_to_object * distance_to_object );
  double coeff           = object_radius * object_radius * U_speed * inv_distance_sq;

  // Fluidodynamic target speeds
  Eigen::Vector2d target_speed = U_speed * center_lane_versor - coeff * Eigen::Vector2d( std::cos( theta_2 ), std::sin( theta_2 ) );

  // Compute vehicle-aligned longitudinal and lateral speeds
  Eigen::Vector2d yaw_versor( std::cos( current_state.yaw_angle ), std::sin( current_state.yaw_angle ) );
  Eigen::Vector2d lateral_direction_versor( -yaw_versor.y(), yaw_versor.x() ); // Perpendicular vector

  double target_longitudinal_speed = yaw_versor.dot( target_speed );
  double target_lateral_speed      = lateral_direction_versor.dot( target_speed );

  return { target_longitudinal_speed, target_lateral_speed };
}

double
MultiAgentPID::sigmoid_activation( double d, double d_threshold, double k )
{
  return 1.0 / ( 1.0 + exp( k * ( d - d_threshold ) ) );
}

} // namespace planner
} // namespace adore
