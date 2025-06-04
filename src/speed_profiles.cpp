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
 *    Marko Mizdrak
 ********************************************************************************/

#include "planning/speed_profiles.hpp"

namespace adore
{
namespace planner
{

double
SpeedProfile::get_speed_at_s( double s ) const
{
  auto it = s_to_speed.lower_bound( s );
  if( it == s_to_speed.end() )
  {
    return s_to_speed.rbegin()->second;
  }
  else if( it == s_to_speed.begin() )
  {
    return it->second;
  }
  else
  {
    auto   it_prev = std::prev( it );
    double ratio   = ( s - it_prev->first ) / ( it->first - it_prev->first );
    return it_prev->second + ratio * ( it->second - it_prev->second );
  }
}

std::unordered_map<int, std::map<double, double>>
SpeedProfile::predict_traffic_participant_trajectories( const dynamics::TrafficParticipantSet& traffic_participants,
                                                        const map::Route& route, double prediction_horizon, double time_step )
{
  std::unordered_map<int, std::map<double, double>> predicted_trajectories;

  for( const auto& [id, participant] : traffic_participants.participants )
  {
    double current_s = route.get_s( participant.state );
    double offset    = math::distance_2d( participant.state, route.get_pose_at_s( current_s ) );
    if( offset > 2.0 )
      continue;

    double                   current_velocity = participant.state.vx;
    std::map<double, double> trajectory;

    double accumulated_time = 0.0;
    while( accumulated_time <= prediction_horizon )
    {
      double predicted_s       = current_s + current_velocity * accumulated_time;
      trajectory[predicted_s]  = current_velocity;
      accumulated_time        += time_step;
    }

    predicted_trajectories[id] = std::move( trajectory );
  }

  return predicted_trajectories;
}

void
SpeedProfile::generate_from_route_and_participants( const map::Route& route, const dynamics::TrafficParticipantSet& traffic_participants,
                                                    double initial_speed, double initial_s, double max_lateral_acceleration,
                                                    double desired_time_headway, double length )
{
  // Clear previous speed profile
  s_to_speed.clear();

  // Compute curvature-based speed limits
  std::map<double, double> s_to_curvature = calculate_curvature_speeds( route, max_lateral_acceleration, initial_s, length );

  // Precompute Traffic Participant Trajectories
  auto predicted_trajectories = predict_traffic_participant_trajectories( traffic_participants, route );

  // Initialize starting conditions
  s_to_speed[initial_s] = initial_speed;

  auto it     = route.center_lane.lower_bound( initial_s );
  auto end_it = route.center_lane.upper_bound( initial_s + length );

  if( it == route.center_lane.end() )
  {
    return;
  }

  // Forward Pass
  auto prev_it = it;
  ++it;

  double safety_distance  = distance_headway + vehicle_params.wheelbase + vehicle_params.front_axle_to_front_border;
  double max_acceleration = vehicle_params.acceleration_max;
  double max_deceleration = -vehicle_params.acceleration_min;
  for( ; it != end_it; ++it, ++prev_it )
  {

    double s_prev                        = prev_it->first;
    double s_curr                        = it->first;
    double delta_s                       = s_curr - s_prev;
    auto [object_distance, object_speed] = get_nearest_object_info( s_curr, predicted_trajectories );

    double max_curvature_speed = s_to_curvature.count( s_curr ) ? s_to_curvature.at( s_curr ) : max_allowed_speed;
    double max_legal_speed     = it->second.max_speed ? *it->second.max_speed : max_allowed_speed;
    double max_current_speed   = std::min( { max_curvature_speed, max_legal_speed } );

    double idm_acc = idm::calculate_idm_acc( route.get_length() - s_curr, object_distance, max_current_speed, desired_time_headway,
                                             safety_distance, s_to_speed[s_prev], max_acceleration, object_speed );

    // Compute speed limits
    double idm_speed = std::sqrt( s_to_speed[s_prev] * s_to_speed[s_prev] + 2 * idm_acc * delta_s );

    double max_reachable_speed = std::sqrt( s_to_speed[s_prev] * s_to_speed[s_prev] + 2 * max_acceleration * delta_s );

    s_to_speed[s_curr] = std::min( { max_reachable_speed, idm_speed } );
  }

  // Backward Pass (Smoothing and Enforcing Deceleration Limits)
  auto current_it  = std::prev( end_it );
  auto previous_it = std::prev( current_it );

  while( previous_it != route.center_lane.lower_bound( initial_s ) )
  {
    double s_prev  = previous_it->first;
    double s_curr  = current_it->first;
    double delta_s = s_curr - s_prev;

    double max_reachable_speed = std::sqrt( s_to_speed[s_curr] * s_to_speed[s_curr] + 2.0 * max_deceleration * delta_s );

    if( s_to_speed[s_prev] > max_reachable_speed )
    {
      s_to_speed[s_prev] = max_reachable_speed;
    }

    if( previous_it == route.center_lane.begin() )
    {
      break;
    }

    --current_it;
    --previous_it;
  }
}

std::map<double, double>
SpeedProfile::calculate_curvature_speeds( const adore::map::Route& route, double max_lateral_acceleration, double initial_s, double length,
                                          double max_curvature )
{
  std::map<double, double> s_to_curvature;

  if( route.center_lane.size() < 3 )
  {
    std::cerr << "Route has less than 5 points, cannot speed profile" << std::endl;
    return s_to_curvature;
  }

  // Get the boundaries for the route segment
  auto begin_it = std::next( route.center_lane.lower_bound( initial_s ) );
  auto end_it   = route.center_lane.upper_bound( initial_s + length );

  // Iterate over the route with a sliding window of 3 points
  for( auto it = begin_it; std::next( it ) != end_it; ++it )
  {
    const auto& [prev_s, prev_point] = *std::prev( it );
    const auto& [curr_s, curr_point] = *it;
    const auto& [next_s, next_point] = *std::next( it );

    // Compute curvature
    double curvature = adore::math::compute_curvature( prev_point.x, prev_point.y, curr_point.x, curr_point.y, next_point.x, next_point.y );
    curvature        = std::min( curvature, max_curvature );

    // Compute maximum speed for that curvature
    double max_curve_speed = std::sqrt( max_lateral_acceleration / std::max( curvature, 1e-6 ) );
    s_to_curvature[curr_s] = max_curve_speed;
  }

  return s_to_curvature;
}

std::pair<double, double>
SpeedProfile::get_nearest_object_info( double                                                   s_curr,
                                       const std::unordered_map<int, std::map<double, double>>& predicted_trajectories ) const
{
  double object_distance = std::numeric_limits<double>::max();
  double object_speed    = 0.0; // Will only be set if we find an object

  for( const auto& [id, trajectory] : predicted_trajectories )
  {
    auto it = trajectory.lower_bound( s_curr );
    if( it == trajectory.end() )
      continue;

    double predicted_s = it->first;
    if( predicted_s > s_curr )
    {
      double distance = predicted_s - s_curr;
      if( distance < object_distance )
      {
        object_distance = distance;
        object_speed    = it->second;
      }
    }
  }

  return { object_distance, object_speed };
}


} // namespace planner
} // namespace adore