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
#pragma once

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
      trajectory[accumulated_time]  = current_s + current_velocity * accumulated_time;
      accumulated_time             += time_step;
    }
    predicted_trajectories[id] = trajectory;
  }

  return predicted_trajectories;
}

void
SpeedProfile::generate_from_route_and_participants( const map::Route& route, const dynamics::TrafficParticipantSet& traffic_participants,
                                                    const map::Map& local_map, double initial_speed, double max_speed,
                                                    double max_lateral_acceleration, double desired_time_headway, double start_s,
                                                    double length )
{
  // Clear previous speed profile
  s_to_speed.clear();

  // Compute curvature-based speed limits
  std::map<double, double> s_to_curvature = calculate_curvature_speeds( route, max_lateral_acceleration, start_s, length, 0.5 );

  // Precompute Traffic Participant Trajectories
  auto predicted_trajectories = predict_traffic_participant_trajectories( traffic_participants, route );

  // Initialize starting conditions
  s_to_speed[start_s] = initial_speed;

  auto it     = route.center_lane.lower_bound( start_s );
  auto end_it = route.center_lane.upper_bound( start_s + length );

  if( it == route.center_lane.end() )
  {
    std::cerr << "No route points found after start_s" << std::endl;
    return;
  }

  // Forward Pass
  auto prev_it = it;
  ++it;

  for( ; it != end_it; ++it, ++prev_it )
  {
    double s_prev = prev_it->first;
    double s_curr = it->first;

    double delta_s = s_curr - s_prev;

    // Lookup the max speed at the current point
    double max_curvature_speed = s_to_curvature.count( s_curr ) ? s_to_curvature.at( s_curr ) : max_speed;
    double max_reachable_speed = std::sqrt( s_to_speed[s_prev] * s_to_speed[s_prev] + 2 * max_lateral_acceleration * delta_s );

    double idm_acc = idm::calculate_idm_acc( route.get_length() - s_curr, 100, max_curvature_speed, 5.0, 5.0, s_to_speed[s_prev], 2.0,
                                             1.0 );

    // Compute speed limits
    double idm_speed = std::sqrt( s_to_speed[s_prev] * s_to_speed[s_prev] + 2 * idm_acc * delta_s );

    // Directly update in the map
    s_to_speed[s_curr] = std::min( { max_reachable_speed, idm_speed, max_curvature_speed } );
  }

  // Backward Pass (Smoothing and Enforcing Deceleration Limits)
  auto current_it  = std::prev( end_it );
  auto previous_it = std::prev( current_it );

  while( previous_it != route.center_lane.lower_bound( start_s ) )
  {
    double s_prev  = previous_it->first;
    double s_curr  = current_it->first;
    double delta_s = s_curr - s_prev;

    double max_reachable_speed = std::sqrt( s_to_speed[s_curr] * s_to_speed[s_curr] + 2.0 * max_lateral_acceleration * delta_s );

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
SpeedProfile::calculate_curvature_speeds( const adore::map::Route& route, double max_lateral_acceleration, double start_s, double length,
                                          double max_curvature )
{
  std::map<double, double> s_to_curvature;

  if( route.center_lane.size() < 3 )
  {
    std::cerr << "Route has less than 5 points, cannot speed profile" << std::endl;
    return s_to_curvature;
  }

  // Get the boundaries for the route segment
  auto begin_it = std::next( route.center_lane.lower_bound( start_s ) );
  auto end_it   = route.center_lane.upper_bound( start_s + length );

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

} // namespace planner
} // namespace adore