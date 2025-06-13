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

double
SpeedProfile::get_acc_at_s( double s ) const
{
  if( s_to_speed.size() < 2 )
  {
    std::cerr << "[WARNING] Cannot compute acceleration: speed profile too small.\n";
    return 0.0;
  }

  auto it = s_to_speed.lower_bound( s );

  // Handle edge cases
  if( it == s_to_speed.begin() )
  {
    auto it_next = std::next( it );
    if( it_next == s_to_speed.end() )
      return 0.0;

    double ds = it_next->first - it->first;
    double dv = it_next->second - it->second;
    return ( ds != 0.0 ) ? ( dv / ds ) : 0.0;
  }

  if( it == s_to_speed.end() )
  {
    auto it_prev  = std::prev( it );
    auto it_prev2 = ( it_prev == s_to_speed.begin() ) ? it_prev : std::prev( it_prev );

    double ds = it_prev->first - it_prev2->first;
    double dv = it_prev->second - it_prev2->second;
    return ( ds != 0.0 ) ? ( dv / ds ) : 0.0;
  }

  // Use central difference
  auto   it_prev = std::prev( it );
  double ds      = it->first - it_prev->first;
  double dv      = it->second - it_prev->second;
  return ( ds != 0.0 ) ? ( dv / ds ) : 0.0;
}

void
SpeedProfile::generate_from_route_and_participants( const map::Route& route, const dynamics::TrafficParticipantSet& traffic_participants,
                                                    double initial_speed, double initial_s, double initial_time,
                                                    double max_lateral_acceleration, double desired_time_headway, double length )
{
  // Clear previous speed profile
  s_to_speed.clear();

  // Compute curvature-based speed limits
  std::map<double, double> s_to_curvature = calculate_curvature_speeds( route, max_lateral_acceleration, initial_s, length );

  // Initialize starting conditions
  s_to_speed[initial_s] = initial_speed;

  auto it     = route.center_lane.lower_bound( initial_s );
  auto end_it = route.center_lane.lower_bound( initial_s + length );
  end_it--;

  if( it == route.center_lane.end() )
  {
    return;
  }

  auto prev_it = it;
  ++it;

  safety_distance  = distance_headway + vehicle_params.wheelbase + vehicle_params.front_axle_to_front_border;
  max_acceleration = vehicle_params.acceleration_max;
  max_deceleration = -vehicle_params.acceleration_min;
  forward_pass( it, end_it, prev_it, s_to_curvature, route, traffic_participants, initial_time );

  // Backward Pass (Smoothing and Enforcing Deceleration Limits)
  auto current_it  = std::prev( end_it );
  auto previous_it = std::prev( current_it );

  backward_pass( previous_it, route, initial_s, current_it, length );
}

void
SpeedProfile::backward_pass( MapPointIter& previous_it, const adore::map::Route& route, double initial_s, MapPointIter& current_it,
                             double length )
{
  while( previous_it != route.center_lane.lower_bound( initial_s ) )
  {
    double s_prev  = previous_it->first;
    double s_curr  = current_it->first;
    double delta_s = s_curr - s_prev;

    double idm_acc = idm::calculate_idm_acc( length, length, s_to_speed[s_prev], desired_time_headway, safety_distance, s_to_speed[s_curr],
                                             max_deceleration, 0.0 );
    idm_acc        = std::clamp( idm_acc, -max_deceleration, max_acceleration );

    double idm_speed = std::sqrt( s_to_speed[s_curr] * s_to_speed[s_curr] + 2.0 * idm_acc * delta_s );

    if( s_to_speed[s_prev] > idm_speed )
    {
      s_to_speed[s_prev] = idm_speed;
    }

    if( previous_it == route.center_lane.begin() )
    {
      break;
    }

    --current_it;
    --previous_it;
  }
}

void
SpeedProfile::forward_pass( MapPointIter& it, MapPointIter& end_it, MapPointIter& prev_it, std::map<double, double>& s_to_curvature,
                            const adore::map::Route& route, const dynamics::TrafficParticipantSet& traffic_participants,
                            double initial_time )
{
  bool stop = false;

  auto get_nearest_object_info_at_time = [&]( double s_curr, double time ) {
    double object_distance = std::numeric_limits<double>::max();
    double object_speed    = 0.0;

    for( const auto& [id, participant] : traffic_participants.participants )
    {
      const auto& traj = participant.trajectory.value();

      auto state = participant.state;
      if( participant.trajectory.has_value() )
        state = traj.get_state_at_time( time );

      double obj_s  = route.get_s( state );
      double offset = adore::math::distance_2d( state, route.get_pose_at_s( obj_s ) );

      if( offset > 4.0 )
        continue;

      if( obj_s > s_curr )
      {
        double distance = obj_s - s_curr;
        if( distance < object_distance )
        {
          object_distance = distance;
          object_speed    = state.vx;
        }
      }
    }

    return std::make_pair( object_distance, object_speed );
  };

  double time = initial_time;

  for( ; it != end_it; ++it, ++prev_it )
  {
    double s_prev                        = prev_it->first;
    double s_curr                        = it->first;
    double delta_s                       = s_curr - s_prev;
    auto [object_distance, object_speed] = get_nearest_object_info_at_time( s_curr, time );
    if( object_distance < safety_distance )
      stop = true;
    if( stop )
    {
      s_to_speed[s_curr] = 0.0;
      continue;
    }

    double max_curvature_speed = s_to_curvature.lower_bound( s_curr )->second;
    double max_legal_speed     = it->second.max_speed ? *it->second.max_speed : max_allowed_speed;
    double max_reachable_speed = std::sqrt( s_to_speed[s_prev] * s_to_speed[s_prev] + 2 * max_acceleration * delta_s );

    double desired_speed = std::min( { max_curvature_speed, max_legal_speed, max_reachable_speed } );

    double idm_acc = idm::calculate_idm_acc( route.get_length() - s_curr, object_distance, desired_speed, desired_time_headway,
                                             safety_distance, s_to_speed[s_prev], max_acceleration, object_speed );
    idm_acc        = std::clamp( idm_acc, -max_deceleration, max_acceleration );

    // Compute speed limits
    double idm_speed = std::sqrt( s_to_speed[s_prev] * s_to_speed[s_prev] + 2 * idm_acc * delta_s );

    if( !std::isfinite( idm_speed ) )
    {
      std::cerr << "Non-finite speed calculated at s = " << s_curr << ", setting to 0.0" << std::endl;
      std::cerr << ", delta_s: " << delta_s << ", idm_acc: " << idm_acc << ", desired_speed: " << desired_speed
                << ", max_curvature_speed: " << max_curvature_speed << ", max_legal_speed: " << max_legal_speed
                << ", max_reachable_speed: " << max_reachable_speed << ", object_distance: " << object_distance
                << ", object_speed: " << object_speed << std::endl;
      idm_speed = 0.0;
    }

    s_to_speed[s_curr]  = idm_speed;
    time               += delta_s / idm_speed;
    if( idm_speed == 0.0 )
      stop = true;
  }
}

std::map<double, double>
SpeedProfile::calculate_curvature_speeds( const adore::map::Route& route, double max_lateral_acceleration, double initial_s, double length,
                                          double max_curvature )
{
  std::map<double, double> s_to_curvature;

  if( route.center_lane.size() < 3 )
  {
    std::cerr << "Route has less than 3 points, cannot speed profile" << std::endl;
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

    // skip if points are too close
    if( std::fabs( curr_s - prev_s ) < 1e-6 || std::fabs( next_s - curr_s ) < 1e-6 )
    {
      continue;
    }

    // Compute curvature
    double curvature = adore::math::compute_curvature( prev_point.x, prev_point.y, curr_point.x, curr_point.y, next_point.x, next_point.y );
    curvature        = std::min( curvature, max_curvature );

    // Compute maximum speed for that curvature
    double max_curve_speed = std::sqrt( max_lateral_acceleration / std::max( std::fabs( curvature ), 1e-6 ) );
    s_to_curvature[curr_s] = max_curve_speed;
  }

  return s_to_curvature;
}


} // namespace planner
} // namespace adore