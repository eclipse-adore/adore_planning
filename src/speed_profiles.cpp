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
  if( it == s_to_speed.end() ) // right-side extrapolation

  {
    auto   it_last = s_to_speed.rbegin();
    auto   it_prev = std::next( it_last );
    double slope   = ( it_last->second - it_prev->second ) / +( it_last->first - it_prev->first );
    return it_last->second + slope * ( s - it_last->first );
  }
  else if( it == s_to_speed.begin() ) // left-side extrapolation

  {
    auto   it_next = std::next( it );
    double slope   = ( it_next->second - it->second ) / +( it_next->first - it->first );
    return it->second + slope * ( s - it->first );
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
  auto it = s_to_acc.lower_bound( s );
  if( it == s_to_acc.end() )
  {
    return s_to_acc.rbegin()->second;
  }
  else if( it == s_to_acc.begin() )
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

void
SpeedProfile::generate_from_route_and_participants( const map::Route& route, const dynamics::TrafficParticipantSet& traffic_participants,
                                                    double initial_speed, double initial_s, double initial_time, double length )
{
  // Clear previous speed profile
  s_to_speed.clear();
  s_to_acc.clear();

  // Compute curvature-based speed limits
  std::map<double, double> s_to_curvature = calculate_curvature_speeds( route, initial_s, length );

  // Initialize starting conditions
  auto   it          = route.center_lane.lower_bound( initial_s );
  double s_curr      = it->first;
  s_to_speed[s_curr] = initial_speed;

  auto end_it = std::prev( route.center_lane.lower_bound( initial_s + length ) );

  if( it == route.center_lane.end() )
  {
    return;
  }

  auto prev_it = it;
  ++it;

  safety_distance = comfort_settings->distance_headway + vehicle_params.wheelbase + vehicle_params.front_axle_to_front_border;
  max_acc         = comfort_settings->max_acceleration;
  max_decel       = -comfort_settings->min_acceleration;
  forward_pass( it, end_it, prev_it, s_to_curvature, route, traffic_participants, initial_time );

  // Backward Pass (Smoothing and Enforcing Deceleration Limits)
  auto current_it  = end_it; // now inclusive
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

    double idm_acc = idm::calculate_idm_acc( length, length, s_to_speed[s_prev], comfort_settings->time_headway, safety_distance,
                                             s_to_speed[s_curr], max_decel, 0.0 );
    idm_acc        = std::clamp( idm_acc, -max_acc, max_decel );

    double idm_speed = std::sqrt( s_to_speed[s_curr] * s_to_speed[s_curr] + 2.0 * idm_acc * delta_s );

    if( s_to_speed[s_prev] > idm_speed )
    {
      s_to_speed[s_prev] = idm_speed;
      s_to_acc[s_prev]   = -idm_acc;
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

    double max_curvature_speed  = s_to_curvature.lower_bound( s_curr )->second;
    double max_legal_speed      = it->second.max_speed ? *it->second.max_speed : comfort_settings->max_speed;
    max_legal_speed            *= comfort_settings->speed_fraction_of_limit;
    double max_reachable_speed  = std::sqrt( s_to_speed[s_prev] * s_to_speed[s_prev] + 2 * max_acc * delta_s );

    double desired_speed = std::min( { max_curvature_speed, max_legal_speed } );

    double idm_acc = idm::calculate_idm_acc( route.get_length() - s_curr, object_distance, desired_speed, comfort_settings->time_headway,
                                             safety_distance, s_to_speed[s_prev], max_acc, object_speed );
    idm_acc        = std::clamp( idm_acc, -max_decel, max_acc );

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
    s_to_acc[s_curr]    = idm_acc;
    time               += delta_s / idm_speed;
    if( idm_speed == 0.0 )
      stop = true;
  }
}

std::map<double, double>
SpeedProfile::calculate_curvature_speeds( const adore::map::Route& route, double initial_s, double length, double max_curvature )
{
  std::map<double, double> s_to_curvature;

  if( route.center_lane.size() < 3 )
  {
    std::cerr << "Route has less than 3 points, cannot speed profile\n";
    return s_to_curvature;
  }

  auto begin_it = std::next( route.center_lane.lower_bound( initial_s ) );
  auto end_it   = route.center_lane.upper_bound( initial_s + length );

  for( auto it = begin_it; std::next( it ) != end_it; ++it )
  {
    double s         = it->first;
    double curvature = route.get_curvature_at_s( s );
    curvature        = std::min( std::fabs( curvature ), max_curvature );

    double vmax       = std::sqrt( comfort_settings->max_lateral_acceleration / std::max( curvature, 1e-6 ) );
    s_to_curvature[s] = vmax;
  }

  const int    half_win = 5; // k ⇒ window = 2k+1
  const size_t N        = s_to_curvature.size();
  if( N < 5 )
    return s_to_curvature; // nothing to smooth

  std::vector<double> keys, vals;
  keys.reserve( N );
  vals.reserve( N );
  for( auto& kv : s_to_curvature )
  {
    keys.push_back( kv.first );
    vals.push_back( kv.second );
  }

  std::vector<double> smoothed( vals );

  for( size_t i = 0; i < N; ++i )
  {
    int lo = static_cast<int>( i ) - half_win;
    int hi = lo + half_win * 2;
    lo     = std::max( lo, 0 );
    hi     = std::min( hi, static_cast<int>( N ) - 1 );

    double sum = 0.0;
    for( int j = lo; j <= hi; ++j )
      sum += vals[j];
    double avg = sum / ( hi - lo + 1 );

    smoothed[i] = avg;
  }

  for( size_t i = 0; i < N; ++i )
    s_to_curvature[keys[i]] = smoothed[i];

  return s_to_curvature;
}


} // namespace planner
} // namespace adore