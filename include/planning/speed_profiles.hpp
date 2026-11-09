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

#pragma once

#include <cmath>

#include <iostream>
#include <optional>
#include <unordered_map>

#include "adore_map/map.hpp"
#include "adore_map/route.hpp"
#include "adore_math/curvature.hpp"

#include "controllers/controller.hpp"
#include "dynamics/comfort_settings.hpp"
#include "dynamics/physical_vehicle_parameters.hpp"
#include "dynamics/traffic_participant.hpp"
#include "dynamics/trajectory.hpp"
#include "dynamics/vehicle_state.hpp"
#include "planning/idm.hpp"

namespace adore
{
namespace planner
{

struct SpeedProfile
{

  using MapPointIter = std::map<double, adore::map::MapPoint>::const_iterator;

  double get_speed_at_s( double s ) const;
  double get_acc_at_s( double s ) const;

  void generate_from_route_and_participants( const map::Route& route, const dynamics::TrafficParticipantSet& traffic_participants,
                                             double initial_speed, double initial_s, double initial_time, double length );

  void backward_pass( MapPointIter& previous_it, const adore::map::Route& route, double initial_s, MapPointIter& current_it,
                      double length );

  void forward_pass( MapPointIter& it, MapPointIter& end_it, MapPointIter& prev_it, std::map<double, double>& s_to_curvature,
                     const adore::map::Route& route, const dynamics::TrafficParticipantSet& traffic_participants, double initial_time );


  SpeedProfile() {};

  void
  set_vehicle_parameters( const dynamics::PhysicalVehicleParameters& params )
  {
    vehicle_params = params;
  }

  void
  set_comfort_settings( const dynamics::ComfortSettings& settings )
  {
    comfort_settings = settings;
  }

  // Overloading the << operator
  friend std::ostream&
  operator<<( std::ostream& os, const SpeedProfile& profile )
  {
    os << "Speed Profile (s -> speed):" << std::endl;
    for( const auto& [s, speed] : profile.s_to_speed )
    {
      os << "s = " << s << " m, speed = " << speed << " m/s" << std::endl;
    }
    return os;
  }

  std::map<double, double> s_to_speed;
  std::map<double, double> s_to_acc;

private:

  std::map<double, double>                   calculate_curvature_speeds( const adore::map::Route& route, double initial_s, double length,
                                                                         double max_curvature = 0.5 );
  dynamics::PhysicalVehicleParameters        vehicle_params;
  dynamics::ComfortSettings comfort_settings;


  // default values overritten by comfort settings
  double max_acc         = 2.0;  // [m/s^2] maximum acceleration
  double max_decel       = -2.0; // [m/s^2]
  double safety_distance = 3.0;  // [m] safety distance to the nearest object
};

static adore::dynamics::Trajectory
generate_trajectory_from_speed_profile( const SpeedProfile& speed_profile, const map::Route& route,
                                        const dynamics::VehicleStateDynamic& start_state, double time_step = 0.1 )
{
  adore::dynamics::Trajectory initial_trajectory;
  double                      accumulated_time = 0.0;

  // basic sanity on time_step
  if( !std::isfinite( time_step ) || time_step <= 0.0 )
  {
    time_step = 0.1;
  }

  // need at least two points to form a segment
  if( speed_profile.s_to_speed.size() < 2 )
  {
    return initial_trajectory;
  }

  constexpr double min_avg_speed  = 0.1;    // m/s, avoid division by ~0
  constexpr double stopped_speed  = 1e-6;   // a zero-speed route point is a stop boundary
  constexpr double max_total_time = 3600.0; // clamp to 1h to avoid insane trajectories

  auto it      = speed_profile.s_to_speed.begin();
  auto next_it = std::next( it );

  while( next_it != speed_profile.s_to_speed.end() )
  {
    const double s1 = it->first;
    const double s2 = next_it->first;
    const double v1 = it->second;
    const double v2 = next_it->second;

    const double delta_s = s2 - s1;

    // skip non-forward or zero-length segments
    if( delta_s <= 0.0 )
    {
      ++it;
      ++next_it;
      continue;
    }

    double avg_v = 0.5 * ( v1 + v2 );

    // avoid zero / negative or tiny average speed → clamp
    if( !std::isfinite( avg_v ) || avg_v < min_avg_speed )
    {
      avg_v = min_avg_speed;
    }

    const double delta_t = delta_s / avg_v;

    if( !std::isfinite( delta_t ) || delta_t <= 0.0 )
    {
      ++it;
      ++next_it;
      continue;
    }

    const auto pose = route.get_pose_at_s( s1 );

    adore::dynamics::VehicleStateDynamic state;
    state.x         = pose.x;
    state.y         = pose.y;
    state.yaw_angle = pose.yaw;
    state.vx        = v1;
    state.time      = accumulated_time;

    accumulated_time += delta_t;

    if( accumulated_time > max_total_time )
    {
      accumulated_time = max_total_time;
    }

    const double dt = accumulated_time - state.time;
    if( dt > 0.0 && std::isfinite( dt ) )
    {
      state.ax = ( v2 - v1 ) / dt;
    }
    else
    {
      state.ax = 0.0;
    }

    initial_trajectory.states.push_back( state );

    // A zero-speed route point is a spatial stop boundary: stop AT it instead of
    // creeping past it (the min_avg_speed clamp above would otherwise keep
    // advancing the pose while vx=0). Key off the segment END speed (v2): break
    // only at a *downstream* stop, never on v1 alone -> a start from standstill
    // (v1=0, v2>0) must keep building and drive off. If the vehicle was still
    // moving into the stop (v1>0), add the stop pose at s2 so it halts AT s2;
    // if it was already stopped (v1~0), hold here and drop later route points.
    if( v2 <= stopped_speed )
    {
      if( v1 > stopped_speed )
      {
        const auto stop_pose = route.get_pose_at_s( s2 );

        adore::dynamics::VehicleStateDynamic stop_state;
        stop_state.x         = stop_pose.x;
        stop_state.y         = stop_pose.y;
        stop_state.yaw_angle = stop_pose.yaw;
        stop_state.vx        = 0.0;
        stop_state.time      = accumulated_time;
        initial_trajectory.states.push_back( stop_state );
      }
      break;
    }

    if( accumulated_time >= max_total_time )
    {
      break;
    }

    ++it;
    ++next_it;
  }

  adore::dynamics::Trajectory trajectory;

  // nothing usable built → just return empty trajectory
  if( initial_trajectory.states.empty() )
  {
    return trajectory;
  }

  const double t_final = initial_trajectory.states.back().time;

  if( !std::isfinite( t_final ) || t_final <= 0.0 )
  {
    // Fully stopped route (e.g. ego already halted at the obstacle / oncoming
    // wait): return a short route-aligned hold at the first pose instead of an
    // empty (untracked) trajectory, so the stop needs no fabricated hold.
    auto hold_state = initial_trajectory.states.front();
    hold_state.vx   = 0.0;
    hold_state.time = 0.0;
    trajectory.states.push_back( hold_state );
    hold_state.time = time_step;
    trajectory.states.push_back( hold_state );
    return trajectory;
  }

  const double   clamped_t_final = std::min( t_final, max_total_time );
  std::size_t    max_steps       = static_cast<std::size_t>( clamped_t_final / time_step ) + 1;
  constexpr auto hard_step_cap   = static_cast<std::size_t>( 100000 ); // safety cap

  if( max_steps > hard_step_cap )
  {
    max_steps = hard_step_cap;
  }

  dynamics::VehicleStateDynamic current_state = start_state;

  for( std::size_t step = 0; step <= max_steps; ++step )
  {
    double t = static_cast<double>( step ) * time_step;
    if( t > clamped_t_final )
    {
      t = clamped_t_final;
    }

    current_state = initial_trajectory.get_state_at_time( t );
    trajectory.states.push_back( current_state );

    if( t >= clamped_t_final )
    {
      break;
    }
  }

  return trajectory;
}

} // namespace planner
} // namespace adore