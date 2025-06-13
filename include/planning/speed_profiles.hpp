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

#include <cmath>

#include <iostream>
#include <optional>
#include <unordered_map>

#include "adore_map/map.hpp"
#include "adore_map/route.hpp"
#include "adore_math/curvature.hpp"

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
  std::map<double, double> s_to_speed;

  using MapPointIter = std::map<double, adore::map::MapPoint>::const_iterator;

  double get_speed_at_s( double s ) const;
  double get_acc_at_s( double s ) const;

  void generate_from_route_and_participants( const map::Route& route, const dynamics::TrafficParticipantSet& traffic_participants,
                                             double initial_speed, double initial_s, double initial_time, double max_lateral_acceleration,
                                             double desired_time_headway, double length );

  void backward_pass( MapPointIter& previous_it, const adore::map::Route& route, double initial_s, MapPointIter& current_it,
                      double length );

  void forward_pass( MapPointIter& it, MapPointIter& end_it, MapPointIter& prev_it, std::map<double, double>& s_to_curvature,
                     const adore::map::Route& route, const dynamics::TrafficParticipantSet& traffic_participants, double initial_time );


  SpeedProfile() {};


  double                              max_allowed_speed = 13.7; // 50 km/h in m/s, can be adjusted based on vehicle parameters
  dynamics::PhysicalVehicleParameters vehicle_params;
  double                              desired_time_headway = 3.0; // seconds
  double                              distance_headway     = 3.0; // meters, safety distance ( after vehicle length )

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

private:

  double max_deceleration;
  double max_acceleration;
  double safety_distance;

  std::map<double, double> calculate_curvature_speeds( const adore::map::Route& route, double max_lateral_acceleration, double initial_s,
                                                       double length, double max_curvature = 0.5 );
};

static adore::dynamics::Trajectory
generate_trajectory_from_speed_profile( const SpeedProfile& speed_profile, const map::Route& route, double time_step = 0.1 )
{
  adore::dynamics::Trajectory initial_trajectory;
  double                      accumulated_time = 0.0;

  // Generate initial trajectory from the speed profile
  auto it      = speed_profile.s_to_speed.begin();
  auto next_it = std::next( it );

  while( next_it != speed_profile.s_to_speed.end() )
  {
    double s1 = it->first;
    double s2 = next_it->first;

    double v1      = it->second;
    double v2      = next_it->second;
    double delta_s = s2 - s1;

    // Get the pose at s1
    auto pose = route.get_pose_at_s( s1 );

    // Create a VehicleStateDynamic
    adore::dynamics::VehicleStateDynamic state;
    state.x         = pose.x;
    state.y         = pose.y;
    state.yaw_angle = pose.yaw;
    state.vx        = v1;
    state.time      = accumulated_time;

    // Add to the initial trajectory
    initial_trajectory.states.push_back( state );

    // Integrate time using the segment speed (average of v1 and v2)
    double avg_speed  = ( v1 + v2 ) / 2.0;
    accumulated_time += ( avg_speed > 1e-3 ) ? delta_s / avg_speed : time_step;

    ++it;
    ++next_it;
  }

  // Re-interpolate to constant time intervals using `get_state_at_time`
  adore::dynamics::Trajectory trajectory;
  double                      final_time = accumulated_time;

  for( double t = 0.0; t <= final_time; t += time_step )
  {
    auto interpolated_state = initial_trajectory.get_state_at_time( t );
    trajectory.states.push_back( interpolated_state );
  }

  return trajectory;
}
} // namespace planner
} // namespace adore