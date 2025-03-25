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

#include <optional>
#include <unordered_map>

#include "adore_map/map.hpp"

#include "dynamics/trajectory.hpp"
#include "dynamics/vehicle_state.hpp"

namespace adore
{
namespace planner
{
class LaneFollowPlanner
{
public:

  LaneFollowPlanner();

  void                 set_parameters( const std::map<std::string, double>& params );
  dynamics::Trajectory plan_trajectory( const dynamics::VehicleStateDynamic& current_state, const std::deque<map::MapPoint>& route_points,
                                        const map::Map& local_map, const dynamics::VehicleCommandLimits& limits );

  double       desired_acceleration     = 0.3;
  double       desired_deceleration     = 0.3;
  double       max_lateral_acceleration = 0.5;
  double       override_max_speed       = 2.5;
  double       spline_length_time       = 1.0;
  double       wheelbase                = 2.7;
  double       dt                       = 0.05;
  const double min_point_distance       = 0.05;
  double       max_speed                = 2.5;


private:

  std::optional<dynamics::VehicleStateDynamic> previous_state;

  dynamics::Trajectory generate_trajectory_from_route( const dynamics::VehicleStateDynamic& current_state, const map::Map& local_map,
                                                       const std::deque<map::MapPoint>& route_points, double initial_speed );


  std::vector<double> compute_curvatures( const std::deque<map::MapPoint>& points );

  std::vector<double> generate_speed_profile( const std::vector<double>& distances, const std::deque<map::MapPoint>& route_points,
                                              const std::vector<double>& curvatures, const map::Map& local_map, double initial_speed );


  std::vector<double> compute_cumulative_times( const std::vector<double>& distances, const std::vector<double>& speeds );


  dynamics::Trajectory resample_trajectory( const dynamics::VehicleStateDynamic& current_state, const std::vector<double>& times,
                                            const std::deque<map::MapPoint>& points, const std::vector<double>& speeds,
                                            const std::vector<double>& curvatures );

  std::vector<double> compute_cumulative_distances( const std::deque<map::MapPoint>& points );

  std::deque<map::MapPoint> filter_close_points( const std::deque<map::MapPoint>& points );

  // Apply the spline_into_start to replace the first 2 seconds of trajectory points
  void spline_into_start( const dynamics::VehicleStateDynamic& current_state, std::deque<map::MapPoint>& route_points );
};
} // namespace planner
} // namespace adore
