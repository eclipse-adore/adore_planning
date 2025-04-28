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
#pragma once

#include <cmath>

#include <optional>
#include <unordered_map>
#include <utility>

#include <Eigen/Dense>

#include "adore_map/map.hpp"

#include "dynamics/traffic_participant.hpp"
#include "dynamics/trajectory.hpp"
#include "dynamics/vehicle_state.hpp"

namespace adore
{
namespace planner
{
using MotionModel = std::function<dynamics::VehicleStateDynamic( const dynamics::VehicleStateDynamic&, const dynamics::VehicleCommand& )>;

class MultiAgentPID
{
public:

  MultiAgentPID();

  void set_parameters( const std::map<std::string, double>& params );
  void plan_trajectories( dynamics::TrafficParticipantSet& traffic_participant_set );


  double       desired_acceleration        = 1.0;
  double       desired_deceleration        = 1.0;
  double       max_lateral_acceleration    = 0.5;
  int          number_of_integration_steps = 50;
  double       dt                          = 0.1;
  const double min_point_distance          = 0.05;
  double       max_speed                   = 3.0;

  double obstacle_avoidance_offset_threshold = 1.0;
  double k_yaw                               = 8.0;
  double k_distance                          = 2.0;
  double k_speed                             = 0.5;
  double k_goal_point                        = 10.0;
  double k_sigmoid                           = 5.0;
  double preview_distance                    = 3.0;

  double lane_width   = 4.0;
  double min_distance = 3.0;
  double time_headway = 3.0;

  dynamics::VehicleCommandLimits limits;


private:

  dynamics::VehicleStateDynamic   get_current_state( const dynamics::TrafficParticipant& participant );
  adore::dynamics::VehicleCommand compute_vehicle_command( const adore::dynamics::VehicleStateDynamic&   current_state,
                                                           const adore::dynamics::TrafficParticipantSet& traffic_participant_set,
                                                           const int                                     id );

  std::pair<double, double> compute_lane_following_errors( const dynamics::VehicleStateDynamic& current_state,
                                                           const dynamics::TrafficParticipant&  participant );

  double compute_error_lateral_distance( const dynamics::VehicleStateDynamic& current_state, const math::Pose2d& target_pose );
  double compute_error_yaw( const double current_yaw, const double yaw_objective );


  double compute_idm_velocity( double dist_to_nearest_object, double distance_to_goal, double nearest_object_speed,
                               const dynamics::VehicleStateDynamic& current_state );

  std::tuple<double, double, double> get_nearest_obstacle_distance_speed_offset(
    const dynamics::TrafficParticipantSet& traffic_participant_set, const int id );
};
} // namespace planner
} // namespace adore
