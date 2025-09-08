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

#include <optional>
#include <unordered_map>
#include <utility>

#include <Eigen/Dense>

#include "adore_map/map.hpp"
#include "adore_math/angles.h"

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
  int plan_trajectories( dynamics::TrafficParticipantSet& traffic_participant_set);


  double       desired_acceleration        = 2.0;
  double       desired_deceleration        = 3.0;
  double       max_lateral_acceleration    = 0.5;
  int          number_of_integration_steps = 160;
  double       dt                          = 0.05;
  const double min_point_distance          = 0.05;
  double       max_allowed_speed           = 10.0;
  double       max_speed                   = 10.0;

  double obstacle_avoidance_offset_threshold = 1.0;
  double k_yaw                               = 8.0;
  double k_distance                          = 2.0;
  double k_speed                             = 0.6;
  double k_goal_point                        = 10.0;
  double k_obstacle_avoidance_longitudinal   = 0.0;
  double k_obstacle_avoidance_lateral        = 2.0;
  double k_sigmoid                           = 5.0;
  double k_lateral_acc                       = 3.5;

  double lane_width   = 4.0;
  double min_distance = 8.0;
  double time_headway = 3.0;

  std::unordered_map<int, double> traffic_light_distances;

  dynamics::VehicleCommandLimits limits;


private:

  dynamics::VehicleStateDynamic   get_current_state( const dynamics::TrafficParticipant& participant );
  adore::dynamics::VehicleCommand compute_vehicle_command( const adore::dynamics::VehicleStateDynamic&   current_state,
                                                           const adore::dynamics::TrafficParticipantSet& traffic_participant_set,
                                                           const int                                     id, const double& ego_goal_distance,
                                                           int &overview_status );

  std::pair<double, double> compute_lane_following_errors( const dynamics::VehicleStateDynamic& current_state,
                                                           const dynamics::TrafficParticipant&  participant );
  double compute_error_lateral_distance( const dynamics::VehicleStateDynamic& current_state, const math::Pose2d& target_pose );
  double compute_error_yaw( const double current_yaw, const double yaw_objective );

  std::tuple<double, double, double> compute_distance_speed_offset_nearest_obstacle(
    const dynamics::TrafficParticipantSet& traffic_participant_set, int id_vehicle );

  double compute_idm_velocity( double dist_to_nearest_object, double distance_to_goal, double nearest_object_speed,
                               const dynamics::VehicleStateDynamic& current_state );

  std::pair<double, double> compute_obstacle_avoidance_speed_component_errors(
    const dynamics::VehicleStateDynamic& current_state, const dynamics::TrafficParticipantSet& traffic_participant_set,
    const int vehicle_id );

  double sigmoid_activation( double d, double d_threshold, double k );

  std::pair<double, double> compute_target_speed_components( const dynamics::VehicleStateDynamic& current_state,
                                                             const dynamics::VehicleStateDynamic& other_participant_state,
                                                             const map::Route&                    route );
};
} // namespace planner
} // namespace adore
