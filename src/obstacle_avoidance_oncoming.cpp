/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

// Oncoming-traffic reasoning: opposite-lane conflict intervals for a planned
// avoidance maneuver, ego-lane oncoming threat detection, predicted arrival /
// ego-clear timing along the route, and the gap-acceptance decision
// (check_oncoming_gap). Depends on the geometry, projection, grouping, shift and
// drivable-area helpers in oa_detail plus the public OncomingConflictResult.

#include "obstacle_avoidance_internal.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace adore
{
namespace planner
{
namespace oa_detail
{

bool
participant_footprint_overlaps_ego_lane(
  const map::Route& route,
  const ParticipantFootprintOnRoute& footprint,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  const auto route_point = find_reference_point_near_s( route, footprint.center_s );

  if( route_point.has_value() )
  {
    const auto current_lane_interval =
      get_allowed_lateral_interval_at_route_point(
        route,
        route_point->second,
        route_point->first,
        0.0,
        false,
        params );

    if( current_lane_interval.has_value() )
    {
      return footprint.l_max >= current_lane_interval->min_l - params.ego_lane_oncoming_lateral_margin &&
             footprint.l_min <= current_lane_interval->max_l + params.ego_lane_oncoming_lateral_margin;
    }
  }

  // Fallback if map/lane boundaries are unavailable: use a narrow ego corridor
  // around the reference line. This still prevents normal oncoming traffic in a
  // clearly separated opposite lane from triggering unless it overlaps the ego
  // corridor.
  const double corridor_half_width =
    0.5 * ego_params.body_width +
    params.ego_corridor_safety_margin +
    params.ego_lane_oncoming_lateral_margin;

  return footprint.l_min <= corridor_half_width &&
         footprint.l_max >= -corridor_half_width;
}

std::optional<EgoLaneOncomingThreat>
find_ego_lane_oncoming_threat(
  const map::Route& route,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::TrafficParticipantSet& traffic_participants,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  const double ego_s = project_s_on_reference_line( route, ego );
  if( !std::isfinite( ego_s ) )
  {
    return std::nullopt;
  }

  const double ego_front_offset =
    ego_params.wheelbase + ego_params.front_axle_to_front_border;

  std::optional<EgoLaneOncomingThreat> best_threat;

  for( const auto& [id, participant] : traffic_participants.participants )
  {
    const double participant_speed =
      std::hypot( participant.state.vx, participant.state.vy );
    if( participant_speed <= params.max_static_object_speed )
    {
      continue;
    }

    const auto footprint =
      project_participant_footprint_to_route( route, participant, params );
    if( !footprint.has_value() )
    {
      continue;
    }

    const double participant_near_s = footprint->s_min;
    const double distance_s = participant_near_s - ego_s - ego_front_offset;

    // Participant has already passed the ego reference point and moves further
    // toward decreasing s. It no longer creates a head-on threat ahead.
    if( footprint->s_max < ego_s )
    {
      continue;
    }

    const auto route_pose = route.get_pose_at_s( footprint->center_s );
    const double yaw_diff = normalize_angle( participant.state.yaw_angle - route_pose.yaw );
    const double v_route =
      participant.state.vx * std::cos( yaw_diff ) -
      participant.state.vy * std::sin( yaw_diff );

    if( v_route >= -params.min_oncoming_route_speed )
    {
      continue;
    }

    if( !participant_footprint_overlaps_ego_lane(
          route,
          footprint.value(),
          ego_params,
          params ) )
    {
      continue;
    }

    const double closing_speed =
      std::max( 0.0, ego.vx ) + std::fabs( v_route );

    const double time_to_conflict =
      std::max( 0.0, distance_s ) / closing_speed;

    if( params.prediction_time_horizon > 0.0 &&
        time_to_conflict > params.prediction_time_horizon )
    {
      continue;
    }

    EgoLaneOncomingThreat threat;
    threat.participant_id = static_cast<int>( id );
    threat.participant_near_s = participant_near_s;
    threat.time_to_conflict = time_to_conflict;

    if( !best_threat.has_value() ||
        threat.time_to_conflict < best_threat->time_to_conflict )
    {
      best_threat = threat;
    }
  }

  return best_threat;
}

OppositeLaneConflictInterval
compute_opposite_lane_conflict_interval(
  const map::Route& route,
  const AvoidanceGroup& group,
  double lateral_shift,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  OppositeLaneConflictInterval interval;

  if( route.reference_line.empty() )
  {
    interval.reason = "route reference line is empty";
    return interval;
  }

  if( !std::isfinite( lateral_shift ) || std::fabs( lateral_shift ) < 1e-6 )
  {
    interval.valid = true;
    interval.occupies_opposite_lane = false;
    interval.reason = "lateral shift is zero";
    return interval;
  }

  const double ego_half_width = 0.5 * ego_params.body_width;

  const bool shift_left = lateral_shift > 0.0;

  bool found_conflict_sample = false;
  bool found_active_sample = false;
  bool map_usable_at_least_once = false;
  bool opposite_lane_present_at_least_once = false;
  bool fallback_used_at_least_once = false;
  bool evaluated_active_sample = false;
  std::size_t invalid_lane_samples = 0;

  double first_conflict_s = std::numeric_limits<double>::infinity();
  double last_conflict_s = -std::numeric_limits<double>::infinity();

  for( const auto& [route_s, route_point] : route.reference_line )
  {
    const double center_l =
      avoidance_shift_offset_at_s(
        route_s,
        group,
        lateral_shift,
        ego_params,
        params );
    if( std::fabs( center_l ) <= 1e-6 )
    {
      continue;
    }

    found_active_sample = true;

    // Use the outer ego edge, not only the reference point.
    const double ego_left_l = center_l + ego_half_width;

    const double ego_right_l = center_l - ego_half_width;

    const auto opposite_query =
      query_opposite_direction_lateral_interval_at_route_point(
        route,
        route_point,
        route_s,
        shift_left,
        params );

    bool conflict_here = false;

    if( opposite_query.map_usable )
    {
      map_usable_at_least_once = true;
      evaluated_active_sample = true;

      if( opposite_query.has_opposite_lane )
      {
        opposite_lane_present_at_least_once = true;

        // Overlap of two 1-D lateral intervals. This is the preferred conflict
        // definition: ego footprint physically overlaps an opposite-direction
        // driving lane.
        const bool ego_overlaps_opposite =
          ego_left_l > opposite_query.interval.min_l &&
          ego_right_l < opposite_query.interval.max_l;

        conflict_here = ego_overlaps_opposite;
      }
      else
      {
        // Important distinction: the map was usable and explicitly found no
        // opposite-direction lane on the shift side. This covers multi-lane
        // same-direction roads. Do NOT fall back to "left lane departure ==
        // oncoming" in this case.
        conflict_here = false;
      }
    }
    else
    {
      // Only when topology is not usable do we use the conservative right-hand-
      // traffic fallback: leaving the current lane on the shift side may mean
      // entering an oncoming lane.
      fallback_used_at_least_once = true;

      const auto current_lane_interval =
        get_allowed_lateral_interval_at_route_point(
          route,
          route_point,
          route_s,
          lateral_shift,
          false,  // current route lane only
          params );

      if( !current_lane_interval.has_value() )
      {
        ++invalid_lane_samples;
        continue;
      }

      evaluated_active_sample = true;

      const bool crosses_left_border =
        shift_left && ego_left_l > current_lane_interval->max_l;

      const bool crosses_right_border =
        !shift_left && ego_right_l < current_lane_interval->min_l;

      conflict_here = crosses_left_border || crosses_right_border;
    }

    if( conflict_here )
    {
      found_conflict_sample = true;
      first_conflict_s = std::min( first_conflict_s, route_s );
      last_conflict_s = std::max( last_conflict_s, route_s );
    }
  }

  if( found_conflict_sample )
  {
    interval.valid = true;
    interval.occupies_opposite_lane = true;
    const double spatial_margin =
      std::max( 0.0, params.oncoming_spatial_margin );
    interval.start_s =
      std::max( 0.0, first_conflict_s - spatial_margin );
    interval.end_s =
      last_conflict_s + spatial_margin;
    interval.reason =
      opposite_lane_present_at_least_once
        ? "ego footprint enters opposite-direction lane"
        : "ego footprint crosses current lane border (map unavailable fallback)";
    return interval;
  }

  if( found_active_sample && evaluated_active_sample )
  {
    interval.valid = true;
    interval.occupies_opposite_lane = false;

    if( map_usable_at_least_once && !opposite_lane_present_at_least_once )
    {
      interval.reason = "map usable: no opposite-direction lane on candidate shift side";
    }
    else if( opposite_lane_present_at_least_once )
    {
      interval.reason = "ego footprint stays clear of opposite-direction lane";
    }
    else if( fallback_used_at_least_once )
    {
      interval.reason = "fallback evaluated: ego footprint stays inside current lane";
    }
    else
    {
      interval.reason = "shifted samples evaluated without opposite-lane occupancy";
    }

    return interval;
  }

  interval.valid = false;
  interval.occupies_opposite_lane = false;
  interval.reason =
    invalid_lane_samples > 0
      ? "could not evaluate lane intervals on shifted samples"
      : "no active shifted samples available";
  return interval;
}

std::optional<double>
compute_arrival_time_from_trajectory(
  const map::Route& route,
  const dynamics::Trajectory& trajectory,
  double now_time,
  double conflict_start_s,
  double conflict_end_s )
{
  if( trajectory.states.size() < 2 )
  {
    return std::nullopt;
  }

  if( !std::isfinite( now_time ) )
  {
    return std::nullopt;
  }

  // Walk the trajectory in time order; remember the previous state's s and t so
  // we can linearly interpolate the crossing into the conflict interval.
  double s_prev = std::numeric_limits<double>::quiet_NaN();
  double t_prev = std::numeric_limits<double>::quiet_NaN();
  bool   has_prev = false;
  bool   any_state_in_future = false;

  for( const auto& state : trajectory.states )
  {
    const double t_now = state.time - now_time;
    if( t_now < 0.0 )
    {
      // Trajectory state already in the past; skip but treat it as a valid
      // anchor for interpolation if a future state is seen later.
      const double s_now_past =
        project_s_on_reference_line( route, state, s_prev );
      if( std::isfinite( s_now_past ) )
      {
        s_prev   = s_now_past;
        t_prev   = t_now;
        has_prev = true;
      }
      continue;
    }

    any_state_in_future = true;

    const double s_now =
      project_s_on_reference_line( route, state, s_prev );
    if( !std::isfinite( s_now ) )
    {
      has_prev = false;
      continue;
    }

    const bool inside_now =
      s_now >= conflict_start_s && s_now <= conflict_end_s;

    if( inside_now )
    {
      if( has_prev && std::isfinite( s_prev ) && std::isfinite( t_prev ) )
      {
        // Refine entry by linear interpolation between the previous (outside)
        // and current (inside) sample. Use the boundary that the previous
        // sample approached: if s_prev was past the end, interpolate at
        // conflict_end_s; otherwise interpolate at conflict_start_s.
        const double boundary = s_prev > conflict_end_s
                                  ? conflict_end_s
                                  : conflict_start_s;

        const double ds = s_now - s_prev;
        if( std::fabs( ds ) > 1e-9 )
        {
          const double alpha = std::clamp( ( boundary - s_prev ) / ds, 0.0, 1.0 );
          return std::max( 0.0, t_prev + alpha * ( t_now - t_prev ) );
        }
      }

      return std::max( 0.0, t_now );
    }

    s_prev   = s_now;
    t_prev   = t_now;
    has_prev = true;
  }

  if( !any_state_in_future )
  {
    return std::nullopt;
  }

  // Trajectory horizon did not cover the conflict interval.
  return std::nullopt;
}

std::optional<double>
compute_ego_clear_time_from_trajectory(
  const map::Route& route,
  const dynamics::Trajectory& trajectory,
  double now_time,
  double conflict_end_s,
  double initial_s_hint,
  double projection_window )
{
  if( trajectory.states.empty() || !std::isfinite( now_time ) ||
      !std::isfinite( conflict_end_s ) )
  {
    return std::nullopt;
  }

  double previous_s = initial_s_hint;

  for( const auto& state : trajectory.states )
  {
    const double t_now = state.time - now_time;
    if( t_now < 0.0 )
    {
      continue;
    }

    double state_s =
      adore::map::get_s_on_reference_line_segments(
        route,
        state,
        std::isfinite( previous_s ) ? previous_s : conflict_end_s,
        std::max( 0.0, projection_window ) );

    if( !std::isfinite( state_s ) )
    {
      state_s = project_s_on_reference_line( route, state, previous_s );
    }

    if( !std::isfinite( state_s ) )
    {
      continue;
    }

    previous_s = state_s;

    if( state_s >= conflict_end_s )
    {
      return std::max( 0.0, t_now );
    }
  }

  return std::nullopt;
}

/**
 * Time-based oncoming traffic gap-acceptance check.
 *
 * Determines whether an obstacle-avoidance maneuver can be safely executed
 * with respect to oncoming traffic in an opposite-direction lane.
 *
 * Conflict-zone definition (from compute_opposite_lane_conflict_interval):
 * - Preferred: the longitudinal range of route s values at which the ego
 *   footprint overlaps the lateral interval of an adjacent opposite-direction
 *   driving lane. The first such s becomes conflict_start_s, the last becomes
 *   conflict_end_s, then both are expanded by oncoming_spatial_margin to
 *   account for geometry, localization and prediction uncertainty.
 * - If adjacent-lane direction cannot be determined from map data, a right-
 *   hand-traffic fallback flags the same interval whenever the ego footprint
 *   leaves the current lane on the shift side.
 * - If no shifted samples can be evaluated at all, a conservative obstacle-
 *   envelope fallback is used.
 *
 * Route-aligned velocity convention (participant vx/vy are body-frame):
 * - v_route = vx*cos(yaw_diff) - vy*sin(yaw_diff)
 * - v_route > 0: participant moves in the ego route direction.
 * - v_route < 0: participant moves against the ego route direction / oncoming.
 *
 * Arrival-time estimation:
 * - If TrafficParticipant.trajectory is available, the participant's predicted
 *   trajectory is walked in time order and projected onto the ego route. The
 *   earliest state that lands in [conflict_start_s, conflict_end_s] defines the
 *   arrival time (linearly interpolated between consecutive samples).
 * - Otherwise, a constant-velocity prediction is used with the route-aligned
 *   speed |v_route| floored by min_oncoming_route_speed.
 *
 * Decision rule:
 *   arrival_time <= ego_clear_time + oncoming_time_margin -> reject maneuver
 *   arrival_time >  ego_clear_time + oncoming_time_margin -> accept maneuver
 */
planner::OncomingConflictResult
check_oncoming_gap( const map::Route& route,
                    const map::Route& driven_route,
                    const dynamics::VehicleStateDynamic& ego,
                    const dynamics::TrafficParticipantSet& traffic_participants,
                    const AvoidanceGroup& group,
                    double lateral_shift,
                    const dynamics::PhysicalVehicleParameters& ego_params,
                    const dynamics::Trajectory* candidate_ego_trajectory,
                    const ObstacleAvoidanceParams& params )
{
  planner::OncomingConflictResult result;
  result.conflict = false;
  result.participant_id = -1;
  result.oncoming_arrival_time = std::numeric_limits<double>::infinity();

  const double ego_s = project_s_on_reference_line( route, ego );
  if( !std::isfinite( ego_s ) )
  {
    result.reason = "ego position invalid on route";
    return result;
  }

  const auto conflict_interval =
    compute_opposite_lane_conflict_interval(
      route,
      group,
      lateral_shift,
      ego_params,
      params );

  if( conflict_interval.valid && !conflict_interval.occupies_opposite_lane )
  {
    result.reason = conflict_interval.reason;

    return result;
  }

  if( conflict_interval.valid )
  {
    result.conflict_start_s = conflict_interval.start_s;
    result.conflict_end_s = conflict_interval.end_s;
  }
  else
  {
    // Conservative fallback if lane boundaries cannot be evaluated. This is not
    // the preferred conflict-zone definition, but it avoids accepting an
    // opposite-lane maneuver blindly when map data is incomplete.
    result.conflict_start_s =
      std::max(
        0.0,
        group.envelope.object_s_min
        - params.front_clearance
        - params.oncoming_spatial_margin );

    result.conflict_end_s =
      group.envelope.object_s_max
      + params.rear_clearance
      + params.oncoming_spatial_margin;

  }

  if( candidate_ego_trajectory != nullptr )
  {
    const auto trajectory_clear_time =
      compute_ego_clear_time_from_trajectory(
        route,
        *candidate_ego_trajectory,
        ego.time,
        result.conflict_end_s,
        ego_s,
        params.route_window_min );

    if( trajectory_clear_time.has_value() )
    {
      result.ego_clear_time = trajectory_clear_time.value();
    }
  }

  if( result.ego_clear_time <= 0.0 )
  {
    const double conflict_distance =
      std::max( 0.0, result.conflict_end_s - ego_s );

    double ego_speed = std::max( 0.0, ego.vx );
    if( params.max_speed_during_avoidance > 0.0 )
    {
      ego_speed = std::min( ego_speed, params.max_speed_during_avoidance );
    }

    result.ego_clear_time =
      conflict_distance <= 0.0
        ? 0.0
        : ( ego_speed > 0.0
              ? conflict_distance / ego_speed
              : std::numeric_limits<double>::infinity() );
  }

  bool oncoming_detected = false;

  for( const auto& [id, participant] : traffic_participants.participants )
  {
    const double participant_speed =
      std::hypot( participant.state.vx, participant.state.vy );

    if( avoidance_group_contains_participant_id(
          group,
          static_cast<int>( id ) ) &&
        participant_speed <= params.max_static_object_speed )
    {
      continue;
    }

    const double participant_s =
      project_s_on_reference_line( route, participant.state, ego_s );
    if( !std::isfinite( participant_s ) )
    {
      continue;
    }

    const auto route_pose = route.get_pose_at_s( participant_s );
    const double route_yaw = route_pose.yaw;

    const double yaw_diff =
      normalize_angle( participant.state.yaw_angle - route_yaw );

    const double v_route =
      participant.state.vx * std::cos( yaw_diff ) -
      participant.state.vy * std::sin( yaw_diff );
    const bool heading_opposite =
      std::fabs( yaw_diff ) >= params.min_oncoming_heading_diff;
    const auto participant_footprint =
      project_participant_footprint_to_route( route, participant, params );
    const bool participant_in_conflict_interval =
      participant_footprint.has_value()
        ? ( participant_footprint->s_max >= result.conflict_start_s &&
            participant_footprint->s_min <= result.conflict_end_s )
        : ( participant_s >= result.conflict_start_s &&
            participant_s <= result.conflict_end_s );
    const double participant_near_s =
      participant_footprint.has_value()
        ? participant_footprint->s_min
        : participant_s;
    const double participant_far_s =
      participant_footprint.has_value()
        ? participant_footprint->s_max
        : participant_s;

    if( participant_speed <= params.max_static_object_speed ||
        v_route >= -params.min_oncoming_route_speed )
    {
      if( heading_opposite && participant_in_conflict_interval )
      {
        // A genuinely stopped participant is a geometric obstacle, not a
        // time-gap problem. If the candidate route keeps the hard corridor
        // clear, it leaves usable space and must not reject the maneuver merely
        // because its formal arrival time in the conflict interval is zero.
        // Intrusions have already been rejected by the candidate-route safety
        // check and remain conflicts here as a conservative fallback.
        if( participant_speed <= params.max_static_object_speed &&
            participant_keeps_hard_clearance_to_route_corridor(
              driven_route, participant, ego_params, params ) )
        {
          continue;
        }

        result.conflict = true;
        result.participant_id = static_cast<int>( id );
        result.oncoming_arrival_time = 0.0;

        char buf[384];
        std::snprintf(
          buf,
          sizeof( buf ),
          "participant id=%d currently occupies conflict interval as slow/stopped opposite-direction traffic, conflict=[%.2f, %.2f]",
          static_cast<int>( id ),
          result.conflict_start_s,
          result.conflict_end_s );
        result.reason = buf;

        return result;
      }

      // Slow oncoming traffic that has not reached the conflict interval yet:
      // its route-aligned speed is below min_oncoming_route_speed, so the main
      // gap check below never sees it. A vehicle that is clearly heading
      // against the route and approaching the interval must still pass the
      // gap-acceptance test with a floored closing speed.
      if( heading_opposite &&
          participant_speed > params.max_static_object_speed &&
          participant_near_s > result.conflict_end_s )
      {
        const double slow_closing_speed =
          std::max( params.min_oncoming_route_speed, std::fabs( v_route ) );
        const double slow_arrival_time =
          std::max( 0.0, participant_near_s - result.conflict_end_s ) /
          slow_closing_speed;

        if( slow_arrival_time <=
            result.ego_clear_time + params.oncoming_time_margin )
        {
          result.conflict = true;
          result.participant_id = static_cast<int>( id );
          result.oncoming_arrival_time = slow_arrival_time;

          char buf[384];
          std::snprintf(
            buf,
            sizeof( buf ),
            "participant id=%d slow oncoming (v_route=%.2f) arrival_time=%.2f <= ego_clear_time=%.2f + margin=%.2f, conflict=[%.2f, %.2f]",
            static_cast<int>( id ),
            v_route,
            slow_arrival_time,
            result.ego_clear_time,
            params.oncoming_time_margin,
            result.conflict_start_s,
            result.conflict_end_s );
          result.reason = buf;

          return result;
        }
      }

      continue;
    }

    oncoming_detected = true;

    const double oncoming_speed =
      std::max(
        params.min_oncoming_route_speed,
        std::fabs( v_route ) );

    double arrival_time = std::numeric_limits<double>::infinity();
    const char* arrival_source = "constant_velocity";

    bool arrival_resolved = false;

    if( participant_in_conflict_interval )
    {
      arrival_time = 0.0;
      arrival_source = "current_footprint";
      arrival_resolved = true;
    }
    else if( participant.trajectory.has_value() &&
        participant.trajectory->states.size() >= 2 )
    {
      const auto trajectory_arrival =
        compute_arrival_time_from_trajectory(
          route,
          participant.trajectory.value(),
          ego.time,
          result.conflict_start_s,
          result.conflict_end_s );

      if( trajectory_arrival.has_value() )
      {
        arrival_time = trajectory_arrival.value();
        arrival_source = "trajectory";
        arrival_resolved = true;
      }
    }

    if( !arrival_resolved )
    {
      if( participant_near_s > result.conflict_end_s )
      {
        // Oncoming traffic is ahead on the route and moves toward decreasing s.
        // Use its nearest footprint edge, not its center.
        const double distance_to_conflict =
          participant_near_s - result.conflict_end_s;
        arrival_time = distance_to_conflict / oncoming_speed;
      }
      else if( participant_far_s < result.conflict_start_s )
      {
        // The complete footprint has passed the interval and keeps moving away.
        continue;
      }
      else
      {
        // Projection uncertainty at an interval boundary: fail closed.
        arrival_time = 0.0;
      }
    }

    if( params.prediction_time_horizon > 0.0 &&
        arrival_time > params.prediction_time_horizon &&
        result.ego_clear_time + params.oncoming_time_margin <= params.prediction_time_horizon )
    {
      continue;
    }

    const double required_arrival_time =
      result.ego_clear_time + params.oncoming_time_margin;

    if( arrival_time <= required_arrival_time )
    {
      result.conflict = true;
      result.participant_id = static_cast<int>( id );
      result.oncoming_arrival_time = arrival_time;

      char buf[384];
      std::snprintf(
        buf,
        sizeof( buf ),
        "participant id=%d arrival_time=%.2f (source=%s) <= ego_clear_time=%.2f + margin=%.2f, conflict=[%.2f, %.2f]",
        static_cast<int>( id ),
        arrival_time,
        arrival_source,
        result.ego_clear_time,
        params.oncoming_time_margin,
        result.conflict_start_s,
        result.conflict_end_s );
      result.reason = buf;

      return result;
    }

  }

  result.conflict = false;
  result.reason = oncoming_detected
    ? "oncoming detected but no collision predicted"
    : "no oncoming conflict detected";

  return result;
}

} // namespace oa_detail
} // namespace planner
} // namespace adore
