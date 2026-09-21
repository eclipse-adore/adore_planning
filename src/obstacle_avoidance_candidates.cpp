/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

// Lateral-shift candidate generation, ordinary/opposite/intersection drivable-
// area validation, planned-trajectory validation against lanes and obstacles,
// and candidate scoring. Depends on the geometry, projection, grouping, shift,
// drivable-area and oncoming helpers in oa_detail.

#include "obstacle_avoidance_internal.hpp"

#include <algorithm>
#include <array>
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
candidate_route_conflict_is_ignorable(
  const RouteCorridorConflict& conflict,
  AvoidanceCandidateType candidate_type,
  bool uses_opposite_lane,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  // A candidate that actually occupies an opposite-direction lane is governed
  // by check_oncoming_gap. Letting the generic route-corridor check reject the
  // same oncoming participant first would bypass its time-gap acceptance rule.
  // Other object classes remain generic route-safety conflicts.
  if( uses_opposite_lane &&
      conflict.object_class == RouteCorridorObjectClass::Oncoming )
  {
    return true;
  }

  // An in-lane shift may see geometrically unrelated traffic in the neighbouring
  // oncoming lane inside the broad route-frame corridor. Ignore it only when the
  // configured side clearance to the route-centred ego footprint is preserved.
  return
    candidate_type == AvoidanceCandidateType::InLane &&
    is_oncoming_other_lane_conflict(
      conflict,
      ego_params,
      params );
}

std::optional<LateralInterval>
allowed_interval_for_candidate_at_route_point(
  const map::Route& route,
  const map::MapPoint& route_point,
  double route_s,
  double lateral_shift,
  AvoidanceCandidateType candidate_type,
  const ObstacleAvoidanceParams& params,
  std::string* failure_reason = nullptr )
{
  if( candidate_type == AvoidanceCandidateType::OppositeDirection )
  {
    const auto opposite_query =
      query_opposite_direction_lateral_interval_at_route_point(
        route,
        route_point,
        route_s,
        lateral_shift > 0.0,
        params );
    if( !opposite_query.map_usable || !opposite_query.has_opposite_lane )
    {
      if( failure_reason != nullptr )
      {
        *failure_reason =
          "no usable opposite-direction lane interval: " +
          opposite_query.reason;
      }
      return std::nullopt;
    }

    // Include every physically connected driving lane on the shift side. This
    // permits crossing an intermediate same-direction lane but rejects a
    // median/shoulder/non-driving gap larger than lane_boundary_join_slack.
    const auto interval = get_allowed_lateral_interval_at_route_point(
      route,
      route_point,
      route_s,
      lateral_shift,
      true,
      params );
    if( !interval.has_value() && failure_reason != nullptr )
    {
      *failure_reason =
        "opposite-direction lane is not connected to the current driving area";
    }
    return interval;
  }

  if( candidate_type == AvoidanceCandidateType::ConnectedIntersection )
  {
    const auto intersection_query =
      query_connected_intersection_driving_area_at_route_point(
        route,
        route_point,
        route_s,
        lateral_shift,
        params );
    if( !intersection_query.map_usable )
    {
      if( failure_reason != nullptr )
      {
        *failure_reason = intersection_query.reason;
      }
      return std::nullopt;
    }
    return intersection_query.interval;
  }

  auto validation_params = params;
  validation_params.opposite_lane_enabled = false;
  const bool allow_adjacent =
    candidate_type == AvoidanceCandidateType::AdjacentSameDirection;
  return get_allowed_lateral_interval_at_route_point(
    route,
    route_point,
    route_s,
    lateral_shift,
    allow_adjacent,
    validation_params );
}

bool
candidate_respects_drivable_area( const map::Route& route,
                                  const AvoidanceGroup& group,
                                  double lateral_shift,
                                  const dynamics::PhysicalVehicleParameters& ego_params,
                                  const ObstacleAvoidanceParams& params,
                                  bool allow_adjacent_driving_lanes )
{
  if( route.reference_line.empty() )
  {
    return false;
  }

  const double ego_half_width = 0.5 * ego_params.body_width;

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

    const double ego_min_l = center_l - ego_half_width;
    const double ego_max_l = center_l + ego_half_width;

    const auto allowed_interval = get_allowed_lateral_interval_at_route_point(
      route,
      route_point,
      route_s,
      lateral_shift,
      allow_adjacent_driving_lanes,
      params );

    if( !allowed_interval.has_value() )
    {
      return false;
    }

    if( ego_min_l < allowed_interval->min_l || ego_max_l > allowed_interval->max_l )
    {
      return false;
    }
  }

  return true;
}

ShiftCandidate
make_candidate_from_obstacle_hulls(
  ShiftDirection direction,
  const AvoidanceGroup& group,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  const double ego_half_width = 0.5 * ego_params.body_width;
  const bool shift_left = direction == ShiftDirection::Left;
  double required_shift = 0.0;

  for( const auto& obs : group.obstacles )
  {
    if( !obs.overlaps_ego_corridor )
    {
      continue;
    }

    double required_for_obs =
      shift_left
        ? obs.object_l_max + params.side_clearance + ego_half_width
        : obs.object_l_min - params.side_clearance - ego_half_width;

    if( obs.has_persistent_profile &&
        ( shift_left
            ? obs.persistent_signed_shift > 0.0
            : obs.persistent_signed_shift < 0.0 ) )
    {
      required_for_obs =
        choose_larger_magnitude_shift(
          required_for_obs,
          obs.persistent_signed_shift );
    }

    required_shift =
      shift_left
        ? std::max( required_shift, required_for_obs )
        : std::min( required_shift, required_for_obs );
  }

  return ShiftCandidate{
    required_shift,
    std::isfinite( required_shift ) &&
      ( shift_left ? required_shift > 0.0 : required_shift < 0.0 ) };
}

bool
candidate_respects_opposite_direction_area(
  const map::Route& route,
  const AvoidanceGroup& group,
  double lateral_shift,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  if( route.reference_line.empty() )
  {
    return false;
  }

  const double ego_half_width = 0.5 * ego_params.body_width;
  const bool shift_left = lateral_shift > 0.0;

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

    const double ego_min_l = center_l - ego_half_width;
    const double ego_max_l = center_l + ego_half_width;

    const auto opposite_query =
      query_opposite_direction_lateral_interval_at_route_point(
        route,
        route_point,
        route_s,
        shift_left,
        params );

    if( !opposite_query.map_usable || !opposite_query.has_opposite_lane )
    {
      return false;
    }

    const auto allowed_interval =
      get_allowed_lateral_interval_at_route_point(
        route,
        route_point,
        route_s,
        lateral_shift,
        true,
        params );

    if( !allowed_interval.has_value() ||
        ego_min_l < allowed_interval->min_l ||
        ego_max_l > allowed_interval->max_l )
    {
      return false;
    }
  }

  return true;
}

bool
candidate_respects_connected_intersection_area(
  const map::Route& route,
  const AvoidanceGroup& group,
  double lateral_shift,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  if( route.reference_line.empty() ||
      !params.intersection_drivable_area_enabled )
  {
    return false;
  }

  const double ego_half_width = 0.5 * ego_params.body_width;
  bool actually_uses_cross_road_area = false;

  auto same_direction_params = params;
  same_direction_params.opposite_lane_enabled = false;

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

    const double ego_min_l = center_l - ego_half_width;
    const double ego_max_l = center_l + ego_half_width;
    const auto base_interval = get_allowed_lateral_interval_at_route_point(
      route,
      route_point,
      route_s,
      lateral_shift,
      true,
      same_direction_params );

    if( base_interval.has_value() &&
        ego_min_l >= base_interval->min_l &&
        ego_max_l <= base_interval->max_l )
    {
      continue;
    }

    const auto intersection_query =
      query_connected_intersection_driving_area_at_route_point(
        route,
        route_point,
        route_s,
        lateral_shift,
        params );
    if( !intersection_query.map_usable ||
        !intersection_query.has_cross_road_connection ||
        ego_min_l < intersection_query.interval.min_l ||
        ego_max_l > intersection_query.interval.max_l )
    {
      return false;
    }

    actually_uses_cross_road_area = true;
  }

  return actually_uses_cross_road_area;
}

void
evaluate_shift_candidate( ShiftCandidate& candidate,
                          const map::Route& route,
                          const AvoidanceGroup& group,
                          const dynamics::PhysicalVehicleParameters& ego_params,
                          const ObstacleAvoidanceParams& params )
{
  if( !candidate.valid )
  {
    return;
  }

  if( candidate.type == AvoidanceCandidateType::OppositeDirection )
  {
    candidate.in_lane = false;

    if( !candidate_respects_opposite_direction_area(
          route,
          group,
          candidate.shift,
          ego_params,
          params ) )
    {
      candidate.valid = false;
    }

    return;
  }

  const bool fits_current_lane = candidate_respects_drivable_area(
    route,
    group,
    candidate.shift,
    ego_params,
    params,
    false );

  if( fits_current_lane )
  {
    candidate.in_lane =
      candidate.type == AvoidanceCandidateType::InLane;
    // Candidate types describe enabled maneuver classes, not merely scoring
    // preferences. An adjacent-lane candidate that never leaves the current
    // lane would otherwise duplicate the in-lane candidate and could bypass a
    // disabled in_lane_shift_enabled mode.
    if( candidate.type == AvoidanceCandidateType::AdjacentSameDirection ||
        candidate.type == AvoidanceCandidateType::ConnectedIntersection )
    {
      candidate.valid = false;
    }
    return;
  }

  candidate.in_lane = false;

  if( candidate.type == AvoidanceCandidateType::InLane )
  {
    candidate.valid = false;
    return;
  }

  auto same_direction_params = params;
  same_direction_params.opposite_lane_enabled = false;

  const bool respects_enabled_area =
    candidate.type == AvoidanceCandidateType::ConnectedIntersection
      ? candidate_respects_connected_intersection_area(
          route,
          group,
          candidate.shift,
          ego_params,
          params )
      : candidate_respects_drivable_area(
          route,
          group,
          candidate.shift,
          ego_params,
          same_direction_params,
          true );

  if( !respects_enabled_area )
  {
    candidate.valid = false;
  }
}

std::vector<ShiftCandidate>
generate_shift_candidate_variants(
  const AvoidanceGroup& group,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  std::vector<ShiftCandidate> candidates;

  const auto left_base =
    make_candidate_from_obstacle_hulls(
      ShiftDirection::Left,
      group,
      ego_params,
      params );
  const auto right_base =
    make_candidate_from_obstacle_hulls(
      ShiftDirection::Right,
      group,
      ego_params,
      params );

  // One minimal-shift candidate per side: the required shift to clear the group by
  // side_clearance. A single adaptive retry may enlarge it later by the measured
  // trajectory deficit. Left base is > 0, right base < 0, so the two can never
  // coincide; no duplicate check is needed.
  auto append_side =
    [&]( const ShiftCandidate& base, double sign )
    {
      ShiftCandidate candidate;
      candidate.shift = base.shift;
      candidate.valid =
        std::isfinite( candidate.shift ) &&
        ( sign > 0.0 ? candidate.shift > 0.0 : candidate.shift < 0.0 );

      if( candidate.valid )
      {
        candidates.push_back( candidate );
      }
    };

  append_side( left_base, 1.0 );
  append_side( right_base, -1.0 );

  return candidates;
}


std::vector<ShiftCandidate>
generate_opposite_lane_candidate_variants(
  const map::Route& route,
  const AvoidanceGroup& group,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  std::vector<ShiftCandidate> candidates;

  const double ego_half_width = 0.5 * ego_params.body_width;
  const double required_lane_margin = 0.0;

  const std::array<double, 3> sample_s_values = {{
    group.envelope.object_s_min,
    0.5 * ( group.envelope.object_s_min + group.envelope.object_s_max ),
    group.envelope.object_s_max
  }};

  auto append_unique_candidate =
    [&]( double shift )
    {
      if( !std::isfinite( shift ) || std::fabs( shift ) < 1e-6 )
      {
        return;
      }

      const bool duplicate =
        std::any_of(
          candidates.begin(),
          candidates.end(),
          [&]( const ShiftCandidate& existing )
          {
            return std::fabs( existing.shift - shift ) < 1e-6;
          } );

      if( duplicate )
      {
        return;
      }

      ShiftCandidate candidate;
      candidate.shift = shift;
      candidate.valid = true;
      candidate.type = AvoidanceCandidateType::OppositeDirection;
      candidate.in_lane = false;
      candidates.push_back( candidate );
    };

  for( const bool shift_left : { true, false } )
  {
    LateralInterval opposite_intersection;
    bool initialized = false;
    bool all_samples_usable = true;

    for( const double sample_s : sample_s_values )
    {
      const auto route_point = find_reference_point_near_s( route, sample_s );
      if( !route_point.has_value() )
      {
        all_samples_usable = false;
        break;
      }

      const auto opposite_query =
        query_opposite_direction_lateral_interval_at_route_point(
          route,
          route_point->second,
          route_point->first,
          shift_left,
          params );

      if( !opposite_query.map_usable || !opposite_query.has_opposite_lane )
      {
        all_samples_usable = false;
        break;
      }

      if( !initialized )
      {
        opposite_intersection = opposite_query.interval;
        initialized = true;
      }
      else
      {
        opposite_intersection.min_l =
          std::max( opposite_intersection.min_l, opposite_query.interval.min_l );
        opposite_intersection.max_l =
          std::min( opposite_intersection.max_l, opposite_query.interval.max_l );
      }
    }

    if( !all_samples_usable || !initialized )
    {
      continue;
    }

    const double center_l_min =
      opposite_intersection.min_l + ego_half_width + required_lane_margin;
    const double center_l_max =
      opposite_intersection.max_l - ego_half_width - required_lane_margin;

    if( center_l_min > center_l_max )
    {
      continue;
    }

    double obstacle_clear_center_l = 0.0;
    if( shift_left )
    {
      const auto hull_shift =
        make_candidate_from_obstacle_hulls(
          ShiftDirection::Left,
          group,
          ego_params,
          params );
      obstacle_clear_center_l = hull_shift.shift;
    }
    else
    {
      const auto hull_shift =
        make_candidate_from_obstacle_hulls(
          ShiftDirection::Right,
          group,
          ego_params,
          params );
      obstacle_clear_center_l = hull_shift.shift;
    }

    const double opposite_center_l =
      0.5 * ( opposite_intersection.min_l + opposite_intersection.max_l );

    std::vector<double> requested_centers;
    requested_centers.push_back( obstacle_clear_center_l );
    requested_centers.push_back(
      std::clamp( opposite_center_l, center_l_min, center_l_max ) );

    for( double requested_center_l : requested_centers )
    {
      const double shift = std::clamp( requested_center_l, center_l_min, center_l_max );

      if( shift_left && shift <= 0.0 )
      {
        continue;
      }
      if( !shift_left && shift >= 0.0 )
      {
        continue;
      }

      append_unique_candidate( shift );
    }

  }

  return candidates;
}

TrajectoryValidationResult
validate_planned_shift_trajectory(
  const map::Route& route,
  const dynamics::Trajectory& trajectory,
  const AvoidanceGroup& group,
  double lateral_shift,
  bool in_lane,
  AvoidanceCandidateType candidate_type,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params,
  double target_side_clearance,
  double initial_s_hint )
{
  TrajectoryValidationResult result;
  result.reason = "trajectory valid";

  const bool type_is_in_lane =
    candidate_type == AvoidanceCandidateType::InLane;
  if( in_lane != type_is_in_lane )
  {
    result.valid = false;
    result.reason =
      "trajectory validation failed: candidate type/in-lane state mismatch";
    return result;
  }

  if( trajectory.states.empty() )
  {
    result.valid = false;
    result.reason = "trajectory validation failed: empty trajectory";
    return result;
  }

  const double ego_half_width = 0.5 * ego_params.body_width;
  const double ego_front_offset =
    ego_params.wheelbase + ego_params.front_axle_to_front_border;
  const double ego_rear_offset =
    ego_params.rear_border_to_rear_axle;
  // Keep collision validation tied to the real rear-axle footprint even though
  // the configured route-shift plateau uses a symmetric half-body-length
  // policy. This prevents the policy from hiding an actual front/rear overlap.
  const double group_timing_s_min =
    group.envelope.object_s_min - ego_front_offset -
    std::max( 0.0, params.front_clearance );
  const double group_timing_s_max =
    group.envelope.object_s_max + ego_rear_offset +
    std::max( 0.0, params.rear_clearance );
  const double target_clearance =
    std::max( 0.0, target_side_clearance );
  const double hard_clearance =
    std::max( 0.0, params.ego_corridor_safety_margin );
  const std::array<double, 4> body_long_offset = {
    ego_front_offset, ego_front_offset, -ego_rear_offset, -ego_rear_offset };
  const std::array<double, 4> body_lat_offset = {
    ego_half_width, -ego_half_width, -ego_half_width, ego_half_width };
  std::vector<std::optional<double>> committed_initial_clearance(
    group.obstacles.size() );
  double previous_s = initial_s_hint;

  // Diagnostic values are captured during the normal first-state pass so the
  // first state is not projected twice.
  double traj_start_s = std::numeric_limits<double>::quiet_NaN();
  double traj_start_l = std::numeric_limits<double>::quiet_NaN();
  double traj_start_route_off = std::numeric_limits<double>::quiet_NaN();

  for( std::size_t state_index = 0;
       state_index < trajectory.states.size();
       ++state_index )
  {
    const auto& state = trajectory.states[state_index];
    double state_s =
      adore::map::get_s_on_reference_line_segments(
        route,
        state,
        std::isfinite( previous_s ) ? previous_s : group.envelope.center_s,
        std::max( 0.0, params.route_window_min ) );

    if( !std::isfinite( state_s ) )
    {
      state_s = project_s_on_reference_line( route, state, previous_s );
    }

    if( !std::isfinite( state_s ) )
    {
      result.valid = false;
      result.reason = "trajectory validation failed: state does not project to route";
      return result;
    }

    previous_s = state_s;

    // Only validate the region where the avoidance route can matter, plus a
    // small buffer. This avoids rejecting future optimizer samples after the
    // maneuver has already returned to the original lane.
    const double planned_offset =
      avoidance_shift_offset_at_s(
        state_s,
        group,
        lateral_shift,
        ego_params,
        params );
    const bool near_obstacle =
      state_s >= group_timing_s_min &&
      state_s <= group_timing_s_max;

    if( std::fabs( planned_offset ) <= 1e-6 && !near_obstacle )
    {
      continue;
    }

    const auto route_point = find_reference_point_near_s( route, state_s );
    if( !route_point.has_value() )
    {
      result.valid = false;
      result.reason = "trajectory validation failed: no route point near state";
      return result;
    }

    const auto frame = make_route_frame( route, state_s );
    const math::Point2d state_xy{ state.x, state.y };
    const double state_l = signed_lateral_offset( frame, state_xy );
    if( state_index == 0 )
    {
      traj_start_s = state_s;
      traj_start_l = state_l;
      traj_start_route_off = planned_offset;
    }

    std::string allowed_interval_failure;
    const auto allowed_interval =
      allowed_interval_for_candidate_at_route_point(
        route,
        route_point->second,
        route_point->first,
        lateral_shift,
        candidate_type,
        params,
        &allowed_interval_failure );

    if( !allowed_interval.has_value() )
    {
      result.valid = false;
      result.reason =
        "trajectory validation failed: no usable lane interval";
      if( !allowed_interval_failure.empty() )
      {
        result.reason += ": " + allowed_interval_failure;
      }
      return result;
    }

    const double lane_ego_min_l = state_l - ego_half_width;
    const double lane_ego_max_l = state_l + ego_half_width;

    const double lane_margin =
      std::min( lane_ego_min_l - allowed_interval->min_l,
                allowed_interval->max_l - lane_ego_max_l );
    if( lane_margin < 0.0 )
    {
      char buf[256];
      std::snprintf(
        buf,
        sizeof( buf ),
        "trajectory validation failed: ego footprint leaves drivable area at s=%.2f margin=%.2f",
        state_s,
        lane_margin );
      result.valid = false;
      result.reason = buf;
      return result;
    }

    // Footprint-accurate ego geometry. The vehicle reference (state.x, state.y)
    // is the rear axle; project the four actual body corners onto the route so
    // the heading is respected. Extruding a single axis-aligned box at the
    // rear-axle lateral over the whole body length ignores that, while turning
    // into the shift, the body-fixed corners project to different lateral
    // offsets: the leading corner has already swung clear (avoiding a false
    // rejection), while the trailing corner may swing out (catching a real tail
    // overhang the centered box missed). Obstacles keep their route-frame
    // bounding box (already a conservative envelope), so only the ego side is
    // refined here.
    const double cos_yaw = std::cos( state.yaw_angle );
    const double sin_yaw = std::sin( state.yaw_angle );

    std::array<double, 4> corner_s{};
    std::array<double, 4> corner_l{};
    double ego_s_lo = std::numeric_limits<double>::infinity();
    double ego_s_hi = -std::numeric_limits<double>::infinity();
    bool corner_projection_ok = true;

    for( std::size_t c = 0; c < 4; ++c )
    {
      const math::Point2d corner_xy{
        state.x + cos_yaw * body_long_offset[c] - sin_yaw * body_lat_offset[c],
        state.y + sin_yaw * body_long_offset[c] + cos_yaw * body_lat_offset[c] };

      double corner_s_value =
        adore::map::get_s_on_reference_line_segments(
          route,
          corner_xy,
          state_s,
          std::max( 0.0, params.route_window_min ) );
      if( !std::isfinite( corner_s_value ) )
      {
        corner_s_value = project_s_on_reference_line( route, corner_xy, state_s );
      }
      if( !std::isfinite( corner_s_value ) )
      {
        corner_projection_ok = false;
        break;
      }

      const auto corner_frame = make_route_frame( route, corner_s_value );
      corner_s[c] = corner_s_value;
      corner_l[c] = signed_lateral_offset( corner_frame, corner_xy );
      ego_s_lo = std::min( ego_s_lo, corner_s[c] );
      ego_s_hi = std::max( ego_s_hi, corner_s[c] );
    }

    if( !corner_projection_ok )
    {
      result.valid = false;
      result.reason =
        "trajectory validation failed: ego footprint corner does not project to route";
      return result;
    }

    // Validate the rotated full-body outline against the lane area as well as
    // against obstacles. The former rear-axle-centred width check above is a
    // useful early rejection, but by itself it misses front/rear corner swing
    // during the lateral transition.
    for( std::size_t c = 0; c < 4; ++c )
    {
      const auto corner_route_point =
        find_reference_point_near_s( route, corner_s[c] );
      if( !corner_route_point.has_value() )
      {
        result.valid = false;
        result.reason =
          "trajectory validation failed: no route point near ego footprint corner";
        return result;
      }

      std::string corner_interval_failure;
      const auto corner_allowed_interval =
        allowed_interval_for_candidate_at_route_point(
          route,
          corner_route_point->second,
          corner_route_point->first,
          lateral_shift,
          candidate_type,
          params,
          &corner_interval_failure );
      if( !corner_allowed_interval.has_value() ||
          corner_l[c] < corner_allowed_interval->min_l ||
          corner_l[c] > corner_allowed_interval->max_l )
      {
        char buf[320];
        std::snprintf(
          buf,
          sizeof( buf ),
          "trajectory validation failed: rotated ego footprint leaves drivable area at corner=%zu s=%.2f l=%.2f",
          c,
          corner_s[c],
          corner_l[c] );
        result.valid = false;
        result.reason = buf;
        if( !corner_interval_failure.empty() )
        {
          result.reason += ": " + corner_interval_failure;
        }
        return result;
      }
    }

    // Lateral extent of the ego footprint restricted to a longitudinal band,
    // from the projected quad (interior vertices + edge crossings at the band
    // limits). Exact for a convex quad; the projection is near-affine over a
    // vehicle length so convexity holds locally.
    const auto ego_lateral_span_in_band =
      [&]( double band_lo, double band_hi, double& span_min_l, double& span_max_l )
    {
      double lo_l = std::numeric_limits<double>::infinity();
      double hi_l = -std::numeric_limits<double>::infinity();

      for( std::size_t c = 0; c < 4; ++c )
      {
        if( corner_s[c] >= band_lo && corner_s[c] <= band_hi )
        {
          lo_l = std::min( lo_l, corner_l[c] );
          hi_l = std::max( hi_l, corner_l[c] );
        }
      }

      for( std::size_t c = 0; c < 4; ++c )
      {
        const std::size_t n = ( c + 1 ) % 4;
        const double sa = corner_s[c];
        const double sb = corner_s[n];
        if( sa == sb )
        {
          continue;
        }
        for( const double bs : { band_lo, band_hi } )
        {
          const double t = ( bs - sa ) / ( sb - sa );
          if( t >= 0.0 && t <= 1.0 )
          {
            const double l = corner_l[c] + t * ( corner_l[n] - corner_l[c] );
            lo_l = std::min( lo_l, l );
            hi_l = std::max( hi_l, l );
          }
        }
      }

      span_min_l = lo_l;
      span_max_l = hi_l;
      return lo_l <= hi_l;
    };

    for( std::size_t obstacle_index = 0;
         obstacle_index < group.obstacles.size();
         ++obstacle_index )
    {
      const auto& obstacle = group.obstacles[obstacle_index];

      double ego_min_l;
      double ego_max_l;

      if( corner_projection_ok )
      {
        const double band_lo = std::max( ego_s_lo, obstacle.object_s_min );
        const double band_hi = std::min( ego_s_hi, obstacle.object_s_max );
        if( band_lo > band_hi )
        {
          continue;  // no longitudinal overlap
        }
        if( !ego_lateral_span_in_band( band_lo, band_hi, ego_min_l, ego_max_l ) )
        {
          continue;
        }
      }
      else
      {
        // Projection glitch: fall back to the conservative rear-axle box so a
        // clearance check is never silently skipped.
        const double ego_s_min = state_s - ego_rear_offset;
        const double ego_s_max = state_s + ego_front_offset;
        if( ego_s_max < obstacle.object_s_min ||
            ego_s_min > obstacle.object_s_max )
        {
          continue;
        }
        ego_min_l = state_l - ego_half_width;
        ego_max_l = state_l + ego_half_width;
      }

      const double left_clearance = ego_min_l - obstacle.object_l_max;
      const double right_clearance = obstacle.object_l_min - ego_max_l;
      const double actual_clearance =
        std::max( left_clearance, right_clearance );
      const double target_lateral_margin =
        actual_clearance - target_clearance;

      result.min_obstacle_lateral_margin =
        std::min(
          result.min_obstacle_lateral_margin,
          target_lateral_margin );

      // A replan can start while ego is already alongside an accepted obstacle
      // and transiently closer than the normal hard margin. That existing state
      // cannot be undone. Record its first-state clearance and require every
      // future overlapping state to be no worse. All committed obstacles that
      // are not alongside at the first state retain the full hard-clearance
      // requirement, so a future committed obstacle is never skipped.
      if( state_index == 0 && obstacle.committed_hold )
      {
        committed_initial_clearance[obstacle_index] =
          actual_clearance;
      }

      double required_clearance = hard_clearance;
      if( committed_initial_clearance[obstacle_index].has_value() )
      {
        required_clearance =
          std::min(
            required_clearance,
            committed_initial_clearance[obstacle_index].value() );
      }

      if( actual_clearance + 1e-9 < required_clearance )
      {
        char buf[448];
        std::snprintf(
          buf,
          sizeof( buf ),
          "trajectory validation failed: obstacle enters hard ego corridor at s=%.2f actual_clearance=%.2f required_clearance=%.2f hard_clearance=%.2f target_clearance=%.2f (planned_offset=%.2f traj_state_l=%.2f ego_l=[%.2f,%.2f] obstacle_l=[%.2f,%.2f]) ego_start=[s=%.1f l=%.2f route_off=%.2f]",
          state_s,
          actual_clearance,
          required_clearance,
          hard_clearance,
          target_clearance,
          planned_offset,
          state_l,
          ego_min_l,
          ego_max_l,
          obstacle.object_l_min,
          obstacle.object_l_max,
          traj_start_s,
          traj_start_l,
          traj_start_route_off );
        result.valid = false;
        result.obstacle_clearance_violation = true;
        result.reason = buf;
        return result;
      }
    }
  }

  if( !std::isfinite( result.min_obstacle_lateral_margin ) )
  {
    result.min_obstacle_lateral_margin = 0.0;
  }

  return result;
}

} // namespace oa_detail
} // namespace planner
} // namespace adore
