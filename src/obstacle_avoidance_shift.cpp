/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

// Lateral-shift profile math: per-obstacle and per-group shift alpha/offset
// curves, hard/soft hull bridges between clustered obstacles, and construction
// of the laterally modified route (plus its avoidance speed profile). Depends on
// the geometry primitives and the public RouteSpeedPolicy.

#include "obstacle_avoidance_internal.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace adore
{
namespace planner
{
namespace oa_detail
{

double
avoidance_shift_alpha_at_s( double s,
                            const ObstacleEnvelope& obstacle,
                            const dynamics::PhysicalVehicleParameters& ego_params,
                            const ObstacleAvoidanceParams& params )
{
  // Build the full-shift plateau symmetrically around the obstacle:
  // [object_s_min - 0.5 * ego_length, object_s_max + 0.5 * ego_length].
  // front_clearance / rear_clearance add the comfort ramps outside this plateau.
  const double ego_half_length =
    symmetric_shift_ego_half_length( ego_params );

  const double hold_start_s =
    obstacle.has_persistent_profile
      ? obstacle.persistent_full_shift_start_s
      : obstacle.object_s_min - ego_half_length;
  const double hold_end_s =
    obstacle.has_persistent_profile
      ? obstacle.persistent_full_shift_end_s
      : obstacle.object_s_max + ego_half_length;

  const double shift_start_s =
    obstacle.has_persistent_profile
      ? obstacle.persistent_ramp_start_s
      : std::max(
          0.0,
          hold_start_s - std::max( 0.0, params.front_clearance ) );
  const double shift_end_s =
    obstacle.has_persistent_profile
      ? obstacle.persistent_ramp_end_s
      : hold_end_s + std::max( 0.0, params.rear_clearance );

  if( s < shift_start_s || s > shift_end_s )
  {
    return 0.0;
  }

  if( s < hold_start_s )
  {
    return smoothstep01(
      ( s - shift_start_s ) /
      std::max( 1e-6, hold_start_s - shift_start_s ) );
  }

  if( s <= hold_end_s )
  {
    return 1.0;
  }

  return 1.0 - smoothstep01(
    ( s - hold_end_s ) /
    std::max( 1e-6, shift_end_s - hold_end_s ) );
}

double
required_signed_shift_for_obstacle(
  const ObstacleEnvelope& obstacle,
  double nominal_lateral_shift,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  if( !std::isfinite( nominal_lateral_shift ) ||
      std::fabs( nominal_lateral_shift ) < 1e-6 )
  {
    return 0.0;
  }

  const double ego_half_width = 0.5 * ego_params.body_width;
  const double side_clearance =
    std::max( 0.0, params.side_clearance );

  if( nominal_lateral_shift > 0.0 )
  {
    const double required =
      obstacle.object_l_max + side_clearance + ego_half_width;
    const double geometric_shift =
      std::clamp( required, 0.0, nominal_lateral_shift );
    if( obstacle.has_persistent_profile &&
        obstacle.persistent_signed_shift > 0.0 )
    {
      return choose_larger_magnitude_shift(
        geometric_shift,
        obstacle.persistent_signed_shift );
    }
    return geometric_shift;
  }

  const double required =
    obstacle.object_l_min - side_clearance - ego_half_width;
  const double geometric_shift =
    std::clamp( required, nominal_lateral_shift, 0.0 );
  if( obstacle.has_persistent_profile &&
      obstacle.persistent_signed_shift < 0.0 )
  {
    return choose_larger_magnitude_shift(
      geometric_shift,
      obstacle.persistent_signed_shift );
  }
  return geometric_shift;
}

double
choose_larger_magnitude_shift( double a, double b )
{
  return std::fabs( b ) > std::fabs( a ) ? b : a;
}

double
avoidance_shift_offset_at_s(
  double s,
  const AvoidanceGroup& group,
  double nominal_lateral_shift,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  // Per-obstacle shift: each obstacle contributes its OWN required shift over its
  // OWN clearance ramp (ramp up over front_clearance, hold alongside, ramp down
  // over rear_clearance), composed by taking the max-magnitude at each s. So ego
  // only ever holds as much shift as the nearest obstacle needs and returns
  // toward its lane as soon as a smaller-shift obstacle takes over - whether the
  // leading or the trailing obstacle sits further at the edge.
  double offset = 0.0;

  for( const auto& obstacle : group.obstacles )
  {
    const double alpha =
      avoidance_shift_alpha_at_s( s, obstacle, ego_params, params );
    if( alpha <= 0.0 )
    {
      continue;
    }

    const double required_shift =
      required_signed_shift_for_obstacle(
        obstacle,
        nominal_lateral_shift,
        ego_params,
        params );
    offset = choose_larger_magnitude_shift( offset, required_shift * alpha );
  }

  // Smooth the transition between consecutive obstacles. The max-of-ramps above
  // dips and kinks (a sharp V) where one obstacle's rear ramp crosses the next
  // obstacle's front ramp. Where the two are close enough that their ramps
  // overlap, bridge the physical gap [prev rear, next front] with a smoothstep
  // from the previous to the next obstacle's own shift. Both plateaus are flat
  // (slope 0), so the bridge hands off C1-smoothly: for equal shifts it is flat
  // (no dip), for unequal a smooth ramp toward the smaller shift instead of a
  // corner. It only ever raises the offset inside the gap, so it never reduces
  // clearance to either obstacle.
  const double ramp_reach =
    std::max( 0.0, params.front_clearance ) +
    std::max( 0.0, params.rear_clearance );

  for( std::size_t i = 1; i < group.obstacles.size(); ++i )
  {
    const auto& prev = group.obstacles[i - 1];
    const auto& next = group.obstacles[i];

    const double gap_start = prev.object_s_max;
    const double gap_end = next.object_s_min;

    // Only bridge a real gap whose per-object ramps actually overlap; for
    // far-apart obstacles the profile genuinely returns toward the lane between
    // them (smoothstep already reaches zero, so no kink there).
    if( gap_end <= gap_start || gap_end - gap_start >= ramp_reach )
    {
      continue;
    }
    if( s < gap_start || s > gap_end )
    {
      continue;
    }

    const double prev_shift =
      required_signed_shift_for_obstacle(
        prev, nominal_lateral_shift, ego_params, params );
    const double next_shift =
      required_signed_shift_for_obstacle(
        next, nominal_lateral_shift, ego_params, params );

    const double t = ( s - gap_start ) / ( gap_end - gap_start );
    const double bridge_shift =
      prev_shift + ( next_shift - prev_shift ) * smoothstep01( t );

    offset = choose_larger_magnitude_shift( offset, bridge_shift );
  }

  return offset;
}

map::Route
build_modified_avoidance_route( const map::Route& route,
                                const AvoidanceGroup& group,
                                double lateral_shift,
                                double ego_s,
                                const dynamics::PhysicalVehicleParameters& ego_params,
                                const ObstacleAvoidanceParams& params )
{
  map::Route modified_route = route;

  for( auto& [s, point] : modified_route.reference_line )
  {
    const double offset =
      avoidance_shift_offset_at_s(
        s,
        group,
        lateral_shift,
        ego_params,
        params );

    if( std::fabs( offset ) <= 1e-6 )
    {
      continue;
    }

    // Important: use the original route as reference frame.
    const auto frame = make_route_frame( route, s );
    const auto shifted_point_xy = shifted_point( frame, offset );

    point.x = shifted_point_xy.x;
    point.y = shifted_point_xy.y;
  }

  // Clearance-based shift window (see avoidance_shift_alpha_at_s): matches the
  // lateral shift window (plateau extended symmetrically by half the ego length,
  // plus the front_clearance / rear_clearance ramps) so the speed profile ramps
  // over the same span.
  const double ego_half_length =
    symmetric_shift_ego_half_length( ego_params );
  std::vector<AvoidanceSpeedSegment> speed_segments;
  speed_segments.reserve( group.obstacles.size() );
  for( const auto& obstacle : group.obstacles )
  {
    const double full_start_s =
      obstacle.has_persistent_profile
        ? obstacle.persistent_full_shift_start_s
        : obstacle.object_s_min - ego_half_length;
    const double full_end_s =
      obstacle.has_persistent_profile
        ? obstacle.persistent_full_shift_end_s
        : obstacle.object_s_max + ego_half_length;
    const double obstacle_shift_start_s =
      obstacle.has_persistent_profile
        ? obstacle.persistent_ramp_start_s
        : std::max(
            0.0,
            full_start_s - std::max( 0.0, params.front_clearance ) );
    const double obstacle_shift_end_s =
      obstacle.has_persistent_profile
        ? obstacle.persistent_ramp_end_s
        : full_end_s + std::max( 0.0, params.rear_clearance );
    speed_segments.push_back(
      AvoidanceSpeedSegment{
        obstacle_shift_start_s,
        obstacle_shift_end_s } );
  }

  modified_route =
    RouteSpeedPolicy::apply_segmented_avoidance_speed_profile(
      modified_route,
      ego_s,
      speed_segments,
      ego_params,
      params );

  return modified_route;
}

} // namespace oa_detail
} // namespace planner
} // namespace adore
