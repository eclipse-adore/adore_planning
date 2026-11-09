/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

// Drivable-area / lane-interval queries: whether a lane has usable borders and
// covers a given s, and the allowed lateral interval at a route point for the
// current lane, adjacent same-direction lanes, connected intersection driving
// surfaces and opposite-direction lanes. Depends on the geometry primitives in
// oa_detail.

#include "obstacle_avoidance_internal.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <optional>
#include <unordered_set>
#include <vector>

namespace adore
{
namespace planner
{
namespace oa_detail
{

namespace
{

constexpr double geometry_epsilon = 1e-7;

bool
intervals_are_connected( const LateralInterval& a,
                         const LateralInterval& b,
                         double slack )
{
  const double nonnegative_slack = std::max( 0.0, slack );
  return a.min_l <= b.max_l + nonnegative_slack &&
         b.min_l <= a.max_l + nonnegative_slack;
}

void
expand_interval( LateralInterval& target, const LateralInterval& addition )
{
  target.min_l = std::min( target.min_l, addition.min_l );
  target.max_l = std::max( target.max_l, addition.max_l );
}

const std::vector<map::MapPoint>&
border_geometry_points( const map::Border& border )
{
  return border.interpolated_points.empty()
    ? border.points
    : border.interpolated_points;
}

std::vector<math::Point2d>
make_lane_polygon( const map::Lane& lane )
{
  const auto& inner = border_geometry_points( lane.borders.inner );
  const auto& outer = border_geometry_points( lane.borders.outer );

  std::vector<math::Point2d> polygon;
  polygon.reserve( inner.size() + outer.size() );

  for( const auto& point : inner )
  {
    if( std::isfinite( point.x ) && std::isfinite( point.y ) )
    {
      polygon.push_back( { point.x, point.y } );
    }
  }
  for( auto it = outer.rbegin(); it != outer.rend(); ++it )
  {
    if( std::isfinite( it->x ) && std::isfinite( it->y ) )
    {
      polygon.push_back( { it->x, it->y } );
    }
  }

  return polygon;
}

bool
point_is_on_segment( const math::Point2d& point,
                     const math::Point2d& a,
                     const math::Point2d& b )
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double px = point.x - a.x;
  const double py = point.y - a.y;
  const double segment_length = std::hypot( dx, dy );

  if( segment_length <= geometry_epsilon )
  {
    return std::hypot( px, py ) <= geometry_epsilon;
  }

  const double perpendicular_distance =
    std::fabs( px * dy - py * dx ) / segment_length;
  if( perpendicular_distance > geometry_epsilon )
  {
    return false;
  }

  const double projection = px * dx + py * dy;
  const double squared_length = dx * dx + dy * dy;
  return projection >= -geometry_epsilon &&
         projection <= squared_length + geometry_epsilon;
}

bool
polygon_contains_or_touches( const std::vector<math::Point2d>& polygon,
                             const math::Point2d& point )
{
  if( polygon.size() < 3 )
  {
    return false;
  }

  bool inside = false;
  for( std::size_t i = 0, j = polygon.size() - 1;
       i < polygon.size();
       j = i++ )
  {
    const auto& a = polygon[j];
    const auto& b = polygon[i];
    if( point_is_on_segment( point, a, b ) )
    {
      return true;
    }

    const bool crosses_y = ( a.y > point.y ) != ( b.y > point.y );
    if( crosses_y )
    {
      const double crossing_x =
        a.x + ( b.x - a.x ) * ( point.y - a.y ) / ( b.y - a.y );
      if( point.x < crossing_x )
      {
        inside = !inside;
      }
    }
  }

  return inside;
}

std::vector<LateralInterval>
intersect_lane_polygon_with_route_normal(
  const map::Lane& lane,
  const RouteFrame& frame,
  double search_radius )
{
  std::vector<LateralInterval> intervals;
  const auto polygon = make_lane_polygon( lane );
  if( polygon.size() < 3 || !std::isfinite( search_radius ) ||
      search_radius <= 0.0 )
  {
    return intervals;
  }

  const double tangent_x = std::cos( frame.yaw );
  const double tangent_y = std::sin( frame.yaw );
  const double normal_x = -tangent_y;
  const double normal_y = tangent_x;

  // Split the finite segment of the route normal at every polygon-boundary
  // intersection. A midpoint test then yields every portion covered by the
  // lane polygon, including angled/crossing lanes whose own lane-s coordinate
  // is unrelated to the current route lane.
  std::vector<double> cuts{ -search_radius, search_radius };
  cuts.reserve( polygon.size() + 2 );

  for( std::size_t i = 0; i < polygon.size(); ++i )
  {
    const auto& a = polygon[i];
    const auto& b = polygon[( i + 1 ) % polygon.size()];
    const double segment_x = b.x - a.x;
    const double segment_y = b.y - a.y;
    const double a_tangent =
      ( a.x - frame.x ) * tangent_x +
      ( a.y - frame.y ) * tangent_y;
    const double segment_tangent =
      segment_x * tangent_x + segment_y * tangent_y;

    if( std::fabs( segment_tangent ) <= geometry_epsilon )
    {
      // An edge lying on the route normal contributes both end points. A
      // parallel edge away from the normal contributes no crossing.
      if( std::fabs( a_tangent ) <= geometry_epsilon )
      {
        for( const auto& endpoint : { a, b } )
        {
          const double l =
            ( endpoint.x - frame.x ) * normal_x +
            ( endpoint.y - frame.y ) * normal_y;
          if( l >= -search_radius - geometry_epsilon &&
              l <= search_radius + geometry_epsilon )
          {
            cuts.push_back( std::clamp( l, -search_radius, search_radius ) );
          }
        }
      }
      continue;
    }

    const double u = -a_tangent / segment_tangent;
    if( u < -geometry_epsilon || u > 1.0 + geometry_epsilon )
    {
      continue;
    }

    const double intersection_x = a.x + std::clamp( u, 0.0, 1.0 ) * segment_x;
    const double intersection_y = a.y + std::clamp( u, 0.0, 1.0 ) * segment_y;
    const double l =
      ( intersection_x - frame.x ) * normal_x +
      ( intersection_y - frame.y ) * normal_y;
    if( l >= -search_radius - geometry_epsilon &&
        l <= search_radius + geometry_epsilon )
    {
      cuts.push_back( std::clamp( l, -search_radius, search_radius ) );
    }
  }

  std::sort( cuts.begin(), cuts.end() );
  cuts.erase(
    std::unique(
      cuts.begin(),
      cuts.end(),
      []( double a, double b )
      {
        return std::fabs( a - b ) <= geometry_epsilon;
      } ),
    cuts.end() );

  for( std::size_t i = 1; i < cuts.size(); ++i )
  {
    const double min_l = cuts[i - 1];
    const double max_l = cuts[i];
    if( max_l - min_l <= geometry_epsilon )
    {
      continue;
    }

    const double midpoint_l = 0.5 * ( min_l + max_l );
    const math::Point2d midpoint{
      frame.x + normal_x * midpoint_l,
      frame.y + normal_y * midpoint_l };
    if( polygon_contains_or_touches( polygon, midpoint ) )
    {
      intervals.push_back( { min_l, max_l } );
    }
  }

  return intervals;
}

} // namespace

bool
lane_has_usable_borders( const map::Lane& lane )
{
  const bool inner_ok =
    !lane.borders.inner.points.empty() ||
    !lane.borders.inner.interpolated_points.empty();

  const bool outer_ok =
    !lane.borders.outer.points.empty() ||
    !lane.borders.outer.interpolated_points.empty();

  return inner_ok && outer_ok;
}

bool
border_contains_s( const map::Border& border, double s, double slack )
{
  if( !border.interpolated_points.empty() )
  {
    const double min_s = std::min( border.interpolated_points.front().s,
                                   border.interpolated_points.back().s );
    const double max_s = std::max( border.interpolated_points.front().s,
                                   border.interpolated_points.back().s );
    return s >= min_s - slack && s <= max_s + slack;
  }

  if( !border.points.empty() )
  {
    const double min_s = std::min( border.points.front().s, border.points.back().s );
    const double max_s = std::max( border.points.front().s, border.points.back().s );
    return s >= min_s - slack && s <= max_s + slack;
  }

  if( border.length > 0.0 )
  {
    return s >= -slack && s <= border.length + slack;
  }

  return false;
}

bool
lane_contains_s( const map::Lane& lane, double s, double slack )
{
  return border_contains_s( lane.borders.inner, s, slack ) &&
         border_contains_s( lane.borders.outer, s, slack );
}

bool
add_lane_lateral_interval( const RouteFrame& frame,
                           const map::Lane& lane,
                           double lane_s,
                           double lane_s_overlap_slack,
                           LateralInterval& interval )
{
  if( !lane_has_usable_borders( lane ) )
  {
    return false;
  }

  if( !lane_contains_s( lane, lane_s, lane_s_overlap_slack ) )
  {
    return false;
  }

  try
  {
    const auto inner = lane.borders.inner.get_interpolated_point( lane_s );
    const auto outer = lane.borders.outer.get_interpolated_point( lane_s );

    const math::Point2d inner_xy{ inner.x, inner.y };
    const math::Point2d outer_xy{ outer.x, outer.y };

    const double inner_l = signed_lateral_offset( frame, inner_xy );
    const double outer_l = signed_lateral_offset( frame, outer_xy );

    const double lane_min_l = std::min( inner_l, outer_l );
    const double lane_max_l = std::max( inner_l, outer_l );

    if( !std::isfinite( lane_min_l ) || !std::isfinite( lane_max_l ) ||
        lane_max_l <= lane_min_l )
    {
      return false;
    }

    interval.min_l = std::min( interval.min_l, lane_min_l );
    interval.max_l = std::max( interval.max_l, lane_max_l );
  }
  catch( const std::exception& )
  {
    return false;
  }

  return true;
}

std::optional<LateralInterval>
get_allowed_lateral_interval_at_route_point(
  const map::Route& route,
  const map::MapPoint& route_point,
  double route_s,
  double lateral_shift,
  bool allow_adjacent_driving_lanes,
  const ObstacleAvoidanceParams& params )
{
  if( !route.map )
  {
    return std::nullopt;
  }

  const auto current_lane_it = route.map->lanes.find( route_point.parent_id );
  if( current_lane_it == route.map->lanes.end() || !current_lane_it->second )
  {
    return std::nullopt;
  }

  const auto& current_lane = *current_lane_it->second;
  const auto frame = make_route_frame( route, route_s );

  std::vector<LateralInterval> intervals;
  LateralInterval current_interval;
  if( !add_lane_lateral_interval(
        frame,
        current_lane,
        route_point.s,
        params.lane_s_overlap_slack,
        current_interval ) )
  {
    return std::nullopt;
  }

  intervals.push_back( current_interval );

  if( allow_adjacent_driving_lanes )
  {
    const auto road_it = route.map->roads.find( current_lane.road_id );
    if( road_it != route.map->roads.end() )
    {
      const bool shift_left = lateral_shift >= 0.0;

      for( const auto& lane_ptr : road_it->second.lanes )
      {
        if( !lane_ptr || lane_ptr->id == current_lane.id )
        {
          continue;
        }

        // The obstacle-avoidance drivable area is limited to actual driving
        // lanes. Parking, shoulders, sidewalks, bus lanes, bike lanes, etc. are
        // not accepted as avoidance area here.
        if( lane_ptr->type != map::driving )
        {
          continue;
        }

        if( !params.opposite_lane_enabled &&
            lane_ptr->left_of_reference != current_lane.left_of_reference )
        {
          continue;
        }

        LateralInterval lane_interval;
        if( !add_lane_lateral_interval(
              frame,
              *lane_ptr,
              route_point.s,
              params.lane_s_overlap_slack,
              lane_interval ) )
        {
          continue;
        }

        const double lane_center_l = 0.5 * ( lane_interval.min_l + lane_interval.max_l );

        // Only include lanes on the side on which the candidate shifts. This
        // prevents a left-shift candidate from gaining artificial drivable area
        // on the right, and vice versa.
        if( shift_left && lane_center_l <= 0.0 )
        {
          continue;
        }

        if( !shift_left && lane_center_l >= 0.0 )
        {
          continue;
        }

        intervals.push_back( lane_interval );
      }
    }
  }

  std::sort(
    intervals.begin(),
    intervals.end(),
    []( const LateralInterval& a, const LateralInterval& b )
    {
      return a.min_l < b.min_l;
    } );

  std::vector<LateralInterval> merged;
  for( const auto& interval : intervals )
  {
    if( merged.empty() ||
        interval.min_l >
          merged.back().max_l + std::max( 0.0, params.lane_boundary_join_slack ) )
    {
      merged.push_back( interval );
    }
    else
    {
      merged.back().max_l = std::max( merged.back().max_l, interval.max_l );
    }
  }

  // Use only the connected drivable interval containing the route centerline.
  // This rejects candidates that would jump over a median, shoulder, sidewalk,
  // or any non-driving gap.
  for( const auto& interval : merged )
  {
    if( interval.min_l <= 0.0 && interval.max_l >= 0.0 )
    {
      return interval;
    }
  }

  return current_interval;
}

ConnectedIntersectionAreaQuery
query_connected_intersection_driving_area_at_route_point(
  const map::Route& route,
  const map::MapPoint& route_point,
  double route_s,
  double lateral_shift,
  const ObstacleAvoidanceParams& params )
{
  ConnectedIntersectionAreaQuery query;

  if( !route.map )
  {
    query.reason = "route has no map";
    return query;
  }

  const auto current_lane_it = route.map->lanes.find( route_point.parent_id );
  if( current_lane_it == route.map->lanes.end() || !current_lane_it->second )
  {
    query.reason = "current route lane not found in map";
    return query;
  }

  const auto& current_lane = *current_lane_it->second;

  // The normal same-direction area is the seed. Foreign-road polygons may
  // enlarge only the connected component containing this seed; they never
  // create an isolated island that the ego would have to jump to.
  auto same_direction_params = params;
  same_direction_params.opposite_lane_enabled = false;
  const auto base_interval = get_allowed_lateral_interval_at_route_point(
    route,
    route_point,
    route_s,
    lateral_shift,
    true,
    same_direction_params );

  if( !base_interval.has_value() )
  {
    query.reason = "current/same-direction driving area is not usable";
    return query;
  }

  query.map_usable = true;
  query.interval = base_interval.value();
  query.reason = "no connected driving-lane polygon from another road";

  if( !params.intersection_drivable_area_enabled )
  {
    query.reason = "connected intersection driving area is disabled";
    return query;
  }

  const double search_radius = params.intersection_drivable_area_search_radius;
  if( !std::isfinite( search_radius ) || search_radius <= 0.0 )
  {
    query.map_usable = false;
    query.reason = "invalid intersection driving-area search radius";
    return query;
  }

  const auto frame = make_route_frame( route, route_s );
  const math::Point2d route_xy{ frame.x, frame.y };
  std::vector<map::MapPoint> nearby_center_points;
  route.map->quadtree.query_range(
    route_xy,
    search_radius,
    nearby_center_points );

  std::unordered_set<std::size_t> nearby_lane_ids;
  nearby_lane_ids.reserve( nearby_center_points.size() + 1 );
  nearby_lane_ids.insert( current_lane.id );
  for( const auto& point : nearby_center_points )
  {
    nearby_lane_ids.insert( point.parent_id );
  }

  struct TaggedInterval
  {
    LateralInterval interval;
    bool foreign_road = false;
    bool included = false;
  };

  std::vector<TaggedInterval> nearby_intervals;
  for( const std::size_t lane_id : nearby_lane_ids )
  {
    const auto lane_it = route.map->lanes.find( lane_id );
    if( lane_it == route.map->lanes.end() || !lane_it->second )
    {
      continue;
    }

    const auto& lane = *lane_it->second;
    if( lane.type != map::driving || !lane_has_usable_borders( lane ) )
    {
      continue;
    }

    const bool foreign_road = lane.road_id != current_lane.road_id;
    const auto lane_intervals =
      intersect_lane_polygon_with_route_normal(
        lane,
        frame,
        search_radius );
    for( const auto& interval : lane_intervals )
    {
      nearby_intervals.push_back( { interval, foreign_road, false } );
    }
  }

  const double join_slack = std::max( 0.0, params.lane_boundary_join_slack );
  auto grow_component =
    [&]( bool foreign_only, bool& included_foreign )
    {
      bool changed = true;
      while( changed )
      {
        changed = false;
        for( auto& candidate : nearby_intervals )
        {
          if( candidate.included ||
              ( foreign_only && !candidate.foreign_road ) ||
              !intervals_are_connected(
                query.interval,
                candidate.interval,
                join_slack ) )
          {
            continue;
          }

          expand_interval( query.interval, candidate.interval );
          candidate.included = true;
          included_foreign = included_foreign || candidate.foreign_road;
          changed = true;
        }
      }
    };

  // A same-road opposite lane must not turn this mode into an unrestricted
  // opposite-lane candidate on an ordinary road. First require a real
  // connection from another Road; only then admit every driving polygon in the
  // resulting shared intersection surface.
  bool included_foreign = false;
  grow_component( true, included_foreign );
  if( !included_foreign )
  {
    return query;
  }

  grow_component( false, included_foreign );
  query.has_cross_road_connection = true;
  query.reason =
    "connected intersection driving surface includes another road";
  return query;
}

OppositeDirectionLaneQuery
query_opposite_direction_lateral_interval_at_route_point(
  const map::Route& route,
  const map::MapPoint& route_point,
  double route_s,
  bool shift_left,
  const ObstacleAvoidanceParams& params )
{
  OppositeDirectionLaneQuery query;

  if( !route.map )
  {
    query.reason = "route has no map";
    return query;
  }

  const auto current_lane_it = route.map->lanes.find( route_point.parent_id );
  if( current_lane_it == route.map->lanes.end() || !current_lane_it->second )
  {
    query.reason = "current route lane not found in map";
    return query;
  }

  const auto& current_lane = *current_lane_it->second;

  const auto road_it = route.map->roads.find( current_lane.road_id );
  if( road_it == route.map->roads.end() )
  {
    query.reason = "current lane road not found in map";
    return query;
  }

  query.map_usable = true;
  query.reason = "map usable; no opposite-direction driving lane on candidate shift side";

  const auto frame = make_route_frame( route, route_s );

  std::vector<LateralInterval> opposite_intervals;
  bool found_opposite_wrong_side = false;
  bool found_opposite_side_without_usable_borders = false;

  for( const auto& lane_ptr : road_it->second.lanes )
  {
    if( !lane_ptr || lane_ptr->id == current_lane.id )
    {
      continue;
    }

    if( lane_ptr->type != map::driving )
    {
      continue;
    }

    // Same direction lanes are deliberately ignored here. This is essential for
    // multi-lane roads: leaving the current lane into a same-direction lane must
    // not trigger a head-on traffic check.
    if( lane_ptr->left_of_reference == current_lane.left_of_reference )
    {
      continue;
    }

    LateralInterval lane_interval;
    if( !add_lane_lateral_interval(
          frame,
          *lane_ptr,
          route_point.s,
          params.lane_s_overlap_slack,
          lane_interval ) )
    {
      found_opposite_side_without_usable_borders = true;
      continue;
    }

    const double lane_center_l =
      0.5 * ( lane_interval.min_l + lane_interval.max_l );

    // Only count lanes on the side toward which the candidate shifts.
    if( shift_left && lane_center_l <= 0.0 )
    {
      found_opposite_wrong_side = true;
      continue;
    }

    if( !shift_left && lane_center_l >= 0.0 )
    {
      found_opposite_wrong_side = true;
      continue;
    }

    opposite_intervals.push_back( lane_interval );
  }

  if( !opposite_intervals.empty() )
  {
    std::sort(
      opposite_intervals.begin(),
      opposite_intervals.end(),
      []( const LateralInterval& a, const LateralInterval& b )
      {
        return a.min_l < b.min_l;
      } );

    std::vector<LateralInterval> connected_opposite;
    for( const auto& interval : opposite_intervals )
    {
      if( connected_opposite.empty() ||
          !intervals_are_connected(
            connected_opposite.back(),
            interval,
            params.lane_boundary_join_slack ) )
      {
        connected_opposite.push_back( interval );
      }
      else
      {
        expand_interval( connected_opposite.back(), interval );
      }
    }

    const auto nearest_it =
      std::min_element(
        connected_opposite.begin(),
        connected_opposite.end(),
        []( const LateralInterval& a, const LateralInterval& b )
        {
          const double a_distance =
            a.min_l <= 0.0 && a.max_l >= 0.0
              ? 0.0
              : std::min( std::fabs( a.min_l ), std::fabs( a.max_l ) );
          const double b_distance =
            b.min_l <= 0.0 && b.max_l >= 0.0
              ? 0.0
              : std::min( std::fabs( b.min_l ), std::fabs( b.max_l ) );
          return a_distance < b_distance;
        } );

    query.has_opposite_lane = true;
    query.interval = *nearest_it;
    query.reason = "opposite-direction driving lane found on candidate shift side";
    return query;
  }

  if( found_opposite_side_without_usable_borders )
  {
    query.reason = "opposite-direction lane exists but its borders are not usable at this s";
  }
  else if( found_opposite_wrong_side )
  {
    query.reason = "opposite-direction driving lane exists only on the other side";
  }

  return query;
}

} // namespace oa_detail
} // namespace planner
} // namespace adore
