/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

// Geometry primitives for the obstacle-avoidance module: route-frame
// construction and point/reference projection onto the route reference line.
// Pure functions with no dependency on other obstacle-avoidance internals.

#include "obstacle_avoidance_internal.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <utility>

namespace adore
{
namespace planner
{
namespace oa_detail
{

double
normalize_angle( double angle )
{
  while( angle > M_PI ) angle -= 2.0 * M_PI;
  while( angle < -M_PI ) angle += 2.0 * M_PI;
  return angle;
}

double
smoothstep01( double t )
{
  // Quintic smootherstep: zero slope and zero curvature at both ends.
  t = std::clamp( t, 0.0, 1.0 );
  return t * t * t * ( t * ( t * 6.0 - 15.0 ) + 10.0 );
}

RouteFrame
make_route_frame( const map::Route& route, double s )
{
  const auto pose = route.get_pose_at_s( s );
  return RouteFrame{ pose.x, pose.y, pose.yaw };
}

double
signed_lateral_offset( const RouteFrame& frame, const math::Point2d& point )
{
  const double dx = point.x - frame.x;
  const double dy = point.y - frame.y;

  // +l = left of route tangent, -l = right of route tangent.
  return -dx * std::sin( frame.yaw ) + dy * std::cos( frame.yaw );
}

math::Point2d
shifted_point( const RouteFrame& frame, double lateral_offset )
{
  return math::Point2d{
    frame.x - std::sin( frame.yaw ) * lateral_offset,
    frame.y + std::cos( frame.yaw ) * lateral_offset
  };
}

std::optional<RouteProjectionSample>
project_point_to_route_segments( const map::Route& route, double x, double y )
{
  if( route.reference_line.size() < 2 )
  {
    return std::nullopt;
  }

  RouteProjectionSample best;

  auto it_prev = route.reference_line.begin();
  auto it      = std::next( it_prev );

  for( ; it != route.reference_line.end(); ++it, ++it_prev )
  {
    const double s0 = it_prev->first;
    const double s1 = it->first;

    const auto& p0 = it_prev->second;
    const auto& p1 = it->second;

    const double dx = p1.x - p0.x;
    const double dy = p1.y - p0.y;

    const double len2 = dx * dx + dy * dy;
    if( len2 < 1e-9 )
    {
      continue;
    }

    const double t_raw =
      ( ( x - p0.x ) * dx + ( y - p0.y ) * dy ) / len2;

    const double t = std::clamp( t_raw, 0.0, 1.0 );

    const double px = p0.x + t * dx;
    const double py = p0.y + t * dy;

    const double ex = x - px;
    const double ey = y - py;

    const double distance2 = ex * ex + ey * ey;

    if( distance2 < best.distance * best.distance )
    {
      const double len = std::sqrt( len2 );

      const double nx = -dy / len;
      const double ny =  dx / len;

      best.s = s0 + t * ( s1 - s0 );
      best.l = ex * nx + ey * ny;
      best.distance = std::sqrt( distance2 );
    }
  }

  if( !std::isfinite( best.s ) || !std::isfinite( best.l ) )
  {
    return std::nullopt;
  }

  return best;
}

std::optional<std::pair<double, map::MapPoint>>
find_reference_point_near_s( const map::Route& route, double query_s )
{
  if( route.reference_line.empty() || !std::isfinite( query_s ) )
  {
    return std::nullopt;
  }

  auto it_upper = route.reference_line.lower_bound( query_s );

  if( it_upper == route.reference_line.begin() )
  {
    return std::make_pair( it_upper->first, it_upper->second );
  }

  if( it_upper == route.reference_line.end() )
  {
    auto it_last = std::prev( route.reference_line.end() );
    return std::make_pair( it_last->first, it_last->second );
  }

  auto it_lower = std::prev( it_upper );

  if( std::fabs( it_upper->first - query_s ) <
      std::fabs( query_s - it_lower->first ) )
  {
    return std::make_pair( it_upper->first, it_upper->second );
  }

  return std::make_pair( it_lower->first, it_lower->second );
}

} // namespace oa_detail
} // namespace planner
} // namespace adore
