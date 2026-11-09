/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

// Projection of traffic-participant geometry onto the route reference line:
// analytic obstacle envelopes and participant footprints. Depends only on the
// geometry primitives in oa_detail.

#include "obstacle_avoidance_internal.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace adore
{
namespace planner
{
namespace oa_detail
{

const char*
route_corridor_object_class_name( RouteCorridorObjectClass object_class )
{
  switch( object_class )
  {
    case RouteCorridorObjectClass::StaticOrSlow:
      return "static_or_slow";
    case RouteCorridorObjectClass::Oncoming:
      return "oncoming";
    case RouteCorridorObjectClass::SameDirection:
      return "same_direction";
    case RouteCorridorObjectClass::CrossingOrUnknown:
    default:
      return "crossing_or_unknown";
  }
}

bool
contains_participant_id( const std::vector<int>& ids, int id )
{
  return std::find( ids.begin(), ids.end(), id ) != ids.end();
}

bool
project_obstacle_to_route_analytic( const map::Route& route,
                                    const dynamics::TrafficParticipant& participant,
                                    const ObstacleAvoidanceParams& params,
                                    double ego_half_width,
                                    ObstacleEnvelope& envelope )
{
  if( route.reference_line.size() < 2 )
  {
    return false;
  }
  if( !participant_has_valid_dimensions( participant ) )
  {
    return false;
  }

  envelope.object_s_min =  std::numeric_limits<double>::infinity();
  envelope.object_s_max = -std::numeric_limits<double>::infinity();
  envelope.object_l_min =  std::numeric_limits<double>::infinity();
  envelope.object_l_max = -std::numeric_limits<double>::infinity();

  const double center_x = participant.state.x;
  const double center_y = participant.state.y;
  const double yaw      = participant.state.yaw_angle;

  const double cos_yaw = std::cos( yaw );
  const double sin_yaw = std::sin( yaw );

  const double half_length =
    0.5 * participant.physical_parameters.body_length;

  const double half_width =
    0.5 * participant.physical_parameters.body_width;

  // Corners plus the midpoints of the long edges: on curved routes the side of
  // a long vehicle can reach closer to the reference line than either corner
  // (sagitta error ~ L^2/8R); the midpoints bound that under-estimation.
  const std::array<std::pair<double, double>, 6> local_corners = {{
    { -half_length, -half_width },
    { -half_length,  half_width },
    {  half_length,  half_width },
    {  half_length, -half_width },
    {          0.0,  half_width },
    {          0.0, -half_width }
  }};

  bool any_valid_projection = false;

  for( const auto& [local_x, local_y] : local_corners )
  {
    const double corner_x =
      center_x + local_x * cos_yaw - local_y * sin_yaw;

    const double corner_y =
      center_y + local_x * sin_yaw + local_y * cos_yaw;

    const auto projection = project_point_to_route_segments( route, corner_x, corner_y );

    if( !projection.has_value() )
    {
      continue;
    }

    any_valid_projection = true;

    envelope.object_s_min =
      std::min( envelope.object_s_min, projection->s );

    envelope.object_s_max =
      std::max( envelope.object_s_max, projection->s );

    envelope.object_l_min =
      std::min( envelope.object_l_min, projection->l );

    envelope.object_l_max =
      std::max( envelope.object_l_max, projection->l );
  }

  if( !any_valid_projection )
  {

    return false;
  }

  if( !std::isfinite( envelope.object_s_min ) ||
      !std::isfinite( envelope.object_s_max ) ||
      !std::isfinite( envelope.object_l_min ) ||
      !std::isfinite( envelope.object_l_max ) )
  {
    return false;
  }

  const double corridor_half_width =
    ego_half_width + params.ego_corridor_safety_margin;

  const bool overlaps_ego_corridor =
    envelope.object_l_min <= corridor_half_width &&
    envelope.object_l_max >= -corridor_half_width;

  envelope.overlaps_ego_corridor = overlaps_ego_corridor;

  envelope.center_s =
    0.5 * ( envelope.object_s_min + envelope.object_s_max );

  return true;
}

std::optional<ParticipantFootprintOnRoute>
project_participant_footprint_to_route(
  const map::Route& route,
  const dynamics::TrafficParticipant& participant,
  const ObstacleAvoidanceParams& params )
{
  if( route.reference_line.size() < 2 )
  {
    return std::nullopt;
  }
  if( !participant_has_valid_dimensions( participant ) )
  {
    return std::nullopt;
  }

  const double center_x = participant.state.x;
  const double center_y = participant.state.y;
  const double yaw      = participant.state.yaw_angle;

  const double cos_yaw = std::cos( yaw );
  const double sin_yaw = std::sin( yaw );
  const double half_length =
    0.5 * participant.physical_parameters.body_length;

  const double half_width =
    0.5 * participant.physical_parameters.body_width;

  // Corners plus long-edge midpoints; see project_obstacle_to_route_analytic
  // for the curved-route rationale.
  const std::array<std::pair<double, double>, 6> local_corners = {{
    { -half_length, -half_width },
    { -half_length,  half_width },
    {  half_length,  half_width },
    {  half_length, -half_width },
    {          0.0,  half_width },
    {          0.0, -half_width }
  }};

  ParticipantFootprintOnRoute footprint;
  bool any_valid_projection = false;

  for( const auto& [local_x, local_y] : local_corners )
  {
    const double corner_x =
      center_x + local_x * cos_yaw - local_y * sin_yaw;

    const double corner_y =
      center_y + local_x * sin_yaw + local_y * cos_yaw;

    const auto projection = project_point_to_route_segments( route, corner_x, corner_y );
    if( !projection.has_value() )
    {
      continue;
    }

    any_valid_projection = true;

    footprint.s_min = std::min( footprint.s_min, projection->s );
    footprint.s_max = std::max( footprint.s_max, projection->s );
    footprint.l_min = std::min( footprint.l_min, projection->l );
    footprint.l_max = std::max( footprint.l_max, projection->l );
  }

  const auto center_projection = project_point_to_route_segments( route, center_x, center_y );
  if( center_projection.has_value() )
  {
    footprint.center_s = center_projection->s;
  }
  else
  {
    footprint.center_s = 0.5 * ( footprint.s_min + footprint.s_max );
  }

  if( !any_valid_projection ||
      !std::isfinite( footprint.s_min ) ||
      !std::isfinite( footprint.s_max ) ||
      !std::isfinite( footprint.l_min ) ||
      !std::isfinite( footprint.l_max ) ||
      !std::isfinite( footprint.center_s ) )
  {
    return std::nullopt;
  }

  return footprint;
}

} // namespace oa_detail
} // namespace planner
} // namespace adore
