/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include "planning/obstacle_avoidance.hpp"

#include "obstacle_avoidance_internal.hpp"

#include "adore_math/point.h"
#include "adore_math/pose.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <exception>
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace adore
{
namespace planner
{

// All internal obstacle-avoidance helpers (geometry, projection, grouping,
// shift, drivable-area, oncoming, candidates, stop-route) now live in the
// adore::planner::oa_detail namespace across the obstacle_avoidance_*.cpp
// translation units. Pull them in so the public API below can call them
// unqualified.
using namespace oa_detail;

std::string
validate_obstacle_avoidance_params( const ObstacleAvoidanceParams& params )
{
  const std::vector<std::pair<const char*, double>> nonnegative = {
    { "max_static_object_speed", params.max_static_object_speed },
    { "ego_corridor_safety_margin", params.ego_corridor_safety_margin },
    { "side_clearance", params.side_clearance },
    { "side_clearance_replan_tolerance", params.side_clearance_replan_tolerance },
    { "front_clearance", params.front_clearance },
    { "rear_clearance", params.rear_clearance },
    { "stop_before_obstacle", params.stop_before_obstacle },
    { "max_speed_during_avoidance", params.max_speed_during_avoidance },
    { "blinker_lead_distance", params.blinker_lead_distance },
    { "lane_s_overlap_slack", params.lane_s_overlap_slack },
    { "lane_boundary_join_slack", params.lane_boundary_join_slack },
    { "oncoming_time_margin", params.oncoming_time_margin },
    { "prediction_time_horizon", params.prediction_time_horizon },
    { "oncoming_spatial_margin", params.oncoming_spatial_margin },
    { "ego_lane_oncoming_lateral_margin", params.ego_lane_oncoming_lateral_margin },
    { "ego_lane_oncoming_stop_distance", params.ego_lane_oncoming_stop_distance },
    { "modified_route_braking_safety_margin", params.modified_route_braking_safety_margin },
    { "route_window_min", params.route_window_min },
    { "projection_progress_tolerance", params.projection_progress_tolerance },
    { "stopped_ego_speed", params.stopped_ego_speed }
  };

  for( const auto& [name, value] : nonnegative )
  {
    if( !std::isfinite( value ) || value < 0.0 )
    {
      return std::string( name ) + " must be finite and >= 0";
    }
  }

  const std::vector<std::pair<const char*, double>> positive = {
    { "avoidance_lateral_accel", params.avoidance_lateral_accel },
    { "min_oncoming_route_speed", params.min_oncoming_route_speed },
    { "planned_braking_deceleration", params.planned_braking_deceleration },
    { "intersection_drivable_area_search_radius", params.intersection_drivable_area_search_radius }
  };

  for( const auto& [name, value] : positive )
  {
    if( !std::isfinite( value ) || value <= 0.0 )
    {
      return std::string( name ) + " must be finite and > 0";
    }
  }

  if( !std::isfinite( params.min_oncoming_heading_diff ) ||
      params.min_oncoming_heading_diff < 0.0 ||
      params.min_oncoming_heading_diff > std::numbers::pi )
  {
    return "min_oncoming_heading_diff must be in [0, pi]";
  }
  if( params.side_clearance < params.ego_corridor_safety_margin )
  {
    return "side_clearance must be >= ego_corridor_safety_margin";
  }
  return {};
}

std::string
validate_obstacle_avoidance_vehicle_params(
  const dynamics::PhysicalVehicleParameters& vehicle_params )
{
  const auto require_positive =
    []( const char* name, double value ) -> std::string
    {
      if( !std::isfinite( value ) || value <= 0.0 )
      {
        return std::string( name ) + " must be finite and > 0";
      }
      return {};
    };
  const auto require_nonnegative =
    []( const char* name, double value ) -> std::string
    {
      if( !std::isfinite( value ) || value < 0.0 )
      {
        return std::string( name ) + " must be finite and >= 0";
      }
      return {};
    };

  if( auto error = require_positive( "body_width", vehicle_params.body_width );
      !error.empty() )
  {
    return error;
  }
  if( auto error = require_positive( "body_length", vehicle_params.body_length );
      !error.empty() )
  {
    return error;
  }
  if( auto error = require_positive( "wheelbase", vehicle_params.wheelbase );
      !error.empty() )
  {
    return error;
  }
  if( auto error = require_nonnegative(
        "front_axle_to_front_border",
        vehicle_params.front_axle_to_front_border );
      !error.empty() )
  {
    return error;
  }
  if( auto error = require_nonnegative(
        "rear_border_to_rear_axle",
        vehicle_params.rear_border_to_rear_axle );
      !error.empty() )
  {
    return error;
  }
  if( !std::isfinite( vehicle_params.acceleration_min ) ||
      vehicle_params.acceleration_min >= 0.0 )
  {
    return "acceleration_min must be finite and < 0";
  }

  return {};
}

bool
participant_has_valid_dimensions(
  const dynamics::TrafficParticipant& participant )
{
  return
    std::isfinite( participant.physical_parameters.body_length ) &&
    participant.physical_parameters.body_length > 0.0 &&
    std::isfinite( participant.physical_parameters.body_width ) &&
    participant.physical_parameters.body_width > 0.0;
}

bool
traffic_participants_have_valid_dimensions(
  const dynamics::TrafficParticipantSet& traffic_participants )
{
  return std::all_of(
    traffic_participants.participants.begin(),
    traffic_participants.participants.end(),
    []( const auto& entry )
    {
      return participant_has_valid_dimensions( entry.second );
    } );
}

double
maximum_braking_deceleration(
  const dynamics::PhysicalVehicleParameters& vehicle_params,
  const ObstacleAvoidanceParams& )
{
  const std::string validation_error =
    validate_obstacle_avoidance_vehicle_params( vehicle_params );
  if( !validation_error.empty() )
  {
    throw std::invalid_argument(
      "invalid obstacle avoidance vehicle parameters: " + validation_error );
  }
  return std::fabs( vehicle_params.acceleration_min );
}

double
planned_braking_deceleration(
  const dynamics::PhysicalVehicleParameters& vehicle_params,
  const ObstacleAvoidanceParams& params )
{
  const double max_deceleration =
    maximum_braking_deceleration( vehicle_params, params );

  if( !std::isfinite( params.planned_braking_deceleration ) ||
      params.planned_braking_deceleration <= 0.0 )
  {
    return max_deceleration;
  }

  return std::min(
    params.planned_braking_deceleration,
    max_deceleration );
}

double
static_obstacle_action_end_s(
  double ego_s,
  double ego_speed,
  const dynamics::PhysicalVehicleParameters& vehicle_params,
  const ObstacleAvoidanceParams& params )
{
  // A non-finite state must not shorten the safety check. Returning infinity
  // makes the caller retain the conservative full-route behavior.
  if( !std::isfinite( ego_s ) || !std::isfinite( ego_speed ) )
  {
    return std::numeric_limits<double>::infinity();
  }

  const double speed = std::max( 0.0, ego_speed );
  const double ego_front_offset =
    vehicle_params.wheelbase + vehicle_params.front_axle_to_front_border;
  const double braking_distance =
    speed * speed /
    ( 2.0 * planned_braking_deceleration( vehicle_params, params ) );
  const double one_cycle_travel =
    speed * obstacle_avoidance_cycle_time_s;

  return
    ego_s + ego_front_offset + one_cycle_travel + braking_distance +
    std::max( 0.0, params.stop_before_obstacle ) +
    std::max( 0.0, params.modified_route_braking_safety_margin );
}

void
set_route_points_from_s_to_zero( map::Route& route, double ego_s )
{
  if( route.reference_line.empty() )
  {
    return;
  }

  if( !std::isfinite( ego_s ) )
  {
    ego_s = route.reference_line.begin()->first;
  }

  for( auto& [s, point] : route.reference_line )
  {
    if( s >= ego_s )
    {
      point.max_speed = 0.0;
    }
  }
}

map::Route
RouteSpeedPolicy::apply_avoidance_speed_profile(
  const map::Route& route,
  double ego_s,
  double shift_start_s,
  double maneuver_end_s,
  const dynamics::PhysicalVehicleParameters& vehicle_params,
  const ObstacleAvoidanceParams& params,
  double ego_v )
{
  map::Route profiled_route = route;

  if( params.max_speed_during_avoidance <= 0.0 ||
      !std::isfinite( ego_s ) ||
      !std::isfinite( shift_start_s ) )
  {
    return profiled_route;
  }

  if( !std::isfinite( maneuver_end_s ) )
  {
    maneuver_end_s = shift_start_s;
  }
  maneuver_end_s = std::max( maneuver_end_s, shift_start_s );

  const double target_v = std::max( 0.0, params.max_speed_during_avoidance );
  const double a_abs =
    planned_braking_deceleration( vehicle_params, params );

  // Coast cap for the approach to the maneuver: never accelerate above the
  // current speed before the shift. Without this the sqrt-profile below is large
  // far from shift_start and lets the planner speed back up to the route cruise
  // cap right before the maneuver, then brake hard - the "accelerate, then brake"
  // peak seen during active obstacle avoidance. Floor at target_v so a slow ego
  // can still roll up to the maneuver speed. Defaults to infinity (no cap) for
  // callers that do not pass a live speed (e.g. candidate generation).
  const double approach_cap =
    std::max( std::max( 0.0, ego_v ), target_v );
  for( auto& [s, point] : profiled_route.reference_line )
  {
    if( s < ego_s )
    {
      continue;
    }

    if( s < shift_start_s )
    {
      const double distance_to_shift_start = shift_start_s - s;
      const double allowed_speed =
        std::min(
          approach_cap,
          std::sqrt(
            target_v * target_v +
            2.0 * a_abs * std::max( 0.0, distance_to_shift_start ) ) );
      point.max_speed =
        std::min(
          point.max_speed.value_or( std::numeric_limits<double>::infinity() ),
          allowed_speed );
      continue;
    }

    if( s <= maneuver_end_s )
    {
      point.max_speed =
        std::min(
          point.max_speed.value_or( std::numeric_limits<double>::infinity() ),
          target_v );
    }
  }

  return profiled_route;
}

map::Route
RouteSpeedPolicy::apply_segmented_avoidance_speed_profile(
  const map::Route& route,
  double ego_s,
  const std::vector<AvoidanceSpeedSegment>& segments,
  const dynamics::PhysicalVehicleParameters& vehicle_params,
  const ObstacleAvoidanceParams& params )
{
  map::Route profiled_route = route;

  if( params.max_speed_during_avoidance <= 0.0 ||
      !std::isfinite( ego_s ) ||
      segments.empty() )
  {
    return profiled_route;
  }

  std::vector<AvoidanceSpeedSegment> merged_segments;
  merged_segments.reserve( segments.size() );
  for( const auto& segment : segments )
  {
    if( !std::isfinite( segment.start_s ) ||
        !std::isfinite( segment.end_s ) )
    {
      continue;
    }

    AvoidanceSpeedSegment normalized = segment;
    if( normalized.end_s < normalized.start_s )
    {
      std::swap( normalized.start_s, normalized.end_s );
    }
    merged_segments.push_back( normalized );
  }

  if( merged_segments.empty() )
  {
    return profiled_route;
  }

  std::sort(
    merged_segments.begin(),
    merged_segments.end(),
    []( const AvoidanceSpeedSegment& lhs,
        const AvoidanceSpeedSegment& rhs )
    {
      return lhs.start_s < rhs.start_s;
    } );

  constexpr double segment_join_tolerance = 1e-6;
  std::size_t write_index = 0;
  for( std::size_t read_index = 1;
       read_index < merged_segments.size();
       ++read_index )
  {
    auto& current = merged_segments[write_index];
    const auto& next = merged_segments[read_index];
    if( next.start_s <= current.end_s + segment_join_tolerance )
    {
      current.end_s = std::max( current.end_s, next.end_s );
      continue;
    }

    ++write_index;
    merged_segments[write_index] = next;
  }
  merged_segments.resize( write_index + 1 );

  const double target_v =
    std::max( 0.0, params.max_speed_during_avoidance );
  const double braking_deceleration =
    planned_braking_deceleration( vehicle_params, params );
  std::size_t segment_index = 0;

  for( auto& [s, point] : profiled_route.reference_line )
  {
    if( s < ego_s )
    {
      continue;
    }

    while( segment_index < merged_segments.size() &&
           s > merged_segments[segment_index].end_s )
    {
      ++segment_index;
    }

    if( segment_index >= merged_segments.size() )
    {
      break;
    }

    const auto& segment = merged_segments[segment_index];
    double allowed_speed = target_v;
    if( s < segment.start_s )
    {
      const double distance_to_segment = segment.start_s - s;
      allowed_speed =
        std::sqrt(
          target_v * target_v +
          2.0 * braking_deceleration * distance_to_segment );
    }

    point.max_speed =
      std::min(
        point.max_speed.value_or( std::numeric_limits<double>::infinity() ),
        allowed_speed );
  }

  return profiled_route;
}

map::Route
RouteSpeedPolicy::apply_stop_profile(
  const map::Route& route,
  double ego_s,
  double ego_v,
  double desired_stop_s,
  const dynamics::PhysicalVehicleParameters& vehicle_params,
  const ObstacleAvoidanceParams& params )
{
  // Unified braking: delegate to the three-regime envelope so every planned stop
  // brakes gently when possible, increases deceleration only as required, and
  // uses the physical maximum (|acceleration_min|) only when the stop point is
  // unreachable. Kept as a named entry point so the existing call sites
  // (plan_stop_on_route, build_stop_route_before_obstacle) stay unchanged.
  return apply_brake_envelope(
    route, ego_s, ego_v, desired_stop_s, vehicle_params, params );
}

map::Route
RouteSpeedPolicy::apply_brake_envelope(
  const map::Route& route,
  double ego_s,
  double ego_v,
  double desired_stop_s,
  const dynamics::PhysicalVehicleParameters& vehicle_params,
  const ObstacleAvoidanceParams& params )
{
  map::Route braked = route;

  if( braked.reference_line.empty() || !std::isfinite( desired_stop_s ) )
  {
    return braked;
  }

  if( !std::isfinite( ego_s ) )
  {
    ego_s = braked.reference_line.begin()->first;
  }

  const double v             = std::max( 0.0, ego_v );
  const double comfort_decel = planned_braking_deceleration( vehicle_params, params );
  const double max_decel     = maximum_braking_deceleration( vehicle_params, params );

  const double available              = desired_stop_s - ego_s;
  const double comfort_brake_distance = v * v / ( 2.0 * comfort_decel );
  const double maximum_brake_distance = v * v / ( 2.0 * max_decel );

  // Three regimes with a stable requested target:
  // 1) enough room -> use the configured comfort deceleration;
  // 2) comfort is insufficient but maximum braking can still reach the requested
  //    point -> use exactly the required intermediate deceleration and keep the
  //    target fixed;
  // 3) even maximum braking cannot reach the requested point -> brake maximally
  //    and stop at the earliest point physics permits.
  double decel;
  double target;
  if( available >= comfort_brake_distance )
  {
    decel  = comfort_decel;
    target = desired_stop_s;
  }
  else if( available > 0.0 &&
           available >= maximum_brake_distance )
  {
    decel =
      std::clamp(
        v * v / ( 2.0 * available ),
        comfort_decel,
        max_decel );
    target = desired_stop_s;
  }
  else
  {
    decel  = max_decel;
    target = ego_s + maximum_brake_distance;
  }

  insert_zero_speed_stop_point( braked, target );

  // Never accelerate toward a stop. Far from the target the sqrt-envelope is high,
  // so without this cap the planner is allowed to speed back up to the route's
  // cruise cap before braking - which shows up as an unwanted "accelerate, then
  // brake" right after an obstacle is detected. Cap the coast portion at the
  // current speed, with a small approach floor (max_speed_during_avoidance) so a
  // stopped ego can still roll up to the stop point. The sqrt-envelope still binds
  // near the target, so this introduces no creep.
  const double coast_cap =
    std::max( v, std::max( 0.0, params.max_speed_during_avoidance ) );

  for( auto& [s, point] : braked.reference_line )
  {
    if( s < ego_s )
    {
      continue;
    }
    const double dist_to_stop = target - s;
    const double allowed_speed =
      std::min(
        coast_cap,
        dist_to_stop > 0.0
          ? std::sqrt( 2.0 * decel * dist_to_stop )
          : 0.0 );
    point.max_speed =
      std::min(
        point.max_speed.value_or( std::numeric_limits<double>::infinity() ),
        allowed_speed );
  }

  return braked;
}

map::Route
RouteSpeedPolicy::apply_max_braking_profile(
  const map::Route& route,
  double ego_s,
  double ego_v,
  const dynamics::PhysicalVehicleParameters& vehicle_params,
  const ObstacleAvoidanceParams& params )
{
  const double v = std::max( 0.0, ego_v );
  const double max_decel = maximum_braking_deceleration( vehicle_params, params );
  // Stop point of a maximum-deceleration brake from the current speed. Passing it
  // as the envelope target lands in the envelope's maximum-braking regime, i.e. a
  // sqrt(2*a_max*(stop-s)) profile from the current position - not a hard zero.
  const double stop_s = ego_s + ( v * v ) / ( 2.0 * max_decel );
  return apply_brake_envelope( route, ego_s, ego_v, stop_s, vehicle_params, params );
}

StopPlan
RouteStopPolicy::plan_stop_on_route(
  const map::Route& route,
  double ego_s,
  double ego_v,
  double desired_stop_s,
  const dynamics::PhysicalVehicleParameters& vehicle_params,
  const ObstacleAvoidanceParams& params )
{
  StopPlan plan;
  plan.route = route;

  if( route.reference_line.empty() ||
      !std::isfinite( ego_s ) ||
      !std::isfinite( desired_stop_s ) )
  {
    plan.valid = false;
    return plan;
  }

  plan.route =
    RouteSpeedPolicy::apply_stop_profile(
      route,
      ego_s,
      ego_v,
      desired_stop_s,
      vehicle_params,
      params );
  plan.valid = true;

  return plan;
}

double
avoidance_ramp_length( double distance_to_obstacle,
                       double ego_front_offset,
                       const ObstacleAvoidanceParams& params )
{
  const double floor = std::max( 0.0, ego_front_offset );
  const double cap = std::max( floor, std::max( 0.0, params.front_clearance ) );
  const double distance = std::isfinite( distance_to_obstacle )
                            ? distance_to_obstacle
                            : cap;
  return std::clamp( distance, floor, cap );
}

double
avoidance_speed_for_shift( double ramp_length,
                           double shift_magnitude,
                           const ObstacleAvoidanceParams& params )
{
  // Preserve the "0 disables speed capping" convention of max_speed_during_avoidance.
  if( params.max_speed_during_avoidance <= 0.0 )
  {
    return 0.0;
  }

  double speed = std::max( 0.0, params.max_speed_during_avoidance );
  const double accel =
    std::max( 1e-6, params.avoidance_lateral_accel );
  if( std::fabs( shift_magnitude ) > 1e-3 )
  {
    const double comfort_speed =
      std::max( 0.0, ramp_length ) *
      std::sqrt( accel / ( 6.0 * std::fabs( shift_magnitude ) ) );
    speed = std::min( speed, comfort_speed );
  }
  return std::clamp(
    speed,
    0.0,
    std::max( 0.0, params.max_speed_during_avoidance ) );
}

ObstacleAvoidanceResult
try_plan_obstacle_avoidance( TrajectoryPlanner& planner,
                             const map::Route& route,
                             const dynamics::VehicleStateDynamic& ego,
                             const dynamics::TrafficParticipantSet& traffic_participants,
                             const ObstacleAvoidanceParams& params,
                             double shift_direction_sign,
                             const std::vector<AvoidanceShiftContribution>*
                               committed_contributions,
                             const std::vector<int>*
                               forced_participant_ids )
{
  ObstacleAvoidanceResult result;
  result.modified_route = route;

  // Find relevant static obstacles on the fixed mission-route frame. During a
  // dynamic replan, accepted contributions are restored first and only
  // driven-corridor intrusions are allowed to join them.
  const auto obstacle_group = find_static_obstacle_group_on_route(
    route,
    ego,
    traffic_participants,
    planner.get_physical_vehicle_parameters(),
    params,
    committed_contributions,
    forced_participant_ids );

  if( !obstacle_group.has_value() )
  {
    result.reason = "no static obstacle in ego corridor";
    return result;
  }

  const auto vehicle_params = planner.get_physical_vehicle_parameters();

  // The nearest relevant obstacle group now drives shift generation, validation,
  // and oncoming checks.
  // Now check if a lateral shift is possible, if it would conflict with oncoming traffic, and plan the modified route and trajectory accordingly.
  // If any of these steps fail, fall back to a stop trajectory before the obstacle.
  auto plan_stop_before_obstacle = [&]( ObstacleAvoidanceMode mode,
                                        const std::string& reason ) -> ObstacleAvoidanceResult
  {
    ObstacleAvoidanceResult stop_result;
    stop_result.mode = mode;
    stop_result.reason = reason;

    // Concise, colleague-facing label for the displayed behavior. The verbose `reason`
    // (obstacle geometry + every rejection category) stays in stop_result.reason for the
    // diagnostic logs; it must not leak into the trajectory label.
    const std::string stop_label =
      mode == ObstacleAvoidanceMode::WaitForOncoming
        ? "driving mission (waiting for oncoming)"
        : "driving mission (stopping for obstacle)";

    const auto& stop_obstacle = obstacle_group->obstacles.front();
    stop_result.obstacle_id = stop_obstacle.id;
    stop_result.obstacle_ids =
      obstacle_group->envelope.participant_ids.empty()
        ? stop_obstacle.participant_ids
        : obstacle_group->envelope.participant_ids;
    stop_result.obstacle_s_min = stop_obstacle.object_s_min;
    const double stop_before_obstacle =
      normalized_stop_before_obstacle( params );
    const double ego_front_offset =
      vehicle_params.wheelbase + vehicle_params.front_axle_to_front_border;
    const double front_stop_s =
      stop_obstacle.object_s_min - std::max( 0.0, stop_before_obstacle );
    const double stop_s =
      front_stop_s - ego_front_offset;

    // Validate that the planned trajectory actually stops before the obstacle;
    // a non-empty trajectory alone does not guarantee a stop.
    RouteCorridorConflict stop_conflict;
    stop_conflict.participant_id = stop_obstacle.id;
    stop_conflict.object_s_min = stop_obstacle.object_s_min;
    stop_conflict.object_s_max = stop_obstacle.object_s_max;

    // While the obstacle is still beyond comfortable braking range, the
    // brake-envelope route correctly keeps cruising now and brakes later, so the
    // planned trajectory legitimately does not stop within this short horizon.
    // Accept it as success instead of treating it as a failed stop: otherwise
    // try_plan_obstacle_avoidance reports failure and the caller falls back to a
    // last-resort hold (a v=0 freeze that flashes for a cycle) even though ego is
    // far from the obstacle and merely approaching. Only skip the stop proof when
    // there is provably enough room to stop comfortably later.
    const double ego_s_for_stop = project_s_on_reference_line( route, ego );
    const double v0_for_stop     = std::max( 0.0, ego.vx );
    const double comfort_braking_distance =
      v0_for_stop * v0_for_stop /
      ( 2.0 * planned_braking_deceleration( vehicle_params, params ) );
    const bool stop_point_still_far =
      std::isfinite( ego_s_for_stop ) &&
      std::isfinite( stop_s ) &&
      ( stop_s - ego_s_for_stop ) >
        comfort_braking_distance +
        std::max( 0.0, params.modified_route_braking_safety_margin );

    if( stop_point_still_far )
    {
      // The obstacle is still beyond braking range. Keep the unmodified mission
      // route and approach it normally; the maneuver is re-evaluated every cycle.
      // Building a distant brake-envelope route here used to make the core verify
      // one route while the behavior layer could publish a separately replanned
      // trajectory.
      try
      {
        stop_result.modified_route = route;
        stop_result.trajectory =
          planner.plan_route_trajectory( route, ego, traffic_participants );
      }
      catch( const std::exception& )
      {
      }

      if( !stop_result.trajectory.states.empty() )
      {
        stop_result.trajectory.label =
          "driving mission (obstacle ahead)";
        stop_result.success = true;
        return stop_result;
      }
    }

    stop_result.modified_route =
      build_stop_route_before_obstacle(
        route,
        stop_obstacle,
        ego,
        vehicle_params,
        stop_before_obstacle,
        params );

    try
    {
      stop_result.trajectory = planner.plan_route_trajectory(
        stop_result.modified_route,
        ego,
        traffic_participants );
    }
    catch( const std::exception& )
    {
    }

    if( !trajectory_stops_before_conflict(
          stop_result.trajectory,
          stop_result.modified_route,
          stop_conflict,
          vehicle_params,
          params ) )
    {

      stop_result.modified_route =
        RouteSpeedPolicy::apply_max_braking_profile(
          route,
          project_s_on_reference_line( route, ego, stop_s ),
          ego.vx,
          vehicle_params,
          params );

      try
      {
        stop_result.trajectory = planner.plan_route_trajectory(
          stop_result.modified_route,
          ego,
          traffic_participants );
      }
      catch( const std::exception& )
      {
      }

      if( !trajectory_stops_before_conflict(
            stop_result.trajectory,
            stop_result.modified_route,
            stop_conflict,
            vehicle_params,
            params,
            true ) )
      {
        stop_result.success = false;
        stop_result.reason =
          reason + ": OA stop route planning failed";
        return stop_result;
      }

    }

    stop_result.trajectory.label = stop_label;
    stop_result.success = true;

    return stop_result;
  };

  const double ego_s_original = project_s_on_reference_line( route, ego );
  if( !std::isfinite( ego_s_original ) )
  {
    return plan_stop_before_obstacle(
      ObstacleAvoidanceMode::StopBeforeObstacle,
      "driving mission (stop before obstacle: invalid ego route projection)" );
  }

  // The selected connected obstacle component must be checked through its
  // complete shift support. Independent static objects farther ahead are hard
  // conflicts only once they enter the braking-derived action horizon; until
  // then the 10 Hz receding-horizon loop may plan them separately.
  const double static_candidate_check_end_s =
    std::max(
      avoidance_group_maneuver_end_s(
        obstacle_group.value(), vehicle_params, params ),
      static_obstacle_action_end_s(
        ego_s_original, ego.vx, vehicle_params, params ) );

  std::vector<RouteShiftPlanCandidate> accepted_candidates;

  bool saw_validated_candidate = false;
  bool saw_oncoming_conflict = false;

  std::string last_drivable_area_rejection;
  std::string last_planning_rejection;
  std::string last_validation_rejection;
  std::string last_safety_rejection;
  std::string last_oncoming_rejection;

  auto describe_candidate_rejection =
    []( const RouteShiftPlanCandidate& candidate,
        const std::string& reason ) -> std::string
    {
      char buf[512];
      std::snprintf(
        buf,
        sizeof( buf ),
        "%s (type=%d in_lane=%d shift=%.2f, front_clearance=%.2f, rear_clearance=%.2f)",
        reason.c_str(),
        static_cast<int>( candidate.shift_candidate.type ),
        static_cast<int>( candidate.shift_candidate.in_lane ),
        candidate.shift_candidate.shift,
        candidate.params.front_clearance,
        candidate.params.rear_clearance );
      return buf;
    };

  const auto base_shift_variants =
    generate_shift_candidate_variants( obstacle_group.value(), vehicle_params, params );
  std::vector<ShiftCandidate> shift_variants;
  shift_variants.reserve( 3 * base_shift_variants.size() + 2 );

  // Candidate-mode enablement is owned here. Geometry evaluation below only
  // decides whether a candidate of the already-enabled type is feasible.
  auto append_base_candidates =
    [&]( AvoidanceCandidateType type )
    {
      for( const auto& base_candidate : base_shift_variants )
      {
        auto candidate = base_candidate;
        candidate.type = type;
        shift_variants.push_back( candidate );
      }
    };

  if( params.in_lane_shift_enabled )
  {
    append_base_candidates( AvoidanceCandidateType::InLane );
  }

  if( params.adjacent_lane_enabled )
  {
    append_base_candidates( AvoidanceCandidateType::AdjacentSameDirection );
  }

  if( params.intersection_drivable_area_enabled )
  {
    append_base_candidates( AvoidanceCandidateType::ConnectedIntersection );
  }

  if( params.opposite_lane_enabled )
  {
    const auto opposite_lane_shift_variants =
      generate_opposite_lane_candidate_variants(
        route,
        obstacle_group.value(),
        vehicle_params,
        params );
    shift_variants.insert(
      shift_variants.end(),
      opposite_lane_shift_variants.begin(),
      opposite_lane_shift_variants.end() );
  }

  // Lock the shift to a single lateral direction when requested. A mid-maneuver
  // extension re-plans on the route ego is already following (so an obstacle it has
  // cleared contributes ~0 shift) and must only ever shift FURTHER in the direction
  // ego is already going. Exploring the opposite direction here would let a smaller
  // opposite shift win and pull ego back toward the obstacle it is passing.
  if( std::fabs( shift_direction_sign ) > 1e-6 )
  {
    const double sign = shift_direction_sign;
    shift_variants.erase(
      std::remove_if(
        shift_variants.begin(),
        shift_variants.end(),
        [sign]( const ShiftCandidate& candidate )
        {
          return !( ( sign > 0.0 && candidate.shift > 0.0 ) ||
                    ( sign < 0.0 && candidate.shift < 0.0 ) );
        } ),
      shift_variants.end() );
  }

  // Physics-sized entry ramp + maneuver speed (see avoidance_ramp_length /
  // avoidance_speed_for_shift). The ramp is sized once to the available distance
  // (shift height is per candidate). Applied to candidate.params so the drivable
  // check, the modified-route build (which bakes the speed profile) and the
  // trajectory validation all use the matched short-ramp + slow-speed maneuver:
  // that is what lets a late replan validate (slow ego turns tight enough to
  // develop the shift over the short remaining distance) instead of failing.
  const double ego_front_offset =
    vehicle_params.wheelbase + vehicle_params.front_axle_to_front_border;
  // Size the entry ramp to the nearest NEW obstacle edge, not the group's front edge.
  // At a mid-maneuver widen the group's front edge is the object ego is already
  // passing (its committed/held shift), right next to ego -> the ramp would collapse to
  // ego_front_offset and the physics-sized speed for the whole remaining maneuver.
  // The committed object needs no new turn-in; only a genuinely
  // new object (not committed_hold) sets the ramp / speed. Falls back to the group
  // front edge for the initial plan, where nothing is committed_hold.
  double new_obstacle_s_min = std::numeric_limits<double>::infinity();
  for( const auto& obstacle : obstacle_group->obstacles )
  {
    if( !obstacle.committed_hold )
    {
      new_obstacle_s_min = std::min( new_obstacle_s_min, obstacle.object_s_min );
    }
  }
  const double distance_reference_s =
    std::isfinite( new_obstacle_s_min ) ? new_obstacle_s_min
                                        : obstacle_group->envelope.object_s_min;
  const double distance_to_obstacle = distance_reference_s - ego_s_original;
  const double avoidance_ramp =
    avoidance_ramp_length( distance_to_obstacle, ego_front_offset, params );

  enum class CandidatePreparationFailure
  {
    None,
    DrivableArea,
    Planning
  };

  auto prepare_candidate =
    [&]( RouteShiftPlanCandidate& candidate,
         std::string& failure_reason ) -> CandidatePreparationFailure
    {
      candidate.params.front_clearance = avoidance_ramp;
      candidate.params.max_speed_during_avoidance =
        avoidance_speed_for_shift(
          avoidance_ramp,
          candidate.shift_candidate.shift,
          params );

      candidate.shift_candidate.valid = true;
      evaluate_shift_candidate(
        candidate.shift_candidate,
        route,
        obstacle_group.value(),
        vehicle_params,
        candidate.params );

      if( !candidate.shift_candidate.valid )
      {
        failure_reason = "candidate rejected by drivable-area check";
        return CandidatePreparationFailure::DrivableArea;
      }

      candidate.mode =
        candidate.shift_candidate.in_lane
          ? ObstacleAvoidanceMode::InLaneShift
          : ( candidate.shift_candidate.shift >= 0.0
                ? ObstacleAvoidanceMode::OvertakeLeft
                : ObstacleAvoidanceMode::OvertakeRight );

      // Candidate mode evaluation already guarantees that InLane stays in the
      // current lane, AdjacentSameDirection uses only same-direction lanes and
      // ConnectedIntersection actually relies on a geometrically connected
      // cross-road driving surface. Compute the opposite-lane interval exactly
      // once for the explicit opposite-direction candidate.
      candidate.uses_opposite_lane =
        candidate.shift_candidate.type ==
          AvoidanceCandidateType::OppositeDirection;
      if( candidate.uses_opposite_lane )
      {
        candidate.opposite_conflict_interval =
          compute_opposite_lane_conflict_interval(
            route,
            obstacle_group.value(),
            candidate.shift_candidate.shift,
            vehicle_params,
            candidate.params );
      }

      candidate.modified_route =
        build_modified_avoidance_route(
          route,
          obstacle_group.value(),
          candidate.shift_candidate.shift,
          ego_s_original,
          vehicle_params,
          candidate.params );

      candidate.ego_s_modified =
        adore::map::get_s_on_reference_line_segments(
          candidate.modified_route,
          ego,
          ego_s_original,
          candidate.params.route_window_min );

      if( !std::isfinite( candidate.ego_s_modified ) )
      {
        candidate.ego_s_modified = ego_s_original;
      }

      auto candidate_planner = planner;
      try
      {
        candidate.trajectory =
          candidate_planner.plan_route_trajectory_from_s(
            candidate.modified_route,
            ego,
            traffic_participants,
            candidate.ego_s_modified );
      }
      catch( const std::exception& e )
      {
        failure_reason =
          std::string( "candidate rejected: planner exception: " ) + e.what();
        return CandidatePreparationFailure::Planning;
      }

      if( candidate.trajectory.states.empty() )
      {
        failure_reason = "candidate rejected: planner returned empty trajectory";
        return CandidatePreparationFailure::Planning;
      }

      candidate.validation =
        validate_planned_shift_trajectory(
          route,
          candidate.trajectory,
          obstacle_group.value(),
          candidate.shift_candidate.shift,
          candidate.shift_candidate.in_lane,
          candidate.shift_candidate.type,
          vehicle_params,
          candidate.params,
          params.side_clearance,
          ego_s_original );

      return CandidatePreparationFailure::None;
    };

  auto record_preparation_failure =
    [&]( const RouteShiftPlanCandidate& candidate,
         CandidatePreparationFailure failure,
         const std::string& reason )
    {
      const std::string described =
        describe_candidate_rejection( candidate, reason );
      switch( failure )
      {
        case CandidatePreparationFailure::DrivableArea:
          last_drivable_area_rejection = described;
          break;
        case CandidatePreparationFailure::Planning:
          last_planning_rejection = described;
          break;
        case CandidatePreparationFailure::None:
          break;
      }
    };

  for( const auto& raw_shift : shift_variants )
  {
    RouteShiftPlanCandidate candidate;
    candidate.shift_candidate = raw_shift;
    candidate.params = params;
    std::string preparation_reason;
    const auto preparation_failure =
      prepare_candidate( candidate, preparation_reason );

    if( preparation_failure != CandidatePreparationFailure::None )
    {
      record_preparation_failure(
        candidate,
        preparation_failure,
        preparation_reason );
      continue;
    }

    // The route targets side_clearance, while the actual trajectory only has
    // to stay outside ego_corridor_safety_margin. If trajectory generation
    // undershoots the target, make one feedback-driven attempt with exactly
    // the measured deficit added to the route target. This is adaptive per
    // candidate and introduces no fixed tracking margin. If the wider route
    // is not drivable, retain the original candidate when it still satisfies
    // the hard trajectory corridor.
    const double target_clearance_deficit =
      std::isfinite( candidate.validation.min_obstacle_lateral_margin )
        ? std::max(
            0.0,
            -candidate.validation.min_obstacle_lateral_margin )
        : 0.0;
    const bool can_refine_for_clearance =
      target_clearance_deficit > 0.0 &&
      ( candidate.validation.valid ||
        candidate.validation.obstacle_clearance_violation );

    if( can_refine_for_clearance )
    {
      RouteShiftPlanCandidate refined;
      refined.shift_candidate = raw_shift;
      refined.shift_candidate.shift =
        candidate.shift_candidate.shift +
        std::copysign(
          target_clearance_deficit,
          candidate.shift_candidate.shift );
      refined.params = params;
      refined.params.side_clearance =
        std::max( 0.0, params.side_clearance ) +
        target_clearance_deficit;

      std::string refined_failure_reason;
      const auto refined_failure =
        prepare_candidate( refined, refined_failure_reason );

      if( refined_failure == CandidatePreparationFailure::None )
      {
        if( refined.validation.valid || !candidate.validation.valid )
        {
          candidate = std::move( refined );
        }
      }
      else
      {
        record_preparation_failure(
          refined,
          refined_failure,
          "adaptive clearance refinement failed: " +
            refined_failure_reason );
      }
    }

    if( !candidate.validation.valid )
    {
      last_validation_rejection =
        describe_candidate_rejection(
          candidate,
          candidate.validation.reason );
      continue;
    }

    saw_validated_candidate = true;

    const auto candidate_route_safety =
      check_route_corridor_safety(
        candidate.modified_route,
        ego,
        traffic_participants,
        vehicle_params,
        candidate.params,
        &candidate.trajectory,
        static_candidate_check_end_s );

    const auto is_ignorable_route_conflict =
      [&]( const RouteCorridorConflict& conflict )
      {
        return candidate_route_conflict_is_ignorable(
          conflict,
          candidate.shift_candidate.type,
          candidate.uses_opposite_lane,
          vehicle_params,
          candidate.params );
      };

    bool has_relevant_route_conflict = false;
    if( !candidate_route_safety.conflicts.empty() )
    {
      has_relevant_route_conflict =
        std::any_of(
          candidate_route_safety.conflicts.begin(),
          candidate_route_safety.conflicts.end(),
          [&]( const RouteCorridorConflict& conflict )
          {
            return !is_ignorable_route_conflict( conflict );
          } );
    }
    else if( candidate_route_safety.has_conflict )
    {
      // Projection and other structural failures report a single conflict
      // without populating the vector and must never be filtered away.
      has_relevant_route_conflict =
        !is_ignorable_route_conflict(
          candidate_route_safety.conflict );
    }

    if( has_relevant_route_conflict )
    {
      last_safety_rejection =
        describe_candidate_rejection(
          candidate,
          "candidate rejected by final active-route safety check: " +
            candidate_route_safety.reason );

      continue;
    }

    if( candidate.uses_opposite_lane )
    {
      const auto oncoming =
        check_oncoming_gap(
          route,
          candidate.modified_route,
          ego,
          traffic_participants,
          obstacle_group.value(),
          candidate.shift_candidate.shift,
          vehicle_params,
          &candidate.trajectory,
          candidate.params );

      if( oncoming.conflict )
      {
        saw_oncoming_conflict = true;
        last_oncoming_rejection =
          describe_candidate_rejection(
            candidate,
            oncoming.reason );
        continue;
      }
    }

    accepted_candidates.push_back( candidate );
  }

  if( accepted_candidates.empty() )
  {
    if( saw_validated_candidate && saw_oncoming_conflict )
    {
      std::string reason =
        "driving mission (waiting before obstacle: oncoming traffic conflict)";

      if( !last_oncoming_rejection.empty() )
      {
        reason += ": " + last_oncoming_rejection;
      }

      return plan_stop_before_obstacle(
        ObstacleAvoidanceMode::WaitForOncoming,
        reason );
    }

    std::string reason =
      "driving mission (stop before obstacle: no validated route-shift candidate)";

    // Diagnostic: surface the obstacle geometry and EVERY rejection category (not
    // just the first) so a stop can be told apart: the required shift capped by the
    // drivable-area check vs a trajectory / oncoming failure. type: 0=InLane,
    // 1=Adjacent, 2=ConnectedIntersection, 3=Opposite
    // (AvoidanceCandidateType order).
    {
      const auto& diag_group = obstacle_group.value();
      char group_buf[320];
      std::snprintf(
        group_buf, sizeof( group_buf ),
        " obstacle{s=[%.1f,%.1f] l=[%.2f,%.2f]}",
        diag_group.envelope.object_s_min, diag_group.envelope.object_s_max,
        diag_group.envelope.object_l_min, diag_group.envelope.object_l_max );
      reason += group_buf;

      if( !last_drivable_area_rejection.empty() )
        reason += " | DRIVABLE: " + last_drivable_area_rejection;
      if( !last_validation_rejection.empty() )
        reason += " | VALIDATION: " + last_validation_rejection;
      if( !last_safety_rejection.empty() )
        reason += " | SAFETY: " + last_safety_rejection;
      if( !last_oncoming_rejection.empty() )
        reason += " | ONCOMING: " + last_oncoming_rejection;
      if( !last_planning_rejection.empty() )
        reason += " | PLANNING: " + last_planning_rejection;
    }

    return plan_stop_before_obstacle(
      ObstacleAvoidanceMode::StopBeforeObstacle,
      reason );
  }

  const auto best_it =
    std::min_element(
      accepted_candidates.begin(),
      accepted_candidates.end(),
      [&]( const RouteShiftPlanCandidate& a,
           const RouteShiftPlanCandidate& b )
      {
        const auto mode_priority =
          []( const RouteShiftPlanCandidate& candidate )
          {
            if( candidate.shift_candidate.in_lane )
            {
              return 0;
            }
            if( candidate.shift_candidate.type ==
                AvoidanceCandidateType::AdjacentSameDirection )
            {
              return 1;
            }
            if( candidate.shift_candidate.type ==
                AvoidanceCandidateType::ConnectedIntersection )
            {
              return 2;
            }
            return 3;
          };

        const int a_priority = mode_priority( a );
        const int b_priority = mode_priority( b );
        if( a_priority != b_priority )
        {
          return a_priority < b_priority;
        }

        const double a_shift = std::fabs( a.shift_candidate.shift );
        const double b_shift = std::fabs( b.shift_candidate.shift );
        if( std::fabs( a_shift - b_shift ) > 1e-6 )
        {
          return a_shift < b_shift;
        }

        if( std::fabs( a.shift_candidate.shift - b.shift_candidate.shift ) < 1e-6 )
        {
          return false;
        }

        if( params.prefer_left_shift )
        {
          return a.shift_candidate.shift > b.shift_candidate.shift;
        }

        return a.shift_candidate.shift < b.shift_candidate.shift;
      } );

  const auto selected = *best_it;

  result.mode = selected.mode;
  result.modified_route = selected.modified_route;
  result.lateral_shift = selected.shift_candidate.shift;
  result.avoidance_speed =
    selected.params.max_speed_during_avoidance;
  result.in_lane = selected.shift_candidate.in_lane;
  result.trajectory = selected.trajectory;

  // No re-validation of the selected candidate here: only candidates that already
  // passed validate_planned_shift_trajectory, check_route_corridor_safety and (for
  // opposite-lane) check_oncoming_gap in the candidate loop above enter
  // accepted_candidates, and result.{trajectory,modified_route,lateral_shift} are
  // verbatim copies of the selected candidate's. Re-running those checks with the
  // identical inputs cannot change the outcome, so the former final_validation /
  // final_route_safety / final_oncoming_check block was pure duplication.

  if( result.mode == ObstacleAvoidanceMode::InLaneShift )
  {
    result.trajectory.label = "driving mission (avoiding obstacle)";
  }
  else if( result.mode == ObstacleAvoidanceMode::OvertakeLeft )
  {
    result.trajectory.label = "driving mission (avoiding obstacle left)";
  }
  else
  {
    result.trajectory.label = "driving mission (avoiding obstacle right)";
  }

  result.success = true;
  if( selected.params.side_clearance >
      std::max( 0.0, params.side_clearance ) )
  {
    char buf[256];
    std::snprintf(
      buf,
      sizeof( buf ),
      "planned validated obstacle avoidance with adaptive route-clearance refinement (configured=%.2f refined=%.2f)",
      std::max( 0.0, params.side_clearance ),
      selected.params.side_clearance );
    result.reason = buf;
  }
  else
  {
    result.reason =
      "planned validated obstacle avoidance by selecting a route-shift candidate";
  }

  // Freeze the per-object shift inputs so the persistent maneuver can rebuild
  // the modified route as each object's own curve. obstacle_group->obstacles is
  // already ordered by object_s_min (find_static_obstacle_group_on_route), which the
  // gap-bridging in avoidance_shift_offset_at_s relies on.
  result.shift_contributions.clear();
  result.shift_contributions.reserve( obstacle_group->obstacles.size() );
  const double contribution_ego_half_length =
    symmetric_shift_ego_half_length( vehicle_params );
  for( const auto& obstacle : obstacle_group->obstacles )
  {
    AvoidanceShiftContribution contribution;
    contribution.participant_ids = obstacle.participant_ids;
    contribution.object_s_min = obstacle.object_s_min;
    contribution.object_s_max = obstacle.object_s_max;
    contribution.object_l_min = obstacle.object_l_min;
    contribution.object_l_max = obstacle.object_l_max;
    contribution.has_persistent_profile = true;
    contribution.signed_shift =
      required_signed_shift_for_obstacle(
        obstacle,
        result.lateral_shift,
        vehicle_params,
        selected.params );

    if( obstacle.has_persistent_profile )
    {
      contribution.ramp_start_s = obstacle.persistent_ramp_start_s;
      contribution.full_shift_start_s =
        obstacle.persistent_full_shift_start_s;
      contribution.full_shift_end_s =
        obstacle.persistent_full_shift_end_s;
      contribution.ramp_end_s = obstacle.persistent_ramp_end_s;
    }
    else
    {
      contribution.full_shift_start_s =
        obstacle.object_s_min - contribution_ego_half_length;
      contribution.full_shift_end_s =
        obstacle.object_s_max + contribution_ego_half_length;
      contribution.ramp_start_s =
        std::max(
          0.0,
          contribution.full_shift_start_s -
            std::max( 0.0, selected.params.front_clearance ) );
      contribution.ramp_end_s =
        contribution.full_shift_end_s +
        std::max( 0.0, selected.params.rear_clearance );
    }

    result.shift_contributions.push_back( contribution );
  }

  // Maneuver bounds are already known exactly from the shift contributions.
  // Recovering them by comparing modified and mission-route points required an
  // arbitrary XY difference threshold and made the lifecycle depend on route
  // discretization.
  double maneuver_start_s = std::numeric_limits<double>::infinity();
  double maneuver_end_s = -std::numeric_limits<double>::infinity();
  for( const auto& contribution : result.shift_contributions )
  {
    if( !std::isfinite( contribution.ramp_start_s ) ||
        !std::isfinite( contribution.ramp_end_s ) ||
        std::fabs( contribution.signed_shift ) <= 1e-6 )
    {
      continue;
    }

    maneuver_start_s =
      std::min( maneuver_start_s, contribution.ramp_start_s );
    maneuver_end_s =
      std::max( maneuver_end_s, contribution.ramp_end_s );
  }

  if( std::isfinite( maneuver_start_s ) &&
      std::isfinite( maneuver_end_s ) &&
      maneuver_end_s >= maneuver_start_s &&
      !route.reference_line.empty() )
  {
    const double route_first_s = route.reference_line.begin()->first;
    const double route_last_s = route.reference_line.rbegin()->first;

    result.has_maneuver_bounds = true;
    result.shift_start_s =
      std::clamp( maneuver_start_s, route_first_s, route_last_s );
    result.shift_end_s =
      std::clamp( maneuver_end_s, result.shift_start_s, route_last_s );

    // The full-shift plateau begins at the obstacle; before that point a new
    // oncoming conflict can still abort the maneuver.
    const double plateau_start_s =
      std::max( 0.0, obstacle_group->envelope.object_s_min );

    result.obstacle_id = obstacle_group->envelope.id;
    result.obstacle_ids = obstacle_group->envelope.participant_ids;
    result.obstacle_s_min = obstacle_group->envelope.object_s_min;

    result.maneuver.active = true;
    result.maneuver.mode = result.mode;
    result.maneuver.obstacle_id = result.obstacle_id;
    result.maneuver.obstacle_ids = result.obstacle_ids;
    result.maneuver.shift_start_s = result.shift_start_s;
    result.maneuver.shift_end_s = result.shift_end_s;
    result.maneuver.release_s = result.shift_end_s;
    result.maneuver.lateral_shift = result.lateral_shift;
    result.maneuver.in_lane = result.in_lane;
    result.maneuver.uses_opposite_lane = selected.uses_opposite_lane;
    result.maneuver.has_opposite_lane_conflict_interval =
      selected.opposite_conflict_interval.valid &&
      selected.opposite_conflict_interval.occupies_opposite_lane;
    result.maneuver.opposite_lane_conflict_start_s =
      selected.opposite_conflict_interval.start_s;
    result.maneuver.opposite_lane_conflict_end_s =
      selected.opposite_conflict_interval.end_s;
    result.maneuver.commitment_s =
      result.maneuver.has_opposite_lane_conflict_interval
        ? result.maneuver.opposite_lane_conflict_start_s
        : plateau_start_s;
  }

  return result;
}

EgoLaneOncomingStopResult
try_plan_ego_lane_oncoming_stop( TrajectoryPlanner& planner,
                                 const map::Route& route,
                                 const dynamics::VehicleStateDynamic& ego,
                                 const dynamics::TrafficParticipantSet& traffic_participants,
                                 const ObstacleAvoidanceParams& params )
{
  EgoLaneOncomingStopResult result;
  result.modified_route = route;

  const auto vehicle_params = planner.get_physical_vehicle_parameters();

  const auto threat = find_ego_lane_oncoming_threat(
    route,
    ego,
    traffic_participants,
    vehicle_params,
    params );

  if( !threat.has_value() )
  {
    return result;
  }

  ObstacleEnvelope pseudo_obstacle;
  pseudo_obstacle.id = threat->participant_id;
  pseudo_obstacle.object_s_min = threat->participant_near_s;
  pseudo_obstacle.object_s_max = threat->participant_near_s;
  result.modified_route = build_stop_route_before_obstacle(
    route,
    pseudo_obstacle,
    ego,
    vehicle_params,
    params.ego_lane_oncoming_stop_distance,
    params );

  try
  {
    result.trajectory = planner.plan_route_trajectory(
      result.modified_route,
      ego,
      traffic_participants );
  }
  catch( const std::exception& )
  {
  }

  result.trajectory.label = "driving mission (waiting for oncoming)";

  result.success = true;
  return result;
}

RouteCorridorCheckResult
check_route_corridor_safety(
  const map::Route& route_to_check,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::TrafficParticipantSet& traffic_participants,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params,
  const dynamics::Trajectory* ego_trajectory,
  double static_check_end_s )
{
  RouteCorridorCheckResult result;
  result.reason = "route corridor not evaluated";

  const double ego_s = project_s_on_reference_line( route_to_check, ego );
  result.ego_s = ego_s;

  if( !std::isfinite( ego_s ) )
  {
    result.safe = false;
    result.has_conflict = true;
    result.reason = "invalid ego projection on route being checked";
    result.conflict.reason = result.reason;
    return result;
  }

  const double ego_half_width = 0.5 * ego_params.body_width;
  const double corridor_half_width =
    ego_half_width + std::max( 0.0, params.ego_corridor_safety_margin );
  const double ego_front_s =
    ego_s + ego_params.wheelbase + ego_params.front_axle_to_front_border;
  const double ego_rear_s =
    ego_s - ego_params.rear_border_to_rear_axle;
  // Moving conflicts are actionable through braking only while they still reach
  // ahead of ego's front bumper. A static intrusion is different: while it overlaps
  // ego longitudinally, perception may still reveal a longer/wider contour and the
  // active avoidance route can still be reshaped. Therefore static objects are
  // scanned from ego's rear border, while fully cleared objects remain excluded.
  // rear_clearance is deliberately not used here; it is a shift ramp-down length,
  // not an object relevance range.
  const double moving_check_start_s = ego_front_s;
  const double static_check_start_s = ego_rear_s;
  // Moving objects keep the complete remaining route/prediction check. Static
  // objects may optionally be bounded by a braking-derived action horizon so a
  // receding-horizon caller can defer an independent later maneuver safely.
  const double route_check_end_s =
    route_to_check.reference_line.empty()
      ? ego_s
      : route_to_check.reference_line.rbegin()->first;
  const double bounded_static_check_end_s =
    std::isfinite( static_check_end_s ) &&
    static_check_end_s >= ego_front_s
      ? std::min(
          route_check_end_s,
          static_check_end_s )
      : route_check_end_s;
  const double corridor_l_min = -corridor_half_width;
  const double corridor_l_max = corridor_half_width;

  std::vector<RouteCorridorConflict> conflicts;

  for( const auto& [id, participant] : traffic_participants.participants )
  {
    const int participant_id = static_cast<int>( id );
    const auto footprint =
      project_participant_footprint_to_route( route_to_check, participant, params );

    if( !footprint.has_value() )
    {
      continue;
    }

    const auto route_pose =
      route_to_check.get_pose_at_s( footprint->center_s );
    const double yaw_diff =
      normalize_angle( participant.state.yaw_angle - route_pose.yaw );
    const double v_route =
      participant.state.vx * std::cos( yaw_diff ) -
      participant.state.vy * std::sin( yaw_diff );

    RouteCorridorObjectClass object_class =
      RouteCorridorObjectClass::CrossingOrUnknown;

    if( participant_is_static_for_avoidance( participant, params ) )
    {
      object_class = RouteCorridorObjectClass::StaticOrSlow;
    }
    else if( v_route <= -params.min_oncoming_route_speed ||
             std::fabs( yaw_diff ) >= params.min_oncoming_heading_diff )
    {
      object_class = RouteCorridorObjectClass::Oncoming;
    }
    else if( v_route >= params.min_oncoming_route_speed )
    {
      object_class = RouteCorridorObjectClass::SameDirection;
    }

    const double object_check_start_s =
      object_class == RouteCorridorObjectClass::StaticOrSlow
        ? static_check_start_s
        : moving_check_start_s;
    const double object_check_end_s =
      object_class == RouteCorridorObjectClass::StaticOrSlow
        ? bounded_static_check_end_s
        : route_check_end_s;

    if( footprint->s_max < object_check_start_s ||
        footprint->s_min > object_check_end_s )
    {
      continue;
    }

    const bool currently_overlaps_route_corridor =
      footprint->l_max >= corridor_l_min &&
      footprint->l_min <= corridor_l_max &&
      footprint->s_max >= object_check_start_s;
    const bool currently_overlaps_ego_footprint =
      footprint->l_max >= -ego_half_width &&
      footprint->l_min <= ego_half_width &&
      footprint->s_max >= ego_rear_s &&
      footprint->s_min <= ego_front_s;

    double ttc = std::numeric_limits<double>::infinity();
    const char* ttc_source = "none";
    bool predicted_enters_corridor = false;
    auto conflict_footprint = footprint.value();

    if( currently_overlaps_ego_footprint )
    {
      ttc = 0.0;
      ttc_source = "ego_footprint_overlap";
    }
    else if( currently_overlaps_route_corridor )
    {
      const double ego_speed_for_ttc = std::max( 0.0, ego.vx );
      const double distance_to_object =
        std::max( 0.0, footprint->s_min - ego_front_s );
      ttc =
        distance_to_object <= 0.0
          ? 0.0
          : ( ego_speed_for_ttc > 0.0
                ? distance_to_object / ego_speed_for_ttc
                : std::numeric_limits<double>::infinity() );
      ttc_source = "route_corridor_ahead";
    }

    auto predict_participant_at_time =
      [&]( double query_time ) -> dynamics::TrafficParticipant
      {
        auto predicted_participant = participant;

        if( participant.trajectory.has_value() &&
            !participant.trajectory->states.empty() )
        {
          const auto& states = participant.trajectory->states;
          auto best_it = states.begin();
          double best_dt = std::fabs( best_it->time - query_time );

          for( auto it = std::next( states.begin() ); it != states.end(); ++it )
          {
            const double dt = std::fabs( it->time - query_time );
            if( dt < best_dt )
            {
              best_it = it;
              best_dt = dt;
            }
          }

          predicted_participant.state = *best_it;
          return predicted_participant;
        }

        const double dt =
          std::max( 0.0, query_time - ego.time );
        const double cos_yaw = std::cos( participant.state.yaw_angle );
        const double sin_yaw = std::sin( participant.state.yaw_angle );
        const double world_vx =
          cos_yaw * participant.state.vx -
          sin_yaw * participant.state.vy;
        const double world_vy =
          sin_yaw * participant.state.vx +
          cos_yaw * participant.state.vy;
        predicted_participant.state.x =
          participant.state.x + world_vx * dt;
        predicted_participant.state.y =
          participant.state.y + world_vy * dt;
        predicted_participant.state.time = query_time;
        return predicted_participant;
      };

    const bool has_usable_ego_trajectory =
      ego_trajectory != nullptr &&
      ego_trajectory->states.size() >= 2;

    if( has_usable_ego_trajectory )
    {
      for( std::size_t ego_state_index = 0;
           ego_state_index < ego_trajectory->states.size();
           ++ego_state_index )
      {
        const auto& ego_state =
          ego_trajectory->states[ego_state_index];
        const double predicted_t = ego_state.time - ego.time;
        if( predicted_t < 0.0 ||
            predicted_t > params.prediction_time_horizon )
        {
          continue;
        }

        const double predicted_ego_s =
          project_s_on_reference_line( route_to_check, ego_state, ego_s );
        if( !std::isfinite( predicted_ego_s ) )
        {
          continue;
        }

        const double predicted_ego_front_s =
          predicted_ego_s + ego_params.wheelbase + ego_params.front_axle_to_front_border;
        const double predicted_ego_rear_s =
          predicted_ego_s - ego_params.rear_border_to_rear_axle;

        const auto predicted_participant =
          predict_participant_at_time( ego_state.time );

        const auto predicted_footprint =
          project_participant_footprint_to_route(
            route_to_check,
            predicted_participant,
            params );

        if( !predicted_footprint.has_value() )
        {
          continue;
        }

        // Compensate the discrete prediction raster by the distance ego can
        // travel to the adjacent trajectory sample. Deriving this from the
        // actual trajectory removes the old hidden fixed geometry margin.
        double sample_time_step = 0.0;
        if( ego_state_index + 1 < ego_trajectory->states.size() )
        {
          sample_time_step =
            ego_trajectory->states[ego_state_index + 1].time -
            ego_state.time;
        }
        else if( ego_state_index > 0 )
        {
          sample_time_step =
            ego_state.time -
            ego_trajectory->states[ego_state_index - 1].time;
        }
        const double predicted_overlap_front_margin =
          std::max( 0.0, ego_state.vx ) *
          std::max( 0.0, sample_time_step );

        const bool predicted_overlap =
          predicted_footprint->s_max >=
            predicted_ego_rear_s &&
          predicted_footprint->s_min <=
            predicted_ego_front_s + predicted_overlap_front_margin &&
          predicted_footprint->l_max >= corridor_l_min &&
          predicted_footprint->l_min <= corridor_l_max;

        if( predicted_overlap && predicted_t < ttc )
        {
          ttc = predicted_t;
          ttc_source =
            participant.trajectory.has_value()
              ? "ego_and_participant_trajectory"
              : "ego_trajectory_constant_velocity";
          predicted_enters_corridor = true;
          conflict_footprint = predicted_footprint.value();
        }
      }
    }
    else if( !currently_overlaps_route_corridor &&
             participant.trajectory.has_value() &&
             participant.trajectory->states.size() >= 2 )
    {
      for( const auto& predicted_state : participant.trajectory->states )
      {
        const double predicted_t = predicted_state.time - ego.time;
        if( predicted_t < 0.0 ||
            predicted_t > params.prediction_time_horizon )
        {
          continue;
        }

        auto predicted_participant = participant;
        predicted_participant.state = predicted_state;

        const auto predicted_footprint =
          project_participant_footprint_to_route(
            route_to_check,
            predicted_participant,
            params );

        if( !predicted_footprint.has_value() )
        {
          continue;
        }

        const bool predicted_overlap =
          predicted_footprint->s_max >= object_check_start_s &&
          predicted_footprint->s_min <= object_check_end_s &&
          predicted_footprint->l_max >= corridor_l_min &&
          predicted_footprint->l_min <= corridor_l_max;

        if( predicted_overlap && predicted_t < ttc )
        {
          ttc = predicted_t;
          ttc_source = "participant_trajectory_corridor";
          predicted_enters_corridor = true;
          conflict_footprint = predicted_footprint.value();
        }
      }
    }

    if( !has_usable_ego_trajectory &&
        !currently_overlaps_route_corridor &&
        std::isfinite( v_route ) &&
        v_route < -params.min_oncoming_route_speed &&
        footprint->s_min > ego_s )
    {
      const double closing_speed =
        std::max( params.min_oncoming_route_speed, std::fabs( v_route ) );
      const double constant_velocity_ttc =
        std::max( 0.0, footprint->s_min - ego_s ) / closing_speed;
      if( constant_velocity_ttc < ttc )
      {
        ttc = constant_velocity_ttc;
        ttc_source = "constant_velocity";
        predicted_enters_corridor = true;
      }
    }

    // Without an ego trajectory, conservatively report a participant that will
    // enter any part of the checked route corridor. When an ego trajectory exists,
    // the loop above has already evaluated both actors at matching times; a second
    // participant-only pass would discard temporal separation and turn a proven
    // non-conflict back into a false positive.
    if( !has_usable_ego_trajectory &&
        !currently_overlaps_route_corridor &&
        !predicted_enters_corridor )
    {
      const double horizon =
        std::max( 0.0, params.prediction_time_horizon );
      const double step = obstacle_avoidance_cycle_time_s;

      for( double predicted_t = step;
           predicted_t <= horizon + 1e-6;
           predicted_t += step )
      {
        const auto predicted_participant =
          predict_participant_at_time( ego.time + predicted_t );
        const auto predicted_footprint =
          project_participant_footprint_to_route(
            route_to_check,
            predicted_participant,
            params );

        if( !predicted_footprint.has_value() )
        {
          continue;
        }

        const bool predicted_overlap =
          predicted_footprint->s_max >= object_check_start_s &&
          predicted_footprint->s_min <= object_check_end_s &&
          predicted_footprint->l_max >= corridor_l_min &&
          predicted_footprint->l_min <= corridor_l_max;

        if( predicted_overlap )
        {
          ttc = predicted_t;
          ttc_source =
            participant.trajectory.has_value()
              ? "participant_trajectory_predicted_corridor"
              : "constant_velocity_predicted_corridor";
          predicted_enters_corridor = true;
          conflict_footprint = predicted_footprint.value();
          break;
        }
      }
    }

    const bool predictive_conflict =
      std::isfinite( ttc ) &&
      ttc <= params.prediction_time_horizon &&
      ( predicted_enters_corridor ||
        ( footprint->l_max >= corridor_l_min &&
          footprint->l_min <= corridor_l_max ) );

    const double actual_lateral_clearance =
      actual_lateral_clearance_to_centered_ego(
        footprint->l_min,
        footprint->l_max,
        ego_half_width );
    const double desired_clearance_replan_trigger =
      std::max(
        std::max( 0.0, params.ego_corridor_safety_margin ),
        std::max( 0.0, params.side_clearance ) -
          std::max( 0.0, params.side_clearance_replan_tolerance ) );
    const bool desired_static_clearance_shortfall =
      object_class == RouteCorridorObjectClass::StaticOrSlow &&
      !currently_overlaps_route_corridor &&
      !predictive_conflict &&
      actual_lateral_clearance <
        desired_clearance_replan_trigger;

    if( !currently_overlaps_route_corridor &&
        !predictive_conflict &&
        !desired_static_clearance_shortfall )
    {
      continue;
    }

    RouteCorridorConflict conflict;
    conflict.participant_id = static_cast<int>( id );
    conflict.object_class = object_class;
    const auto& reported_footprint =
      desired_static_clearance_shortfall
        ? footprint.value()
        : conflict_footprint;
    conflict.object_s_min = reported_footprint.s_min;
    conflict.object_s_max = reported_footprint.s_max;
    conflict.object_l_min = reported_footprint.l_min;
    conflict.object_l_max = reported_footprint.l_max;
    conflict.distance_s = std::max( 0.0, reported_footprint.s_min - ego_s );
    conflict.time_to_conflict = ttc;
    conflict.actual_lateral_clearance =
      actual_lateral_clearance_to_centered_ego(
        reported_footprint.l_min,
        reported_footprint.l_max,
        ego_half_width );
    conflict.currently_overlaps_route_corridor = currently_overlaps_route_corridor;
    conflict.currently_overlaps_ego_footprint = currently_overlaps_ego_footprint;
    conflict.predicted_spatiotemporal_conflict = predicted_enters_corridor;
    char buf[384];
    if( desired_static_clearance_shortfall )
    {
      std::snprintf(
        buf,
        sizeof( buf ),
        "static desired-clearance shortfall actual=%.2f target=%.2f s=[%.2f,%.2f] l=[%.2f,%.2f]",
        conflict.actual_lateral_clearance,
        std::max( 0.0, params.side_clearance ),
        conflict.object_s_min,
        conflict.object_s_max,
        conflict.object_l_min,
        conflict.object_l_max );
    }
    else
    {
      std::snprintf(
        buf,
        sizeof( buf ),
        "%s conflict source=%s s=[%.2f,%.2f] l=[%.2f,%.2f]",
        currently_overlaps_ego_footprint
          ? "ego footprint overlap"
          : ( currently_overlaps_route_corridor
                ? "route corridor ahead"
                : "predicted spatiotemporal" ),
        ttc_source,
        conflict.object_s_min,
        conflict.object_s_max,
        conflict.object_l_min,
        conflict.object_l_max );
    }
    conflict.reason = buf;

    if( desired_static_clearance_shortfall )
    {
      result.static_clearance_improvements.push_back( conflict );
    }
    else
    {
      conflicts.push_back( conflict );
    }
  }

  if( conflicts.empty() )
  {
    result.safe = true;
    result.has_conflict = false;
    result.reason =
      result.static_clearance_improvements.empty()
        ? "no conflict in route corridor"
        : "no hard conflict; static objects fall short of desired side clearance";
    return result;
  }

  const auto best_it =
    std::min_element(
      conflicts.begin(),
      conflicts.end(),
      []( const RouteCorridorConflict& a,
          const RouteCorridorConflict& b )
      {
        if( a.currently_overlaps_ego_footprint != b.currently_overlaps_ego_footprint )
        {
          return a.currently_overlaps_ego_footprint;
        }
        if( a.predicted_spatiotemporal_conflict != b.predicted_spatiotemporal_conflict )
        {
          return a.predicted_spatiotemporal_conflict;
        }
        if( a.currently_overlaps_route_corridor != b.currently_overlaps_route_corridor )
        {
          return a.currently_overlaps_route_corridor;
        }
        if( std::fabs( a.time_to_conflict - b.time_to_conflict ) > 1e-6 )
        {
          return a.time_to_conflict < b.time_to_conflict;
        }
        return a.distance_s < b.distance_s;
      } );

  result.safe = false;
  result.has_conflict = true;
  result.conflict = *best_it;
  result.conflicts = conflicts;

  char buf[512];
  std::snprintf(
    buf,
    sizeof( buf ),
    "conflict id=%d class=%s s=[%.2f,%.2f] l=[%.2f,%.2f] ttc=%.2f reason=%s",
    result.conflict.participant_id,
    route_corridor_object_class_name( result.conflict.object_class ),
    result.conflict.object_s_min,
    result.conflict.object_s_max,
    result.conflict.object_l_min,
    result.conflict.object_l_max,
    result.conflict.time_to_conflict,
    result.conflict.reason.c_str() );
  result.reason = buf;

  return result;
}

bool
trajectory_stops_by_route_s(
  const dynamics::Trajectory& trajectory,
  const map::Route& route,
  double required_stop_s,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params,
  bool use_maximum_braking_deceleration )
{
  if( trajectory.states.empty() || !std::isfinite( required_stop_s ) )
  {
    return false;
  }

  const double stopped_speed =
    std::max( 0.0, params.stopped_ego_speed );
  bool saw_stopped_state_at_or_before_target = false;

  for( const auto& state : trajectory.states )
  {
    const double state_s =
      project_s_on_reference_line( route, state, required_stop_s );
    if( !std::isfinite( state_s ) )
    {
      continue;
    }

    const double speed = std::fabs( state.vx );
    if( state_s >= required_stop_s && speed > stopped_speed )
    {
      return false;
    }
    if( state_s <= required_stop_s && speed <= stopped_speed )
    {
      saw_stopped_state_at_or_before_target = true;
    }
  }

  const auto& final_state = trajectory.states.back();
  const double final_s =
    project_s_on_reference_line( route, final_state, required_stop_s );
  if( !std::isfinite( final_s ) )
  {
    return false;
  }

  if( final_s < required_stop_s )
  {
    const double a_abs =
      use_maximum_braking_deceleration
        ? maximum_braking_deceleration( ego_params, params )
        : planned_braking_deceleration( ego_params, params );
    const double remaining_braking_distance =
      final_state.vx * final_state.vx / ( 2.0 * a_abs );

    return final_s + remaining_braking_distance <= required_stop_s;
  }

  return std::fabs( final_state.vx ) <= stopped_speed &&
         saw_stopped_state_at_or_before_target;
}

bool
trajectory_stops_before_conflict(
  const dynamics::Trajectory& trajectory,
  const map::Route& route,
  const RouteCorridorConflict& conflict,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params,
  bool use_maximum_braking_deceleration )
{
  if( trajectory.states.empty() )
  {
    return false;
  }

  const double ego_front_offset =
    ego_params.wheelbase + ego_params.front_axle_to_front_border;
  const double front_stop_s =
    conflict.object_s_min - std::max( 0.0, normalized_stop_before_obstacle( params ) );
  const double stop_s =
    front_stop_s - ego_front_offset;
  const double collision_entry_s = conflict.object_s_min;
  const double stopped_speed =
    std::max( 0.0, params.stopped_ego_speed );

  bool saw_state_before_conflict = false;
  double min_speed_before_conflict = std::numeric_limits<double>::infinity();

  for( const auto& state : trajectory.states )
  {
    const double state_s =
      project_s_on_reference_line( route, state, conflict.object_s_min );
    if( !std::isfinite( state_s ) )
    {
      continue;
    }

    const double ego_front_s = state_s + ego_front_offset;
    const double speed = std::fabs( state.vx );

    if( ego_front_s < collision_entry_s )
    {
      saw_state_before_conflict = true;
      min_speed_before_conflict =
        std::min( min_speed_before_conflict, speed );
    }

    if( ego_front_s >= collision_entry_s && speed > stopped_speed )
    {
      return false;
    }

    if( state_s >= stop_s && speed > stopped_speed )
    {
      return false;
    }
  }

  const auto& final_state = trajectory.states.back();
  const double final_s =
    project_s_on_reference_line( route, final_state, conflict.object_s_min );
  if( !std::isfinite( final_s ) )
  {
    return false;
  }

  const double final_front_s = final_s + ego_front_offset;
  if( final_s < stop_s )
  {
    // The trajectory horizon ends before the stop point. Accept it only if the
    // remaining speed can still be braked away before the stop point; a
    // trajectory that is merely "not there yet" but too fast does not stop.
    const double a_abs =
      use_maximum_braking_deceleration
        ? maximum_braking_deceleration( ego_params, params )
        : planned_braking_deceleration( ego_params, params );
    const double remaining_braking_distance =
      final_state.vx * final_state.vx / ( 2.0 * a_abs );

    return final_front_s < collision_entry_s &&
           final_s + remaining_braking_distance <= stop_s;
  }

  const bool final_before_conflict =
    final_front_s <= collision_entry_s;
  const bool final_stopped =
    std::fabs( final_state.vx ) <= stopped_speed;
  const bool stopped_before_conflict =
    saw_state_before_conflict &&
    min_speed_before_conflict <= stopped_speed;

  return final_before_conflict && final_stopped && stopped_before_conflict;
}


bool
participant_keeps_hard_clearance_to_route_corridor(
  const map::Route& route,
  const dynamics::TrafficParticipant& participant,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  const auto footprint =
    project_participant_footprint_to_route( route, participant, params );
  if( !footprint.has_value() )
  {
    return false;
  }

  const double ego_half_width = 0.5 * ego_params.body_width;
  return actual_lateral_clearance_to_centered_ego(
           footprint->l_min, footprint->l_max, ego_half_width )
         >= std::max( 0.0, params.ego_corridor_safety_margin );
}

ObstacleAvoidanceMonitorResult
monitor_active_obstacle_avoidance_maneuver(
  const map::Route& route,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::TrafficParticipantSet& traffic_participants,
  const ObstacleAvoidanceManeuver& maneuver,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params,
  const dynamics::Trajectory* candidate_ego_trajectory )
{
  ObstacleAvoidanceMonitorResult result;
  result.safe_to_continue = true;
  result.should_abort_before_commitment = false;
  result.reason = "active OA maneuver does not use an opposite-direction lane";

  if( !maneuver.active )
  {
    result.reason = "no active OA maneuver";
    return result;
  }

  if( !maneuver.uses_opposite_lane )
  {
    return result;
  }

  const double ego_s = project_s_on_reference_line( route, ego );
  if( !std::isfinite( ego_s ) )
  {
    result.safe_to_continue = false;
    result.should_abort_before_commitment = true;
    result.reason = "cannot monitor active OA maneuver: invalid ego route projection";
    result.oncoming.conflict = true;
    result.oncoming.reason = result.reason;
    return result;
  }

  result.already_committed =
    std::isfinite( maneuver.commitment_s ) &&
    ego_s >= maneuver.commitment_s;

  if( !maneuver.has_opposite_lane_conflict_interval )
  {
    result.safe_to_continue = result.already_committed;
    result.should_abort_before_commitment = !result.already_committed;
    result.reason =
      result.already_committed
        ? "opposite-lane maneuver is committed but has no monitorable conflict interval"
        : "opposite-lane maneuver has no monitorable conflict interval";
    result.oncoming.conflict = !result.already_committed;
    result.oncoming.reason = result.reason;
    return result;
  }

  result.oncoming.conflict_start_s =
    maneuver.opposite_lane_conflict_start_s;
  result.oncoming.conflict_end_s =
    maneuver.opposite_lane_conflict_end_s;

  double ego_speed = std::max( 0.0, ego.vx );
  if( params.max_speed_during_avoidance > 0.0 )
  {
    ego_speed = std::min( ego_speed, params.max_speed_during_avoidance );
  }
  if( candidate_ego_trajectory != nullptr )
  {
    const auto trajectory_clear_time =
      compute_ego_clear_time_from_trajectory(
        route,
        *candidate_ego_trajectory,
        ego.time,
        result.oncoming.conflict_end_s,
        ego_s,
        params.route_window_min );

    if( trajectory_clear_time.has_value() )
    {
      result.oncoming.ego_clear_time = trajectory_clear_time.value();
    }
  }

  if( result.oncoming.ego_clear_time <= 0.0 )
  {
    const double conflict_distance =
      std::max( 0.0, result.oncoming.conflict_end_s - ego_s );
    result.oncoming.ego_clear_time =
      conflict_distance <= 0.0
        ? 0.0
        : ( ego_speed > 0.0
              ? conflict_distance / ego_speed
              : std::numeric_limits<double>::infinity() );
  }

  for( const auto& [id, participant] : traffic_participants.participants )
  {
    const int participant_id = static_cast<int>( id );
    const double participant_speed =
      std::hypot( participant.state.vx, participant.state.vy );

    const bool expected_active_obstacle =
      ( participant_id == maneuver.obstacle_id ||
        contains_participant_id( maneuver.obstacle_ids, participant_id ) ) &&
      participant_speed <= params.max_static_object_speed;

    if( expected_active_obstacle )
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
    const double yaw_diff =
      normalize_angle( participant.state.yaw_angle - route_pose.yaw );
    const double v_route =
      participant.state.vx * std::cos( yaw_diff ) -
      participant.state.vy * std::sin( yaw_diff );
    const bool heading_opposite =
      std::fabs( yaw_diff ) >= params.min_oncoming_heading_diff;
    const auto participant_footprint =
      project_participant_footprint_to_route( route, participant, params );
    const bool participant_in_conflict_interval =
      participant_footprint.has_value()
        ? ( participant_footprint->s_max >= result.oncoming.conflict_start_s &&
            participant_footprint->s_min <= result.oncoming.conflict_end_s )
        : ( participant_s >= result.oncoming.conflict_start_s &&
            participant_s <= result.oncoming.conflict_end_s );
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
        // A fully stopped opposite-direction vehicle is handled geometrically.
        // If it stays outside the hard ego corridor of the current/modified
        // route, it leaves a safe path and does not block the maneuver. If it
        // intrudes, the behavior layer's normal static-object cascade replans
        // the shift or stops. Moving participants remain in the time-gap logic.
        if( participant_speed <= params.max_static_object_speed &&
            participant_keeps_hard_clearance_to_route_corridor(
              route, participant, ego_params, params ) )
        {
          continue;
        }

        result.oncoming.conflict = true;
        result.oncoming.participant_id = participant_id;
        result.oncoming.oncoming_arrival_time = 0.0;

        char buf[384];
        std::snprintf(
          buf,
          sizeof( buf ),
          "active OA monitor: participant id=%d currently occupies conflict interval as slow/stopped opposite-direction traffic, committed=%s",
          participant_id,
          result.already_committed ? "true" : "false" );
        result.reason = buf;
        result.oncoming.reason = buf;
        result.safe_to_continue = false;
        result.should_abort_before_commitment = !result.already_committed;

        return result;
      }

      // Slow oncoming traffic approaching the conflict interval: same gap
      // logic as in check_oncoming_gap, with a floored closing speed, so the
      // monitor does not ignore vehicles below min_oncoming_route_speed.
      if( heading_opposite &&
          participant_speed > params.max_static_object_speed &&
          participant_near_s > result.oncoming.conflict_end_s )
      {
        const double slow_closing_speed =
          std::max( params.min_oncoming_route_speed, std::fabs( v_route ) );
        const double slow_arrival_time =
          std::max( 0.0, participant_near_s - result.oncoming.conflict_end_s ) /
          slow_closing_speed;

        if( slow_arrival_time <=
            result.oncoming.ego_clear_time + params.oncoming_time_margin )
        {
          result.oncoming.conflict = true;
          result.oncoming.participant_id = participant_id;
          result.oncoming.oncoming_arrival_time = slow_arrival_time;

          char buf[384];
          std::snprintf(
            buf,
            sizeof( buf ),
            "active OA monitor: participant id=%d slow oncoming (v_route=%.2f) arrival_time=%.2f <= ego_clear_time=%.2f + margin=%.2f, committed=%s",
            participant_id,
            v_route,
            slow_arrival_time,
            result.oncoming.ego_clear_time,
            params.oncoming_time_margin,
            result.already_committed ? "true" : "false" );
          result.reason = buf;
          result.oncoming.reason = buf;
          result.safe_to_continue = false;
          result.should_abort_before_commitment = !result.already_committed;

          return result;
        }
      }

      continue;
    }

    const double oncoming_speed =
      std::max( params.min_oncoming_route_speed,
                std::fabs( v_route ) );

    double arrival_time = std::numeric_limits<double>::infinity();
    const char* arrival_source = "constant_velocity";

    if( participant_in_conflict_interval )
    {
      arrival_time = 0.0;
      arrival_source = "current_footprint";
    }
    else if( participant.trajectory.has_value() &&
        participant.trajectory->states.size() >= 2 )
    {
      const auto trajectory_arrival =
        compute_arrival_time_from_trajectory(
          route,
          participant.trajectory.value(),
          ego.time,
          result.oncoming.conflict_start_s,
          result.oncoming.conflict_end_s );

      if( trajectory_arrival.has_value() )
      {
        arrival_time = trajectory_arrival.value();
        arrival_source = "trajectory";
      }
    }

    if( !std::isfinite( arrival_time ) )
    {
      if( participant_near_s > result.oncoming.conflict_end_s )
      {
        arrival_time =
          ( participant_near_s - result.oncoming.conflict_end_s ) /
          oncoming_speed;
      }
      else if( participant_far_s < result.oncoming.conflict_start_s )
      {
        continue;
      }
      else
      {
        arrival_time = 0.0;
      }
    }

    const double required_arrival_time =
      result.oncoming.ego_clear_time + params.oncoming_time_margin;

    if( arrival_time <= required_arrival_time )
    {
      result.oncoming.conflict = true;
      result.oncoming.participant_id = participant_id;
      result.oncoming.oncoming_arrival_time = arrival_time;

      char buf[384];
      std::snprintf(
        buf,
        sizeof( buf ),
        "active OA monitor: participant id=%d arrival_time=%.2f (source=%s) <= ego_clear_time=%.2f + margin=%.2f, committed=%s",
        participant_id,
        arrival_time,
        arrival_source,
        result.oncoming.ego_clear_time,
        params.oncoming_time_margin,
        result.already_committed ? "true" : "false" );
      result.reason = buf;
      result.oncoming.reason = buf;
      result.safe_to_continue = false;
      result.should_abort_before_commitment = !result.already_committed;

      return result;
    }
  }

  result.oncoming.conflict = false;
  result.oncoming.reason = "active OA monitor: no oncoming conflict detected";
  result.reason = result.oncoming.reason;
  return result;
}

} // namespace planner
} // namespace adore
