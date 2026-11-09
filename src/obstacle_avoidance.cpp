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
#include <cstdio>
#include <exception>
#include <limits>
#include <optional>
#include <string>
#include <vector>
#include <cstddef>

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
  const ObstacleAvoidanceParams& params )
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
    std::max( params.min_braking_deceleration, std::fabs( vehicle_params.acceleration_min ) );
  std::size_t approach_points = 0;
  std::size_t capped_points = 0;

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
        std::sqrt(
          target_v * target_v +
          2.0 * a_abs * std::max( 0.0, distance_to_shift_start ) );
      point.max_speed =
        std::min(
          point.max_speed.value_or( std::numeric_limits<double>::infinity() ),
          allowed_speed );
      ++approach_points;
      continue;
    }

    if( s <= maneuver_end_s )
    {
      point.max_speed =
        std::min(
          point.max_speed.value_or( std::numeric_limits<double>::infinity() ),
          target_v );
      ++capped_points;
    }
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
  map::Route stop_route = route;

  if( stop_route.reference_line.empty() ||
      !std::isfinite( ego_s ) ||
      !std::isfinite( desired_stop_s ) )
  {
    return stop_route;
  }

  const double braking_deceleration =
    std::max( params.min_braking_deceleration, std::fabs( vehicle_params.acceleration_min ) );
  const double current_speed = std::max( 0.0, ego_v );
  const double available_distance = desired_stop_s - ego_s;
  const double required_braking_distance =
    current_speed * current_speed / ( 2.0 * braking_deceleration );

  if( available_distance <= 0.0 ||
      available_distance < required_braking_distance )
  {
    set_route_points_from_s_to_zero( stop_route, ego_s );
    return stop_route;
  }

  insert_zero_speed_stop_point( stop_route, desired_stop_s );

  const double brake_start_s =
    ( required_braking_distance >= available_distance )
      ? ego_s
      : desired_stop_s - required_braking_distance;

  for( auto& [s, point] : stop_route.reference_line )
  {
    if( s >= brake_start_s )
    {
      const double dist_to_stop = desired_stop_s - s;
      const double allowed_speed =
        std::sqrt(
          2.0 * braking_deceleration * std::max( 0.0, dist_to_stop ) );
      point.max_speed =
        std::min(
          point.max_speed.value_or( std::numeric_limits<double>::infinity() ),
          allowed_speed );
    }
  }

  return stop_route;
}

StopPlan
RouteStopPolicy::plan_stop_on_route(
  const map::Route& route,
  double ego_s,
  double ego_v,
  double desired_stop_s,
  StopReason reason,
  const dynamics::PhysicalVehicleParameters& vehicle_params,
  const ObstacleAvoidanceParams& params )
{
  StopPlan plan;
  plan.route = route;
  plan.stop_s = desired_stop_s;
  plan.reason = reason;

  if( route.reference_line.empty() ||
      !std::isfinite( ego_s ) ||
      !std::isfinite( desired_stop_s ) )
  {
    plan.valid = false;
    plan.reason =
      !std::isfinite( ego_s ) || !std::isfinite( desired_stop_s )
        ? StopReason::InvalidProjection
        : reason;
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

ObstacleAvoidanceResult
try_plan_obstacle_avoidance( TrajectoryPlanner& planner,
                             const map::Route& route,
                             const dynamics::VehicleStateDynamic& ego,
                             const dynamics::TrafficParticipantSet& traffic_participants,
                             const ObstacleAvoidanceParams& params,
                             const std::vector<int>* additional_ignored_participant_ids )
{
  ObstacleAvoidanceResult result;
  result.modified_route = route;

  // Find nearest relevant static obstacle group on route. During a dynamic
  // replan the obstacles already handled by the active maneuver are passed as
  // ignored so detection locks onto the genuinely new obstacle rather than the
  // one ego is currently passing.
  const auto obstacle_group = find_static_obstacle_group_on_route(
    route,
    ego,
    traffic_participants,
    planner.get_physical_vehicle_parameters(),
    params,
    additional_ignored_participant_ids );

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

    const auto& stop_obstacle = obstacle_group->obstacles.front();
    stop_result.obstacle_id = stop_obstacle.id;
    stop_result.obstacle_ids =
      obstacle_group->envelope.participant_ids.empty()
        ? stop_obstacle.participant_ids
        : obstacle_group->envelope.participant_ids;
    stop_result.obstacle_s_min = stop_obstacle.object_s_min;
    stop_result.obstacle_s_max = stop_obstacle.object_s_max;
    const double stop_before_obstacle =
      normalized_stop_before_obstacle( params );
    const double ego_front_offset =
      vehicle_params.wheelbase + vehicle_params.front_axle_to_front_border;
    const double front_stop_s =
      stop_obstacle.object_s_min - std::max( 0.0, stop_before_obstacle );
    const double stop_s =
      front_stop_s - ego_front_offset;

    stop_result.modified_route =
      build_stop_route_before_obstacle(
        route,
        stop_obstacle,
        ego,
        vehicle_params,
        stop_before_obstacle,
        std::max( 0.0, params.modified_route_braking_safety_margin ) +
        std::max( 0.0, params.min_valid_stop_margin ),
        params );

    try
    {
      stop_result.trajectory = planner.plan_route_trajectory(
        stop_result.modified_route,
        ego,
        traffic_participants, std::nullopt );
    }
    catch( const std::exception& e )
    {
      static_cast<void>( e );
      // std::fprintf(
        // stderr,
        // "[OA][STOP_ROUTE] planner exception on stop route: %s\n",
        // e.what() );
      // std::fflush( stderr );
    }

    // Validate that the planned trajectory actually stops before the obstacle;
    // a non-empty trajectory alone does not guarantee a stop.
    RouteCorridorConflict stop_conflict;
    stop_conflict.participant_id = stop_obstacle.id;
    stop_conflict.object_s_min = stop_obstacle.object_s_min;
    stop_conflict.object_s_max = stop_obstacle.object_s_max;
    stop_conflict.inflated_s_min = stop_obstacle.object_s_min;
    stop_conflict.inflated_s_max = stop_obstacle.object_s_max;

    if( stop_result.trajectory.states.empty() ||
        !trajectory_stops_before_conflict(
          stop_result.trajectory,
          stop_result.modified_route,
          stop_conflict,
          vehicle_params,
          params ) )
    {
      // std::fprintf(
        // stderr,
        // "[OA][STOP_ROUTE] invalid stop trajectory; retrying with maximum route-based braking\n" );
      // std::fflush( stderr );

      stop_result.modified_route = route;
      set_route_points_from_s_to_zero(
        stop_result.modified_route,
        project_s_on_reference_line( route, ego, stop_s ) );

      try
      {
        stop_result.trajectory = planner.plan_route_trajectory(
          stop_result.modified_route,
          ego,
          traffic_participants, std::nullopt );
      }
      catch( const std::exception& e )
      {
        static_cast<void>( e );
        // std::fprintf(
          // stderr,
          // "[OA][STOP_ROUTE] planner exception on maximum route-based braking: %s\n",
          // e.what() );
        // std::fflush( stderr );
      }

      if( stop_result.trajectory.states.empty() ||
          !trajectory_stops_before_conflict(
            stop_result.trajectory,
            stop_result.modified_route,
            stop_conflict,
            vehicle_params,
            params ) )
      {
        stop_result.success = false;
        stop_result.reason = "OA stop route planning failed";
        // std::fprintf(
          // stderr,
          // "[OA][STOP_ROUTE] maximum route-based braking failed or did not stop before conflict; no valid route trajectory\n" );
        // std::fflush( stderr );
        return stop_result;
      }

    }

    stop_result.trajectory.label = reason;
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

  std::vector<int> ignored_participant_ids =
    obstacle_group->envelope.participant_ids;
  if( additional_ignored_participant_ids != nullptr )
  {
    for( const int participant_id : *additional_ignored_participant_ids )
    {
      if( !contains_participant_id( ignored_participant_ids, participant_id ) )
      {
        ignored_participant_ids.push_back( participant_id );
      }
    }
  }

  std::vector<RouteShiftPlanCandidate> accepted_candidates;

  bool saw_validated_candidate = false;
  bool saw_oncoming_conflict = false;

  std::string last_drivable_area_rejection;
  std::string last_projection_rejection;
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
        "%s (shift=%.2f, front_clearance=%.2f, rear_clearance=%.2f)",
        reason.c_str(),
        candidate.shift_candidate.shift,
        candidate.params.front_clearance,
        candidate.params.rear_clearance );
      return buf;
    };

  auto in_lane_shift_variants =
    generate_shift_candidate_variants( obstacle_group.value(), vehicle_params, params );
  for( auto& candidate : in_lane_shift_variants )
  {
    candidate.type = AvoidanceCandidateType::InLane;
  }

  auto adjacent_same_direction_shift_variants =
    generate_shift_candidate_variants( obstacle_group.value(), vehicle_params, params );
  for( auto& candidate : adjacent_same_direction_shift_variants )
  {
    candidate.type = AvoidanceCandidateType::AdjacentSameDirection;
  }

  auto opposite_lane_shift_variants =
    generate_opposite_lane_candidate_variants(
      route,
      obstacle_group.value(),
      vehicle_params,
      params );

  std::vector<ShiftCandidate> shift_variants;
  shift_variants.reserve(
    in_lane_shift_variants.size() +
    adjacent_same_direction_shift_variants.size() +
    opposite_lane_shift_variants.size() );

  // =========================================================================
  // Filter candidates by test-mode parameters
  // =========================================================================
  if( params.in_lane_shift_enabled )
  {
    shift_variants.insert(
      shift_variants.end(),
      in_lane_shift_variants.begin(),
      in_lane_shift_variants.end() );
  }

  if( params.adjacent_lane_enabled )
  {
    shift_variants.insert(
      shift_variants.end(),
      adjacent_same_direction_shift_variants.begin(),
      adjacent_same_direction_shift_variants.end() );
  }

  if( params.opposite_lane_enabled )
  {
    shift_variants.insert(
      shift_variants.end(),
      opposite_lane_shift_variants.begin(),
      opposite_lane_shift_variants.end() );
  }

  for( const auto& raw_shift : shift_variants )
  {
      RouteShiftPlanCandidate candidate;
      candidate.shift_candidate = raw_shift;
      candidate.params = params;

      evaluate_shift_candidate(
        candidate.shift_candidate,
        route,
        obstacle_group.value(),
        vehicle_params,
        candidate.params );

      if( !candidate.shift_candidate.valid )
      {
        last_drivable_area_rejection =
          describe_candidate_rejection(
            candidate,
            "candidate rejected by drivable-area check" );
        continue;
      }

      candidate.mode =
        candidate.shift_candidate.in_lane
          ? ObstacleAvoidanceMode::InLaneShift
          : ( candidate.shift_candidate.shift >= 0.0
                ? ObstacleAvoidanceMode::OvertakeLeft
                : ObstacleAvoidanceMode::OvertakeRight );

      candidate.uses_opposite_lane =
        candidate_uses_opposite_direction_lane(
          route,
          obstacle_group.value(),
          candidate.shift_candidate.shift,
          candidate.shift_candidate.in_lane,
          candidate.shift_candidate.type,
          vehicle_params,
          candidate.params );

      candidate.opposite_conflict_interval =
        candidate.uses_opposite_lane
          ? compute_opposite_lane_conflict_interval(
              route,
              obstacle_group.value(),
              candidate.shift_candidate.shift,
              vehicle_params,
              candidate.params )
          : OppositeLaneConflictInterval{};

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
          candidate.params.route_window_min,
          candidate.params.max_projection_distance_from_route );

      if( !std::isfinite( candidate.ego_s_modified ) )
      {
        candidate.ego_s_modified = ego_s_original;
      }

      if( !std::isfinite( candidate.ego_s_modified ) )
      {
        last_projection_rejection =
          describe_candidate_rejection(
            candidate,
            "candidate rejected: invalid modified-route projection" );
        continue;
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
        last_planning_rejection =
          describe_candidate_rejection(
            candidate,
            std::string( "candidate rejected: planner exception: " ) + e.what() );
        continue;
      }

      if( candidate.trajectory.states.empty() )
      {
        last_planning_rejection =
          describe_candidate_rejection(
            candidate,
            "candidate rejected: planner returned empty trajectory" );
        continue;
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
          ego_s_original );

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
          &ignored_participant_ids );

      const bool ignore_other_lane_oncoming =
        candidate.shift_candidate.type == AvoidanceCandidateType::InLane &&
        candidate_route_safety.has_conflict &&
        is_oncoming_other_lane_conflict(
          candidate_route_safety.conflict,
          vehicle_params,
          candidate.params );

      if( ( candidate_route_safety.has_conflict || !candidate_route_safety.safe ) &&
          !ignore_other_lane_oncoming )
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
        candidate.oncoming =
          check_oncoming_gap(
            route,
            ego,
            traffic_participants,
            obstacle_group.value(),
            candidate.shift_candidate.shift,
            vehicle_params,
            &candidate.trajectory,
            candidate.params );

        if( candidate.oncoming.conflict )
        {
          saw_oncoming_conflict = true;
          last_oncoming_rejection =
            describe_candidate_rejection(
              candidate,
              candidate.oncoming.reason );
          continue;
        }
      }

      candidate.score = score_route_shift_candidate( candidate, candidate.params );
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

    if( !last_validation_rejection.empty() )
    {
      reason += ": " + last_validation_rejection;
    }
    else if( !last_planning_rejection.empty() )
    {
      reason += ": " + last_planning_rejection;
    }
    else if( !last_projection_rejection.empty() )
    {
      reason += ": " + last_projection_rejection;
    }
    else if( !last_drivable_area_rejection.empty() )
    {
      reason += ": " + last_drivable_area_rejection;
    }
    else if( !last_safety_rejection.empty() )
    {
      reason += ": " + last_safety_rejection;
    }
    else if( !last_oncoming_rejection.empty() )
    {
      reason += ": " + last_oncoming_rejection;
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
        if( std::fabs( a.score - b.score ) > 1e-6 )
        {
          return a.score < b.score;
        }

        if( a.shift_candidate.in_lane != b.shift_candidate.in_lane )
        {
          return a.shift_candidate.in_lane;
        }

        if( a.uses_opposite_lane != b.uses_opposite_lane )
        {
          return !a.uses_opposite_lane;
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
  result.in_lane = selected.shift_candidate.in_lane;
  result.trajectory = selected.trajectory;

  if( result.trajectory.states.empty() )
  {
    return plan_stop_before_obstacle(
      ObstacleAvoidanceMode::StopBeforeObstacle,
      "driving mission (stop before obstacle: selected route-shift planning failed)" );
  }

  const auto final_validation =
    validate_planned_shift_trajectory(
      route,
      result.trajectory,
      obstacle_group.value(),
      result.lateral_shift,
      result.in_lane,
      selected.shift_candidate.type,
      vehicle_params,
      selected.params,
      ego_s_original );

  if( !final_validation.valid )
  {
    return plan_stop_before_obstacle(
      ObstacleAvoidanceMode::StopBeforeObstacle,
      "driving mission (stop before obstacle: selected route-shift trajectory validation failed)" );
  }

  const auto final_route_safety =
    check_route_corridor_safety(
      result.modified_route,
      ego,
      traffic_participants,
      vehicle_params,
      selected.params,
      &result.trajectory,
      &ignored_participant_ids );

  const bool ignore_final_other_lane_oncoming =
    selected.shift_candidate.type == AvoidanceCandidateType::InLane &&
    final_route_safety.has_conflict &&
    is_oncoming_other_lane_conflict(
      final_route_safety.conflict,
      vehicle_params,
      selected.params );

  if( ( final_route_safety.has_conflict || !final_route_safety.safe ) &&
      !ignore_final_other_lane_oncoming )
  {
    return plan_stop_before_obstacle(
      ObstacleAvoidanceMode::StopBeforeObstacle,
      "driving mission (stop before obstacle: selected route-shift failed active-route safety check)" );
  }

  if( selected.uses_opposite_lane )
  {
    const auto final_oncoming_check =
      check_oncoming_gap(
        route,
        ego,
        traffic_participants,
        obstacle_group.value(),
        result.lateral_shift,
        vehicle_params,
        &result.trajectory,
        selected.params );

    if( final_oncoming_check.conflict )
    {
      return plan_stop_before_obstacle(
        ObstacleAvoidanceMode::WaitForOncoming,
        "driving mission (waiting before obstacle: oncoming traffic conflict)" );
    }
  }

  if( result.mode == ObstacleAvoidanceMode::InLaneShift )
  {
    result.trajectory.label = "driving mission (in-lane obstacle avoidance)";
  }
  else if( result.mode == ObstacleAvoidanceMode::OvertakeLeft )
  {
    result.trajectory.label = "driving mission (obstacle avoidance left)";
  }
  else
  {
    result.trajectory.label = "driving mission (obstacle avoidance right)";
  }

  result.success = true;
  result.reason = "planned validated obstacle avoidance by selecting a route-shift candidate";


    const auto route_diff_bounds =
    find_route_difference_bounds(
      route,
      result.modified_route,
      0.02 ); // 2cm point distance threshold for difference detection

    if( route_diff_bounds.has_value() )
    {
      const double route_equal_again_s =
        route_diff_bounds->has_equal_point_after_last_difference
          ? route_diff_bounds->first_equal_s_after_last_difference
          : route_diff_bounds->last_different_s;

      result.has_maneuver_bounds = true;
      result.shift_start_s = route_diff_bounds->first_different_s;
      result.shift_end_s = route_equal_again_s;

      // Optional: keep analytical values for debug
      result.plateau_start_s =
        std::max( 0.0, obstacle_group->envelope.object_s_min - selected.params.front_clearance );

      result.plateau_end_s =
        obstacle_group->envelope.object_s_max + selected.params.rear_clearance;

      result.obstacle_id = obstacle_group->envelope.id;
      result.obstacle_ids = obstacle_group->envelope.participant_ids;
      result.obstacle_s_min = obstacle_group->envelope.object_s_min;
      result.obstacle_s_max = obstacle_group->envelope.object_s_max;

      result.maneuver.active = true;
      result.maneuver.mode = result.mode;
      result.maneuver.obstacle_id = result.obstacle_id;
      result.maneuver.obstacle_ids = result.obstacle_ids;
      result.maneuver.obstacle_s_min = result.obstacle_s_min;
      result.maneuver.obstacle_s_max = result.obstacle_s_max;
      result.maneuver.shift_start_s = result.shift_start_s;
      result.maneuver.plateau_start_s = result.plateau_start_s;
      result.maneuver.plateau_end_s = result.plateau_end_s;
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
          : result.plateau_start_s;

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
    result.reason = "no ego-lane oncoming participant";
    return result;
  }

  ObstacleEnvelope pseudo_obstacle;
  pseudo_obstacle.id = threat->participant_id;
  pseudo_obstacle.object_s_min = threat->participant_near_s;
  pseudo_obstacle.object_s_max = threat->participant_near_s;
  pseudo_obstacle.s_min = threat->participant_near_s;
  pseudo_obstacle.s_max = threat->participant_near_s;

  result.modified_route = build_stop_route_before_obstacle(
    route,
    pseudo_obstacle,
    ego,
    vehicle_params,
    params.ego_lane_oncoming_stop_distance,
    std::max( 0.0, params.modified_route_braking_safety_margin ) +
    std::max( 0.0, params.min_valid_stop_margin ),
    params );

  try
  {
    result.trajectory = planner.plan_route_trajectory(
      result.modified_route,
      ego,
      traffic_participants, std::nullopt );
  }
  catch( const std::exception& e )
  {
    static_cast<void>( e );
    // std::fprintf(
      // stderr,
      // "[OA][STOP_ROUTE] planner exception; keeping ego-lane stop route without free-space fallback: %s\n",
      // e.what() );
    // std::fflush( stderr );
  }

  if( result.trajectory.states.empty() )
  {
    // std::fprintf(
      // stderr,
      // "[OA][STOP_ROUTE] planner returned empty trajectory; keeping ego-lane stop route without free-space fallback\n" );
    // std::fflush( stderr );
  }

  result.trajectory.label =
    "driving mission (waiting: oncoming vehicle on ego lane)";

  result.success = true;
  result.reason = threat->reason;
  result.participant_id = threat->participant_id;
  result.participant_s = threat->participant_center_s;
  result.participant_distance_s = threat->distance_s;
  result.participant_route_speed = threat->v_route;
  result.time_to_conflict = threat->time_to_conflict;
  result.stop_s = threat->stop_s;

  if( params.debug_oncoming_check )
  {
    // std::fprintf(
      // stderr,
      // "[OA][ego_lane_oncoming] STOP: %s\n",
      // result.reason.c_str() );
    // std::fflush( stderr );
  }

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
  const std::vector<int>* ignored_participant_ids )
{
  RouteCorridorCheckResult result;
  result.reason = "modified-route corridor safety check disabled";

  if( !params.modified_route_safety_check_enabled )
  {
    return result;
  }

  const double ego_s = project_s_on_reference_line( route_to_check, ego );
  result.ego_s = ego_s;

  if( !std::isfinite( ego_s ) )
  {
    result.safe = false;
    result.has_conflict = true;
    result.reason = "invalid ego projection on route being checked";
    result.conflict.currently_inside_corridor = true;
    result.conflict.reason = result.reason;
    return result;
  }

  const double ego_half_width =
    0.5 * std::max( params.min_vehicle_dimension, ego_params.body_width );
  const double corridor_half_width =
    ego_half_width + std::max( 0.0, params.side_clearance );
  const double check_start_s =
    ego_s - std::max( 0.0, params.rear_clearance );
  const double check_end_s =
    ego_s + std::max( 0.0, params.modified_route_max_check_distance );
  const double corridor_l_min = -corridor_half_width;
  const double corridor_l_max = corridor_half_width;

  std::vector<RouteCorridorConflict> conflicts;

  for( const auto& [id, participant] : traffic_participants.participants )
  {
    const int participant_id = static_cast<int>( id );
    const double participant_speed = std::fabs( participant.state.vx );
    const bool ignored_active_obstacle =
      ignored_participant_ids != nullptr &&
      contains_participant_id( *ignored_participant_ids, participant_id ) &&
      participant_speed <= std::max( params.max_static_object_speed,
                                     params.ignored_obstacle_release_speed );

    if( ignored_active_obstacle )
    {
      continue;
    }

    const auto footprint =
      project_participant_footprint_to_route( route_to_check, participant, params );

    if( !footprint.has_value() )
    {
      continue;
    }

    if( footprint->s_max < check_start_s || footprint->s_min > check_end_s )
    {
      continue;
    }

    const double ego_front_s =
      ego_s + ego_params.wheelbase + ego_params.front_axle_to_front_border;
    const double ego_rear_s =
      ego_s - ego_params.rear_border_to_rear_axle;

    const bool currently_overlaps_route_corridor =
      footprint->l_max >= corridor_l_min &&
      footprint->l_min <= corridor_l_max &&
      footprint->s_max >= check_start_s;
    const bool currently_overlaps_ego_footprint =
      footprint->l_max >= -ego_half_width &&
      footprint->l_min <= ego_half_width &&
      footprint->s_max >= ego_rear_s &&
      footprint->s_min <= ego_front_s;

    const auto route_pose =
      route_to_check.get_pose_at_s( footprint->center_s );
    const double yaw_diff =
      normalize_angle( participant.state.yaw_angle - route_pose.yaw );
    const double v_route = participant_speed * std::cos( yaw_diff );

    RouteCorridorObjectClass object_class =
      RouteCorridorObjectClass::CrossingOrUnknown;

    if( participant_speed <= params.max_static_object_speed )
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

    double ttc = std::numeric_limits<double>::infinity();
    const char* ttc_source = "none";
    bool predicted_enters_corridor = false;
    auto conflict_footprint = footprint.value();
    auto conflict_world_participant = participant;

    if( currently_overlaps_ego_footprint )
    {
      ttc = 0.0;
      ttc_source = "ego_footprint_overlap";
    }
    else if( currently_overlaps_route_corridor )
    {
      const double ego_speed_for_ttc =
        std::max( params.min_ego_speed_for_gap_check, std::max( 0.0, ego.vx ) );
      ttc = std::max( 0.0, footprint->s_min - ego_front_s ) / ego_speed_for_ttc;
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
        predicted_participant.state.x =
          participant.state.x +
          std::cos( participant.state.yaw_angle ) * participant.state.vx * dt;
        predicted_participant.state.y =
          participant.state.y +
          std::sin( participant.state.yaw_angle ) * participant.state.vx * dt;
        predicted_participant.state.time = query_time;
        return predicted_participant;
      };

    if( ego_trajectory != nullptr &&
        ego_trajectory->states.size() >= 2 )
    {
      for( const auto& ego_state : ego_trajectory->states )
      {
        const double predicted_t = ego_state.time - ego.time;
        if( predicted_t < 0.0 ||
            predicted_t > params.modified_route_time_horizon + std::max( 0.0, params.modified_route_ttc_margin ) )
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

        const bool predicted_overlap =
          predicted_footprint->s_max >=
            predicted_ego_rear_s - std::max( 0.0, params.rear_clearance ) &&
          predicted_footprint->s_min <=
            predicted_ego_front_s + std::max( 0.0, params.front_clearance ) &&
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
          conflict_world_participant = predicted_participant;
        }
      }
    }
    else if( participant.trajectory.has_value() &&
             participant.trajectory->states.size() >= 2 )
    {
      for( const auto& predicted_state : participant.trajectory->states )
      {
        const double predicted_t = predicted_state.time - ego.time;
        if( predicted_t < 0.0 ||
            predicted_t > params.modified_route_time_horizon + std::max( 0.0, params.modified_route_ttc_margin ) )
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
          predicted_footprint->s_max >= check_start_s &&
          predicted_footprint->s_min <= check_end_s &&
          predicted_footprint->l_max >= corridor_l_min &&
          predicted_footprint->l_min <= corridor_l_max;

        if( predicted_overlap && predicted_t < ttc )
        {
          ttc = predicted_t;
          ttc_source = "participant_trajectory_corridor";
          predicted_enters_corridor = true;
          conflict_footprint = predicted_footprint.value();
          conflict_world_participant = predicted_participant;
        }
      }
    }

    if( !currently_overlaps_route_corridor &&
        std::isfinite( v_route ) &&
        v_route < -params.min_oncoming_route_speed &&
        footprint->s_min > ego_s )
    {
      const double closing_speed =
        std::max( params.min_oncoming_speed_for_gap_check, std::fabs( v_route ) );
      const double constant_velocity_ttc =
        std::max( 0.0, footprint->s_min - ego_s ) / closing_speed;
      if( constant_velocity_ttc < ttc )
      {
        ttc = constant_velocity_ttc;
        ttc_source = "constant_velocity";
        predicted_enters_corridor = true;
      }
    }

    if( !predicted_enters_corridor )
    {
      const double horizon =
        std::max( 0.0, params.modified_route_time_horizon );
      const double step = 0.5;

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
          predicted_footprint->s_max >= check_start_s &&
          predicted_footprint->s_min <= check_end_s &&
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
          conflict_world_participant = predicted_participant;
          break;
        }
      }
    }

    const bool predictive_conflict =
      std::isfinite( ttc ) &&
      ttc <= params.modified_route_time_horizon + std::max( 0.0, params.modified_route_ttc_margin ) &&
      ( predicted_enters_corridor ||
        ( footprint->l_max >= corridor_l_min &&
          footprint->l_min <= corridor_l_max ) );

    if( !currently_overlaps_route_corridor && !predictive_conflict )
    {
      continue;
    }

    RouteCorridorConflict conflict;
    conflict.participant_id = static_cast<int>( id );
    conflict.object_class = object_class;
    conflict.object_s_min = conflict_footprint.s_min;
    conflict.object_s_max = conflict_footprint.s_max;
    conflict.object_l_min = conflict_footprint.l_min;
    conflict.object_l_max = conflict_footprint.l_max;
    // Keep object geometry uninflated. Safety margins are represented by the
    // route/ego corridor checks above, not by modifying the object footprint.
    conflict.inflated_s_min = conflict_footprint.s_min;
    conflict.inflated_s_max = conflict_footprint.s_max;
    conflict.inflated_l_min = conflict_footprint.l_min;
    conflict.inflated_l_max = conflict_footprint.l_max;
    conflict.distance_s = std::max( 0.0, conflict_footprint.s_min - ego_s );
    conflict.time_to_conflict = ttc;
    conflict.currently_overlaps_route_corridor = currently_overlaps_route_corridor;
    conflict.currently_overlaps_ego_footprint = currently_overlaps_ego_footprint;
    conflict.predicted_spatiotemporal_conflict = predicted_enters_corridor;
    conflict.requires_stop =
      currently_overlaps_ego_footprint ||
      currently_overlaps_route_corridor ||
      predicted_enters_corridor;
    conflict.allows_replan = !currently_overlaps_ego_footprint;
    conflict.ttc_source = ttc_source;
    conflict.currently_inside_corridor = currently_overlaps_route_corridor;

    fill_world_footprint_from_participant( conflict, conflict_world_participant, params );

    char buf[384];
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
    conflict.reason = buf;

    conflicts.push_back( conflict );
  }

  if( conflicts.empty() )
  {
    result.safe = true;
    result.has_conflict = false;
    result.reason = "no conflict in route corridor";
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
trajectory_stops_before_conflict(
  const dynamics::Trajectory& trajectory,
  const map::Route& route,
  const RouteCorridorConflict& conflict,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
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
  constexpr double stopped_speed = 0.20;

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
      std::max( params.min_braking_deceleration, std::fabs( ego_params.acceleration_min ) );
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


ObstacleAvoidanceMonitorResult
monitor_active_obstacle_avoidance_maneuver(
  const map::Route& route,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::TrafficParticipantSet& traffic_participants,
  const ObstacleAvoidanceManeuver& maneuver,
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
  ego_speed = std::max( params.min_ego_speed_for_gap_check, ego_speed );

  if( candidate_ego_trajectory != nullptr )
  {
    const auto trajectory_clear_time =
      compute_ego_clear_time_from_trajectory(
        route,
        *candidate_ego_trajectory,
        ego.time,
        result.oncoming.conflict_end_s,
        ego_s );

    if( trajectory_clear_time.has_value() )
    {
      result.oncoming.ego_clear_time = trajectory_clear_time.value();
    }
  }

  if( result.oncoming.ego_clear_time <= 0.0 )
  {
    result.oncoming.ego_clear_time =
      std::max( 0.0, result.oncoming.conflict_end_s - ego_s ) /
      ego_speed;
  }

  for( const auto& [id, participant] : traffic_participants.participants )
  {
    const int participant_id = static_cast<int>( id );
    const double participant_speed = std::fabs( participant.state.vx );

    const bool expected_active_obstacle =
      ( participant_id == maneuver.obstacle_id ||
        contains_participant_id( maneuver.obstacle_ids, participant_id ) ) &&
      participant_speed <= std::max( params.max_static_object_speed,
                                     params.ignored_obstacle_release_speed );

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
    const double v_route = participant_speed * std::cos( yaw_diff );
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

    if( participant_speed <= params.max_static_object_speed ||
        v_route >= -params.min_oncoming_route_speed )
    {
      if( heading_opposite && participant_in_conflict_interval )
      {
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
          participant_s > result.oncoming.conflict_end_s )
      {
        const double slow_closing_speed =
          std::max( params.min_oncoming_speed_for_gap_check, std::fabs( v_route ) );
        const double slow_front_s =
          participant_footprint.has_value()
            ? participant_footprint->s_min
            : participant_s;
        const double slow_arrival_time =
          std::max( 0.0, slow_front_s - result.oncoming.conflict_end_s ) /
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
      std::max( params.min_oncoming_speed_for_gap_check,
                std::fabs( v_route ) );

    double arrival_time = std::numeric_limits<double>::infinity();
    const char* arrival_source = "constant_velocity";

    if( participant.trajectory.has_value() &&
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
      if( participant_s >= result.oncoming.conflict_start_s &&
          participant_s <= result.oncoming.conflict_end_s )
      {
        arrival_time = 0.0;
      }
      else if( participant_s > result.oncoming.conflict_end_s )
      {
        arrival_time =
          ( participant_s - result.oncoming.conflict_end_s ) /
          oncoming_speed;
      }
      else
      {
        continue;
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
