/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include "planning/active_avoidance.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace adore
{
namespace planner
{
void
start_active_avoidance_state(
    ActiveAvoidanceState& state,
    const planner::ObstacleAvoidanceResult& oa_result,
    const map::Route& mission_route_baseline )
{
    state.active = true;
    state.base_modified_route = oa_result.modified_route;
    state.mission_route_baseline = mission_route_baseline;

    state.shift_start_s = oa_result.shift_start_s;
    state.shift_end_s = oa_result.shift_end_s;
    state.release_s = oa_result.shift_end_s;
    state.obstacle_s_min = oa_result.obstacle_s_min;

    state.lateral_shift = oa_result.lateral_shift;
    state.avoidance_speed = oa_result.avoidance_speed;
    state.in_lane = oa_result.in_lane;

    // Persist the monotone per-object hull union used to rebuild the active
    // modified route from the fixed mission-route frame.
    state.committed_contributions = oa_result.shift_contributions;

    state.maneuver = oa_result.maneuver;

    state.last_modified_s = std::numeric_limits<double>::quiet_NaN();
    state.last_modified_time = std::numeric_limits<double>::quiet_NaN();
}

bool
should_stop_for_oncoming_monitor_result(
    const ObstacleAvoidanceMonitorResult& monitor_result )
{
    if( monitor_result.should_abort_before_commitment )
    {
        return true;
    }

    if( monitor_result.safe_to_continue )
    {
        return false;
    }

    return true;
}

// Build a RouteCorridorConflict from an active opposite-lane monitor result so
// the route-based stop policy can consume it. reference_s is the ego s on the
// route used to derive the longitudinal distance to the conflict.
planner::RouteCorridorConflict
make_oncoming_monitor_conflict(
    const planner::ObstacleAvoidanceMonitorResult& monitor_result,
    double reference_s )
{
    planner::RouteCorridorConflict conflict;
    conflict.participant_id = monitor_result.oncoming.participant_id;
    conflict.object_class = planner::RouteCorridorObjectClass::Oncoming;
    conflict.object_s_min = monitor_result.oncoming.conflict_start_s;
    conflict.object_s_max = monitor_result.oncoming.conflict_end_s;
    conflict.distance_s =
        std::max( 0.0, monitor_result.oncoming.conflict_start_s - reference_s );
    conflict.time_to_conflict = monitor_result.oncoming.oncoming_arrival_time;
    conflict.predicted_spatiotemporal_conflict = true;
    conflict.reason = monitor_result.reason;
    return conflict;
}

RouteStopPlan
compute_route_stop_plan(
    const map::Route& active_route,
    const dynamics::VehicleStateDynamic& ego,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const planner::RouteCorridorConflict& conflict,
    const planner::ObstacleAvoidanceParams& params )
{
    RouteStopPlan plan;
    const double conflict_hint =
        std::isfinite( conflict.object_s_min )
            ? conflict.object_s_min
            : std::numeric_limits<double>::quiet_NaN();
    plan.ego_s =
        project_s_on_reference_line(
            active_route,
            ego,
            conflict_hint );

    if( !std::isfinite( plan.ego_s ) )
    {
        return plan;
    }

    const double conflict_s =
        std::isfinite( conflict.object_s_min )
            ? conflict.object_s_min
            : plan.ego_s + params.stop_before_obstacle;
    const double ego_front_offset =
        vehicle_params.wheelbase + vehicle_params.front_axle_to_front_border;

    plan.stop_s =
        conflict_s -
        std::max( 0.0, params.stop_before_obstacle ) -
        ego_front_offset;

    plan.braking_deceleration =
        planned_braking_deceleration( vehicle_params, params );
    plan.required_braking_distance =
        std::max( 0.0, ego.vx ) * std::max( 0.0, ego.vx ) /
        ( 2.0 * plan.braking_deceleration );
    plan.brake_start_s =
        plan.stop_s -
        plan.required_braking_distance -
        std::max( 0.0, params.modified_route_braking_safety_margin );
    plan.valid =
        std::isfinite( plan.stop_s ) &&
        std::isfinite( plan.brake_start_s );

    return plan;
}

bool
should_stop_for_active_conflict(
    const map::Route& active_route,
    const dynamics::VehicleStateDynamic& ego,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const planner::RouteCorridorConflict& conflict,
    const planner::ObstacleAvoidanceParams& params )
{
    if( conflict.currently_overlaps_ego_footprint )
    {
        return true;
    }

    const auto stop_plan =
        compute_route_stop_plan(
            active_route,
            ego,
            vehicle_params,
            conflict,
            params );

    if( !stop_plan.valid )
    {
        return true;
    }

    const bool relevant_conflict =
        conflict.predicted_spatiotemporal_conflict ||
        conflict.currently_overlaps_route_corridor;

    // The stop decision is purely spatial and vehicle-dependent: begin once ego
    // reaches the braking point derived from current speed, planned deceleration,
    // stand-off and braking safety margin. A separate fixed TTC threshold can
    // otherwise request braking too late at high speed or too early at low speed.
    return relevant_conflict &&
           stop_plan.ego_s >= stop_plan.brake_start_s;
}

std::optional<double>
compute_monotonic_ego_s_modified(
    double ego_s_modified_raw,
    const dynamics::VehicleStateDynamic& vehicle_state_dynamic,
    ActiveAvoidanceState& state,
    const ObstacleAvoidanceParams& params )
{
    double ego_s_modified = ego_s_modified_raw;

    if( std::isfinite( ego_s_modified_raw ) &&
        std::isfinite( state.last_modified_s ) )
    {
        // Enforce monotonic progression: never go backward. Forward
        // jumps beyond what the vehicle can have travelled since the
        // last cycle are projection artifacts (e.g. matches at the
        // search-window edge); advance by odometry instead of
        // latching the jump permanently.
        const double dt =
            std::isfinite( state.last_modified_time )
                ? std::max(
                      0.0,
                      vehicle_state_dynamic.time - state.last_modified_time )
                : 0.0;
        const double odometry_advance =
            std::max( 0.0, vehicle_state_dynamic.vx ) * dt;
        const double max_plausible_advance =
            odometry_advance +
            std::max( 0.0, params.projection_progress_tolerance );

        if( ego_s_modified_raw < state.last_modified_s )
        {
            ego_s_modified = state.last_modified_s;
        }
        else if( ego_s_modified_raw - state.last_modified_s >
                 max_plausible_advance )
        {
            ego_s_modified = state.last_modified_s + odometry_advance;
        }
    }
    else if( !std::isfinite( ego_s_modified_raw ) &&
             std::isfinite( state.last_modified_s ) )
    {
        // Fallback: projection lost, use last valid value
        ego_s_modified = state.last_modified_s;
    }

    if( !std::isfinite( ego_s_modified ) )
    {
        return std::nullopt;
    }

    state.last_modified_s = ego_s_modified;
    state.last_modified_time = vehicle_state_dynamic.time;
    return ego_s_modified;
}

} // namespace planner
} // namespace adore
