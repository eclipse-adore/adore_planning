/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#pragma once

#include "adore_map/route.hpp"
#include "dynamics/traffic_participant.hpp"
#include "dynamics/trajectory.hpp"
#include "planning/trajectory_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace adore
{
namespace planner
{

// Obstacle avoidance is evaluated with the decision maker's 10 Hz cycle. The
// same interval is used by the conservative constant-velocity fallback when no
// ego trajectory is available.
inline constexpr double obstacle_avoidance_cycle_time_s = 0.1;

struct ObstacleAvoidanceParams
{
  // --------------------------------------------------------------------------
  // Public baseline parameters
  // --------------------------------------------------------------------------

  // Master enable/disable for obstacle avoidance.
  bool enabled = true;

  double max_static_object_speed      = 0.1;

  // Detection corridor: obstacle is relevant only if its raw footprint intersects
  // [-0.5 * ego_width - ego_corridor_safety_margin, +0.5 * ego_width + ego_corridor_safety_margin].
  // The same edge-to-edge distance is the hard lower bound for the dynamically
  // planned trajectory. It is deliberately smaller than side_clearance.
  double ego_corridor_safety_margin       = 0.5;

  // Lateral route-planning target from the ego outer edge to the real obstacle
  // outer edge. A trajectory that undershoots this target remains usable while
  // it stays outside ego_corridor_safety_margin.
  double side_clearance               = 1.0;

  // Allowed shortfall [m] from side_clearance before the active-route monitor
  // requests another optional best-effort replan. This does not alter the route
  // target or the hard ego_corridor_safety_margin.
  double side_clearance_replan_tolerance = 0.03;

  // Longitudinal planning distances for stop/shift timing. These do not
  // inflate stored obstacle geometry. front_clearance is now the MAXIMUM entry
  // ramp; the effective ramp is sized down to the available distance to the
  // obstacle (avoidance_ramp_length), so a late obstacle where ego is closer
  // than front_clearance still gets a fitting (shorter) ramp.
  double front_clearance              = 7.0;
  double rear_clearance               = 7.0;
  double stop_before_obstacle         = 8.0;

  // Allow lateral shifts within the current lane (without changing lanes).
  bool in_lane_shift_enabled = true;

  // Allow use of adjacent driving lanes in the same direction as the route.
  bool adjacent_lane_enabled = true;

  // Allow a route-guided avoidance maneuver to use the connected driving
  // surface of an intersection. Unlike adjacent_lane_enabled, this may include
  // geometrically connected driving lanes belonging to other roads and with a
  // different orientation. The mission route remains the longitudinal guide;
  // the foreign lanes only enlarge the permitted lateral area locally.
  bool intersection_drivable_area_enabled = false;

  // Allow use of opposite-direction lanes (oncoming lanes). Requires special
  // oncoming traffic checks.
  bool opposite_lane_enabled = true;

  // Upper speed cap during an avoidance. The actual maneuver speed is sized down
  // from this so the lateral shift stays within avoidance_lateral_accel over the
  // (possibly short) entry ramp: v = ramp * sqrt(a / (6*D)) (see
  // avoidance_speed_for_shift). 0.0 disables speed capping. Raise this to let
  // small shifts drive faster than a big-shift crawl.
  double max_speed_during_avoidance    = 4.167; // ~15 km/h upper cap

  // Comfort lateral acceleration used to couple maneuver speed and entry-ramp
  // length: shorter ramp / larger shift => lower speed so the turn-in stays
  // drivable; smaller shift => higher speed (no needless crawl).
  double avoidance_lateral_accel       = 2.0; // m/s^2

  // Distance ahead of the lateral-shift start (shift_start_s) at which the turn
  // indicator is switched on for an avoidance maneuver. The indicator is derived
  // downstream (trajectory_tracker) purely from the trajectory label; until ego
  // is within this distance of where it actually begins to move over, the
  // avoidance label carries no direction so the blinker stays off. This keeps
  // the vehicle from signaling at the (often far-ahead) moment the maneuver is
  // merely decided. Larger = signal earlier.
  double blinker_lead_distance         = 25.0;

  // Every accepted avoidance candidate is validated against the actually planned
  // trajectory, and every active maneuver is monitored against the route ego is
  // driving. These are safety invariants, not runtime switches.
  // Shared time horizon for candidate gap acceptance, the active modified-route
  // monitor and defensive ego-lane oncoming detection. This limits prediction
  // relevance, never perception/sensor range.
  double prediction_time_horizon = 15.0;

  // --------------------------------------------------------------------------
  // Internal/advanced parameters
  // --------------------------------------------------------------------------

  double min_oncoming_heading_diff    = 2.35; // rad, about 135 deg

  // If left and right are equally good, prefer left. This matches right-hand traffic overtaking behavior.
  bool prefer_left_shift               = true;

  // Longitudinal tolerance for considering a neighbouring lane available at the
  // same lane-local s position. Prevents using a lane outside its actual s range.
  double lane_s_overlap_slack          = 0.50;

  // Lateral tolerance for joining adjacent lane-border intervals into one
  // connected drivable area. Small map gaps below this value are ignored.
  double lane_boundary_join_slack      = 0.25;

  // Radius [m] around a route point in which lane-center samples are queried
  // when constructing the connected driving surface of an intersection.
  // The resulting area is still restricted to the component connected to the
  // current route lane, so nearby but disconnected roads are not admitted.
  double intersection_drivable_area_search_radius = 20.0;

  // ============================================================================
  // Internal/advanced oncoming traffic gap-acceptance parameters.
  // ============================================================================

  // Minimum time margin before an oncoming vehicle arrives at the conflict area.
  // If the computed oncoming arrival time is <= ego_clear_time + oncoming_time_margin,
  // the maneuver is rejected.
  double oncoming_time_margin = 1.0;

  // Minimum route-aligned speed for an oncoming participant to be considered
  // as moving in the opposite direction. Slow opposite-heading traffic below
  // this threshold uses the same value as a conservative arrival-speed floor.
  double min_oncoming_route_speed = 1.0;

  // Spatial uncertainty reserve around the longitudinal interval in which ego
  // occupies an opposite-direction lane. This complements oncoming_time_margin:
  // distance handles geometry/localization uncertainty and stopped traffic,
  // while time margin handles arrival-time separation.
  double oncoming_spatial_margin = 2.0;

  // ============================================================================
  // Internal/advanced ego-lane oncoming stop behavior.
  // ============================================================================

  // Additional lateral tolerance around the current ego lane when deciding
  // whether the participant footprint overlaps the ego lane.
  double ego_lane_oncoming_lateral_margin = 0.20;

  // Desired distance between the ego front and the nearest footprint point of
  // the oncoming participant when the ego vehicle comes to rest.
  double ego_lane_oncoming_stop_distance = 8.0;

  // ============================================================================
  // Internal/advanced active modified-route safety monitor parameters.
  // ============================================================================

  double modified_route_braking_safety_margin = 2.0;

  // ============================================================================
  // Internal/advanced trajectory and geometry parameters.
  // ============================================================================

  // Minimum search window for route projection. Prevents degenerate route segments.
  double route_window_min = 20.0;

  // Maximum tolerated unexplained forward projection progress on the active
  // modified route. Larger jumps are replaced by odometry-based progress.
  double projection_progress_tolerance = 2.0;

  // Ego speed at or below which a planned trajectory is considered stopped.
  double stopped_ego_speed = 0.20;

  // Nominal positive deceleration used for planned OA speed/stop profiles.
  // This should be gentler than the vehicle's physical acceleration_min
  // magnitude so maximum braking still has reserve.
  double planned_braking_deceleration = 1.0;

};

// Empty string means valid. A non-empty result identifies the first invalid
// value or cross-parameter relationship.
std::string
validate_obstacle_avoidance_params( const ObstacleAvoidanceParams& params );

// Validate the ego geometry and braking inputs consumed by obstacle avoidance.
// Empty means valid. Invalid ego data is a startup configuration error; it must
// not be replaced by a small synthetic footprint or braking capability.
std::string
validate_obstacle_avoidance_vehicle_params(
  const dynamics::PhysicalVehicleParameters& vehicle_params );

// Participant length and width are perception inputs. Invalid dimensions must
// trigger the behavior-layer fail-safe instead of creating an underestimated
// synthetic footprint.
bool
participant_has_valid_dimensions(
  const dynamics::TrafficParticipant& participant );

bool
traffic_participants_have_valid_dimensions(
  const dynamics::TrafficParticipantSet& traffic_participants );

double
maximum_braking_deceleration(
  const dynamics::PhysicalVehicleParameters& vehicle_params,
  const ObstacleAvoidanceParams& params );

double
planned_braking_deceleration(
  const dynamics::PhysicalVehicleParameters& vehicle_params,
  const ObstacleAvoidanceParams& params );

// Furthest route-s at which a currently static obstacle must become actionable
// in this planning cycle. The horizon contains the ego front overhang, one
// decision-maker cycle of travel, the planned-braking distance, the desired
// stand-off and the configured braking safety reserve. Objects beyond it remain
// perception inputs, but may be deferred to a later receding-horizon cycle.
double
static_obstacle_action_end_s(
  double ego_s,
  double ego_speed,
  const dynamics::PhysicalVehicleParameters& vehicle_params,
  const ObstacleAvoidanceParams& params );

struct AvoidanceSpeedSegment
{
  double start_s = std::numeric_limits<double>::infinity();
  double end_s = -std::numeric_limits<double>::infinity();
};

class RouteSpeedPolicy
{
public:
  static map::Route apply_avoidance_speed_profile(
    const map::Route& route,
    double ego_s,
    double shift_start_s,
    double maneuver_end_s,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const ObstacleAvoidanceParams& params,
    double ego_v = std::numeric_limits<double>::infinity() );

  // Apply the avoidance-speed cap only where the modified route actually shifts.
  // Between disconnected shift segments the original route speed is retained as
  // far as the braking envelope for the next segment permits.
  static map::Route apply_segmented_avoidance_speed_profile(
    const map::Route& route,
    double ego_s,
    const std::vector<AvoidanceSpeedSegment>& segments,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const ObstacleAvoidanceParams& params );

  static map::Route apply_stop_profile(
    const map::Route& route,
    double ego_s,
    double ego_v,
    double desired_stop_s,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const ObstacleAvoidanceParams& params );

  // Three-regime braking envelope: brake comfortably when possible, use the
  // required intermediate deceleration while retaining desired_stop_s when it is
  // still physically reachable, and use maximum braking toward the earliest
  // reachable point only when desired_stop_s cannot be reached.
  static map::Route apply_brake_envelope(
    const map::Route& route,
    double ego_s,
    double ego_v,
    double desired_stop_s,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const ObstacleAvoidanceParams& params );

  // Maximum-deceleration braking profile from the current position: decelerate at
  // the physical maximum (|acceleration_min|) from the current speed down to a
  // stop, laying sqrt(2*a_max*(stop-s)) on the route. Replaces the hard
  // zero-from-ego fallback so unreachable / invalid stops still get a smooth
  // speed reduction instead of an instant zero.
  static map::Route apply_max_braking_profile(
    const map::Route& route,
    double ego_s,
    double ego_v,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const ObstacleAvoidanceParams& params );
};

struct StopPlan
{
  bool valid = false;
  map::Route route;
};

class RouteStopPolicy
{
public:
  static StopPlan plan_stop_on_route(
    const map::Route& route,
    double ego_s,
    double ego_v,
    double desired_stop_s,
    const dynamics::PhysicalVehicleParameters& vehicle_params,
    const ObstacleAvoidanceParams& params );
};

// Set max_speed to zero for every route point at or beyond ego_s (immediate /
// maximum-braking stop request on the given route). Shared with the decision
// maker so the node does not reimplement this route mutation.
void
set_route_points_from_s_to_zero( map::Route& route, double ego_s );


enum class ObstacleAvoidanceMode
{
  None,
  InLaneShift,
  OvertakeLeft,
  OvertakeRight,
  WaitForOncoming,
  StopBeforeObstacle
};

/**
 * Detailed result of an oncoming traffic gap-acceptance check.
 *
 * This structure provides comprehensive diagnostics explaining whether a candidate
 * obstacle-avoidance maneuver can be safely executed with respect to oncoming traffic
 * in the opposite-direction lane.
 *
 * Assumptions:
 * - "Conflict interval" refers to the longitudinal range [conflict_start_s, conflict_end_s]
 *   where the ego vehicle's footprint occupies an opposite-direction lane during avoidance.
 * - Time calculations assume constant-velocity prediction for traffic participants.
 * - Times are measured from now (current timestamp).
 * - Route-aligned velocity is computed as: v_route = speed * cos(yaw_delta_from_route_yaw)
 *   Positive v_route means moving in route direction; negative means oncoming.
 * - The maneuver is rejected if: oncoming_arrival_time <= ego_clear_time + oncoming_time_margin
 *   to provide a safety buffer.
 */
struct OncomingConflictResult
{
  // True if the maneuver must be rejected due to oncoming traffic conflict.
  bool conflict = false;

  // The traffic participant ID causing the conflict (or -1 if no conflict).
  int participant_id = -1;

  // Longitudinal range [conflict_start_s, conflict_end_s] where ego is in opposite lane.
  // Typically derived from the maneuver's shift_start_s and shift_end_s or from the
  // obstacle's geometry.
  double conflict_start_s = 0.0;
  double conflict_end_s = 0.0;

  // Estimated time for ego to safely clear the conflict interval,
  // including the maneuver's return phase, measured from now.
  double ego_clear_time = 0.0;

  // Estimated arrival time of the oncoming vehicle at the rear of the conflict interval,
  // measured from now. Set to +infinity if no conflict or if the vehicle is not moving
  // oncoming.
  double oncoming_arrival_time = std::numeric_limits<double>::infinity();

  // Human-readable explanation of the decision (e.g., "participant id=5 arrival_time=2.5s <= ego_clear_time=3.0s + margin=2.0s").
  std::string reason;

};

struct EgoLaneOncomingStopResult
{
  bool success = false;

  map::Route modified_route;
  dynamics::Trajectory trajectory;
};

enum class RouteCorridorObjectClass
{
  StaticOrSlow,
  Oncoming,
  SameDirection,
  CrossingOrUnknown
};

struct RouteCorridorConflict
{
  int participant_id = -1;
  RouteCorridorObjectClass object_class = RouteCorridorObjectClass::CrossingOrUnknown;

  double object_s_min = std::numeric_limits<double>::infinity();
  double object_s_max = -std::numeric_limits<double>::infinity();
  double object_l_min = std::numeric_limits<double>::infinity();
  double object_l_max = -std::numeric_limits<double>::infinity();

  double distance_s = std::numeric_limits<double>::infinity();
  double time_to_conflict = std::numeric_limits<double>::infinity();
  double actual_lateral_clearance = std::numeric_limits<double>::quiet_NaN();

  bool currently_overlaps_route_corridor = false;
  bool currently_overlaps_ego_footprint = false;
  bool predicted_spatiotemporal_conflict = false;

  std::string reason;
};

struct RouteCorridorCheckResult
{
  bool safe = true;
  bool has_conflict = false;
  double ego_s = std::numeric_limits<double>::quiet_NaN();
  RouteCorridorConflict conflict;
  // All hard conflicts found this cycle (conflict above is the most relevant
  // one). Consumers that maintain per-obstacle memory must see every detection,
  // not only the best one, so simultaneously visible obstacles do not age out.
  std::vector<RouteCorridorConflict> conflicts;
  // Static objects that remain outside the hard ego corridor but fall short of
  // the desired side_clearance. They are non-blocking best-effort replan inputs:
  // failure to improve their clearance must not turn a still-safe route into a
  // stop request.
  std::vector<RouteCorridorConflict> static_clearance_improvements;
  std::string reason;
};

struct ObstacleAvoidanceManeuver
{
  bool active = false;

  ObstacleAvoidanceMode mode = ObstacleAvoidanceMode::None;

  int obstacle_id = -1;
  std::vector<int> obstacle_ids;

  double shift_start_s = 0.0;
  double shift_end_s = 0.0;
  double release_s = 0.0;

  double lateral_shift = 0.0;
  bool in_lane = false;

  // True if the maneuver occupies a lane whose direction is opposite to the
  // current route direction. These fields make active maneuver monitoring
  // possible from the decision maker without recomputing private obstacle
  // envelopes.
  bool uses_opposite_lane = false;
  bool has_opposite_lane_conflict_interval = false;
  double opposite_lane_conflict_start_s = 0.0;
  double opposite_lane_conflict_end_s = 0.0;

  // Before this point, an unsafe oncoming result is marked as an abort before
  // entry into the opposite-lane interval. After this point it is a committed
  // conflict. Both cases trigger a controlled stop on the driven route.
  double commitment_s = 0.0;
};

struct ObstacleAvoidanceMonitorResult
{
  bool safe_to_continue = true;
  bool should_abort_before_commitment = false;
  bool already_committed = false;

  OncomingConflictResult oncoming;
  std::string reason;
};

// Persistent per-object input to the lateral-shift composition. Besides the
// monotone object hull, this stores the accepted signed shift and complete
// ramp/plateau geometry. A later object therefore cannot reshape an earlier
// object's curve by changing a global ramp parameter.
struct AvoidanceShiftContribution
{
  std::vector<int> participant_ids;

  double object_s_min = std::numeric_limits<double>::infinity();
  double object_s_max = -std::numeric_limits<double>::infinity();
  double object_l_min = std::numeric_limits<double>::infinity();
  double object_l_max = -std::numeric_limits<double>::infinity();

  bool has_persistent_profile = false;
  double signed_shift = 0.0;
  double ramp_start_s = std::numeric_limits<double>::infinity();
  double full_shift_start_s = std::numeric_limits<double>::infinity();
  double full_shift_end_s = -std::numeric_limits<double>::infinity();
  double ramp_end_s = -std::numeric_limits<double>::infinity();
};

struct ObstacleAvoidanceResult
{
  bool success = false;
  std::string reason;
  ObstacleAvoidanceMode mode = ObstacleAvoidanceMode::None;

  map::Route modified_route;
  dynamics::Trajectory trajectory;

  bool has_maneuver_bounds = false;

  int obstacle_id = -1;
  std::vector<int> obstacle_ids;
  double obstacle_s_min = std::numeric_limits<double>::infinity();

  double shift_start_s = 0.0;
  double shift_end_s = 0.0;

  double lateral_shift = 0.0;
  double avoidance_speed = 0.0;
  bool in_lane = false;

  // Per-object frozen shift inputs of the selected maneuver, ordered by
  // object_s_min. Threaded into ActiveAvoidanceState so the persistent maneuver can
  // rebuild the modified route from each object's own curve, instead of
  // reconstructing a single merged span.
  std::vector<AvoidanceShiftContribution> shift_contributions;

  ObstacleAvoidanceManeuver maneuver;
};

// ----------------------------------------------------------------------------
// Shared helpers used by both the planner library and the decision-maker node.
// Kept here (instead of duplicated in each translation unit) so the projection
// and oncoming-clearance logic stays single-sourced.
// ----------------------------------------------------------------------------

// Project an ego/participant state onto the (possibly laterally modified) route
// reference line via segment projection. hint_s seeds the coarse search; the
// search window defaults to the full route span and the projection distance is
// unbounded by default.
template<typename State>
double
project_s_on_reference_line(
  const map::Route& route,
  const State& state,
  double hint_s = std::numeric_limits<double>::quiet_NaN(),
  double search_window = std::numeric_limits<double>::infinity(),
  double max_projection_distance = std::numeric_limits<double>::infinity() )
{
  if( route.reference_line.size() < 2 )
  {
    return std::numeric_limits<double>::infinity();
  }

  const double first_s = route.reference_line.begin()->first;
  const double last_s = route.reference_line.rbegin()->first;
  const double route_window =
    std::max( ObstacleAvoidanceParams{}.route_window_min, last_s - first_s );
  const double coarse_s =
    std::isfinite( hint_s ) ? hint_s : 0.5 * ( first_s + last_s );
  const double window =
    std::isfinite( search_window ) ? search_window : route_window;

  return adore::map::get_s_on_reference_line_segments(
    route, state, coarse_s, window, max_projection_distance );
}

// Lateral clearance between a route-centered ego footprint of half-width
// ego_half_width and an object whose route-frame lateral extent is
// [object_l_min, object_l_max]. +l is left of the route tangent.
inline double
actual_lateral_clearance_to_centered_ego(
  double object_l_min,
  double object_l_max,
  double ego_half_width )
{
  const double left_clearance = -ego_half_width - object_l_max;
  const double right_clearance = object_l_min - ego_half_width;
  return std::max( left_clearance, right_clearance );
}

// True if an oncoming object in the other lane keeps at least side_clearance to a
// route-centered ego footprint, i.e. it does not actually obstruct the ego lane.
inline bool
is_oncoming_other_lane_conflict(
  const RouteCorridorConflict& conflict,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params )
{
  if( conflict.object_class != RouteCorridorObjectClass::Oncoming ||
      conflict.currently_overlaps_ego_footprint )
  {
    return false;
  }

  const double ego_half_width = 0.5 * ego_params.body_width;
  const double actual_clearance =
    actual_lateral_clearance_to_centered_ego(
      conflict.object_l_min,
      conflict.object_l_max,
      ego_half_width );

  return actual_clearance >= std::max( 0.0, params.side_clearance );
}

// True if the participant keeps at least the hard ego-corridor safety margin
// to a route-centered ego footprint. Projects the participant footprint onto
// the route internally so the behavior layer and active monitor use the same
// geometric pass/fail boundary for stopped oncoming traffic.
bool
participant_keeps_hard_clearance_to_route_corridor(
  const map::Route& route,
  const dynamics::TrafficParticipant& participant,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

/**
 * Plan obstacle avoidance by modifying a copy of the route reference line.
 *
 * The planner does not publish or create a separate shifted path here. It modifies
 * the points of a copy of the original route and then calls TrajectoryPlanner on
 * that modified route.
 */
EgoLaneOncomingStopResult
try_plan_ego_lane_oncoming_stop( TrajectoryPlanner& planner,
                                 const map::Route& route,
                                 const dynamics::VehicleStateDynamic& ego,
                                 const dynamics::TrafficParticipantSet& traffic_participants,
                                 const ObstacleAvoidanceParams& params = {} );

// Check the route occupied by ego. static_check_end_s can bound only
// StaticOrSlow participants for receding-horizon operation; moving, crossing
// and oncoming participants retain the complete route/prediction check. The
// default and invalid/backwards bounds preserve the conservative full-route
// static check.
RouteCorridorCheckResult
check_route_corridor_safety(
  const map::Route& route_to_check,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::TrafficParticipantSet& traffic_participants,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params = {},
  const dynamics::Trajectory* ego_trajectory = nullptr,
  double static_check_end_s = std::numeric_limits<double>::infinity() );

bool
trajectory_stops_before_conflict(
  const dynamics::Trajectory& trajectory,
  const map::Route& route,
  const RouteCorridorConflict& conflict,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params = {},
  bool use_maximum_braking_deceleration = false );

/**
 * Verify that a route-derived trajectory stops at or before an exact route-s.
 *
 * This is used when a caller has already converted a conflict and all safety
 * margins into a rear-axle stop position. A horizon ending before required_stop_s
 * is accepted only if the remaining speed can still be braked away before it.
 */
bool
trajectory_stops_by_route_s(
  const dynamics::Trajectory& trajectory,
  const map::Route& route,
  double required_stop_s,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params = {},
  bool use_maximum_braking_deceleration = false );

ObstacleAvoidanceMonitorResult
monitor_active_obstacle_avoidance_maneuver(
  const map::Route& route,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::TrafficParticipantSet& traffic_participants,
  const ObstacleAvoidanceManeuver& maneuver,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params = {},
  const dynamics::Trajectory* candidate_ego_trajectory = nullptr );

ObstacleAvoidanceResult
try_plan_obstacle_avoidance( TrajectoryPlanner& planner,
                             const map::Route& route,
                             const dynamics::VehicleStateDynamic& ego,
                             const dynamics::TrafficParticipantSet& traffic_participants,
                             const ObstacleAvoidanceParams& params = {},
                             // When non-zero, restrict the shift to this lateral
                             // direction (+left / -right). Used by a mid-maneuver
                             // extension so it only ever widens the current side.
                             double shift_direction_sign = 0.0,
                             // Frozen obstacle hulls already accepted for an active
                             // maneuver. They are expressed in the mission-route
                             // frame and are composed again from that fixed baseline.
                             const std::vector<AvoidanceShiftContribution>*
                               committed_contributions = nullptr,
                             // Static participants selected by the corridor check
                             // on the currently driven modified route. They must be
                             // included even when they do not intersect the original
                             // mission-route trigger corridor.
                             const std::vector<int>*
                               forced_participant_ids = nullptr );

// Effective lateral-shift entry-ramp length: the distance to the obstacle,
// floored at the ego front overhang (below it the front reaches the obstacle
// before ego can begin turning) and capped at params.front_clearance (comfort).
// So a late obstacle where ego is closer than front_clearance gets a fitting
// shorter ramp instead of one that no longer fits the remaining distance.
double
avoidance_ramp_length( double distance_to_obstacle,
                       double ego_front_offset,
                       const ObstacleAvoidanceParams& params );

// Maneuver speed sized so a lateral shift of |shift_magnitude| driven over
// ramp_length stays within params.avoidance_lateral_accel:
//   v = min( ramp * sqrt(a / (6*|D|)), max_speed_during_avoidance ).
// Short ramp / big shift => slower (drivable tight turn-in); small shift =>
// faster (no needless crawl).
double
avoidance_speed_for_shift( double ramp_length,
                           double shift_magnitude,
                           const ObstacleAvoidanceParams& params );

} // namespace planner
} // namespace adore
