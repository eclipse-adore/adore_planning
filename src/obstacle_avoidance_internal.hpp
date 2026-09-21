/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#pragma once

// Private implementation detail of the adore_planning obstacle-avoidance module.
// This header intentionally lives in src/ (not include/) so it is never installed
// and is NOT part of the public API. It holds the shared internal types and helper
// declarations used to split the formerly monolithic obstacle_avoidance.cpp into
// cohesive translation units. All symbols live in adore::planner::oa_detail.

#include "planning/obstacle_avoidance.hpp"

#include "adore_math/point.h"
#include "adore_math/pose.h"

#include <limits>
#include <optional>
#include <utility>

namespace adore
{
namespace planner
{
namespace oa_detail
{

// ---------------------------------------------------------------------------
// Geometry primitives (obstacle_avoidance_geometry.cpp)
// ---------------------------------------------------------------------------

struct RouteFrame
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct RouteProjectionSample
{
  double s = std::numeric_limits<double>::quiet_NaN();
  double l = std::numeric_limits<double>::quiet_NaN();
  double distance = std::numeric_limits<double>::infinity();
};

double
normalize_angle( double angle );

double
smoothstep01( double t );

RouteFrame
make_route_frame( const map::Route& route, double s );

double
signed_lateral_offset( const RouteFrame& frame, const math::Point2d& point );

math::Point2d
shifted_point( const RouteFrame& frame, double lateral_offset );

// Project a world point onto the route reference line. Returns arc-length s,
// signed lateral offset l (+left of the tangent) and the orthogonal distance of
// the nearest polyline segment. Returns nullopt if the route has fewer than two
// points or no finite projection exists.
std::optional<RouteProjectionSample>
project_point_to_route_segments( const map::Route& route, double x, double y );

std::optional<std::pair<double, map::MapPoint>>
find_reference_point_near_s( const map::Route& route, double query_s );

// ---------------------------------------------------------------------------
// Obstacle / participant projection onto the route (obstacle_avoidance_projection.cpp)
// ---------------------------------------------------------------------------

struct ObstacleEnvelope
{
  int id = -1;
  std::vector<int> participant_ids;

  double object_s_min = std::numeric_limits<double>::infinity();
  double object_s_max = -std::numeric_limits<double>::infinity();
  double object_l_min = std::numeric_limits<double>::infinity();
  double object_l_max = -std::numeric_limits<double>::infinity();

  double center_s = std::numeric_limits<double>::infinity();

  bool overlaps_ego_corridor = false;

  // Accepted curve of a persistent contribution. New observations may expand
  // these bounds conservatively but a replan may never replace them with a
  // shorter/new global ramp.
  bool has_persistent_profile = false;
  double persistent_signed_shift = 0.0;
  double persistent_ramp_start_s = std::numeric_limits<double>::infinity();
  double persistent_full_shift_start_s = std::numeric_limits<double>::infinity();
  double persistent_full_shift_end_s = -std::numeric_limits<double>::infinity();
  double persistent_ramp_end_s = -std::numeric_limits<double>::infinity();

  // Belongs to the maneuver ego is already executing. If ego is already
  // alongside it at the first replanned trajectory state, validation permits
  // the existing clearance but never a smaller future clearance. Committed
  // obstacles farther ahead still require the normal hard clearance.
  bool committed_hold = false;
};

struct ParticipantFootprintOnRoute
{
  double s_min = std::numeric_limits<double>::infinity();
  double s_max = -std::numeric_limits<double>::infinity();
  double l_min = std::numeric_limits<double>::infinity();
  double l_max = -std::numeric_limits<double>::infinity();

  double center_s = std::numeric_limits<double>::infinity();
};

const char*
route_corridor_object_class_name( RouteCorridorObjectClass object_class );

bool
contains_participant_id( const std::vector<int>& ids, int id );

bool
project_obstacle_to_route_analytic( const map::Route& route,
                                    const dynamics::TrafficParticipant& participant,
                                    const ObstacleAvoidanceParams& params,
                                    double ego_half_width,
                                    ObstacleEnvelope& envelope );

std::optional<ParticipantFootprintOnRoute>
project_participant_footprint_to_route(
  const map::Route& route,
  const dynamics::TrafficParticipant& participant,
  const ObstacleAvoidanceParams& params );

// ---------------------------------------------------------------------------
// Static-obstacle detection / clustering (obstacle_avoidance_grouping.cpp)
// ---------------------------------------------------------------------------

// Obstacles selected for one planning cycle. Their individual hulls and
// persistent profiles remain separate; envelope is only their coarse union for
// search bounds, ids and diagnostics.
struct AvoidanceGroup
{
  std::vector<ObstacleEnvelope> obstacles;
  ObstacleEnvelope envelope;
};

bool
participant_is_static_for_avoidance(
  const dynamics::TrafficParticipant& participant,
  const ObstacleAvoidanceParams& params );

bool
participant_heading_is_opposite_to_route(
  const map::Route& route,
  const dynamics::TrafficParticipant& participant,
  double participant_s,
  const ObstacleAvoidanceParams& params );

AvoidanceGroup
make_avoidance_group_from_obstacles( std::vector<ObstacleEnvelope> obstacles );

std::pair<double, double>
avoidance_shift_support_interval(
  const ObstacleEnvelope& obstacle,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

double
avoidance_group_maneuver_end_s(
  const AvoidanceGroup& group,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

bool
avoidance_group_contains_participant_id( const AvoidanceGroup& group, int id );

std::optional<AvoidanceGroup>
find_static_obstacle_group_on_route(
  const map::Route& route,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::TrafficParticipantSet& traffic_participants,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params,
  const std::vector<AvoidanceShiftContribution>*
    committed_contributions = nullptr,
  const std::vector<int>* forced_participant_ids = nullptr );

// ---------------------------------------------------------------------------
// Lateral-shift profile math + modified-route construction (obstacle_avoidance_shift.cpp)
// ---------------------------------------------------------------------------

inline double
symmetric_shift_ego_half_length(
  const dynamics::PhysicalVehicleParameters& ego_params )
{
  return 0.5 * ego_params.body_length;
}

double
avoidance_shift_alpha_at_s( double s,
                            const ObstacleEnvelope& obstacle,
                            const dynamics::PhysicalVehicleParameters& ego_params,
                            const ObstacleAvoidanceParams& params );

double
required_signed_shift_for_obstacle(
  const ObstacleEnvelope& obstacle,
  double nominal_lateral_shift,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

double
choose_larger_magnitude_shift( double a, double b );

double
avoidance_shift_offset_at_s(
  double s,
  const AvoidanceGroup& group,
  double nominal_lateral_shift,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

map::Route
build_modified_avoidance_route( const map::Route& route,
                                const AvoidanceGroup& group,
                                double lateral_shift,
                                double ego_s,
                                const dynamics::PhysicalVehicleParameters& ego_params,
                                const ObstacleAvoidanceParams& params );

// ---------------------------------------------------------------------------
// Drivable-area / lane-interval queries (obstacle_avoidance_drivable_area.cpp)
// ---------------------------------------------------------------------------

struct LateralInterval
{
  double min_l = std::numeric_limits<double>::infinity();
  double max_l = -std::numeric_limits<double>::infinity();
};

struct OppositeDirectionLaneQuery
{
  // true: map topology and the current lane/road could be evaluated reliably.
  // false: map/topology is incomplete; callers may use a conservative fallback.
  bool map_usable = false;

  // true only if at least one adjacent driving lane with opposite direction was
  // found on the candidate shift side. If map_usable=true and this is false,
  // the map explicitly says that the candidate does not enter oncoming traffic.
  bool has_opposite_lane = false;

  LateralInterval interval;
  std::string reason;
};

struct ConnectedIntersectionAreaQuery
{
  // true when the current route lane and its same-direction neighbours could
  // be evaluated. A false result must never be used to enlarge the lane area.
  bool map_usable = false;

  // true only if at least one driving-lane polygon from another Road is part
  // of the connected component containing the current route lane.
  bool has_cross_road_connection = false;

  LateralInterval interval;
  std::string reason;
};

bool
lane_has_usable_borders( const map::Lane& lane );

bool
border_contains_s( const map::Border& border, double s, double slack );

bool
lane_contains_s( const map::Lane& lane, double s, double slack );

bool
add_lane_lateral_interval( const RouteFrame& frame,
                           const map::Lane& lane,
                           double lane_s,
                           double lane_s_overlap_slack,
                           LateralInterval& interval );

std::optional<LateralInterval>
get_allowed_lateral_interval_at_route_point(
  const map::Route& route,
  const map::MapPoint& route_point,
  double route_s,
  double lateral_shift,
  bool allow_adjacent_driving_lanes,
  const ObstacleAvoidanceParams& params );

OppositeDirectionLaneQuery
query_opposite_direction_lateral_interval_at_route_point(
  const map::Route& route,
  const map::MapPoint& route_point,
  double route_s,
  bool shift_left,
  const ObstacleAvoidanceParams& params );

ConnectedIntersectionAreaQuery
query_connected_intersection_driving_area_at_route_point(
  const map::Route& route,
  const map::MapPoint& route_point,
  double route_s,
  double lateral_shift,
  const ObstacleAvoidanceParams& params );

// ---------------------------------------------------------------------------
// Oncoming-traffic reasoning (obstacle_avoidance_oncoming.cpp)
// ---------------------------------------------------------------------------

struct OppositeLaneConflictInterval
{
  bool valid = false;
  bool occupies_opposite_lane = false;

  double start_s = 0.0;
  double end_s = 0.0;

  std::string reason;
};

struct EgoLaneOncomingThreat
{
  int participant_id = -1;
  double participant_near_s = 0.0;
  double time_to_conflict = std::numeric_limits<double>::infinity();
};

bool
participant_footprint_overlaps_ego_lane(
  const map::Route& route,
  const ParticipantFootprintOnRoute& footprint,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

std::optional<EgoLaneOncomingThreat>
find_ego_lane_oncoming_threat(
  const map::Route& route,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::TrafficParticipantSet& traffic_participants,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

OppositeLaneConflictInterval
compute_opposite_lane_conflict_interval(
  const map::Route& route,
  const AvoidanceGroup& group,
  double lateral_shift,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

std::optional<double>
compute_arrival_time_from_trajectory(
  const map::Route& route,
  const dynamics::Trajectory& trajectory,
  double now_time,
  double conflict_start_s,
  double conflict_end_s );

std::optional<double>
compute_ego_clear_time_from_trajectory(
  const map::Route& route,
  const dynamics::Trajectory& trajectory,
  double now_time,
  double conflict_end_s,
  double initial_s_hint,
  double projection_window );

planner::OncomingConflictResult
check_oncoming_gap( const map::Route& route,
                    const map::Route& driven_route,
                    const dynamics::VehicleStateDynamic& ego,
                    const dynamics::TrafficParticipantSet& traffic_participants,
                    const AvoidanceGroup& group,
                    double lateral_shift,
                    const dynamics::PhysicalVehicleParameters& ego_params,
                    const dynamics::Trajectory* candidate_ego_trajectory,
                    const ObstacleAvoidanceParams& params );

// ---------------------------------------------------------------------------
// Lateral-shift candidate generation / validation / scoring (obstacle_avoidance_candidates.cpp)
// ---------------------------------------------------------------------------

enum class AvoidanceCandidateType
{
  InLane,
  AdjacentSameDirection,
  ConnectedIntersection,
  OppositeDirection
};

enum class ShiftDirection
{
  Left,
  Right
};

struct ShiftCandidate
{
  double shift = 0.0; // +left, -right
  bool valid = false;

  // Intended validation mode. This is deliberately explicit so an opposite-lane
  // maneuver is not accidentally rejected by the current-lane check before the
  // oncoming-gap logic can run.
  AvoidanceCandidateType type = AvoidanceCandidateType::InLane;

  // True if the complete ego footprint stays inside the current route lane.
  // False means the shift uses adjacent driving-lane space and is therefore an
  // overtaking / lane-crossing maneuver.
  bool in_lane = false;
};

struct TrajectoryValidationResult
{
  bool valid = true;
  bool obstacle_clearance_violation = false;
  // Margin to the configured route-planning target (side_clearance), retained
  // even though trajectory rejection uses ego_corridor_safety_margin. A
  // negative value feeds one adaptive route-expansion retry.
  double min_obstacle_lateral_margin = std::numeric_limits<double>::infinity();
  std::string reason;
};

struct RouteShiftPlanCandidate
{
  ShiftCandidate shift_candidate;
  ObstacleAvoidanceParams params;

  ObstacleAvoidanceMode mode = ObstacleAvoidanceMode::None;

  map::Route modified_route;
  dynamics::Trajectory trajectory;

  bool uses_opposite_lane = false;
  OppositeLaneConflictInterval opposite_conflict_interval;
  TrajectoryValidationResult validation;

  double ego_s_modified = std::numeric_limits<double>::quiet_NaN();
};

bool
candidate_route_conflict_is_ignorable(
  const RouteCorridorConflict& conflict,
  AvoidanceCandidateType candidate_type,
  bool uses_opposite_lane,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

bool
candidate_respects_drivable_area( const map::Route& route,
                                  const AvoidanceGroup& group,
                                  double lateral_shift,
                                  const dynamics::PhysicalVehicleParameters& ego_params,
                                  const ObstacleAvoidanceParams& params,
                                  bool allow_adjacent_driving_lanes );

ShiftCandidate
make_candidate_from_obstacle_hulls(
  ShiftDirection direction,
  const AvoidanceGroup& group,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

bool
candidate_respects_opposite_direction_area(
  const map::Route& route,
  const AvoidanceGroup& group,
  double lateral_shift,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

bool
candidate_respects_connected_intersection_area(
  const map::Route& route,
  const AvoidanceGroup& group,
  double lateral_shift,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

void
evaluate_shift_candidate( ShiftCandidate& candidate,
                          const map::Route& route,
                          const AvoidanceGroup& group,
                          const dynamics::PhysicalVehicleParameters& ego_params,
                          const ObstacleAvoidanceParams& params );

std::vector<ShiftCandidate>
generate_shift_candidate_variants(
  const AvoidanceGroup& group,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

std::vector<ShiftCandidate>
generate_opposite_lane_candidate_variants(
  const map::Route& route,
  const AvoidanceGroup& group,
  const dynamics::PhysicalVehicleParameters& ego_params,
  const ObstacleAvoidanceParams& params );

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
  double initial_s_hint );

// ---------------------------------------------------------------------------
// Stop-route construction (obstacle_avoidance_stop_route.cpp)
// ---------------------------------------------------------------------------

double
normalized_stop_before_obstacle( const ObstacleAvoidanceParams& params );

// set_route_points_from_s_to_zero is now public (declared in
// planning/obstacle_avoidance.hpp) so the decision maker can reuse it; oa_detail
// callers resolve it via the enclosing planner namespace.

void
insert_zero_speed_stop_point( map::Route& route, double stop_s );

map::Route
build_stop_route_before_obstacle(
  const map::Route& route,
  const ObstacleEnvelope& obstacle,
  const dynamics::VehicleStateDynamic& ego,
  const dynamics::PhysicalVehicleParameters& vehicle_params,
  double stop_before_obstacle,
  const ObstacleAvoidanceParams& params );

} // namespace oa_detail
} // namespace planner
} // namespace adore
