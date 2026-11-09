/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/
#include "planning/unstructured_planner.hpp"

#include <algorithm>
#include <deque>

using namespace adore::math;

namespace adore
{
namespace planner
{

HybridAStarPlanner::HybridAStarPlanner() {}

void
HybridAStarPlanner::set_parameters( const std::map<std::string, double>& )
{}

void
HybridAStarPlanner::set_comfort_settings( const std::shared_ptr<dynamics::ComfortSettings>& settings )
{
  comfort_settings = settings;
}

void
HybridAStarPlanner::set_vehicle_parameters( const dynamics::PhysicalVehicleParameters& params )
{
  vehicle_params = params;
}

void
HybridAStarPlanner::set_goal( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                              const dynamics::TrafficParticipantSet& traffic_participants )
{
  double current_s     = latest_route.get_s( current_state );
  bool   route_blocked = false;
  double free_point_s  = current_s;

  constexpr double check_distance   = 20.0;
  constexpr double blocking_offset  = 1.5;
  constexpr double stopped_velocity = 0.2;
  constexpr double object_clearance = 5.0;

  double furthest_blocking_s = 0.0;

  // ----------------------------------------------------------
  // Find the furthest object blocking the route
  // ----------------------------------------------------------
  for( const auto& [id, participant] : traffic_participants.participants )
  {
    const auto& state = participant.state;

    // Ignore moving participants
    if( state.vx > stopped_velocity )
      continue;

    // Longitudinal position of the participant on the route
    const double obj_s = latest_route.get_s( state );

    // Ignore objects behind the vehicle
    if( obj_s <= current_s )
      continue;

    // Check that the object is actually on the route
    const double offset = adore::math::distance_2d( state, latest_route.get_pose_at_s( obj_s ) );

    if( offset > blocking_offset )
      continue;

    // Object is blocking the route.
    // Keep the furthest one.
    furthest_blocking_s = std::max( furthest_blocking_s, obj_s - current_s + object_clearance + participant.physical_parameters.body_length );
  }
  if( furthest_blocking_s == 0 )
  {
    furthest_blocking_s = 7.0;
    // unstructured_speed = 2.5;
  }

  auto free_point_ahead = latest_route.get_pose_at_s( current_s + furthest_blocking_s );
  goal_x   = free_point_ahead.x;
  goal_y   = free_point_ahead.y;
  goal_yaw = free_point_ahead.yaw;
}

std::tuple<int, int, int>
HybridAStarPlanner::Node::grid_index() const
{
  int ix   = static_cast<int>( x / XY_RES );
  int iy   = static_cast<int>( y / XY_RES );
  int iyaw = static_cast<int>( yaw / YAW_RES );

  return { ix, iy, iyaw };
}

size_t
HybridAStarPlanner::GridHash::operator()( const std::tuple<int, int, int>& k ) const
{
  auto [x, y, yaw] = k;

  size_t h1 = std::hash<int>()( x );
  size_t h2 = std::hash<int>()( y );
  size_t h3 = std::hash<int>()( yaw );

  return h1 ^ ( h2 << 1 ) ^ ( h3 << 2 );
}

// ======================================================
// CONFIG
// ======================================================

static constexpr double LOCAL_GOAL_MAX_DIST = 30.0;
static constexpr double GOAL_REACHED_RADIUS = 2.0;

static constexpr double VEHICLE_LENGTH = 4.5;
static constexpr double VEHICLE_WIDTH  = 2.0;

static constexpr double LOCAL_GOAL_HYSTERESIS_DIST   = 3.0;
static constexpr double LOCAL_GOAL_CONTINUITY_WEIGHT = 5.0;
static constexpr double LOCAL_GOAL_CONTINUITY_RADIUS = 6.0;

static constexpr int MAX_EXPANSIONS = 4000;

static constexpr double PREV_ROUTE_CONTINUITY_CLAMP = 5.0;

static constexpr double STITCH_DISTANCE = 3.0;

static constexpr double MIN_PREV_PATH_LENGTH_FOR_STITCH = 0.5;
static constexpr double MIN_REMAINING_ROUTE_LENGTH      = 8.0;

static constexpr double FREE_SPACE_GOAL_CONE_HALF_ANGLE = 1.3;
static constexpr double PREV_ROUTE_LOOKUP_MARGIN  = 5.0; // meters padding around the search horizon when building the continuity window
static constexpr double PREV_ROUTE_HEADING_WEIGHT = 1.0; // cost weight per rad^2 of heading mismatch vs. the previous plan - tune alongside
                                                         // the 0.3 position weight below

static constexpr double GOAL_HEADING_TOLERANCE      = 0.15; // rad (~8.6 deg) - required heading accuracy to accept the goal connection
static constexpr double GOAL_HEADING_BLEND_DISTANCE = 9.0; // meters - start blending target heading toward goal_yaw within this distance of
                                                           // the goal
static constexpr int GOAL_CONNECTION_MAX_STEPS = 120; // was a hardcoded 80 in try_goal_connection - a bit more room to converge heading as
                                                      // well as position

static const PathState*
find_state_at_or_after_s( const std::vector<PathState>& states, double target_s )
{
  if( states.empty() )
    return nullptr;

  for( const auto& st : states )
  {
    if( st.s >= target_s )
      return &st;
  }

  return &states.back();
}

static std::vector<PathState>
trim_states_from_s( const std::vector<PathState>& states, double s_from )
{
  std::vector<PathState> trimmed;

  for( const auto& st : states )
  {
    if( st.s < s_from )
      continue;

    PathState shifted  = st;
    shifted.s         -= s_from;

    trimmed.push_back( shifted );
  }

  return trimmed;
}

bool
HybridAStarPlanner::inside_drivable_area( double x, double y, double yaw, const std::optional<math::Polygon2d>& drivable_area )
{
  // No drivable-area information:
  // everything is considered geometrically valid.
  if( !drivable_area.has_value() )
  {
    return true;
  }

  math::Point2d p;
  p.x = x;
  p.y = y;

  return drivable_area.value().point_inside( p );
}

bool
HybridAStarPlanner::collision( double x, double y, const dynamics::TrafficParticipantSet& participants )
{
  for( const auto& [id, p] : participants.participants )
  {
    double dx = x - p.state.x;
    double dy = y - p.state.y;
    double yaw = p.state.yaw_angle;

    double cos_yaw = math::fast_cos( yaw );
    double sin_yaw = math::fast_sin( yaw );

    double local_x = dx * cos_yaw + dy * sin_yaw;
    double local_y = -dx * sin_yaw + dy * cos_yaw;

    double half_length = 0.5 * p.physical_parameters.body_length + 1.5;
    double half_width = 0.5 * p.physical_parameters.body_width + 1.5;

    if( std::abs( local_x ) < half_length && std::abs( local_y ) < half_width )
      return true;

    // if( std::hypot( dx, dy ) < VEHICLE_RADIUS )
    //   return true;
  }

  return false;
}

double
HybridAStarPlanner::distance_to_polygon_boundary( const math::Point2d& p, const math::Polygon2d& polygon )
{
  double best = std::numeric_limits<double>::max();

  for( size_t i = 0; i < polygon.points.size(); i++ )
  {
    auto a = polygon.points[i];
    auto b = polygon.points[( i + 1 ) % polygon.points.size()];

    double d = distance_point_to_segment( p, a, b );

    best = std::min( best, d );
  }

  return best;
}

bool
HybridAStarPlanner::simulate_motion( double& x, double& y, double& yaw, double steer, const dynamics::TrafficParticipantSet& participants,
                                     const std::optional<math::Polygon2d>& drivable_area )
{
  double distance = 0.0;

  const double start_x   = x;
  const double start_y   = y;
  const double start_yaw = yaw;

  while( distance < STEP )
  {
    x += MOTION_RESOLUTION * std::cos( yaw );
    y += MOTION_RESOLUTION * std::sin( yaw );

    yaw += MOTION_RESOLUTION / WHEEL_BASE * std::tan( steer );

    // --------------------------------------------------
    // Collision checking
    //
    // ALWAYS active, even in free-space mode.
    // --------------------------------------------------

    if( collision( x, y, participants ) )
    {
      x   = start_x;
      y   = start_y;
      yaw = start_yaw;

      return false;
    }

    distance += MOTION_RESOLUTION;
  }

  // ----------------------------------------------------
  // Drivable-area checking
  //
  // inside_drivable_area() returns true automatically
  // when drivable_area is std::nullopt.
  // ----------------------------------------------------

  if( !inside_drivable_area( x, y, yaw, drivable_area ) )
  {
    x   = start_x;
    y   = start_y;
    yaw = start_yaw;

    return false;
  }

  return true;
}

math::Point2d
HybridAStarPlanner::compute_local_goal( const dynamics::VehicleStateDynamic& ego, const std::optional<math::Polygon2d>& drivable_area )
{
  // ============================================================
  // FREE-SPACE MODE
  // ============================================================

  if( !drivable_area.has_value() )
  {
    const double dx = goal_x - ego.x;

    const double dy = goal_y - ego.y;

    const double goal_distance = std::hypot( dx, dy );

    if( goal_distance < 15.0 )
    {
      math::Point2d final_goal;
      final_goal.x = goal_x;
      final_goal.y = goal_y;
      return final_goal;
    }

    // ----------------------------------------------------------
    // Clamp the local goal to within a forward-reachable cone of
    // the current heading - same restriction the drivable-area
    // branch already applies (FREE_SPACE_GOAL_CONE_HALF_ANGLE
    // matches its +-1.3 rad search cone). Motion primitives are
    // forward-only, so pointing straight at a goal that's behind
    // or sharply beside the vehicle can make the search unable
    // to connect within GOAL_REACHED_RADIUS/MAX_EXPANSIONS at
    // all. Clamping lets the vehicle curve toward the goal over
    // successive planning cycles instead of needing an
    // unreachable point in one shot.
    // ----------------------------------------------------------

    const double bearing_to_goal = std::atan2( dy, dx );

    const double heading_diff = math::normalize_angle( bearing_to_goal - ego.yaw_angle );

    const double clamped_diff = std::max( -FREE_SPACE_GOAL_CONE_HALF_ANGLE, std::min( FREE_SPACE_GOAL_CONE_HALF_ANGLE, heading_diff ) );

    const double theta = ego.yaw_angle + clamped_diff;

    const double local_goal_dist = std::min( goal_distance, LOCAL_GOAL_MAX_DIST );

    math::Point2d local_goal;

    local_goal.x = ego.x + local_goal_dist * std::cos( theta );

    local_goal.y = ego.y + local_goal_dist * std::sin( theta );

    // ----------------------------------------------------------
    // Hysteresis
    //
    // Don't keep the previous goal if it has already fallen
    // behind the vehicle.
    // ----------------------------------------------------------

    if( has_local_goal )
    {
      const double previous_dx = current_local_goal.x - ego.x;

      const double previous_dy = current_local_goal.y - ego.y;

      const double previous_forward_distance = previous_dx * std::cos( ego.yaw_angle ) + previous_dy * std::sin( ego.yaw_angle );

      const double goal_dx = local_goal.x - current_local_goal.x;

      const double goal_dy = local_goal.y - current_local_goal.y;

      const double goal_change = std::hypot( goal_dx, goal_dy );

      // Only reuse previous local goal if it is:
      //
      // 1. still ahead of the vehicle
      // 2. sufficiently close to the newly calculated goal
      //
      if( previous_forward_distance > 2.0 && goal_change < LOCAL_GOAL_HYSTERESIS_DIST )
      {
        return current_local_goal;
      }
    }

    current_local_goal = local_goal;
    has_local_goal     = true;

    return local_goal;
  }

  // ============================================================
  // DRIVABLE-AREA MODE
  // ============================================================

  const math::Polygon2d& area = *drivable_area;

  math::Point2d best_point;

  double best_score = -std::numeric_limits<double>::max();

  // ------------------------------------------------------------
  // Use CURRENT VEHICLE HEADING
  //
  // This is important for zig-zag corridors.
  // ------------------------------------------------------------

  const double search_heading = ego.yaw_angle;

  // ------------------------------------------------------------
  // Search reachable local frontier.
  //
  // Integer-indexed loops avoid floating-point accumulation.
  // ------------------------------------------------------------

  const int R_STEPS = static_cast<int>( ( LOCAL_GOAL_MAX_DIST - 15.0 ) / 0.5 + 1e-6 ) + 1;

  const int ANGLE_STEPS = static_cast<int>( 2.6 / 0.1 + 1e-6 ) + 1;

  for( int ri = 0; ri < R_STEPS; ++ri )
  {
    const double r = 15.0 + ri * 0.5;

    // ----------------------------------------------------------
    // Search cone around current heading
    // ----------------------------------------------------------

    for( int ai = 0; ai < ANGLE_STEPS; ++ai )
    {
      const double angle = -1.3 + ai * 0.1;

      const double theta = search_heading + angle;

      math::Point2d p;

      p.x = ego.x + r * std::cos( theta );

      p.y = ego.y + r * std::sin( theta );

      // --------------------------------------------------------
      // Must remain inside drivable area
      // --------------------------------------------------------

      if( !area.point_inside( p ) )
      {
        continue;
      }

      // --------------------------------------------------------
      // Progress toward final goal
      // --------------------------------------------------------

      const double goal_progress = -std::hypot( goal_x - p.x, goal_y - p.y );

      // --------------------------------------------------------
      // Prefer farther points
      // --------------------------------------------------------

      const double distance_score = r;

      // --------------------------------------------------------
      // Prefer forward points
      // --------------------------------------------------------

      double dir_x = p.x - ego.x;

      double dir_y = p.y - ego.y;

      const double norm = std::hypot( dir_x, dir_y );

      if( norm > 1e-3 )
      {
        dir_x /= norm;
        dir_y /= norm;
      }

      const double heading_alignment = dir_x * std::cos( ego.yaw_angle ) + dir_y * std::sin( ego.yaw_angle );

      const double forward_score = 8.0 * heading_alignment;

      // --------------------------------------------------------
      // Penalize large turning angles
      // --------------------------------------------------------

      const double turning_penalty = 2.0 * std::abs( angle );

      // --------------------------------------------------------
      // Clearance from polygon boundary
      // --------------------------------------------------------

      const double clearance = distance_to_polygon_boundary( p, area );

      const double clearance_score = 6.0 * clearance;

      // --------------------------------------------------------
      // Continuity
      // --------------------------------------------------------

      double continuity_score = 0.0;

      if( has_local_goal )
      {
        const double dcx = p.x - current_local_goal.x;

        const double dcy = p.y - current_local_goal.y;

        const double dist_to_prev_goal = std::hypot( dcx, dcy );

        continuity_score = LOCAL_GOAL_CONTINUITY_WEIGHT * std::max( 0.0, 1.0 - dist_to_prev_goal / LOCAL_GOAL_CONTINUITY_RADIUS );
      }

      // --------------------------------------------------------
      // Final score
      // --------------------------------------------------------

      const double score = 3.0 * goal_progress + 1.0 * distance_score + forward_score + clearance_score - turning_penalty
                         + continuity_score;

      if( score > best_score )
      {
        best_score = score;
        best_point = p;
      }
    }
  }

  // ============================================================
  // FALLBACK
  // ============================================================

  if( best_score < -std::numeric_limits<double>::max() / 2.0 )
  {
    planner_message += " No valid local goal found";

    best_point.x = ego.x + 5.0 * std::cos( ego.yaw_angle );

    best_point.y = ego.y + 5.0 * std::sin( ego.yaw_angle );
  }

  // ============================================================
  // GOAL HYSTERESIS
  // ============================================================

  if( has_local_goal )
  {
    const double dx = best_point.x - current_local_goal.x;

    const double dy = best_point.y - current_local_goal.y;

    const double d = std::hypot( dx, dy );

    if( d < LOCAL_GOAL_HYSTERESIS_DIST )
    {
      return current_local_goal;
    }
  }

  current_local_goal = best_point;
  has_local_goal     = true;

  // ============================================================
  // DEBUG
  // ============================================================

  return best_point;
}

// Nearest-sample lookup against a (small, pre-filtered) window
// of previous-path states, returning both position distance and
// heading mismatch at that nearest sample - replaces the old
// position-only, full-route distance_to_previous_route() in the
// search's hot path. distance_to_previous_route() itself is left
// untouched in case anything else still calls it.
static ContinuityCost
nearest_previous_state_cost( const std::vector<PathState>& relevant_states, double x, double y, double yaw )
{
  ContinuityCost result{ 0.0, 0.0 };

  double best_dist = std::numeric_limits<double>::max();

  for( const auto& st : relevant_states )
  {
    const double dx = x - st.x;
    const double dy = y - st.y;

    const double d = std::hypot( dx, dy );

    if( d < best_dist )
    {
      best_dist           = d;
      result.heading_diff = std::fabs( math::normalize_angle( yaw - st.yaw ) );
    }
  }

  if( best_dist < std::numeric_limits<double>::max() )
  {
    result.distance = best_dist;
  }

  return result;
}

double
HybridAStarPlanner::heuristic( double x, double y, double yaw, const math::Point2d& local_goal )
{
  const double dx = local_goal.x - x;
  const double dy = local_goal.y - y;
  
  const double dist = std::hypot( dx, dy );

  double target_heading = std::atan2( dy, dx );

  // When steering toward the LOCKED final goal, bias the
  // heuristic's target heading toward goal_yaw as distance
  // shrinks - same blend used in try_goal_connection() below, so
  // the coarse search already favors approaches that end up
  // roughly facing the right way, instead of relying entirely on
  // the short final connector to fix heading at the last moment.
  if( final_goal_locked )
  {
    const double blend = std::max( 0.0, std::min( 1.0, 1.0 - dist / GOAL_HEADING_BLEND_DISTANCE ) );

    const double heading_gap = math::normalize_angle( goal_yaw - target_heading );

    target_heading = math::normalize_angle( target_heading + blend * heading_gap );
  }

  const double heading_error = std::fabs( math::normalize_angle( target_heading - yaw ) );

  return distance_weight * dist + heading_weight * heading_error;
}

bool
HybridAStarPlanner::try_goal_connection( Node* node, const math::Point2d& local_goal, const dynamics::TrafficParticipantSet& participants,
                                         const std::optional<math::Polygon2d>& drivable_area, std::vector<std::pair<double, double>>& path )
{
  double x   = node->x;
  double y   = node->y;
  double yaw = node->yaw;

  for( int i = 0; i < GOAL_CONNECTION_MAX_STEPS; ++i )
  {
    const double dx = local_goal.x - x;

    const double dy = local_goal.y - y;

    const double dist = std::hypot( dx, dy );

    const double target_position_bearing = std::atan2( dy, dx );

    double target = target_position_bearing;

    bool heading_ok = true;

    // ----------------------------------------------------------
    // local_goal equals (goal_x, goal_y) exactly whenever
    // final_goal_locked is true (see compute_local_goal), so this
    // check is enough to know "this connection attempt is aiming
    // at the real final goal, heading requirement included" vs.
    // "this is just a local frontier waypoint, heading doesn't
    // matter." No signature change needed to pass that through.
    // ----------------------------------------------------------

    if( final_goal_locked )
    {
      const double blend = std::max( 0.0, std::min( 1.0, 1.0 - dist / GOAL_HEADING_BLEND_DISTANCE ) );

      const double heading_gap = math::normalize_angle( goal_yaw - target_position_bearing );

      target = math::normalize_angle( target_position_bearing + blend * heading_gap );

      heading_ok = std::fabs( math::normalize_angle( yaw - goal_yaw ) ) < GOAL_HEADING_TOLERANCE;
    }

    // ----------------------------------------------------------
    // Goal reached - position AND (when applicable) heading
    // ----------------------------------------------------------

    if( dist < GOAL_REACHED_RADIUS && heading_ok )
    {
      path.push_back( { local_goal.x, local_goal.y } );

      return true;
    }

    const double steer = std::max( -MAX_STEER, std::min( MAX_STEER, math::normalize_angle( target - yaw ) ) );

    x += MOTION_RESOLUTION * std::cos( yaw );

    y += MOTION_RESOLUTION * std::sin( yaw );

    yaw += MOTION_RESOLUTION / WHEEL_BASE * std::tan( steer );

    // ----------------------------------------------------------
    // Collision checking ALWAYS active.
    // ----------------------------------------------------------

    if( collision( x, y, participants ) )
    {
      return false;
    }

    // ----------------------------------------------------------
    // Polygon checking only when a polygon exists.
    //
    // inside_drivable_area() handles std::nullopt.
    // ----------------------------------------------------------

    if( !inside_drivable_area( x, y, yaw, drivable_area ) )
    {
      return false;
    }

    path.push_back( { x, y } );
  }

  return false;
}

std::vector<HybridAStarPlanner::Node*>
HybridAStarPlanner::reconstruct( Node* node )
{
  std::vector<Node*> path;

  while( node )
  {
    path.push_back( node );
    node = node->parent;
  }

  std::reverse( path.begin(), path.end() );

  return path;
}

double
HybridAStarPlanner::distance_to_previous_route( double x, double y )
{
  if( !has_previous_route || previous_route.reference_line.empty() )
  {
    return 0.0;
  }

  double best = std::numeric_limits<double>::max();

  for( const auto& [s, p] : previous_route.reference_line )
  {
    double dx = x - p.x;
    double dy = y - p.y;

    best = std::min( best, std::hypot( dx, dy ) );
  }

  return best;
}

double
HybridAStarPlanner::route_difference( const map::Route& r1, const map::Route& r2 )
{
  if( r1.reference_line.empty() || r2.reference_line.empty() )
  {
    return std::numeric_limits<double>::max();
  }

  double max_diff = 0.0;

  for( const auto& [s, p] : r1.reference_line )
  {
    auto pose = r2.get_pose_at_s( s );

    double dx = p.x - pose.x;
    double dy = p.y - pose.y;

    max_diff = std::max( max_diff, std::hypot( dx, dy ) );
  }

  return max_diff;
}

double
HybridAStarPlanner::find_closest_s_on_route( const map::Route& route, const dynamics::VehicleStateDynamic& ego )
{
  double best_s    = 0.0;
  double best_dist = std::numeric_limits<double>::max();

  for( const auto& [s, p] : route.reference_line )
  {
    double dx = ego.x - p.x;
    double dy = ego.y - p.y;

    double d = std::hypot( dx, dy );

    if( d < best_dist )
    {
      best_dist = d;
      best_s    = s;
    }
  }

  return best_s;
}

map::Route
HybridAStarPlanner::trim_route_from_ego( const map::Route& route, const dynamics::VehicleStateDynamic& ego )
{
  map::Route trimmed;

  if( route.reference_line.empty() )
    return trimmed;

  double s_ego = find_closest_s_on_route( route, ego );

  for( const auto& [s, p] : route.reference_line )
  {
    if( s < s_ego )
      continue;

    map::MapPoint mp;

    mp.x = p.x;
    mp.y = p.y;

    trimmed.reference_line[s - s_ego] = mp;
  }

  return trimmed;
}

// ======================================================
// MAIN PLANNER
// ======================================================

map::Route
HybridAStarPlanner::plan( const dynamics::VehicleStateDynamic& ego, const dynamics::TrafficParticipantSet& participants,
                          const std::optional<math::Polygon2d>& drivable_area, const map::Route& latest_route )
{
  std::priority_queue<QueueNode> open_set;

  std::unordered_map<std::tuple<int, int, int>, double, GridHash> visited;

  std::deque<Node> nodes;

  // ============================================================
  // PLANNING MODE
  // ============================================================

  const bool has_drivable_area = drivable_area.has_value();

  // std::cerr << "Hybrid A* planning mode: " << ( has_drivable_area ? "DRIVABLE AREA" : "FREE SPACE" ) << std::endl;

  // ============================================================
  // START VALIDATION
  // ============================================================

  if( !inside_drivable_area( ego.x, ego.y, ego.yaw_angle, drivable_area ) )
  {
    std::cerr << "Start outside drivable area" << std::endl;

    return map::Route();
  }

  // ============================================================
  // COMPUTE LOCAL GOAL
  // ============================================================

  math::Point2d local_goal = compute_local_goal( ego, drivable_area );
  const double dx = local_goal.x - ego.x;
  const double dy = local_goal.y - ego.y;
  
  const double dist = std::hypot( dx, dy );
  if( dist < 8.0 )
  {
    distance_weight = 2.5;
    heading_weight = 3.0;
  }
  // planner_message += " distance: " + std::to_string(dist);

  Node* start = nullptr;

  map::Route frozen_route;

  std::vector<PathState> frozen_states;

  double frozen_length = 0.0;

  bool stitched = false;

  // Bounded window of previous-path states used for the per-node
  // continuity cost during search (see COSTS below, step 7).
  // Restricted to roughly the region this cycle's search can
  // actually reach, instead of scanning the whole stored history
  // for every single expanded node.
  std::vector<PathState> relevant_prev_states;

  // ============================================================
  // PREVIOUS ROUTE STITCHING + CONTINUITY WINDOW
  // ============================================================

  if( has_previous_route && !previous_path_states.empty() )
  {
    const double s_ego = find_closest_s_on_route( previous_route, ego );

    // ----------------------------------------------------------
    // Continuity window
    // ----------------------------------------------------------

    const double window_min = s_ego - PREV_ROUTE_LOOKUP_MARGIN;
    const double window_max = s_ego + LOCAL_GOAL_MAX_DIST + PREV_ROUTE_LOOKUP_MARGIN;

    for( const auto& st : previous_path_states )
    {
      if( st.s < window_min || st.s > window_max )
      {
        continue;
      }

      relevant_prev_states.push_back( st );
    }

    // ----------------------------------------------------------
    // Stitching (unchanged from before, just re-using s_ego
    // computed above instead of recomputing it)
    // ----------------------------------------------------------

    std::vector<PathState> shifted = trim_states_from_s( previous_path_states, s_ego );

    const PathState* stitch_state = find_state_at_or_after_s( shifted, STITCH_DISTANCE );

    if( stitch_state != nullptr && stitch_state->s > MIN_PREV_PATH_LENGTH_FOR_STITCH )
    {
      std::vector<PathState> candidate_frozen;

      bool safe = true;

      for( const auto& st : shifted )
      {
        if( st.s > stitch_state->s + 1e-6 )
        {
          break;
        }

        if( collision( st.x, st.y, participants ) )
        {
          safe = false;
          break;
        }

        if( !inside_drivable_area( st.x, st.y, st.yaw, drivable_area ) )
        {
          safe = false;
          break;
        }

        candidate_frozen.push_back( st );
      }

      if( safe && !candidate_frozen.empty() )
      {
        frozen_states = candidate_frozen;

        frozen_length = frozen_states.back().s;

        for( const auto& st : frozen_states )
        {
          map::MapPoint mp;

          mp.x = st.x;
          mp.y = st.y;

          frozen_route.reference_line[st.s] = mp;
        }

        const PathState& seed = frozen_states.back();

        nodes.emplace_back( seed.x, seed.y, seed.yaw, 0.0, seed.steer, nullptr );

        start = &nodes.back();

        stitched = true;
      }
    }
  }

  // ============================================================
  // START NODE
  // ============================================================

  if( !stitched )
  {
    nodes.emplace_back( ego.x, ego.y, ego.yaw_angle, 0.0, ego.steering_angle, nullptr );

    start = &nodes.back();
  }


  // ============================================================
  // INIT
  // ============================================================

  int counter = 0;

  open_set.push( { 0.0, counter++, start } );

  static const std::vector<double> steering_set = { -MAX_STEER,       -0.66 * MAX_STEER, -0.33 * MAX_STEER, 0.0,
                                                    0.33 * MAX_STEER, 0.66 * MAX_STEER,  MAX_STEER };

  // ============================================================
  // BEST NODE TRACKING
  // ============================================================

  Node* best_node = start;

  double best_goal_dist = std::hypot( local_goal.x - start->x, local_goal.y - start->y );

  // ============================================================
  // SEARCH
  // ============================================================

  int expansions = 0;

  while( !open_set.empty() )
  {
    if( ++expansions > MAX_EXPANSIONS )
    {
      planner_message += " Hybrid A* expansion limit reached (" + std::to_string( MAX_EXPANSIONS ) + "), using best node found so far ";
      break;
    }

    Node* current = open_set.top().node;

    open_set.pop();

    auto grid = current->grid_index();

    if( visited.count( grid ) && visited[grid] <= current->g )
    {
      continue;
    }

    visited[grid] = current->g;

    // ==========================================================
    // TRACK BEST FRONTIER NODE
    // ==========================================================

    const double local_goal_dist = std::hypot( local_goal.x - current->x, local_goal.y - current->y );

    if( local_goal_dist < best_goal_dist )
    {
      best_goal_dist = local_goal_dist;

      best_node = current;
    }

    // ==========================================================
    // GOAL REACHED
    // ==========================================================

    if( local_goal_dist < GOAL_REACHED_RADIUS )
    {
      std::vector<std::pair<double, double>> connection;

      if( try_goal_connection( current, local_goal, participants, drivable_area, connection ) )
      {
        best_node = current;

        break;
      }
    }

    // ==========================================================
    // NODE EXPANSION
    // ==========================================================

    for( double steer : steering_set )
    {
      double nx = current->x;

      double ny = current->y;

      double nyaw = current->yaw;

      const bool valid = simulate_motion( nx, ny, nyaw, steer, participants, drivable_area );

      if( !valid )
      {
        continue;
      }

      // --------------------------------------------------------
      // LOCAL HORIZON
      //
      // Keep this restriction only when a drivable area exists.
      //
      // In free-space mode we allow the search to continue
      // toward the final goal.
      // --------------------------------------------------------

      if( has_drivable_area )
      {
        const double dist_from_ego = std::hypot( nx - ego.x, ny - ego.y );

        if( dist_from_ego > LOCAL_GOAL_MAX_DIST )
        {
          continue;
        }
      }

      // --------------------------------------------------------
      // COSTS
      // --------------------------------------------------------

      const double steer_change = std::abs( steer - current->steer );

      const double steer_cost = 3.0 * std::abs( steer );

      const double smooth_cost = 5.0 * steer_change;

      // Continuity cost vs. the previous plan - now penalizes
      // heading mismatch against the nearest previous path state
      // as well as position. A node that lands on top of the old
      // path but crosses it at a sharp angle used to score as
      // "free" here; it no longer does.
      const ContinuityCost cc = nearest_previous_state_cost( relevant_prev_states, nx, ny, nyaw );

      const double prev_dist_clamped = std::min( cc.distance, PREV_ROUTE_CONTINUITY_CLAMP );

      const double continuity_cost = 0.3 * prev_dist_clamped * prev_dist_clamped
                                   + PREV_ROUTE_HEADING_WEIGHT * cc.heading_diff * cc.heading_diff;

      const double g = current->g + STEP + steer_cost + smooth_cost + continuity_cost;


      nodes.emplace_back( nx, ny, nyaw, g, steer, current );

      Node* node = &nodes.back();

      const double h = heuristic( nx, ny, nyaw, local_goal );

      open_set.push( { g + h, counter++, node } );
    }
  }

  // ============================================================
  // RECONSTRUCT BEST PATH
  // ============================================================

  if( best_node )
  {
    auto path_nodes = reconstruct( best_node );

    map::Route candidate_route = frozen_route;

    std::vector<PathState> candidate_states = frozen_states;

    double s = frozen_length;

    const size_t start_i = ( frozen_length > 0.0 ) ? 1 : 0;

    for( size_t i = start_i; i < path_nodes.size(); ++i )
    {
      map::MapPoint p;

      p.x = path_nodes[i]->x;

      p.y = path_nodes[i]->y;

      if( i > 0 )
      {
        const double dx = path_nodes[i]->x - path_nodes[i - 1]->x;

        const double dy = path_nodes[i]->y - path_nodes[i - 1]->y;

        s += std::hypot( dx, dy );
      }

      candidate_route.reference_line[s] = p;

      PathState ps;

      ps.s = s;

      ps.x = path_nodes[i]->x;

      ps.y = path_nodes[i]->y;

      ps.yaw = path_nodes[i]->yaw;

      ps.steer = path_nodes[i]->steer;

      candidate_states.push_back( ps );
    }

    // ==========================================================
    // PREVIOUS ROUTE REUSE
    // ==========================================================

    if( has_previous_route )
    {
      const double diff = route_difference( candidate_route, previous_route );

      bool blocked = false;

      for( const auto& [s_prev, p] : previous_route.reference_line )
      {
        if( collision( p.x, p.y, participants ) )
        {
          blocked = true;
          break;
        }
      }

      if( !blocked && diff < 4.0 )
      {
        map::Route reusable = trim_route_from_ego( previous_route, ego );

        const double remaining_length = reusable.reference_line.empty() ? 0.0 : reusable.reference_line.rbegin()->first;

        if( remaining_length >= MIN_REMAINING_ROUTE_LENGTH )
        {
          return reusable;
        }
      }
    }

    // ==========================================================
    // SAVE CURRENT ROUTE
    // ==========================================================

    previous_route = candidate_route;

    previous_path_states = candidate_states;

    has_previous_route = true;

    return candidate_route;
  }

  return map::Route();
}

double
HybridAStarPlanner::compute_idm_velocity( const adore::map::Route& route, const adore::dynamics::VehicleStateDynamic& ego,
                      const adore::dynamics::TrafficParticipantSet& participants, double goal_distance )
{
  // IDM parameters
  const double v0    = unstructured_speed;
  const double a     = 1.5;
  const double b     = 1.5;
  const double T     = 1.2;
  double       s0    = 2.0;
  const double delta = 4.0;

  // Route corridor parameters
  const double lane_half_width = 1.75; // ~3.5m lane
  const double margin          = 0.5;

  double ego_v = ego.vx;

  double min_gap = goal_distance;
  double lead_v  = 0.0;

  // ---------------------------------------------------
  // Find closest blocking participant
  // ---------------------------------------------------
  for( const auto& [id, p] : participants.participants )
  {
    double best_dist = std::numeric_limits<double>::infinity();
    double best_s    = -1.0;
    double best_dx   = 0.0;
    double best_dy   = 0.0;

    // project participant onto route
    for( const auto& [s, pt] : route.reference_line )
    {
      double dx = p.state.x - pt.x;
      double dy = p.state.y - pt.y;

      double dist = dx * dx + dy * dy;

      if( dist < best_dist )
      {
        best_dist = dist;
        best_s    = s;
        best_dx   = dx;
        best_dy   = dy;
      }
    }

    if( best_s <= 0.0 )
      continue;

    // lateral distance from route
    double lateral_dist = std::sqrt( best_dx * best_dx + best_dy * best_dy );

    // check if object blocks route
    if( lateral_dist > lane_half_width + margin )
      continue;

    // update closest obstacle
    if( best_s < min_gap )
    {
      min_gap = best_s;
      lead_v  = p.state.vx;
    }
  }
  if( goal_distance < 4 )
    s0 = 0.5;

  // ---------------------------------------------------
  // IDM calculation
  // ---------------------------------------------------

  double delta_v = ego_v - lead_v;

  double s_star = s0 + ego_v * T + ( ego_v * delta_v ) / ( 2 * std::sqrt( a * b ) );

  double accel = a * ( 1 - std::pow( ego_v / v0, delta ) - std::pow( s_star / std::max( min_gap, 0.1 ), 2 ) );

  double v_next = ego_v + accel * 0.1;

  return std::clamp( v_next, 0.0, v0 );
}

mas::MotionModel
HybridAStarPlanner::get_planning_model( const dynamics::PhysicalVehicleParameters& params )
{
  return [params]( const mas::State& x, const mas::Control& u ) -> mas::StateDerivative {
    mas::StateDerivative dxdt;
    dxdt.setZero( 6 );
    dxdt( 0 ) = x( 3 ) * std::cos( x( 2 ) );                    // x
    dxdt( 1 ) = x( 3 ) * std::sin( x( 2 ) );                    // y
    dxdt( 2 ) = x( 3 ) * std::tan( x( 5 ) ) / params.wheelbase; // yaw_angle
    dxdt( 3 ) = u( 1 );                                         // v
    dxdt( 4 ) = x( 3 );                                         // s
    dxdt( 5 ) = u( 0 );                                         // delta
    return dxdt;
  };
}

mas::StageCostFunction
HybridAStarPlanner::make_trajectory_cost( const map::Route& ref_route )
{
  return [start_state = start_state, ref_route = ref_route, ref_velocity = ref_velocity, weights = weights,
          dt = dt]( const mas::State& x, const mas::Control& u, std::size_t k ) -> double {
    double cost = 0.0;

    const double t   = k * dt;
    const auto   ref = ref_route.get_pose_at_s( x( 4 ) );

    const double dx = x( 0 ) - ref.x;
    const double dy = x( 1 ) - ref.y;

    const double c = math::fast_cos( ref.yaw );
    const double s = math::fast_sin( ref.yaw );

    const double lon_err = dx * c + dy * s;
    const double lat_err = -dx * s + dy * c;
    const double hdg_err = math::normalize_angle( x( 2 ) - ref.yaw );
    const double spd_err = x( 3 ) - ref_velocity;

    cost += weights.lane_error * lat_err * lat_err;
    cost += weights.long_error * lon_err * lon_err;
    cost += weights.heading_error * hdg_err * hdg_err;
    cost += weights.speed_error * spd_err * spd_err;
    cost += weights.steering_angle * u( 0 ) * u( 0 ) + weights.acceleration * u( 1 ) * u( 1 );
    return cost;
  };
}

PlannerResult
HybridAStarPlanner::plan_trajectory( const dynamics::VehicleStateDynamic&   current_state,
                                     const dynamics::TrafficParticipantSet& participants,
                                     const std::optional<math::Polygon2d>& drivable_area, const map::Route& latest_route )
{
  PlannerResult planner_output;
  planner_message.clear();
  all_participants = participants;
  set_goal( latest_route, current_state, participants );
  map::Route ref_route = plan( current_state, participants, drivable_area, latest_route );
  if( ref_route.reference_line.size() < 2 )
  {
    planner_message += " no route found to goal";
    planner_output.planner_message = planner_message;
    return planner_output;
  }
  double goal_distance          = ref_route.reference_line.rbegin()->first - 1.0;
  ref_velocity                  = compute_idm_velocity( ref_route, current_state, all_participants, goal_distance );
  planner_output.modified_route = ref_route;
  planner_output.trajectory     = optimize_trajectory( current_state, ref_route );
  planner_output.planner_message = planner_message;
  return planner_output;
}

dynamics::Trajectory
HybridAStarPlanner::optimize_trajectory( const dynamics::VehicleStateDynamic& current_state, const map::Route& ref_route )
{
  start_state     = current_state;
  reference_route = ref_route;
  setup_problem();
  solve_problem();
  auto out_trajectory = extract_trajectory();
  return out_trajectory;
}

void
HybridAStarPlanner::solve_problem()
{
  mas::SolverParams params;
  params["max_iterations"] = solver_params.max_iterations;
  params["tolerance"]      = solver_params.tolerance;
  params["max_ms"]         = solver_params.max_ms;
  params["debug"]          = solver_params.debug;

  auto solve_with = [&]( auto&& solver, double max_ms ) {
    params["max_ms"] = max_ms;
    solver.set_params( params );
    solver.solve( *problem );
    problem->update_initial_with_best();
  };

  // first pass with collocation
  solve_with( mas::OSQPCollocation{}, 60 );
}

dynamics::Trajectory
HybridAStarPlanner::extract_trajectory()
{
  dynamics::Trajectory trajectory;
  trajectory.states.reserve( problem->horizon_steps );
  for( size_t i = 0; i < problem->horizon_steps; ++i )
  {
    dynamics::VehicleStateDynamic state;
    auto                          x = problem->best_states.col( i );
    auto                          u = problem->best_controls.col( i );

    state.x              = x( 0 );
    state.y              = x( 1 );
    state.yaw_angle      = math::normalize_angle( x( 2 ) );
    state.vx             = x( 3 );
    state.time           = start_state.time + i * dt;
    state.steering_angle = math::normalize_angle( x( 5 ) );
    state.ax             = u( 1 );

    trajectory.states.push_back( state );
  }

  return trajectory;
}

void
HybridAStarPlanner::setup_problem()
{
  problem = std::make_shared<mas::OCP>();

  problem->state_dim     = 6;
  problem->control_dim   = 2;
  problem->horizon_steps = horizon_steps;
  problem->dt            = dt;
  problem->initial_state = Eigen::VectorXd( 6 );
  problem->dynamics      = get_planning_model( vehicle_params );


  Eigen::VectorXd lower_bounds( problem->control_dim ), upper_bounds( problem->control_dim );
  lower_bounds << -0.3, vehicle_params.acceleration_min;
  upper_bounds << 0.3, vehicle_params.acceleration_max;
  problem->input_lower_bounds = lower_bounds;
  problem->input_upper_bounds = upper_bounds;
  problem->stage_cost         = make_trajectory_cost( reference_route );

  problem->initial_state << start_state.x, start_state.y, start_state.yaw_angle, start_state.vx, 0.0, start_state.steering_angle;

  // initialize best guess controls from guess trajectory
  problem->initial_controls = mas::ControlTrajectory::Zero( problem->control_dim, problem->horizon_steps );
  for( size_t i = 0; i < problem->horizon_steps; ++i )
  {
    double t                          = i * dt;
    problem->initial_controls( 1, i ) = start_state.ax; // steering
    problem->initial_controls( 0, i ) = 0.0;
  }

  problem->initialize_problem();
  problem->verify_problem();
}

} // namespace planner
} // namespace adore