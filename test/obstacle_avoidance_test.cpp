/********************************************************************************
 * Copyright (c) 2026 Contributors to the Eclipse Foundation
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <utility>

#include "planning/active_avoidance.hpp"
#include "planning/obstacle_avoidance.hpp"
#include "planning/speed_profiles.hpp"
#include "../src/obstacle_avoidance_internal.hpp"

namespace
{

adore::map::Route
make_straight_route( double length, double step )
{
  adore::map::Route route;
  for( double s = 0.0; s <= length + 1e-9; s += step )
  {
    adore::map::MapPoint point;
    point.x = s;
    point.y = 0.0;
    point.s = s;
    route.reference_line[s] = point;
  }
  return route;
}

adore::map::Route
make_straight_route_with_lane(
  double length,
  double step,
  double lane_half_width )
{
  auto route = make_straight_route( length, step );
  constexpr std::size_t lane_id = 1;
  constexpr std::size_t road_id = 1;

  for( auto& [s, point] : route.reference_line )
  {
    static_cast<void>( s );
    point.parent_id = lane_id;
  }

  const auto make_border =
    [&]( double y )
    {
      adore::map::Border border;
      adore::map::MapPoint start{ 0.0, y, lane_id };
      start.s = 0.0;
      adore::map::MapPoint end{ length, y, lane_id };
      end.s = length;
      border.points = { start, end };
      border.length = length;
      return border;
    };

  auto lane = std::make_shared<adore::map::Lane>();
  lane->id = lane_id;
  lane->road_id = road_id;
  lane->type = adore::map::driving;
  lane->length = length;
  lane->borders.inner = make_border( -lane_half_width );
  lane->borders.outer = make_border( lane_half_width );

  route.map = std::make_shared<adore::map::Map>();
  route.map->lanes[lane_id] = lane;
  adore::map::Road road;
  road.id = road_id;
  road.lanes.insert( lane );
  route.map->roads[road_id] = road;

  return route;
}

std::shared_ptr<adore::map::Lane>
add_horizontal_lane( adore::map::Route& route,
                     std::size_t lane_id,
                     std::size_t road_id,
                     double x_min,
                     double x_max,
                     double y_min,
                     double y_max,
                     bool left_of_reference,
                     adore::map::LaneType type = adore::map::driving )
{
  const double length = x_max - x_min;
  const auto make_border =
    [&]( double y )
    {
      adore::map::Border border;
      adore::map::MapPoint start{ x_min, y, lane_id };
      start.s = 0.0;
      adore::map::MapPoint end{ x_max, y, lane_id };
      end.s = length;
      border.points = { start, end };
      border.length = length;
      return border;
    };

  auto lane = std::make_shared<adore::map::Lane>();
  lane->id = lane_id;
  lane->road_id = road_id;
  lane->type = type;
  lane->left_of_reference = left_of_reference;
  lane->length = length;
  lane->borders.inner = make_border( y_min );
  lane->borders.outer = make_border( y_max );

  route.map->lanes[lane_id] = lane;
  auto& road = route.map->roads[road_id];
  road.id = road_id;
  road.lanes.insert( lane );
  return lane;
}

std::shared_ptr<adore::map::Lane>
add_vertical_lane( adore::map::Route& route,
                   std::size_t lane_id,
                   std::size_t road_id,
                   double x_min,
                   double x_max,
                   double y_min,
                   double y_max,
                   adore::map::LaneType type = adore::map::driving )
{
  const double length = y_max - y_min;
  const auto make_border =
    [&]( double x )
    {
      adore::map::Border border;
      adore::map::MapPoint start{ x, y_min, lane_id };
      start.s = 0.0;
      adore::map::MapPoint end{ x, y_max, lane_id };
      end.s = length;
      border.points = { start, end };
      border.length = length;
      return border;
    };

  auto lane = std::make_shared<adore::map::Lane>();
  lane->id = lane_id;
  lane->road_id = road_id;
  lane->type = type;
  lane->left_of_reference = false;
  lane->length = length;
  lane->borders.inner = make_border( x_min );
  lane->borders.outer = make_border( x_max );

  const double center_x = 0.5 * ( x_min + x_max );
  for( const double y : { y_min, 0.5 * ( y_min + y_max ), y_max } )
  {
    adore::map::MapPoint center{ center_x, y, lane_id };
    center.s = y - y_min;
    lane->borders.center.interpolated_points.push_back( center );
    route.map->quadtree.insert( center );
  }

  route.map->lanes[lane_id] = lane;
  auto& road = route.map->roads[road_id];
  road.id = road_id;
  road.lanes.insert( lane );
  return lane;
}

adore::map::Route
make_intersection_route( double foreign_y_min,
                         double foreign_y_max,
                         adore::map::LaneType foreign_type = adore::map::driving,
                         std::size_t foreign_road_id = 2 )
{
  auto route = make_straight_route_with_lane( 30.0, 1.0, 1.5 );
  using Tree = Quadtree<adore::map::MapPoint>;
  route.map->quadtree = Tree(
    Tree::Boundary{ -50.0, 50.0, -50.0, 50.0 },
    8 );
  add_vertical_lane(
    route,
    2,
    foreign_road_id,
    7.0,
    23.0,
    foreign_y_min,
    foreign_y_max,
    foreign_type );
  return route;
}

adore::planner::oa_detail::AvoidanceGroup
make_validation_group()
{
  adore::planner::oa_detail::ObstacleEnvelope obstacle;
  obstacle.object_s_min = 10.0;
  obstacle.object_s_max = 20.0;
  obstacle.object_l_min = -1.0;
  obstacle.object_l_max = 1.0;
  obstacle.overlaps_ego_corridor = true;

  adore::planner::oa_detail::AvoidanceGroup group;
  group.obstacles.push_back( obstacle );
  group.envelope = obstacle;
  return group;
}

adore::dynamics::PhysicalVehicleParameters
validation_vehicle_params()
{
  adore::dynamics::PhysicalVehicleParameters vehicle_params;
  vehicle_params.acceleration_min = -2.0;
  vehicle_params.body_width = 2.0;
  vehicle_params.wheelbase = 2.0;
  vehicle_params.front_axle_to_front_border = 1.0;
  vehicle_params.rear_border_to_rear_axle = 1.0;
  return vehicle_params;
}

adore::dynamics::Trajectory
make_single_state_trajectory( double x, double y )
{
  adore::dynamics::VehicleStateDynamic state;
  state.x = x;
  state.y = y;
  state.yaw_angle = 0.0;

  adore::dynamics::Trajectory trajectory;
  trajectory.states.push_back( state );
  return trajectory;
}

double
max_speed_at_or_inf( const adore::map::Route& route, double s )
{
  const auto it = route.reference_line.find( s );
  if( it == route.reference_line.end() )
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return it->second.max_speed.value_or( std::numeric_limits<double>::infinity() );
}

adore::planner::ObstacleAvoidanceParams
test_params()
{
  adore::planner::ObstacleAvoidanceParams params;
  params.max_speed_during_avoidance = 2.0;
  params.planned_braking_deceleration = 1.0;
  return params;
}

adore::dynamics::PhysicalVehicleParameters
test_vehicle_params()
{
  adore::dynamics::PhysicalVehicleParameters vehicle_params;
  vehicle_params.acceleration_min = -2.0;
  return vehicle_params;
}

adore::dynamics::TrafficParticipant
make_participant(
  int id,
  double x,
  double y,
  double yaw,
  double speed,
  double length,
  double width )
{
  adore::dynamics::VehicleStateDynamic state;
  state.x = x;
  state.y = y;
  state.yaw_angle = yaw;
  state.vx = speed;
  state.time = 0.0;

  adore::dynamics::PhysicalVehicleParameters physical;
  physical.body_length = length;
  physical.body_width = width;

  return adore::dynamics::TrafficParticipant(
    state,
    id,
    adore::dynamics::CAR,
    physical );
}

adore::dynamics::Trajectory
make_linear_ego_trajectory(
  double start_x,
  double speed,
  double duration,
  double step )
{
  adore::dynamics::Trajectory trajectory;
  for( double t = 0.0; t <= duration + 1e-9; t += step )
  {
    adore::dynamics::VehicleStateDynamic state;
    state.x = start_x + speed * t;
    state.y = 0.0;
    state.yaw_angle = 0.0;
    state.vx = speed;
    state.time = t;
    trajectory.states.push_back( state );
  }
  return trajectory;
}

} // namespace

TEST( ObstacleAvoidance, ParameterValidationRejectsUnsafeRelationships )
{
  auto params = test_params();
  EXPECT_TRUE(
    adore::planner::validate_obstacle_avoidance_params( params ).empty() );

  params.side_clearance = 0.1;
  params.ego_corridor_safety_margin = 0.5;
  EXPECT_FALSE(
    adore::planner::validate_obstacle_avoidance_params( params ).empty() );

  params = test_params();
  params.side_clearance_replan_tolerance = -0.01;
  EXPECT_FALSE(
    adore::planner::validate_obstacle_avoidance_params( params ).empty() );

  params = test_params();
  params.intersection_drivable_area_search_radius = 0.0;
  EXPECT_FALSE(
    adore::planner::validate_obstacle_avoidance_params( params ).empty() );

}

TEST( ObstacleAvoidance, VehicleInputValidationRejectsInvalidGeometryAndBraking )
{
  auto vehicle_params = validation_vehicle_params();
  EXPECT_TRUE(
    adore::planner::validate_obstacle_avoidance_vehicle_params(
      vehicle_params ).empty() );

  vehicle_params.body_width = 0.0;
  EXPECT_FALSE(
    adore::planner::validate_obstacle_avoidance_vehicle_params(
      vehicle_params ).empty() );

  vehicle_params = validation_vehicle_params();
  vehicle_params.body_length = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
    adore::planner::validate_obstacle_avoidance_vehicle_params(
      vehicle_params ).empty() );

  vehicle_params = validation_vehicle_params();
  vehicle_params.acceleration_min = 0.0;
  EXPECT_FALSE(
    adore::planner::validate_obstacle_avoidance_vehicle_params(
      vehicle_params ).empty() );
  EXPECT_THROW(
    adore::planner::maximum_braking_deceleration(
      vehicle_params, test_params() ),
    std::invalid_argument );

  vehicle_params = validation_vehicle_params();
  vehicle_params.acceleration_min = -0.2;
  EXPECT_DOUBLE_EQ(
    adore::planner::maximum_braking_deceleration(
      vehicle_params, test_params() ),
    0.2 );
  EXPECT_DOUBLE_EQ(
    adore::planner::planned_braking_deceleration(
      vehicle_params, test_params() ),
    0.2 );
}

TEST( ObstacleAvoidance, ParticipantDimensionValidationRejectsZeroOrNonfinite )
{
  auto participant =
    make_participant( 99, 10.0, 0.0, 0.0, 0.0, 4.0, 2.0 );
  EXPECT_TRUE(
    adore::planner::participant_has_valid_dimensions( participant ) );

  participant.physical_parameters.body_length = 0.0;
  EXPECT_FALSE(
    adore::planner::participant_has_valid_dimensions( participant ) );

  participant.physical_parameters.body_length = 4.0;
  participant.physical_parameters.body_width =
    std::numeric_limits<double>::infinity();
  EXPECT_FALSE(
    adore::planner::participant_has_valid_dimensions( participant ) );

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace( participant.id, participant );
  EXPECT_FALSE(
    adore::planner::traffic_participants_have_valid_dimensions(
      participants ) );

  const auto footprint =
    adore::planner::oa_detail::project_participant_footprint_to_route(
      make_straight_route( 50.0, 1.0 ),
      participant,
      test_params() );
  EXPECT_FALSE( footprint.has_value() );
}

TEST( ObstacleAvoidance, SensorRangeIsNotTruncatedByPlannerDistanceLimits )
{
  const auto route = make_straight_route( 400.0, 1.0 );

  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 0.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    79,
    make_participant( 79, 250.0, 0.0, 0.0, 0.0, 4.0, 2.0 ) );

  auto params = test_params();
  params.max_static_object_speed = 0.1;

  const auto group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      participants,
      validation_vehicle_params(),
      params );

  ASSERT_TRUE( group.has_value() );
  ASSERT_EQ( group->obstacles.size(), 1U );
  EXPECT_EQ( group->obstacles.front().id, 79 );

  const auto corridor_result =
    adore::planner::check_route_corridor_safety(
      route,
      ego,
      participants,
      validation_vehicle_params(),
      params );

  ASSERT_TRUE( corridor_result.has_conflict );
  EXPECT_EQ( corridor_result.conflict.participant_id, 79 );
}

TEST( ObstacleAvoidance, StaticActionHorizonUsesReactionBrakingAndStandOffDistances )
{
  auto vehicle_params = validation_vehicle_params();
  vehicle_params.body_length = 4.0;
  auto params = test_params();
  params.stop_before_obstacle = 8.0;
  params.modified_route_braking_safety_margin = 2.0;
  params.planned_braking_deceleration = 1.0;

  // ego_s + front offset + one 100 ms cycle + v^2/(2a) + stand-off + reserve
  // = 10 + 3 + 0.4 + 8 + 8 + 2 = 31.4 m.
  EXPECT_NEAR(
    adore::planner::static_obstacle_action_end_s(
      10.0, 4.0, vehicle_params, params ),
    31.4,
    1e-9 );

  EXPECT_TRUE(
    std::isinf(
      adore::planner::static_obstacle_action_end_s(
        10.0,
        std::numeric_limits<double>::quiet_NaN(),
        vehicle_params,
        params ) ) );
}

TEST( ObstacleAvoidance, NewPlanSelectsOnlyNearestConnectedShiftComponent )
{
  const auto route = make_straight_route( 120.0, 1.0 );
  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 0.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    91,
    make_participant( 91, 20.0, 0.0, 0.0, 0.0, 4.0, 2.0 ) );
  participants.participants.emplace(
    92,
    make_participant( 92, 80.0, 0.0, 0.0, 0.0, 4.0, 2.0 ) );

  auto vehicle_params = validation_vehicle_params();
  vehicle_params.body_length = 4.0;
  const auto group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      participants,
      vehicle_params,
      test_params() );

  ASSERT_TRUE( group.has_value() );
  ASSERT_EQ( group->obstacles.size(), 1U );
  EXPECT_EQ( group->obstacles.front().id, 91 );
}

TEST( ObstacleAvoidance, OverlappingShiftSupportsRemainOneManeuver )
{
  const auto route = make_straight_route( 100.0, 1.0 );
  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 0.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    93,
    make_participant( 93, 20.0, 0.0, 0.0, 0.0, 4.0, 2.0 ) );
  participants.participants.emplace(
    94,
    make_participant( 94, 38.0, 0.0, 0.0, 0.0, 4.0, 2.0 ) );

  auto vehicle_params = validation_vehicle_params();
  vehicle_params.body_length = 4.0;
  const auto group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      participants,
      vehicle_params,
      test_params() );

  ASSERT_TRUE( group.has_value() );
  ASSERT_EQ( group->obstacles.size(), 2U );
  EXPECT_EQ( group->obstacles.front().id, 93 );
  EXPECT_EQ( group->obstacles.back().id, 94 );
}

TEST( ObstacleAvoidance, FarStaticObjectDoesNotRequireItsMissingLocalLaneMap )
{
  auto route = make_straight_route_with_lane( 120.0, 1.0, 5.0 );
  // Model Mission Control's full reference line with a local map that has not
  // loaded the later route lane yet.
  for( auto& [s, point] : route.reference_line )
  {
    if( s >= 50.0 )
    {
      point.parent_id = 2;
    }
  }

  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 0.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;

  adore::dynamics::TrafficParticipantSet near_and_far;
  near_and_far.participants.emplace(
    95,
    make_participant( 95, 20.0, 0.0, 0.0, 0.0, 4.0, 2.0 ) );
  near_and_far.participants.emplace(
    96,
    make_participant( 96, 80.0, 0.0, 0.0, 0.0, 4.0, 2.0 ) );

  auto vehicle_params = validation_vehicle_params();
  vehicle_params.body_length = 4.0;
  const auto params = test_params();
  const auto nearest_group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      near_and_far,
      vehicle_params,
      params );

  ASSERT_TRUE( nearest_group.has_value() );
  ASSERT_EQ( nearest_group->obstacles.size(), 1U );
  EXPECT_EQ( nearest_group->obstacles.front().id, 95 );
  EXPECT_TRUE(
    adore::planner::oa_detail::candidate_respects_drivable_area(
      route,
      nearest_group.value(),
      3.0,
      vehicle_params,
      params,
      false ) );

  adore::dynamics::TrafficParticipantSet far_only;
  far_only.participants.emplace(
    96,
    near_and_far.participants.at( 96 ) );
  const auto far_group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      far_only,
      vehicle_params,
      params );
  ASSERT_TRUE( far_group.has_value() );

  auto combined_obstacles = nearest_group->obstacles;
  combined_obstacles.insert(
    combined_obstacles.end(),
    far_group->obstacles.begin(),
    far_group->obstacles.end() );
  const auto old_all_objects_group =
    adore::planner::oa_detail::make_avoidance_group_from_obstacles(
      std::move( combined_obstacles ) );
  EXPECT_FALSE(
    adore::planner::oa_detail::candidate_respects_drivable_area(
      route,
      old_all_objects_group,
      3.0,
      vehicle_params,
      params,
      false ) );
}

TEST( ObstacleAvoidance, StaticCorridorScopeDefersOnlyStaticObjectsBeyondIt )
{
  const auto route = make_straight_route( 120.0, 1.0 );
  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 0.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;
  ego.vx = 2.0;

  auto vehicle_params = validation_vehicle_params();
  vehicle_params.body_length = 4.0;
  const auto params = test_params();

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    97,
    make_participant( 97, 80.0, 0.0, 0.0, 0.0, 4.0, 2.0 ) );

  const auto deferred_static =
    adore::planner::check_route_corridor_safety(
      route,
      ego,
      participants,
      vehicle_params,
      params,
      nullptr,
      30.0 );
  EXPECT_TRUE( deferred_static.safe );
  EXPECT_FALSE( deferred_static.has_conflict );

  const auto actionable_static =
    adore::planner::check_route_corridor_safety(
      route,
      ego,
      participants,
      vehicle_params,
      params,
      nullptr,
      90.0 );
  EXPECT_FALSE( actionable_static.safe );
  ASSERT_TRUE( actionable_static.has_conflict );
  EXPECT_EQ( actionable_static.conflict.participant_id, 97 );
  EXPECT_EQ(
    actionable_static.conflict.object_class,
    adore::planner::RouteCorridorObjectClass::StaticOrSlow );

  // An invalid backwards limit must fail safe to the full route rather than
  // silently suppressing static conflicts.
  const auto invalid_scope =
    adore::planner::check_route_corridor_safety(
      route,
      ego,
      participants,
      vehicle_params,
      params,
      nullptr,
      -1.0 );
  EXPECT_FALSE( invalid_scope.safe );
  EXPECT_TRUE( invalid_scope.has_conflict );

  participants.participants.at( 97 ).state.vx = 2.0;
  const auto moving_still_checked =
    adore::planner::check_route_corridor_safety(
      route,
      ego,
      participants,
      vehicle_params,
      params,
      nullptr,
      30.0 );
  EXPECT_FALSE( moving_still_checked.safe );
  ASSERT_TRUE( moving_still_checked.has_conflict );
  EXPECT_EQ( moving_still_checked.conflict.participant_id, 97 );
  EXPECT_NE(
    moving_still_checked.conflict.object_class,
    adore::planner::RouteCorridorObjectClass::StaticOrSlow );
}

TEST( ObstacleAvoidance, WideIntrudingObjectIsNotDiscardedByFarOuterEdge )
{
  const auto route = make_straight_route( 100.0, 1.0 );

  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 0.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;

  adore::dynamics::TrafficParticipantSet participants;
  // The near edge reaches y=-1 m and therefore intrudes into the ego corridor,
  // while the far edge reaches y=19 m. A maximum absolute lateral-distance
  // filter used to discard the complete object because of that far edge.
  participants.participants.emplace(
    81,
    make_participant( 81, 40.0, 9.0, 0.0, 0.0, 4.0, 20.0 ) );

  const auto group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      participants,
      validation_vehicle_params(),
      test_params() );

  ASSERT_TRUE( group.has_value() );
  ASSERT_EQ( group->obstacles.size(), 1U );
  EXPECT_EQ( group->obstacles.front().id, 81 );
  EXPECT_TRUE( group->obstacles.front().overlaps_ego_corridor );
}

TEST( ObstacleAvoidance, EgoLaneOncomingUsesSensorRangeAndTimeHorizonOnly )
{
  const auto route = make_straight_route( 400.0, 1.0 );

  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 0.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    80,
    make_participant(
      80,
      250.0,
      0.0,
      std::numbers::pi,
      5.0,
      4.0,
      2.0 ) );

  auto params = test_params();
  params.prediction_time_horizon = 0.0;

  const auto threat =
    adore::planner::oa_detail::find_ego_lane_oncoming_threat(
      route,
      ego,
      participants,
      validation_vehicle_params(),
      params );

  ASSERT_TRUE( threat.has_value() );
  EXPECT_EQ( threat->participant_id, 80 );
}

TEST( ObstacleAvoidance, ActiveConflictStopUsesSpatialBrakingPointNotFixedTtc )
{
  const auto route = make_straight_route( 100.0, 1.0 );
  auto params = test_params();
  params.stop_before_obstacle = 8.0;
  params.modified_route_braking_safety_margin = 5.0;

  adore::planner::RouteCorridorConflict conflict;
  conflict.object_class =
    adore::planner::RouteCorridorObjectClass::CrossingOrUnknown;
  conflict.object_s_min = 30.0;
  conflict.object_s_max = 32.0;
  conflict.time_to_conflict = 1.0;
  conflict.predicted_spatiotemporal_conflict = true;

  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 0.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;
  ego.vx = 4.0;

  EXPECT_FALSE(
    adore::planner::should_stop_for_active_conflict(
      route,
      ego,
      validation_vehicle_params(),
      conflict,
      params ) );

  ego.x = 7.0;
  EXPECT_TRUE(
    adore::planner::should_stop_for_active_conflict(
      route,
      ego,
      validation_vehicle_params(),
      conflict,
      params ) );
}

TEST( ObstacleAvoidance, PreShiftStopUsesOnlyConfiguredRampAndStandOff )
{
  auto params = test_params();
  params.front_clearance = 7.0;
  params.stop_before_obstacle = 5.0;

  EXPECT_NEAR(
    adore::planner::oa_detail::normalized_stop_before_obstacle( params ),
    7.0,
    1e-9 );
}

TEST( ObstacleAvoidance, AvoidanceSpeedProfileCapsOnlyAtShiftStart )
{
  const auto route = make_straight_route( 100.0, 10.0 );
  const auto params = test_params();
  const auto vehicle_params = test_vehicle_params();

  const auto profiled_route =
    adore::planner::RouteSpeedPolicy::apply_avoidance_speed_profile(
      route,
      0.0,
      50.0,
      70.0,
      vehicle_params,
      params );

  EXPECT_GT( max_speed_at_or_inf( profiled_route, 0.0 ), params.max_speed_during_avoidance );
  EXPECT_GT( max_speed_at_or_inf( profiled_route, 40.0 ), params.max_speed_during_avoidance );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 50.0 ), params.max_speed_during_avoidance, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 70.0 ), params.max_speed_during_avoidance, 1e-9 );
  EXPECT_TRUE( std::isinf( max_speed_at_or_inf( profiled_route, 80.0 ) ) );
}

TEST( ObstacleAvoidance, SegmentedAvoidanceSpeedRestoresMissionSpeedInLongGap )
{
  auto route = make_straight_route( 110.0, 10.0 );
  for( auto& [s, point] : route.reference_line )
  {
    static_cast<void>( s );
    point.max_speed = 8.0;
  }

  const auto params = test_params();
  const auto vehicle_params = test_vehicle_params();
  const std::vector<adore::planner::AvoidanceSpeedSegment> segments{
    { 20.0, 30.0 },
    { 80.0, 90.0 } };

  const auto profiled_route =
    adore::planner::RouteSpeedPolicy::
      apply_segmented_avoidance_speed_profile(
        route,
        0.0,
        segments,
        vehicle_params,
        params );

  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 20.0 ), 2.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 30.0 ), 2.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 40.0 ), 8.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 50.0 ), 8.0, 1e-9 );
  EXPECT_NEAR(
    max_speed_at_or_inf( profiled_route, 60.0 ),
    std::sqrt( 44.0 ),
    1e-9 );
  EXPECT_NEAR(
    max_speed_at_or_inf( profiled_route, 70.0 ),
    std::sqrt( 24.0 ),
    1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 80.0 ), 2.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 90.0 ), 2.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 100.0 ), 8.0, 1e-9 );
}

TEST( ObstacleAvoidance, SegmentedAvoidanceSpeedKeepsShortGapBrakeSafe )
{
  auto route = make_straight_route( 70.0, 5.0 );
  for( auto& [s, point] : route.reference_line )
  {
    static_cast<void>( s );
    point.max_speed = 8.0;
  }

  const auto params = test_params();
  const auto vehicle_params = test_vehicle_params();
  const std::vector<adore::planner::AvoidanceSpeedSegment> segments{
    { 20.0, 30.0 },
    { 40.0, 50.0 } };

  const auto profiled_route =
    adore::planner::RouteSpeedPolicy::
      apply_segmented_avoidance_speed_profile(
        route,
        0.0,
        segments,
        vehicle_params,
        params );

  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 30.0 ), 2.0, 1e-9 );
  EXPECT_NEAR(
    max_speed_at_or_inf( profiled_route, 35.0 ),
    std::sqrt( 14.0 ),
    1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 40.0 ), 2.0, 1e-9 );
}

TEST( ObstacleAvoidance, SegmentedAvoidanceSpeedMergesOverlappingRamps )
{
  auto route = make_straight_route( 80.0, 10.0 );
  for( auto& [s, point] : route.reference_line )
  {
    static_cast<void>( s );
    point.max_speed = 8.0;
  }

  const auto params = test_params();
  const auto vehicle_params = test_vehicle_params();
  const std::vector<adore::planner::AvoidanceSpeedSegment> segments{
    { 20.0, 40.0 },
    { 35.0, 60.0 } };

  const auto profiled_route =
    adore::planner::RouteSpeedPolicy::
      apply_segmented_avoidance_speed_profile(
        route,
        0.0,
        segments,
        vehicle_params,
        params );

  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 20.0 ), 2.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 30.0 ), 2.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 40.0 ), 2.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 50.0 ), 2.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 60.0 ), 2.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( profiled_route, 70.0 ), 8.0, 1e-9 );
}

TEST( ObstacleAvoidance, AvoidanceSpeedNeverOverridesPhysicsWithCreepFloor )
{
  auto params = test_params();
  params.max_speed_during_avoidance = 1.0;
  params.avoidance_lateral_accel = 2.0;

  EXPECT_NEAR(
    adore::planner::avoidance_speed_for_shift(
      1.0, 4.0, params ),
    std::sqrt( 2.0 / 24.0 ),
    1e-9 );
}

TEST( ObstacleAvoidance, SafeOncomingMonitorResultNeverStops )
{
  adore::planner::ObstacleAvoidanceMonitorResult monitor_result;
  monitor_result.safe_to_continue = true;

  EXPECT_FALSE(
    adore::planner::should_stop_for_oncoming_monitor_result(
      monitor_result ) );
}

TEST( ObstacleAvoidance, PostCommitmentOncomingConflictAlwaysStops )
{
  adore::planner::ObstacleAvoidanceMonitorResult monitor_result;
  monitor_result.safe_to_continue = false;
  monitor_result.already_committed = true;
  monitor_result.oncoming.conflict = true;
  monitor_result.oncoming.oncoming_arrival_time = 5.0;

  EXPECT_TRUE(
    adore::planner::should_stop_for_oncoming_monitor_result(
      monitor_result ) );
}

TEST( ObstacleAvoidance, ImmediateOrPreCommitmentOncomingAlwaysStops )
{
  adore::planner::ObstacleAvoidanceMonitorResult pre_commitment;
  pre_commitment.safe_to_continue = false;
  pre_commitment.should_abort_before_commitment = true;
  pre_commitment.oncoming.conflict = true;
  pre_commitment.oncoming.oncoming_arrival_time = 5.0;
  EXPECT_TRUE(
    adore::planner::should_stop_for_oncoming_monitor_result(
      pre_commitment ) );

  adore::planner::ObstacleAvoidanceMonitorResult already_inside;
  already_inside.safe_to_continue = false;
  already_inside.already_committed = true;
  already_inside.oncoming.conflict = true;
  already_inside.oncoming.oncoming_arrival_time = 0.0;
  EXPECT_TRUE(
    adore::planner::should_stop_for_oncoming_monitor_result(
      already_inside ) );
}

TEST( ObstacleAvoidance, StopBeforeObstacleDoesNotImmediatelySetZeroEverywhere )
{
  const auto route = make_straight_route( 100.0, 5.0 );
  const auto params = test_params();
  const auto vehicle_params = test_vehicle_params();

  const auto stop_plan =
    adore::planner::RouteStopPolicy::plan_stop_on_route(
      route,
      0.0,
      4.0,
      30.0,
      vehicle_params,
      params );

  ASSERT_TRUE( stop_plan.valid );
  EXPECT_GT( max_speed_at_or_inf( stop_plan.route, 0.0 ), 0.0 );
  EXPECT_GT( max_speed_at_or_inf( stop_plan.route, 20.0 ), 0.0 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 25.0 ), std::sqrt( 10.0 ), 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 30.0 ), 0.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 40.0 ), 0.0, 1e-9 );
}

TEST( ObstacleAvoidance, StopProfileFallsBackToMaximumBrakingWhenUnreachable )
{
  const auto route = make_straight_route( 100.0, 10.0 );
  const auto params = test_params();
  const auto vehicle_params = test_vehicle_params();

  const auto stop_plan =
    adore::planner::RouteStopPolicy::plan_stop_on_route(
      route,
      20.0,
      10.0,
      25.0,
      vehicle_params,
      params );

  ASSERT_TRUE( stop_plan.valid );
  EXPECT_TRUE( std::isinf( max_speed_at_or_inf( stop_plan.route, 10.0 ) ) );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 20.0 ), 10.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 30.0 ), std::sqrt( 60.0 ), 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 40.0 ), std::sqrt( 20.0 ), 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 45.0 ), 0.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 50.0 ), 0.0, 1e-9 );
}

TEST( ObstacleAvoidance, ReachableStopTargetStaysFixedWithIntermediateBraking )
{
  const auto route = make_straight_route( 100.0, 10.0 );
  const auto params = test_params();
  const auto vehicle_params = test_vehicle_params();

  // From 10 m/s, comfort braking at 1 m/s² needs 50 m and maximum braking
  // at 2 m/s² needs 25 m. The requested target is 40 m ahead, so it must
  // remain fixed and use the required intermediate 1.25 m/s².
  const auto stop_plan =
    adore::planner::RouteStopPolicy::plan_stop_on_route(
      route,
      20.0,
      10.0,
      60.0,
      vehicle_params,
      params );

  ASSERT_TRUE( stop_plan.valid );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 20.0 ), 10.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 30.0 ), std::sqrt( 75.0 ), 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 50.0 ), 5.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 60.0 ), 0.0, 1e-9 );
  EXPECT_NEAR( max_speed_at_or_inf( stop_plan.route, 70.0 ), 0.0, 1e-9 );
}

TEST( ObstacleAvoidance, ExactRouteStopProofChecksTargetAndRemainingBrakingDistance )
{
  const auto route = make_straight_route( 30.0, 1.0 );
  const auto params = test_params();
  const auto vehicle_params = test_vehicle_params();

  adore::dynamics::Trajectory horizon_before_target;
  horizon_before_target.states =
    make_single_state_trajectory( 6.0, 0.0 ).states;
  horizon_before_target.states.back().vx = 2.0;
  EXPECT_TRUE(
    adore::planner::trajectory_stops_by_route_s(
      horizon_before_target,
      route,
      10.0,
      vehicle_params,
      params ) );

  adore::dynamics::Trajectory crosses_target_moving;
  crosses_target_moving.states =
    make_single_state_trajectory( 10.0, 0.0 ).states;
  crosses_target_moving.states.back().vx = 1.0;
  EXPECT_FALSE(
    adore::planner::trajectory_stops_by_route_s(
      crosses_target_moving,
      route,
      10.0,
      vehicle_params,
      params ) );

  adore::dynamics::Trajectory stopped_at_target;
  stopped_at_target.states =
    make_single_state_trajectory( 10.0, 0.0 ).states;
  stopped_at_target.states.back().vx = 0.0;
  EXPECT_TRUE(
    adore::planner::trajectory_stops_by_route_s(
      stopped_at_target,
      route,
      10.0,
      vehicle_params,
      params ) );
}

TEST( ObstacleAvoidance, PhysicalRouteStopProofAcceptsReachableIntermediateBraking )
{
  const auto route = make_straight_route( 100.0, 1.0 );
  const auto params = test_params();
  const auto vehicle_params = test_vehicle_params();

  // At 10 m/s, stopping in the remaining 40 m needs 1.25 m/s². That is
  // stronger than the configured 1.0 m/s² comfort deceleration, but below the
  // vehicle's 2.0 m/s² maximum. The route envelope intentionally supports this
  // intermediate regime, so the physical safety proof must accept it.
  auto trajectory = make_single_state_trajectory( 20.0, 0.0 );
  trajectory.states.back().vx = 10.0;

  EXPECT_FALSE(
    adore::planner::trajectory_stops_by_route_s(
      trajectory,
      route,
      60.0,
      vehicle_params,
      params ) );
  EXPECT_TRUE(
    adore::planner::trajectory_stops_by_route_s(
      trajectory,
      route,
      60.0,
      vehicle_params,
      params,
      true ) );
}

TEST( ObstacleAvoidance, RouteTrajectoryStopsAtFirstZeroSpeedPoint )
{
  const auto route = make_straight_route( 30.0, 10.0 );
  adore::planner::SpeedProfile speed_profile;
  speed_profile.s_to_speed = {
    { 0.0, 4.0 },
    { 10.0, 0.0 },
    { 20.0, 0.0 },
    { 30.0, 2.0 }
  };

  adore::dynamics::VehicleStateDynamic ego;
  ego.time = 42.0;

  const auto trajectory =
    adore::planner::generate_trajectory_from_speed_profile(
      speed_profile, route, ego, 0.1 );

  ASSERT_FALSE( trajectory.states.empty() );
  // Relative clock: the sampler emits t=0..t_final; the caller re-bases to ego.time.
  EXPECT_NEAR( trajectory.states.front().time, 0.0, 1e-9 );
  EXPECT_NEAR( trajectory.states.back().x, 10.0, 1e-9 );
  EXPECT_NEAR( trajectory.states.back().vx, 0.0, 1e-9 );
  for( const auto& state : trajectory.states )
  {
    EXPECT_LE( state.x, 10.0 + 1e-9 );
  }
}

// Regression: the stop-at-zero guard must key off the segment END speed, not the
// start. A route whose first point is zero speed (ego starting from standstill)
// must still build a moving trajectory and drive off, not truncate to a hold.
TEST( ObstacleAvoidance, RouteFromStandstillDrivesOff )
{
  const auto route = make_straight_route( 50.0, 10.0 );
  adore::planner::SpeedProfile speed_profile;
  speed_profile.s_to_speed = {
    { 0.0, 0.0 },   // ego currently stopped
    { 10.0, 3.0 },
    { 20.0, 5.0 },
    { 30.0, 5.0 }
  };

  adore::dynamics::VehicleStateDynamic ego;
  ego.time = 0.0;

  const auto trajectory =
    adore::planner::generate_trajectory_from_speed_profile(
      speed_profile, route, ego, 0.1 );

  ASSERT_GT( trajectory.states.size(), 2U );
  EXPECT_NEAR( trajectory.states.front().x, 0.0, 1e-9 );
  EXPECT_GT( trajectory.states.back().x, 1.0 );

  bool moves = false;
  for( const auto& state : trajectory.states )
  {
    if( state.vx > 1.0 )
    {
      moves = true;
    }
  }
  EXPECT_TRUE( moves );
}

TEST( ObstacleAvoidance, ZeroSpeedRouteProducesRouteAlignedHold )
{
  const auto route = make_straight_route( 20.0, 10.0 );
  adore::planner::SpeedProfile speed_profile;
  speed_profile.s_to_speed = {
    { 0.0, 0.0 },
    { 10.0, 0.0 },
    { 20.0, 0.0 }
  };

  adore::dynamics::VehicleStateDynamic ego;
  ego.time = 7.0;

  const auto trajectory =
    adore::planner::generate_trajectory_from_speed_profile(
      speed_profile, route, ego, 0.1 );

  ASSERT_EQ( trajectory.states.size(), 2U );
  // Relative clock (see above): hold is [t=0, t=time_step].
  EXPECT_NEAR( trajectory.states.front().time, 0.0, 1e-9 );
  EXPECT_NEAR( trajectory.states.back().time, 0.1, 1e-9 );
  EXPECT_NEAR( trajectory.states.front().x, 0.0, 1e-9 );
  EXPECT_NEAR( trajectory.states.back().x, 0.0, 1e-9 );
  EXPECT_NEAR( trajectory.states.back().vx, 0.0, 1e-9 );
}

// The configured shift policy is symmetric around the obstacle: the full-shift
// plateau starts and ends exactly half an ego body length outside its s-bounds.
TEST( ObstacleAvoidance, ShiftPlateauUsesSymmetricEgoHalfLength )
{
  adore::planner::oa_detail::ObstacleEnvelope obstacle;
  obstacle.object_s_min = 20.0;
  obstacle.object_s_max = 25.0;
  obstacle.object_l_min = -0.5;
  obstacle.object_l_max = 0.5;
  obstacle.overlaps_ego_corridor = true;

  adore::planner::oa_detail::AvoidanceGroup group;
  group.obstacles.push_back( obstacle );

  auto vehicle_params = test_vehicle_params();
  vehicle_params.body_width = 2.0;

  auto params = test_params();
  params.front_clearance = 7.0;
  params.rear_clearance = 7.0;
  params.side_clearance = 1.0;
  constexpr double nominal_left_shift = 3.0;

  const auto offset_at = [&]( double s ) {
    return adore::planner::oa_detail::avoidance_shift_offset_at_s(
      s, group, nominal_left_shift, vehicle_params, params );
  };

  // Full shift (0.5 obstacle + 1.0 side_clearance + 1.0 ego half-width = 2.5)
  // spans the obstacle itself.
  EXPECT_NEAR( offset_at( 20.0 ), 2.5, 1e-9 );
  EXPECT_NEAR( offset_at( 22.5 ), 2.5, 1e-9 );
  EXPECT_NEAR( offset_at( 25.0 ), 2.5, 1e-9 );

  const double ego_half_length = 0.5 * vehicle_params.body_length;
  const double hold_start = 20.0 - ego_half_length;
  const double hold_end = 25.0 + ego_half_length;
  const double shift_start = hold_start - params.front_clearance;
  const double shift_end = hold_end + params.rear_clearance;

  EXPECT_NEAR( offset_at( shift_start - 0.1 ), 0.0, 1e-9 );
  EXPECT_NEAR( offset_at( shift_end + 0.1 ), 0.0, 1e-9 );
  EXPECT_NEAR( offset_at( hold_start ), 2.5, 1e-9 );
  EXPECT_NEAR( offset_at( hold_end ), 2.5, 1e-9 );
  EXPECT_NEAR( 20.0 - hold_start, hold_end - 25.0, 1e-9 );
}

TEST( ObstacleAvoidance, CommittedHullSurvivesMissingDetection )
{
  const auto route = make_straight_route( 100.0, 1.0 );
  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 0.0;
  ego.y = 0.0;

  adore::dynamics::TrafficParticipantSet participants;
  const auto vehicle_params = test_vehicle_params();
  auto params = test_params();

  adore::planner::AvoidanceShiftContribution contribution;
  contribution.participant_ids = { 17 };
  contribution.object_s_min = 25.0;
  contribution.object_s_max = 30.0;
  contribution.object_l_min = -0.5;
  contribution.object_l_max = 0.5;
  const std::vector<adore::planner::AvoidanceShiftContribution> committed = {
    contribution
  };

  const auto group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      participants,
      vehicle_params,
      params,
      &committed,
      nullptr );

  ASSERT_TRUE( group.has_value() );
  ASSERT_EQ( group->obstacles.size(), 1U );
  EXPECT_EQ( group->obstacles.front().participant_ids.front(), 17 );
  EXPECT_NEAR( group->obstacles.front().object_s_min, 25.0, 1e-9 );
  EXPECT_NEAR( group->obstacles.front().object_l_max, 0.5, 1e-9 );
}

TEST( ObstacleAvoidance, DrivenCorridorObjectIsForcedIntoMissionFrame )
{
  const auto route = make_straight_route( 100.0, 1.0 );
  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 0.0;
  ego.y = 0.0;

  adore::dynamics::VehicleStateDynamic object_state;
  object_state.x = 30.0;
  object_state.y = 3.0;
  object_state.yaw_angle = 0.0;
  object_state.vx = 0.0;

  adore::dynamics::PhysicalVehicleParameters object_params;
  object_params.body_length = 4.0;
  object_params.body_width = 2.0;

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    23,
    adore::dynamics::TrafficParticipant(
      object_state,
      23,
      adore::dynamics::CAR,
      object_params ) );

  auto vehicle_params = test_vehicle_params();
  vehicle_params.body_width = 2.0;
  auto params = test_params();
  params.ego_corridor_safety_margin = 0.5;

  const auto without_force =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route, ego, participants, vehicle_params, params );
  EXPECT_FALSE( without_force.has_value() );

  const std::vector<int> forced_ids = { 23 };
  const auto forced_group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      participants,
      vehicle_params,
      params,
      nullptr,
      &forced_ids );

  ASSERT_TRUE( forced_group.has_value() );
  ASSERT_EQ( forced_group->obstacles.size(), 1U );
  EXPECT_TRUE( forced_group->obstacles.front().overlaps_ego_corridor );
  EXPECT_GT( forced_group->obstacles.front().object_l_min, 1.5 );
}

TEST( ObstacleAvoidance, ForcedDrivenCorridorObjectMayProjectBehindRearAxle )
{
  const auto route = make_straight_route( 100.0, 1.0 );
  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 30.0;
  ego.y = 0.0;

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    24,
    make_participant( 24, 28.0, 0.0, 0.0, 0.0, 1.0, 1.0 ) );

  auto vehicle_params = test_vehicle_params();
  vehicle_params.body_width = 2.0;
  auto params = test_params();
  params.ego_corridor_safety_margin = 0.5;

  const auto without_force =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route, ego, participants, vehicle_params, params );
  EXPECT_FALSE( without_force.has_value() );

  const std::vector<int> forced_ids = { 24 };
  const auto forced_group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      participants,
      vehicle_params,
      params,
      nullptr,
      &forced_ids );

  ASSERT_TRUE( forced_group.has_value() );
  ASSERT_EQ( forced_group->obstacles.size(), 1U );
  EXPECT_LT( forced_group->obstacles.front().object_s_max, ego.x );
}

TEST( ObstacleAvoidance, LaterObservationOnlyExpandsCommittedHull )
{
  const auto route = make_straight_route( 100.0, 1.0 );
  adore::dynamics::VehicleStateDynamic ego;

  adore::planner::AvoidanceShiftContribution contribution;
  contribution.participant_ids = { 31 };
  contribution.object_s_min = 28.0;
  contribution.object_s_max = 32.0;
  contribution.object_l_min = -0.5;
  contribution.object_l_max = 0.5;
  const std::vector<adore::planner::AvoidanceShiftContribution> committed = {
    contribution
  };

  adore::dynamics::VehicleStateDynamic observed_state;
  observed_state.x = 30.0;
  observed_state.y = 0.0;
  observed_state.yaw_angle = 0.0;

  adore::dynamics::PhysicalVehicleParameters observed_params;
  observed_params.body_length = 6.0;
  observed_params.body_width = 2.0;

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    31,
    adore::dynamics::TrafficParticipant(
      observed_state,
      31,
      adore::dynamics::CAR,
      observed_params ) );

  auto ego_params = test_vehicle_params();
  ego_params.body_width = 2.0;
  auto params = test_params();
  params.ego_corridor_safety_margin = 0.5;

  const auto group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      participants,
      ego_params,
      params,
      &committed,
      nullptr );

  ASSERT_TRUE( group.has_value() );
  ASSERT_EQ( group->obstacles.size(), 1U );
  EXPECT_LE( group->obstacles.front().object_s_min, 27.0 );
  EXPECT_GE( group->obstacles.front().object_s_max, 33.0 );
  EXPECT_LE( group->obstacles.front().object_l_min, -1.0 );
  EXPECT_GE( group->obstacles.front().object_l_max, 1.0 );
}

// Regression for a dimension refinement that arrives only while ego is already
// passing the object. The updated near edge lies behind the ego front, but the
// farther edge extends into the old ramp-down. A forced active-corridor replan must
// still be able to merge the same-id observation and hold the accepted shift farther
// downstream; the decision maker decides whether the resulting route is safe.
TEST( ObstacleAvoidance, SameIdLengthGrowthAlongsideEgoExtendsPersistentShift )
{
  const auto route = make_straight_route( 100.0, 1.0 );

  auto ego_params = test_vehicle_params();
  ego_params.body_width = 2.0;
  ego_params.wheelbase = 2.7;
  ego_params.front_axle_to_front_border = 1.0;
  ego_params.rear_border_to_rear_axle = 1.0;

  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 31.0;
  ego.y = 2.5;

  adore::planner::AvoidanceShiftContribution contribution;
  contribution.participant_ids = { 31 };
  contribution.object_s_min = 28.0;
  contribution.object_s_max = 32.0;
  contribution.object_l_min = -0.5;
  contribution.object_l_max = 0.5;
  contribution.has_persistent_profile = true;
  contribution.signed_shift = 2.5;
  contribution.ramp_start_s = 17.3;
  contribution.full_shift_start_s = 24.3;
  contribution.full_shift_end_s = 33.0;
  contribution.ramp_end_s = 40.0;
  const std::vector<adore::planner::AvoidanceShiftContribution> committed = {
    contribution
  };

  adore::dynamics::VehicleStateDynamic observed_state;
  observed_state.x = 32.0;
  observed_state.y = 0.0;
  observed_state.yaw_angle = 0.0;
  observed_state.vx = 0.0;

  adore::dynamics::PhysicalVehicleParameters observed_params;
  observed_params.body_length = 12.0;
  observed_params.body_width = 1.0;

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    31,
    adore::dynamics::TrafficParticipant(
      observed_state,
      31,
      adore::dynamics::CAR,
      observed_params ) );

  auto params = test_params();
  params.front_clearance = 7.0;
  params.rear_clearance = 7.0;
  params.side_clearance = 1.0;
  params.ego_corridor_safety_margin = 0.5;

  const auto old_group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      adore::dynamics::TrafficParticipantSet{},
      ego_params,
      params,
      &committed,
      nullptr );
  ASSERT_TRUE( old_group.has_value() );

  const std::vector<int> forced_ids = { 31 };
  const auto expanded_group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      participants,
      ego_params,
      params,
      &committed,
      &forced_ids );

  ASSERT_TRUE( expanded_group.has_value() );
  ASSERT_EQ( expanded_group->obstacles.size(), 1U );
  const auto& expanded = expanded_group->obstacles.front();

  const double ego_front_s =
    ego.x + ego_params.wheelbase +
    ego_params.front_axle_to_front_border;
  EXPECT_LE( expanded.object_s_min, ego_front_s );
  EXPECT_LE( expanded.object_s_min, 26.0 );
  EXPECT_GE( expanded.object_s_max, 38.0 );
  EXPECT_TRUE( expanded.committed_hold );
  const double ego_half_length = 0.5 * ego_params.body_length;
  EXPECT_NEAR(
    expanded.persistent_full_shift_start_s,
    std::min(
      contribution.full_shift_start_s,
      expanded.object_s_min - ego_half_length ),
    1e-9 );
  EXPECT_NEAR(
    expanded.persistent_full_shift_end_s,
    std::max(
      contribution.full_shift_end_s,
      expanded.object_s_max + ego_half_length ),
    1e-9 );
  EXPECT_GE(
    expanded.persistent_ramp_end_s,
    expanded.persistent_full_shift_end_s + params.rear_clearance );

  constexpr double downstream_s = 36.0;
  const double old_offset =
    adore::planner::oa_detail::avoidance_shift_offset_at_s(
      downstream_s,
      old_group.value(),
      contribution.signed_shift,
      ego_params,
      params );
  const double expanded_offset =
    adore::planner::oa_detail::avoidance_shift_offset_at_s(
      downstream_s,
      expanded_group.value(),
      contribution.signed_shift,
      ego_params,
      params );

  EXPECT_LT( old_offset, contribution.signed_shift );
  EXPECT_NEAR( expanded_offset, contribution.signed_shift, 1e-9 );
}

TEST( ObstacleAvoidance, ChangedIdLengthGrowthAlongsideEgoExtendsPersistentShift )
{
  const auto route = make_straight_route( 100.0, 1.0 );

  auto ego_params = test_vehicle_params();
  ego_params.body_width = 2.0;
  ego_params.wheelbase = 2.7;
  ego_params.front_axle_to_front_border = 1.0;
  ego_params.rear_border_to_rear_axle = 1.0;

  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 31.0;
  ego.y = 2.5;

  adore::planner::AvoidanceShiftContribution committed_observation;
  committed_observation.participant_ids = { 31 };
  committed_observation.object_s_min = 28.0;
  committed_observation.object_s_max = 32.0;
  committed_observation.object_l_min = -0.5;
  committed_observation.object_l_max = 0.5;
  committed_observation.has_persistent_profile = true;
  committed_observation.signed_shift = 2.5;
  committed_observation.ramp_start_s = 17.3;
  committed_observation.full_shift_start_s = 24.3;
  committed_observation.full_shift_end_s = 33.0;
  committed_observation.ramp_end_s = 40.0;
  const std::vector<adore::planner::AvoidanceShiftContribution> committed = {
    committed_observation
  };

  // Perception now reports the same physical object with a new ID and its
  // newly visible full length. No identity heuristic is required: the old
  // frozen hull remains and the current geometry becomes a second contribution
  // to the same geometric shift envelope.
  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    32,
    make_participant( 32, 32.0, 0.0, 0.0, 0.0, 12.0, 1.0 ) );

  auto params = test_params();
  params.front_clearance = 7.0;
  params.rear_clearance = 7.0;
  params.side_clearance = 1.0;
  params.ego_corridor_safety_margin = 0.5;

  const std::vector<int> forced_ids = { 32 };
  const auto expanded_group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      participants,
      ego_params,
      params,
      &committed,
      &forced_ids );

  ASSERT_TRUE( expanded_group.has_value() );
  ASSERT_EQ( expanded_group->obstacles.size(), 2U );
  EXPECT_TRUE(
    adore::planner::oa_detail::avoidance_group_contains_participant_id(
      expanded_group.value(), 31 ) );
  EXPECT_TRUE(
    adore::planner::oa_detail::avoidance_group_contains_participant_id(
      expanded_group.value(), 32 ) );
  EXPECT_GE( expanded_group->envelope.object_s_max, 38.0 );

  constexpr double downstream_s = 36.0;
  const double expanded_offset =
    adore::planner::oa_detail::avoidance_shift_offset_at_s(
      downstream_s,
      expanded_group.value(),
      committed_observation.signed_shift,
      ego_params,
      params );

  EXPECT_NEAR(
    expanded_offset,
    committed_observation.signed_shift,
    1e-9 );
}

// The driven-corridor check is authoritative for an active reshape. Once it has
// selected an object that still overlaps ego longitudinally, the initial
// behind-ego filter must not discard it merely because its far edge projects
// just behind ego's reference point.
TEST( ObstacleAvoidance, ForcedAlongsideObjectBypassesInitialBehindEgoFilter )
{
  const auto route = make_straight_route( 100.0, 1.0 );

  auto ego_params = test_vehicle_params();
  ego_params.body_width = 2.0;
  ego_params.wheelbase = 2.7;
  ego_params.front_axle_to_front_border = 1.0;
  ego_params.rear_border_to_rear_axle = 1.0;

  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 38.0;
  ego.y = 2.5;

  adore::planner::AvoidanceShiftContribution contribution;
  contribution.participant_ids = { 31 };
  contribution.object_s_min = 28.0;
  contribution.object_s_max = 32.0;
  contribution.object_l_min = -0.5;
  contribution.object_l_max = 0.5;
  const std::vector<adore::planner::AvoidanceShiftContribution> committed = {
    contribution
  };

  adore::dynamics::VehicleStateDynamic observed_state;
  observed_state.x = 32.0;
  observed_state.y = 0.0;
  observed_state.yaw_angle = 0.0;
  observed_state.vx = 0.0;

  adore::dynamics::PhysicalVehicleParameters observed_params;
  observed_params.body_length = 12.0;
  observed_params.body_width = 1.0;

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    31,
    adore::dynamics::TrafficParticipant(
      observed_state,
      31,
      adore::dynamics::CAR,
      observed_params ) );

  auto params = test_params();
  params.front_clearance = 0.0;
  params.rear_clearance = 0.0;

  const std::vector<int> forced_ids = { 31 };
  const auto expanded_group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      participants,
      ego_params,
      params,
      &committed,
      &forced_ids );

  ASSERT_TRUE( expanded_group.has_value() );
  ASSERT_EQ( expanded_group->obstacles.size(), 1U );
  EXPECT_GE( expanded_group->obstacles.front().object_s_max, ego.x );
  EXPECT_TRUE( expanded_group->obstacles.front().committed_hold );
}

TEST( ObstacleAvoidance, ActiveMergeAddsOnlyDrivenCorridorIntrusions )
{
  const auto route = make_straight_route( 100.0, 1.0 );
  adore::dynamics::VehicleStateDynamic ego;

  adore::planner::AvoidanceShiftContribution contribution;
  contribution.participant_ids = { 41 };
  contribution.object_s_min = 20.0;
  contribution.object_s_max = 24.0;
  contribution.object_l_min = -0.5;
  contribution.object_l_max = 0.5;
  const std::vector<adore::planner::AvoidanceShiftContribution> committed = {
    contribution
  };

  adore::dynamics::VehicleStateDynamic unrelated_state;
  unrelated_state.x = 45.0;
  unrelated_state.y = 0.0;
  unrelated_state.yaw_angle = 0.0;

  adore::dynamics::PhysicalVehicleParameters object_params;
  object_params.body_length = 4.0;
  object_params.body_width = 2.0;

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    42,
    adore::dynamics::TrafficParticipant(
      unrelated_state,
      42,
      adore::dynamics::CAR,
      object_params ) );

  auto ego_params = test_vehicle_params();
  ego_params.body_width = 2.0;
  auto params = test_params();
  params.ego_corridor_safety_margin = 0.5;

  const auto group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      participants,
      ego_params,
      params,
      &committed,
      nullptr );

  ASSERT_TRUE( group.has_value() );
  ASSERT_EQ( group->obstacles.size(), 1U );
  EXPECT_EQ( group->obstacles.front().participant_ids.front(), 41 );
}

TEST( ObstacleAvoidance, ActiveReplanKeepsFarForcedObjectWithCommittedManeuver )
{
  const auto route = make_straight_route( 120.0, 1.0 );
  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 0.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;

  adore::planner::AvoidanceShiftContribution contribution;
  contribution.participant_ids = { 101 };
  contribution.object_s_min = 18.0;
  contribution.object_s_max = 22.0;
  contribution.object_l_min = -1.0;
  contribution.object_l_max = 1.0;
  const std::vector<adore::planner::AvoidanceShiftContribution> committed = {
    contribution
  };

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    102,
    make_participant( 102, 80.0, 0.0, 0.0, 0.0, 4.0, 2.0 ) );
  const std::vector<int> forced_ids = { 102 };

  auto vehicle_params = validation_vehicle_params();
  vehicle_params.body_length = 4.0;
  const auto group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      participants,
      vehicle_params,
      test_params(),
      &committed,
      &forced_ids );

  ASSERT_TRUE( group.has_value() );
  ASSERT_EQ( group->obstacles.size(), 2U );
  EXPECT_EQ( group->obstacles.front().participant_ids.front(), 101 );
  EXPECT_EQ( group->obstacles.back().participant_ids.front(), 102 );
}

TEST( ObstacleAvoidance, MonotonicActiveProgressRejectsProjectionJumps )
{
  auto params = test_params();
  params.projection_progress_tolerance = 2.0;

  adore::planner::ActiveAvoidanceState state;
  state.last_modified_s = 10.0;
  state.last_modified_time = 5.0;

  adore::dynamics::VehicleStateDynamic ego;
  ego.vx = 2.0;
  ego.time = 5.5;

  const auto jump_filtered =
    adore::planner::compute_monotonic_ego_s_modified(
      30.0, ego, state, params );
  ASSERT_TRUE( jump_filtered.has_value() );
  EXPECT_NEAR( jump_filtered.value(), 11.0, 1e-9 );

  ego.time = 6.0;
  const auto backward_filtered =
    adore::planner::compute_monotonic_ego_s_modified(
      9.0, ego, state, params );
  ASSERT_TRUE( backward_filtered.has_value() );
  EXPECT_NEAR( backward_filtered.value(), 11.0, 1e-9 );
}

TEST( ObstacleAvoidance, PersistentProfileIgnoresLaterGlobalRampChanges )
{
  const auto route = make_straight_route( 100.0, 1.0 );
  adore::dynamics::VehicleStateDynamic ego;
  adore::dynamics::TrafficParticipantSet participants;
  auto ego_params = test_vehicle_params();
  ego_params.body_width = 2.0;

  adore::planner::AvoidanceShiftContribution contribution;
  contribution.participant_ids = { 51 };
  contribution.object_s_min = 20.0;
  contribution.object_s_max = 30.0;
  contribution.object_l_min = -0.5;
  contribution.object_l_max = 0.5;
  contribution.has_persistent_profile = true;
  contribution.signed_shift = 2.5;
  contribution.ramp_start_s = 10.0;
  contribution.full_shift_start_s = 20.0;
  contribution.full_shift_end_s = 30.0;
  contribution.ramp_end_s = 40.0;

  const std::vector<adore::planner::AvoidanceShiftContribution> committed = {
    contribution
  };
  auto params = test_params();
  params.side_clearance = 1.0;
  params.front_clearance = 2.0;
  params.rear_clearance = 2.0;

  const auto group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      participants,
      ego_params,
      params,
      &committed,
      nullptr );

  ASSERT_TRUE( group.has_value() );
  const double offset_with_short_global_ramp =
    adore::planner::oa_detail::avoidance_shift_offset_at_s(
      15.0, group.value(), 3.0, ego_params, params );

  params.front_clearance = 20.0;
  params.rear_clearance = 20.0;
  const double offset_with_long_global_ramp =
    adore::planner::oa_detail::avoidance_shift_offset_at_s(
      15.0, group.value(), 3.0, ego_params, params );

  EXPECT_NEAR(
    offset_with_short_global_ramp,
    offset_with_long_global_ramp,
    1e-9 );
  EXPECT_GT( offset_with_short_global_ramp, 0.0 );
  EXPECT_LT( offset_with_short_global_ramp, contribution.signed_shift );
}

TEST( ObstacleAvoidance, DrivableAreaAlwaysRejectsBlindInLaneCandidate )
{
  adore::planner::oa_detail::ShiftCandidate candidate;
  candidate.shift = 2.0;
  candidate.valid = true;
  candidate.type =
    adore::planner::oa_detail::AvoidanceCandidateType::InLane;

  adore::planner::oa_detail::evaluate_shift_candidate(
    candidate,
    make_straight_route( 20.0, 1.0 ),
    make_validation_group(),
    validation_vehicle_params(),
    test_params() );

  EXPECT_FALSE( candidate.valid );
}

TEST( ObstacleAvoidance, AdjacentCandidateThatStaysInLaneIsNotADuplicateMode )
{
  adore::planner::oa_detail::ShiftCandidate candidate;
  candidate.shift = 3.0;
  candidate.valid = true;
  candidate.type =
    adore::planner::oa_detail::AvoidanceCandidateType::AdjacentSameDirection;

  adore::planner::oa_detail::evaluate_shift_candidate(
    candidate,
    make_straight_route_with_lane( 30.0, 1.0, 5.0 ),
    make_validation_group(),
    validation_vehicle_params(),
    test_params() );

  EXPECT_FALSE( candidate.valid );
  EXPECT_FALSE( candidate.in_lane );
}

TEST( ObstacleAvoidance, DrivableAreaAlwaysRejectsBlindAdjacentMode )
{
  adore::planner::oa_detail::ShiftCandidate candidate;
  candidate.shift = 3.0;
  candidate.valid = true;
  candidate.type =
    adore::planner::oa_detail::AvoidanceCandidateType::AdjacentSameDirection;

  adore::planner::oa_detail::evaluate_shift_candidate(
    candidate,
    make_straight_route( 30.0, 1.0 ),
    make_validation_group(),
    validation_vehicle_params(),
    test_params() );

  EXPECT_FALSE( candidate.valid );
}

TEST( ObstacleAvoidance, ConnectedIntersectionUsesDrivingSurfaceAcrossRoadIds )
{
  auto route = make_intersection_route( -8.0, 8.0 );
  auto params = test_params();
  params.intersection_drivable_area_enabled = true;
  params.front_clearance = 0.0;
  params.rear_clearance = 0.0;

  const auto route_point =
    adore::planner::oa_detail::find_reference_point_near_s( route, 15.0 );
  ASSERT_TRUE( route_point.has_value() );
  const auto area =
    adore::planner::oa_detail::
      query_connected_intersection_driving_area_at_route_point(
        route,
        route_point->second,
        route_point->first,
        3.0,
        params );

  ASSERT_TRUE( area.map_usable );
  ASSERT_TRUE( area.has_cross_road_connection );
  EXPECT_LE( area.interval.min_l, -7.9 );
  EXPECT_GE( area.interval.max_l, 7.9 );

  auto disabled_params = params;
  disabled_params.intersection_drivable_area_enabled = false;
  const auto disabled_area =
    adore::planner::oa_detail::
      query_connected_intersection_driving_area_at_route_point(
        route,
        route_point->second,
        route_point->first,
        3.0,
        disabled_params );
  EXPECT_TRUE( disabled_area.map_usable );
  EXPECT_FALSE( disabled_area.has_cross_road_connection );
  EXPECT_GT( disabled_area.interval.min_l, -2.0 );
  EXPECT_LT( disabled_area.interval.max_l, 2.0 );

  adore::planner::oa_detail::ShiftCandidate candidate;
  candidate.shift = 3.0;
  candidate.valid = true;
  candidate.type =
    adore::planner::oa_detail::AvoidanceCandidateType::ConnectedIntersection;
  adore::planner::oa_detail::evaluate_shift_candidate(
    candidate,
    route,
    make_validation_group(),
    validation_vehicle_params(),
    params );

  EXPECT_TRUE( candidate.valid );
  EXPECT_FALSE( candidate.in_lane );
}

TEST( ObstacleAvoidance, ConnectedIntersectionRejectsDisconnectedForeignRoad )
{
  auto route = make_intersection_route( 3.0, 8.0 );
  auto params = test_params();
  params.intersection_drivable_area_enabled = true;
  params.front_clearance = 0.0;
  params.rear_clearance = 0.0;

  adore::planner::oa_detail::ShiftCandidate candidate;
  candidate.shift = 3.0;
  candidate.valid = true;
  candidate.type =
    adore::planner::oa_detail::AvoidanceCandidateType::ConnectedIntersection;
  adore::planner::oa_detail::evaluate_shift_candidate(
    candidate,
    route,
    make_validation_group(),
    validation_vehicle_params(),
    params );

  EXPECT_FALSE( candidate.valid );
}

TEST( ObstacleAvoidance, ConnectedIntersectionRequiresAnotherRoad )
{
  auto route =
    make_intersection_route( -8.0, 8.0, adore::map::driving, 1 );
  auto params = test_params();
  params.intersection_drivable_area_enabled = true;

  const auto route_point =
    adore::planner::oa_detail::find_reference_point_near_s( route, 15.0 );
  ASSERT_TRUE( route_point.has_value() );
  const auto area =
    adore::planner::oa_detail::
      query_connected_intersection_driving_area_at_route_point(
        route,
        route_point->second,
        route_point->first,
        3.0,
        params );

  EXPECT_TRUE( area.map_usable );
  EXPECT_FALSE( area.has_cross_road_connection );
  EXPECT_GT( area.interval.min_l, -2.0 );
  EXPECT_LT( area.interval.max_l, 2.0 );
}

TEST( ObstacleAvoidance, ConnectedIntersectionFailsClosedWithEmptySpatialIndex )
{
  const auto route = make_straight_route_with_lane( 30.0, 1.0, 1.5 );
  auto params = test_params();
  params.intersection_drivable_area_enabled = true;

  const auto route_point =
    adore::planner::oa_detail::find_reference_point_near_s( route, 15.0 );
  ASSERT_TRUE( route_point.has_value() );
  const auto area =
    adore::planner::oa_detail::
      query_connected_intersection_driving_area_at_route_point(
        route,
        route_point->second,
        route_point->first,
        3.0,
        params );

  EXPECT_TRUE( area.map_usable );
  EXPECT_FALSE( area.has_cross_road_connection );
  EXPECT_NEAR( area.interval.min_l, -1.5, 1e-9 );
  EXPECT_NEAR( area.interval.max_l, 1.5, 1e-9 );
}

TEST( ObstacleAvoidance, ConnectedIntersectionIgnoresNonDrivingSurface )
{
  auto route =
    make_intersection_route( -8.0, 8.0, adore::map::sidewalk );
  auto params = test_params();
  params.intersection_drivable_area_enabled = true;
  params.front_clearance = 0.0;
  params.rear_clearance = 0.0;

  adore::planner::oa_detail::ShiftCandidate candidate;
  candidate.shift = 3.0;
  candidate.valid = true;
  candidate.type =
    adore::planner::oa_detail::AvoidanceCandidateType::ConnectedIntersection;
  adore::planner::oa_detail::evaluate_shift_candidate(
    candidate,
    route,
    make_validation_group(),
    validation_vehicle_params(),
    params );

  EXPECT_FALSE( candidate.valid );
}

TEST( ObstacleAvoidance, OppositeLaneCandidateRejectsMedianGap )
{
  auto route = make_straight_route_with_lane( 30.0, 1.0, 1.5 );
  add_horizontal_lane(
    route,
    2,
    1,
    0.0,
    30.0,
    3.0,
    6.0,
    true );

  auto params = test_params();
  params.front_clearance = 0.0;
  params.rear_clearance = 0.0;
  params.opposite_lane_enabled = true;

  adore::planner::oa_detail::ShiftCandidate candidate;
  candidate.shift = 4.5;
  candidate.valid = true;
  candidate.type =
    adore::planner::oa_detail::AvoidanceCandidateType::OppositeDirection;
  adore::planner::oa_detail::evaluate_shift_candidate(
    candidate,
    route,
    make_validation_group(),
    validation_vehicle_params(),
    params );

  EXPECT_FALSE( candidate.valid );
}

TEST( ObstacleAvoidance, CandidateEvaluationDoesNotDuplicateClearanceValidation )
{
  const auto route =
    make_straight_route_with_lane( 30.0, 1.0, 5.0 );
  const auto group = make_validation_group();
  const auto vehicle_params = validation_vehicle_params();

  auto params = test_params();
  params.side_clearance = 1.0;
  params.in_lane_shift_enabled = true;

  // Clearance is already encoded in the generated shift. A small target
  // undershoot is handled by trajectory validation against the smaller hard
  // ego corridor and by adaptive refinement, not by a duplicate route check.
  adore::planner::oa_detail::ShiftCandidate short_by_one_centimetre;
  short_by_one_centimetre.shift = 2.99;
  short_by_one_centimetre.valid = true;
  short_by_one_centimetre.type =
    adore::planner::oa_detail::AvoidanceCandidateType::InLane;
  adore::planner::oa_detail::evaluate_shift_candidate(
    short_by_one_centimetre,
    route,
    group,
    vehicle_params,
    params );
  EXPECT_TRUE( short_by_one_centimetre.valid );
  EXPECT_TRUE( short_by_one_centimetre.in_lane );
}

TEST( ObstacleAvoidance, TrajectoryMayUndershootRouteClearanceAboveHardCorridor )
{
  const auto route =
    make_straight_route_with_lane( 30.0, 1.0, 5.0 );
  const auto group = make_validation_group();
  const auto vehicle_params = validation_vehicle_params();

  auto params = test_params();
  params.side_clearance = 1.0;
  params.ego_corridor_safety_margin = 0.5;

  // Object left edge is l=1.0 and ego half-width is 1.0. At ego center
  // l=2.98 the actual edge-to-edge clearance is 0.98 m: below the route target
  // but safely outside the deliberately smaller hard corridor.
  const auto result =
    adore::planner::oa_detail::validate_planned_shift_trajectory(
      route,
      make_single_state_trajectory( 12.0, 2.98 ),
      group,
      3.0,
      true,
      adore::planner::oa_detail::AvoidanceCandidateType::InLane,
      vehicle_params,
      params,
      params.side_clearance,
      12.0 );

  EXPECT_TRUE( result.valid ) << result.reason;
  EXPECT_FALSE( result.obstacle_clearance_violation );
  EXPECT_NEAR( result.min_obstacle_lateral_margin, -0.02, 1e-9 );
}

TEST( ObstacleAvoidance, TrajectoryInsideHardEgoCorridorIsRejected )
{
  const auto route =
    make_straight_route_with_lane( 30.0, 1.0, 5.0 );
  const auto group = make_validation_group();
  const auto vehicle_params = validation_vehicle_params();

  auto params = test_params();
  params.side_clearance = 1.0;
  params.ego_corridor_safety_margin = 0.5;

  const auto result =
    adore::planner::oa_detail::validate_planned_shift_trajectory(
      route,
      make_single_state_trajectory( 12.0, 2.49 ),
      group,
      3.0,
      true,
      adore::planner::oa_detail::AvoidanceCandidateType::InLane,
      vehicle_params,
      params,
      params.side_clearance,
      12.0 );

  EXPECT_FALSE( result.valid );
  EXPECT_TRUE( result.obstacle_clearance_violation );
  EXPECT_NE( result.reason.find( "hard ego corridor" ), std::string::npos );
}

TEST( ObstacleAvoidance, TrajectoryLeavingDrivableAreaRemainsRejected )
{
  const auto route =
    make_straight_route_with_lane( 30.0, 1.0, 5.0 );
  const auto group = make_validation_group();
  const auto vehicle_params = validation_vehicle_params();

  auto params = test_params();
  params.side_clearance = 1.0;
  params.ego_corridor_safety_margin = 0.5;

  const auto result =
    adore::planner::oa_detail::validate_planned_shift_trajectory(
      route,
      make_single_state_trajectory( 12.0, 4.01 ),
      group,
      3.0,
      true,
      adore::planner::oa_detail::AvoidanceCandidateType::InLane,
      vehicle_params,
      params,
      params.side_clearance,
      12.0 );

  EXPECT_FALSE( result.valid );
  EXPECT_FALSE( result.obstacle_clearance_violation );
  EXPECT_NE( result.reason.find( "leaves drivable area" ), std::string::npos );
}

TEST( ObstacleAvoidance, RotatedEgoFootprintMustRemainInsideDrivableArea )
{
  const auto route =
    make_straight_route_with_lane( 30.0, 1.0, 1.2 );
  const auto group = make_validation_group();
  const auto vehicle_params = validation_vehicle_params();

  auto trajectory = make_single_state_trajectory( 12.0, 0.0 );
  trajectory.states.front().yaw_angle = 0.30;

  auto params = test_params();
  const auto result =
    adore::planner::oa_detail::validate_planned_shift_trajectory(
      route,
      trajectory,
      group,
      3.0,
      true,
      adore::planner::oa_detail::AvoidanceCandidateType::InLane,
      vehicle_params,
      params,
      params.side_clearance,
      12.0 );

  EXPECT_FALSE( result.valid );
  EXPECT_NE(
    result.reason.find( "rotated ego footprint leaves drivable area" ),
    std::string::npos );
}

TEST( ObstacleAvoidance, EmptyTrajectoryCanNeverBypassValidation )
{
  const auto params = test_params();

  const auto validation =
    adore::planner::oa_detail::validate_planned_shift_trajectory(
      make_straight_route( 30.0, 1.0 ),
      adore::dynamics::Trajectory{},
      make_validation_group(),
      3.0,
      true,
      adore::planner::oa_detail::AvoidanceCandidateType::InLane,
      validation_vehicle_params(),
      params,
      params.side_clearance,
      12.0 );

  EXPECT_FALSE( validation.valid );
  EXPECT_NE( validation.reason.find( "empty trajectory" ), std::string::npos );
}

TEST( ObstacleAvoidance, FutureCommittedObstacleStillRequiresHardClearance )
{
  const auto route =
    make_straight_route_with_lane( 30.0, 1.0, 5.0 );
  auto group = make_validation_group();
  group.obstacles.front().committed_hold = true;
  group.envelope.committed_hold = true;

  adore::dynamics::Trajectory trajectory;
  trajectory.states =
    make_single_state_trajectory( 0.0, 0.0 ).states;
  trajectory.states.push_back(
    make_single_state_trajectory( 12.0, 0.0 ).states.front() );

  auto params = test_params();
  params.ego_corridor_safety_margin = 0.5;

  const auto validation =
    adore::planner::oa_detail::validate_planned_shift_trajectory(
      route,
      trajectory,
      group,
      3.0,
      true,
      adore::planner::oa_detail::AvoidanceCandidateType::InLane,
      validation_vehicle_params(),
      params,
      params.side_clearance,
      0.0 );

  EXPECT_FALSE( validation.valid );
  EXPECT_TRUE( validation.obstacle_clearance_violation );
}

TEST( ObstacleAvoidance, CommittedAlongsideTrajectoryMayMoveAwayFromCurrentClearance )
{
  const auto route =
    make_straight_route_with_lane( 30.0, 1.0, 5.0 );
  auto group = make_validation_group();
  group.obstacles.front().committed_hold = true;
  group.envelope.committed_hold = true;

  adore::dynamics::Trajectory trajectory;
  trajectory.states =
    make_single_state_trajectory( 12.0, 2.4 ).states;
  trajectory.states.push_back(
    make_single_state_trajectory( 13.0, 2.6 ).states.front() );

  auto params = test_params();
  params.side_clearance = 1.0;
  params.ego_corridor_safety_margin = 0.5;

  const auto validation =
    adore::planner::oa_detail::validate_planned_shift_trajectory(
      route,
      trajectory,
      group,
      3.0,
      true,
      adore::planner::oa_detail::AvoidanceCandidateType::InLane,
      validation_vehicle_params(),
      params,
      params.side_clearance,
      12.0 );

  EXPECT_TRUE( validation.valid ) << validation.reason;
  EXPECT_LT( validation.min_obstacle_lateral_margin, 0.0 );
}

TEST( ObstacleAvoidance, StaticCorridorIntrusionAlongsideEgoIsReported )
{
  const auto route = make_straight_route( 100.0, 1.0 );

  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 30.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;
  ego.vx = 2.0;
  ego.time = 0.0;

  auto ego_params = validation_vehicle_params();
  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    71,
    make_participant( 71, 30.0, 0.0, 0.0, 0.0, 2.0, 2.0 ) );

  auto params = test_params();
  params.ego_corridor_safety_margin = 0.2;

  const auto result =
    adore::planner::check_route_corridor_safety(
      route,
      ego,
      participants,
      ego_params,
      params );

  ASSERT_TRUE( result.has_conflict );
  ASSERT_FALSE( result.conflicts.empty() );
  EXPECT_EQ(
    result.conflicts.front().object_class,
    adore::planner::RouteCorridorObjectClass::StaticOrSlow );
  EXPECT_TRUE( result.conflicts.front().currently_overlaps_ego_footprint );
}

TEST( ObstacleAvoidance, StaticSideClearanceUsesBestEffortAndHardThresholds )
{
  const auto route = make_straight_route( 100.0, 1.0 );

  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 0.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;
  ego.vx = 2.0;
  ego.time = 0.0;

  const auto ego_params = validation_vehicle_params();
  auto params = test_params();
  params.ego_corridor_safety_margin = 0.5;
  params.side_clearance = 1.0;
  params.side_clearance_replan_tolerance = 0.03;

  const auto check_clearance =
    [&]( int id, double edge_to_edge_clearance )
    {
      adore::dynamics::TrafficParticipantSet participants;
      const double object_center_y =
        0.5 * ego_params.body_width +
        edge_to_edge_clearance +
        1.0;  // object half-width
      participants.participants.emplace(
        id,
        make_participant(
          id,
          20.0,
          object_center_y,
          0.0,
          0.0,
          4.0,
          2.0 ) );

      return adore::planner::check_route_corridor_safety(
        route,
        ego,
        participants,
        ego_params,
        params );
    };

  // The desired planning clearance is already met: no replan input.
  const auto desired_clearance = check_clearance( 88, 1.1 );
  EXPECT_TRUE( desired_clearance.safe );
  EXPECT_FALSE( desired_clearance.has_conflict );
  EXPECT_TRUE( desired_clearance.static_clearance_improvements.empty() );

  // A few centimetres below the desired target are intentionally accepted for
  // the optional replan trigger. The route target itself remains 1.0 m.
  const auto tolerated_clearance = check_clearance( 91, 0.98 );
  EXPECT_TRUE( tolerated_clearance.safe );
  EXPECT_FALSE( tolerated_clearance.has_conflict );
  EXPECT_TRUE( tolerated_clearance.static_clearance_improvements.empty() );

  const auto meaningful_shortfall = check_clearance( 92, 0.96 );
  EXPECT_TRUE( meaningful_shortfall.safe );
  EXPECT_FALSE( meaningful_shortfall.has_conflict );
  ASSERT_EQ(
    meaningful_shortfall.static_clearance_improvements.size(),
    1U );

  // The route is still hard-safe at 0.7 m, but the planner should attempt to
  // restore the 1.0 m side_clearance without turning the shortfall into a stop.
  const auto best_effort_clearance = check_clearance( 89, 0.7 );
  EXPECT_TRUE( best_effort_clearance.safe );
  EXPECT_FALSE( best_effort_clearance.has_conflict );
  ASSERT_EQ(
    best_effort_clearance.static_clearance_improvements.size(),
    1U );
  EXPECT_NEAR(
    best_effort_clearance.static_clearance_improvements.front()
      .actual_lateral_clearance,
    0.7,
    1e-6 );
  EXPECT_FALSE(
    best_effort_clearance.static_clearance_improvements.front()
      .currently_overlaps_route_corridor );

  // Below the 0.5 m hard margin the existing mandatory replan/stop cascade
  // remains authoritative.
  const auto hard_clearance = check_clearance( 90, 0.4 );
  EXPECT_FALSE( hard_clearance.safe );
  EXPECT_TRUE( hard_clearance.has_conflict );
  ASSERT_EQ( hard_clearance.conflicts.size(), 1U );
  EXPECT_TRUE( hard_clearance.static_clearance_improvements.empty() );
  EXPECT_NEAR(
    hard_clearance.conflicts.front().actual_lateral_clearance,
    0.4,
    1e-6 );
  EXPECT_TRUE(
    hard_clearance.conflicts.front().currently_overlaps_route_corridor );
}

TEST( ObstacleAvoidance, OppositeHeadingSpeedNoiseRemainsAStaticObstacle )
{
  const auto route = make_straight_route( 100.0, 1.0 );

  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 0.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;

  auto participant =
    make_participant(
      77,
      20.0,
      0.0,
      std::numbers::pi,
      0.06,
      4.0,
      2.0 );

  auto params = test_params();
  params.max_static_object_speed = 0.1;

  EXPECT_TRUE(
    adore::planner::oa_detail::participant_is_static_for_avoidance(
      participant,
      params ) );

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace( participant.id, participant );

  const auto group =
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      participants,
      validation_vehicle_params(),
      params );

  ASSERT_TRUE( group.has_value() );
  ASSERT_EQ( group->obstacles.size(), 1U );
  EXPECT_EQ( group->obstacles.front().id, 77 );
}

TEST( ObstacleAvoidance, ActiveObstacleUsesSharedStaticSpeedForOncomingClassification )
{
  const auto route = make_straight_route( 100.0, 1.0 );

  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 0.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;
  ego.vx = 2.0;

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    84,
    make_participant(
      84,
      40.0,
      0.0,
      std::numbers::pi,
      0.05,
      4.0,
      2.0 ) );

  adore::planner::ObstacleAvoidanceManeuver maneuver;
  maneuver.active = true;
  maneuver.obstacle_id = 84;
  maneuver.obstacle_ids = { 84 };
  maneuver.uses_opposite_lane = true;
  maneuver.has_opposite_lane_conflict_interval = true;
  maneuver.opposite_lane_conflict_start_s = 1.0;
  maneuver.opposite_lane_conflict_end_s = 29.0;
  maneuver.commitment_s = 10.0;

  auto params = test_params();
  params.max_static_object_speed = 0.1;

  // The already-avoided object is not classified a second time as oncoming
  // while it is genuinely static according to the shared motion threshold.
  const auto static_result =
    adore::planner::monitor_active_obstacle_avoidance_maneuver(
      route,
      ego,
      participants,
      maneuver,
      validation_vehicle_params(),
      params );
  EXPECT_TRUE( static_result.safe_to_continue );

  // As soon as the same ID exceeds max_static_object_speed it is a moving
  // participant like every other object and must enter the oncoming monitor.
  participants.participants.at( 84 ).state.vx = 0.5;
  const auto moving_result =
    adore::planner::monitor_active_obstacle_avoidance_maneuver(
      route,
      ego,
      participants,
      maneuver,
      validation_vehicle_params(),
      params );
  EXPECT_FALSE( moving_result.safe_to_continue );
  EXPECT_TRUE( moving_result.oncoming.conflict );

  const auto corridor_result =
    adore::planner::check_route_corridor_safety(
      route,
      ego,
      participants,
      validation_vehicle_params(),
      params );
  EXPECT_TRUE( corridor_result.has_conflict );
}

TEST( ObstacleAvoidance, ActiveOncomingMonitorIsIndependentOfParticipantId )
{
  const auto route = make_straight_route( 100.0, 1.0 );

  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 12.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;
  ego.vx = 2.0;

  adore::planner::ObstacleAvoidanceManeuver maneuver;
  maneuver.active = true;
  maneuver.obstacle_id = 999;
  maneuver.obstacle_ids = { 999 };
  maneuver.uses_opposite_lane = true;
  maneuver.has_opposite_lane_conflict_interval = true;
  maneuver.opposite_lane_conflict_start_s = 10.0;
  maneuver.opposite_lane_conflict_end_s = 30.0;
  maneuver.commitment_s = 10.0;

  auto params = test_params();
  const auto ego_params = validation_vehicle_params();
  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    85,
    make_participant(
      85,
      40.0,
      3.0,
      std::numbers::pi,
      4.0,
      4.0,
      2.0 ) );

  const auto first_id_result =
    adore::planner::monitor_active_obstacle_avoidance_maneuver(
      route, ego, participants, maneuver, ego_params, params );
  ASSERT_TRUE( first_id_result.oncoming.conflict );
  EXPECT_EQ( first_id_result.oncoming.participant_id, 85 );

  // Re-identification changes only diagnostic identity. The same geometry and
  // motion under a new ID must still produce the same conflict.
  auto reidentified = participants.participants.at( 85 );
  reidentified.id = 86;
  participants.participants.clear();
  participants.participants.emplace( 86, reidentified );

  const auto second_id_result =
    adore::planner::monitor_active_obstacle_avoidance_maneuver(
      route, ego, participants, maneuver, ego_params, params );
  ASSERT_TRUE( second_id_result.oncoming.conflict );
  EXPECT_EQ( second_id_result.oncoming.participant_id, 86 );

  // Once the currently observed participant turns into the route direction, it
  // is no longer an oncoming conflict. The generic driven-corridor monitor
  // remains responsible if its actual path still crosses ego's route.
  participants.participants.at( 86 ).state.yaw_angle = 0.0;
  const auto turned_result =
    adore::planner::monitor_active_obstacle_avoidance_maneuver(
      route, ego, participants, maneuver, ego_params, params );
  EXPECT_TRUE( turned_result.safe_to_continue );
  EXPECT_FALSE( turned_result.oncoming.conflict );
}

TEST( ObstacleAvoidance, OncomingPassageUsesCompleteFootprint )
{
  const auto route = make_straight_route( 100.0, 1.0 );

  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 12.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;
  ego.vx = 2.0;

  adore::planner::ObstacleAvoidanceManeuver maneuver;
  maneuver.active = true;
  maneuver.obstacle_id = 999;
  maneuver.obstacle_ids = { 999 };
  maneuver.uses_opposite_lane = true;
  maneuver.has_opposite_lane_conflict_interval = true;
  maneuver.opposite_lane_conflict_start_s = 10.0;
  maneuver.opposite_lane_conflict_end_s = 30.0;
  maneuver.commitment_s = 10.0;

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    87,
    make_participant(
      87,
      9.0,
      3.0,
      std::numbers::pi,
      2.0,
      4.0,
      2.0 ) );

  const auto overlapping_result =
    adore::planner::monitor_active_obstacle_avoidance_maneuver(
      route,
      ego,
      participants,
      maneuver,
      validation_vehicle_params(),
      test_params() );
  EXPECT_TRUE( overlapping_result.oncoming.conflict );

  // The center is already below conflict_start_s in both cases. Release is
  // permitted only after the trailing footprint edge has also passed it.
  participants.participants.at( 87 ).state.x = 7.0;
  const auto fully_passed_result =
    adore::planner::monitor_active_obstacle_avoidance_maneuver(
      route,
      ego,
      participants,
      maneuver,
      validation_vehicle_params(),
      test_params() );
  EXPECT_TRUE( fully_passed_result.safe_to_continue );
  EXPECT_FALSE( fully_passed_result.oncoming.conflict );
}

TEST( ObstacleAvoidance, StoppedOncomingUsesHardCorridorAndStaticFallback )
{
  const auto route = make_straight_route( 100.0, 1.0 );

  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 12.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;
  ego.vx = 2.0;

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    85,
    make_participant(
      85,
      20.0,
      2.75,
      std::numbers::pi,
      0.0,
      4.0,
      2.0 ) );

  adore::planner::ObstacleAvoidanceManeuver maneuver;
  maneuver.active = true;
  maneuver.obstacle_id = 999;
  maneuver.obstacle_ids = { 999 };
  maneuver.uses_opposite_lane = true;
  maneuver.has_opposite_lane_conflict_interval = true;
  maneuver.opposite_lane_conflict_start_s = 1.0;
  maneuver.opposite_lane_conflict_end_s = 29.0;
  maneuver.commitment_s = 10.0;

  auto params = test_params();
  params.max_static_object_speed = 0.1;
  params.ego_corridor_safety_margin = 0.5;
  params.side_clearance = 1.0;
  const auto ego_params = validation_vehicle_params();

  // The stopped participant leaves 0.75 m edge-to-edge clearance. This is
  // below the 1.0 m planning target but above the 0.5 m hard safety margin, so
  // the already committed maneuver may continue through the usable space.
  const auto clear_result =
    adore::planner::monitor_active_obstacle_avoidance_maneuver(
      route,
      ego,
      participants,
      maneuver,
      ego_params,
      params );
  EXPECT_TRUE( clear_result.safe_to_continue );
  EXPECT_FALSE(
    adore::planner::check_route_corridor_safety(
      route, ego, participants, ego_params, params )
      .has_conflict );
  const auto initial_clear_result =
    adore::planner::oa_detail::check_oncoming_gap(
      route,
      route,
      ego,
      participants,
      make_validation_group(),
      2.0,
      ego_params,
      nullptr,
      params );
  EXPECT_FALSE( initial_clear_result.conflict );

  // If the same stopped participant intrudes into the hard corridor, the
  // oncoming monitor blocks and the generic corridor cascade exposes it as a
  // static replan target rather than allowing continued motion.
  participants.participants.at( 85 ).state.y = 1.0;
  const auto blocked_result =
    adore::planner::monitor_active_obstacle_avoidance_maneuver(
      route,
      ego,
      participants,
      maneuver,
      ego_params,
      params );
  EXPECT_FALSE( blocked_result.safe_to_continue );
  EXPECT_TRUE( blocked_result.oncoming.conflict );

  const auto static_conflict =
    adore::planner::check_route_corridor_safety(
      route, ego, participants, ego_params, params );
  ASSERT_TRUE( static_conflict.has_conflict );
  EXPECT_EQ(
    static_conflict.conflict.object_class,
    adore::planner::RouteCorridorObjectClass::StaticOrSlow );
  const auto initial_blocked_result =
    adore::planner::oa_detail::check_oncoming_gap(
      route,
      route,
      ego,
      participants,
      make_validation_group(),
      2.0,
      ego_params,
      nullptr,
      params );
  EXPECT_TRUE( initial_blocked_result.conflict );
}

TEST( ObstacleAvoidance, PredictedTrajectoryDoesNotOverrideCurrentStaticSpeed )
{
  const auto route = make_straight_route( 100.0, 1.0 );

  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 0.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;
  ego.vx = 2.0;
  ego.time = 0.0;

  auto participant =
    make_participant( 78, 20.0, 0.0, 0.0, 0.0, 4.0, 2.0 );
  adore::dynamics::Trajectory prediction;
  prediction.states.push_back( participant.state );
  auto future_state = participant.state;
  future_state.x = 25.0;
  future_state.vx = 2.0;
  future_state.time = 2.0;
  prediction.states.push_back( future_state );
  participant.trajectory = prediction;

  auto params = test_params();
  params.max_static_object_speed = 0.1;

  EXPECT_TRUE(
    adore::planner::oa_detail::participant_is_static_for_avoidance(
      participant,
      params ) );

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace( participant.id, participant );

  EXPECT_TRUE(
    adore::planner::oa_detail::find_static_obstacle_group_on_route(
      route,
      ego,
      participants,
      validation_vehicle_params(),
      params )
      .has_value() );

  const auto corridor_result =
    adore::planner::check_route_corridor_safety(
      route,
      ego,
      participants,
      validation_vehicle_params(),
      params );

  ASSERT_TRUE( corridor_result.has_conflict );
  EXPECT_EQ(
    corridor_result.conflict.object_class,
    adore::planner::RouteCorridorObjectClass::StaticOrSlow );
}

TEST( ObstacleAvoidance, LateralVelocityPreventsStaticClassification )
{
  auto participant =
    make_participant( 82, 20.0, 0.0, 0.0, 0.0, 4.0, 2.0 );
  participant.state.vy = 0.2;

  auto params = test_params();
  params.max_static_object_speed = 0.1;

  EXPECT_FALSE(
    adore::planner::oa_detail::participant_is_static_for_avoidance(
      participant,
      params ) );
}

TEST( ObstacleAvoidance, LateralVelocityFallbackPredictsCorridorEntry )
{
  const auto route = make_straight_route( 100.0, 1.0 );

  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 0.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;
  ego.time = 0.0;

  auto participant =
    make_participant( 83, 20.0, 5.0, 0.0, 0.0, 4.0, 2.0 );
  participant.state.vy = -1.0;

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace( participant.id, participant );

  const auto corridor_result =
    adore::planner::check_route_corridor_safety(
      route,
      ego,
      participants,
      validation_vehicle_params(),
      test_params() );

  ASSERT_TRUE( corridor_result.has_conflict );
  EXPECT_TRUE( corridor_result.conflict.predicted_spatiotemporal_conflict );
}

TEST( ObstacleAvoidance, StaticCorridorObjectFullyBehindEgoIsIgnored )
{
  const auto route = make_straight_route( 100.0, 1.0 );

  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 30.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;
  ego.vx = 2.0;
  ego.time = 0.0;

  auto ego_params = validation_vehicle_params();
  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    72,
    make_participant( 72, 25.0, 0.0, 0.0, 0.0, 2.0, 2.0 ) );

  const auto result =
    adore::planner::check_route_corridor_safety(
      route,
      ego,
      participants,
      ego_params,
      test_params() );

  EXPECT_TRUE( result.safe );
  EXPECT_FALSE( result.has_conflict );
  EXPECT_TRUE( result.static_clearance_improvements.empty() );
}

TEST( ObstacleAvoidance, CurrentMovingCorridorConflictKeepsPhysicalTtc )
{
  const auto route = make_straight_route( 100.0, 1.0 );

  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 0.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;
  ego.vx = 2.0;
  ego.time = 0.0;

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    76,
    make_participant( 76, 20.0, 0.0, 0.0, 2.0, 2.0, 2.0 ) );

  const auto result =
    adore::planner::check_route_corridor_safety(
      route,
      ego,
      participants,
      validation_vehicle_params(),
      test_params() );

  ASSERT_TRUE( result.has_conflict );
  EXPECT_TRUE( result.conflict.currently_overlaps_route_corridor );
  EXPECT_FALSE( result.conflict.predicted_spatiotemporal_conflict );
  EXPECT_GT(
    result.conflict.time_to_conflict,
    adore::planner::obstacle_avoidance_cycle_time_s );
}

TEST( ObstacleAvoidance, EgoTrajectoryTemporalSeparationIsNotOverriddenByCorridorForecast )
{
  const auto route = make_straight_route( 100.0, 1.0 );

  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 0.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;
  ego.vx = 2.0;
  ego.time = 0.0;

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    73,
    make_participant(
      73,
      40.0,
      10.0,
      -0.5 * std::numbers::pi,
      2.0,
      2.0,
      2.0 ) );

  auto params = test_params();
  params.prediction_time_horizon = 10.0;
  const auto ego_trajectory =
    make_linear_ego_trajectory( 0.0, 2.0, 10.0, 1.0 );

  const auto result =
    adore::planner::check_route_corridor_safety(
      route,
      ego,
      participants,
      validation_vehicle_params(),
      params,
      &ego_trajectory );

  EXPECT_TRUE( result.safe ) << result.reason;
  EXPECT_FALSE( result.has_conflict );
}

TEST( ObstacleAvoidance, EgoTrajectorySpatiotemporalOverlapRemainsConflict )
{
  const auto route = make_straight_route( 100.0, 1.0 );

  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 0.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;
  ego.vx = 2.0;
  ego.time = 0.0;

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    74,
    make_participant(
      74,
      10.0,
      10.0,
      -0.5 * std::numbers::pi,
      2.0,
      2.0,
      2.0 ) );

  auto params = test_params();
  params.prediction_time_horizon = 10.0;
  const auto ego_trajectory =
    make_linear_ego_trajectory( 0.0, 2.0, 10.0, 1.0 );

  const auto result =
    adore::planner::check_route_corridor_safety(
      route,
      ego,
      participants,
      validation_vehicle_params(),
      params,
      &ego_trajectory );

  EXPECT_FALSE( result.safe );
  EXPECT_TRUE( result.has_conflict );
  EXPECT_TRUE( result.conflict.predicted_spatiotemporal_conflict );
}

TEST( ObstacleAvoidance, MissingEgoTrajectoryKeepsConservativeCorridorForecast )
{
  const auto route = make_straight_route( 100.0, 1.0 );

  adore::dynamics::VehicleStateDynamic ego;
  ego.x = 0.0;
  ego.y = 0.0;
  ego.yaw_angle = 0.0;
  ego.vx = 2.0;
  ego.time = 0.0;

  adore::dynamics::TrafficParticipantSet participants;
  participants.participants.emplace(
    75,
    make_participant(
      75,
      40.0,
      10.0,
      -0.5 * std::numbers::pi,
      2.0,
      2.0,
      2.0 ) );

  auto params = test_params();
  params.prediction_time_horizon = 10.0;

  const auto result =
    adore::planner::check_route_corridor_safety(
      route,
      ego,
      participants,
      validation_vehicle_params(),
      params );

  EXPECT_FALSE( result.safe );
  EXPECT_TRUE( result.has_conflict );
  EXPECT_TRUE( result.conflict.predicted_spatiotemporal_conflict );
}

TEST( ObstacleAvoidance, OppositeLaneOncomingIsOwnedByGapCheck )
{
  adore::planner::RouteCorridorConflict oncoming;
  oncoming.object_class =
    adore::planner::RouteCorridorObjectClass::Oncoming;
  oncoming.currently_overlaps_route_corridor = true;

  EXPECT_TRUE(
    adore::planner::oa_detail::candidate_route_conflict_is_ignorable(
      oncoming,
      adore::planner::oa_detail::AvoidanceCandidateType::OppositeDirection,
      true,
      validation_vehicle_params(),
      test_params() ) );

  EXPECT_FALSE(
    adore::planner::oa_detail::candidate_route_conflict_is_ignorable(
      oncoming,
      adore::planner::oa_detail::AvoidanceCandidateType::ConnectedIntersection,
      false,
      validation_vehicle_params(),
      test_params() ) );

  auto crossing = oncoming;
  crossing.object_class =
    adore::planner::RouteCorridorObjectClass::CrossingOrUnknown;
  EXPECT_FALSE(
    adore::planner::oa_detail::candidate_route_conflict_is_ignorable(
      crossing,
      adore::planner::oa_detail::AvoidanceCandidateType::OppositeDirection,
      true,
      validation_vehicle_params(),
      test_params() ) );
}
