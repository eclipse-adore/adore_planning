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

#pragma once

#include <cmath>

#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <unordered_map>
#include <vector>

#include "adore_map/map_point.hpp"
#include "adore_map/route.hpp"
#include "adore_math/fast_trig.h"
#include "adore_math/point.h"

#include "dynamics/comfort_settings.hpp"
#include "dynamics/physical_vehicle_model.hpp"
#include "dynamics/traffic_participant.hpp"
#include "dynamics/trajectory.hpp"
#include "dynamics/vehicle_state.hpp"
#include "multi_agent_solver/multi_agent_solver.hpp"

namespace adore
{
namespace planner
{

struct PlannerResult
{
  std::optional<dynamics::Trajectory> trajectory;
  map::Route                          modified_route;
  std::string                         planner_message;
};

struct PathState
{
  double s;
  double x;
  double y;
  double yaw;
  double steer;
};

struct ContinuityCost
{
  double distance;
  double heading_diff;
};

class HybridAStarPlanner
{
public:

  HybridAStarPlanner();

  void set_parameters( const std::map<std::string, double>& params );
  void set_comfort_settings( const std::shared_ptr<dynamics::ComfortSettings>& settings );
  void set_vehicle_parameters( const dynamics::PhysicalVehicleParameters& params );
  void set_goal( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                 const dynamics::TrafficParticipantSet& traffic_participants );

  map::Route    plan( const adore::dynamics::VehicleStateDynamic& ego, const adore::dynamics::TrafficParticipantSet& participants,
                      const std::optional<math::Polygon2d>& drivable_area, const map::Route& latest_route );
  PlannerResult plan_trajectory( const dynamics::VehicleStateDynamic& current_state, const dynamics::TrafficParticipantSet& participants,
                                 const std::optional<math::Polygon2d>& drivable_area, const map::Route& latest_route );

  dynamics::Trajectory optimize_trajectory( const dynamics::VehicleStateDynamic& current_state, const map::Route& ref_route );

private:

  dynamics::PhysicalVehicleParameters        vehicle_params;
  std::shared_ptr<dynamics::ComfortSettings> comfort_settings;
  dynamics::TrafficParticipantSet            all_participants;
  dynamics::VehicleStateDynamic              start_state;

  double goal_x        = 0;
  double goal_y        = 0;
  double goal_yaw      = 0;
  double dt            = 0.1;
  size_t horizon_steps = 40;
  double ref_velocity  = 0.0;
  double local_goal_x  = 0.0;
  double local_goal_y  = 0.0;

  double distance_weight = 1.0;
  double heading_weight  = 3.5;
  double unstructured_speed = 2.8;

  std::shared_ptr<mas::OCP> problem;
  map::Route                reference_route;
  map::Route                previous_route;
  bool                      has_previous_route = false;
  math::Point2d             current_local_goal;
  bool                      has_local_goal    = false;
  bool                      final_goal_locked = true;
  std::string               planner_message;

  std::vector<PathState> previous_path_states;

  struct SolverParams
  {
    double max_iterations = 1000;
    double tolerance      = 1e-3;
    double max_ms         = 50;
    double debug          = 0.0;
  } solver_params;

  struct PlannerCostWeights
  {
    double lane_error     = 5.0;
    double long_error     = 0.1;
    double speed_error    = 5.0;
    double heading_error  = 10.0;
    double steering_angle = 1.5;
    double acceleration   = 0.1;
  } weights;

  static constexpr double WHEEL_BASE = 2.7;
  static constexpr double MAX_STEER  = 30.0 * M_PI / 180.0;

  static constexpr double STEP              = 1.0;
  static constexpr double MOTION_RESOLUTION = 0.1;

  static constexpr double XY_RES  = 1.0;
  static constexpr double YAW_RES = 5.0 * M_PI / 180.0;

  static constexpr double VEHICLE_RADIUS     = 2.0;
  static constexpr double OBSTACLE_INFLATION = 1.6;

  //------------------------------------------
  // NODE
  //------------------------------------------

  struct Node
  {
    double x;
    double y;
    double yaw;
    double g;
    double steer;

    Node* parent;

    Node( double x_, double y_, double yaw_, double g_, double steer_, Node* p ) :
      x( x_ ),
      y( y_ ),
      yaw( yaw_ ),
      g( g_ ),
      steer( steer_ ),
      parent( p )
    {}

    std::tuple<int, int, int> grid_index() const;
  };

  //------------------------------------------
  // PRIORITY QUEUE NODE
  //------------------------------------------

  struct QueueNode
  {
    double cost;
    int    counter;
    Node*  node;

    bool
    operator<( const QueueNode& other ) const
    {
      return cost > other.cost;
    }
  };

  //------------------------------------------
  // HASH FOR GRID INDEX
  //------------------------------------------

  struct GridHash
  {
    size_t operator()( const std::tuple<int, int, int>& k ) const;
  };

  //------------------------------------------
  // INTERNAL FUNCTIONS
  //------------------------------------------

  bool simulate_motion( double& x, double& y, double& yaw, double steer, const dynamics::TrafficParticipantSet& participants,
                        const std::optional<math::Polygon2d>& drivable_area );
  bool inside_drivable_area( double x, double y, double yaw, const std::optional<math::Polygon2d>& drivable_area );

  bool collision( double x, double y, const adore::dynamics::TrafficParticipantSet& participants );

  double heuristic( double x, double y, double yaw, const math::Point2d& local_goal );
  double distance_to_previous_route( double x, double y );
  double route_difference( const map::Route& r1, const map::Route& r2 );
  double find_closest_s_on_route( const map::Route& route, const dynamics::VehicleStateDynamic& ego );
  double distance_to_polygon_boundary( const math::Point2d& p, const math::Polygon2d& polygon );

  double compute_idm_velocity( const adore::map::Route& route, const adore::dynamics::VehicleStateDynamic& ego,
                               const adore::dynamics::TrafficParticipantSet& participants, double goal_distance );

  math::Point2d compute_local_goal( const dynamics::VehicleStateDynamic& ego, const std::optional<math::Polygon2d>& drivable_area );

  bool try_goal_connection( Node* node, const math::Point2d& local_goal, const dynamics::TrafficParticipantSet& participants,
                            const std::optional<math::Polygon2d>& drivable_area, std::vector<std::pair<double, double>>& path );

  map::Route trim_route_from_ego( const map::Route& route, const dynamics::VehicleStateDynamic& ego );

  std::vector<Node*> reconstruct( Node* node );


  void                   setup_problem();
  void                   solve_problem();
  mas::StageCostFunction make_trajectory_cost( const map::Route& ref_route );

  mas::MotionModel     get_planning_model( const dynamics::PhysicalVehicleParameters& params );
  dynamics::Trajectory extract_trajectory();
};

} // namespace planner
} // namespace adore
