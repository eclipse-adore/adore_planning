#pragma once

#include <cmath>

#include <chrono>
#include <iostream>
#include <limits>
#include <map>
#include <vector>

#include "adore_map/map.hpp"
#include "adore_map/route.hpp"
#include "adore_math/angles.h"
#include "adore_math/distance.h"

#include "dynamics/comfort_settings.hpp"
#include "dynamics/traffic_participant.hpp"
#include "dynamics/trajectory.hpp"
#include "multi_agent_solver/models/single_track_model.hpp"
#include "multi_agent_solver/multi_agent_solver.hpp"
#include "planning/speed_profiles.hpp"

namespace adore
{
namespace planner
{

class TrajectoryPlanner
{


public:


  dynamics::Trajectory plan_route_trajectory( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                                              const dynamics::TrafficParticipantSet& traffic_participants );

  dynamics::Trajectory optimize_trajectory( const dynamics::VehicleStateDynamic& current_state,
                                            const dynamics::Trajectory&          reference_trajectory );

  void set_parameters( const std::map<std::string, double>& params );
  void set_vehicle_parameters( const dynamics::PhysicalVehicleParameters& params );
  void set_comfort_settings( const std::shared_ptr<dynamics::ComfortSettings>& settings );

private:

  struct SolverParams
  {
    double max_iterations = 1000;
    double tolerance      = 1e-3;
    double max_ms         = 50;
    double debug          = 1.0;
  } solver_params;

  struct PlannerCostWeights
  {
    double lane_error     = 5.0;
    double long_error     = 0.1;
    double speed_error    = 5.0;
    double heading_error  = 10.0;
    double steering_angle = 1.0;
    double acceleration   = 0.1;
  } weights;

  double dt              = 0.1;
  size_t horizon_steps   = 30;
  double ref_traj_length = 200;

  std::shared_ptr<mas::OCP>     problem;
  dynamics::Trajectory          reference_trajectory; // Reference trajectory for the planner
  dynamics::VehicleStateDynamic start_state;          // Current state of the vehicle

  dynamics::PhysicalVehicleParameters        vehicle_params;
  std::shared_ptr<dynamics::ComfortSettings> comfort_settings;


  void                   setup_problem();
  mas::StageCostFunction make_trajectory_cost( const dynamics::Trajectory& ref_traj );
  mas::MotionModel       get_planning_model( const dynamics::PhysicalVehicleParameters& params );
  dynamics::Trajectory   extract_trajectory();
  void                   solve_problem();
};

} // namespace planner
} // namespace adore
