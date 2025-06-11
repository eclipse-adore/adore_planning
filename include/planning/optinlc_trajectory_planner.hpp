// #pragma once

// #include <cmath>

// #include <chrono>
// #include <iostream>
// #include <limits>
// #include <map>
// #include <vector>

// #include "adore_map/map.hpp"
// #include "adore_map/route.hpp"
// #include "adore_math/angles.h"
// #include "adore_math/distance.h"

// #include "OptiNLC_Data.h"
// #include "OptiNLC_OCP.h"
// #include "OptiNLC_Options.h"
// #include "OptiNLC_Solver.h"
// #include "dynamics/traffic_participant.hpp"
// #include "dynamics/trajectory.hpp"
// #include "planning/speed_profiles.hpp"

// namespace adore
// {
// namespace planner
// {

// class OptiNLCTrajectoryPlanner
// {
// public:

// enum STATES
// {
//   X,
//   Y,
//   PSI,
//   V,
//   DELTA,
//   S,
//   L
// };

// enum CONTROLS
// {
//   dDELTA
// };

// static constexpr int    state_size       = 7;
// static constexpr int    input_size       = 1;
// static constexpr int    control_points   = 30;
// static constexpr int    constraints_size = 0;
// static constexpr double sim_time         = 3.0;

// private:

// // Core configuration
// OptiNLC_Options options;

// // Parameters
// double wheelbase                 = 2.69;
// double max_forward_speed         = 13.6;
// double max_reverse_speed         = -2.0;
// double max_steering_velocity     = 0.5;
// double max_steering_acceleration = 10.5;

// // IDM & Planning
// double desired_time_headway     = 1.5;
// double max_acceleration         = 2.0;
// double max_deceleration         = 2.5;
// double max_lateral_acceleration = 1.0; // m/s^2, lateral acceleration limit
// double planning_horizon_s       = 100.0;

// // MPC failure tracking
// double               bad_counter   = 0;
// bool                 bad_condition = false;
// dynamics::Trajectory previous_trajectory;

// // Inputs from planner interface
// map::Route                     route;
// dynamics::VehicleCommandLimits limits;

// // Internal setup helpers
// void setup_dynamic_model( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp );
// void setup_objective_function( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp );
// void setup_constraints( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp );

// public:

// SpeedProfile speed_profile;

// OptiNLCTrajectoryPlanner();
// void set_parameters( const std::map<std::string, double>& params );

// dynamics::Trajectory plan_trajectory( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
//                                       const map::Map& latest_map, const dynamics::TrafficParticipantSet& traffic_participants );
// };

// } // namespace planner
// } // namespace adore
