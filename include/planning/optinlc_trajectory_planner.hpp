/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Sanath Himasekhar Konthala
 ********************************************************************************/
#pragma once

#include <cmath>

#include <chrono>
#include <iostream>
#include <limits>
#include <vector>

#include "adore_map/map.hpp"
#include "adore_map/route.hpp"
#include "adore_math/PiecewisePolynomial.h"
#include "adore_math/angles.h"
#include "adore_math/curvature.hpp"
#include "adore_math/distance.h"

#include "OptiNLC_Data.h"
#include "OptiNLC_OCP.h"
#include "OptiNLC_Options.h"
#include "OptiNLC_Solver.h"
#include "dynamics/traffic_participant.hpp"
#include "dynamics/trajectory.hpp"

namespace adore
{
namespace planner
{
struct route_to_piecewise_polynomial
{
  adore::math::PiecewisePolynomial::PiecewiseStruct x;
  adore::math::PiecewisePolynomial::PiecewiseStruct y;
  adore::math::PiecewisePolynomial::PiecewiseStruct heading;
};

class OptiNLCTrajectoryPlanner
{
public:

  enum STATES
  {
    X,
    Y,
    PSI,
    V,
    DELTA,
    dDELTA,
    S,
    L
  };

  enum CONTROLS
  {
    ddDELTA
  };

  static constexpr int    state_size       = 8;
  static constexpr int    input_size       = 1;
  static constexpr int    control_points   = 30;
  static constexpr double sim_time         = 3.0; // Simulation time for the Planner
  static constexpr int    constraints_size = 0;


private:


  struct Path
  {
    std::vector<double> s; // progress
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> v;
    std::vector<double> psi; // heading
    std::vector<double> width;
  } route_to_follow;

  route_to_piecewise_polynomial setup_optimizer_parameters_using_route( const adore::map::Route&             latest_route,
                                                                        const dynamics::VehicleStateDynamic& current_state );

  double lateral_weight            = 0.01;
  double heading_weight            = 0.06;
  double steering_weight           = 1.0;
  double dt                        = 0.1; // 10ms frequency of the node
  double wheelbase                 = 2.69;
  double max_forward_speed         = 13.6;
  double max_reverse_speed         = -2.0;
  double max_steering_velocity     = 0.5;
  double max_steering_acceleration = 0.5;

  double min_distance_in_route     = 0.1;
  double position_smoothing_factor = 0.9;
  double heading_smoothing_factor  = 0.9;
  double threshold_bad_output      = 20.0; // value of cost function above which is considered bad

  // Curvature based velocity calculation members
  double              maximum_velocity   = 5.0; // Maximum set velocity
  double              reference_velocity = 5.0; // Reference velocity for planner
  double              lookahead_time     = 3.0; // 3 seconds lookahead for curvature
  double              distance_moved     = 0.0;
  std::vector<double> curvature_behind;
  double              look_behind_for_curvature = 3.0; // 3 meters look behind for curvature based speed reduction
  int                 distance_to_add_behind    = 1;
  int                 safe_index                = 10;  // safe index for curvature
  double              lateral_acceleration      = 1.0; // max lateral acceleration 1.0 m/s²
  double              minimum_velocity_in_curve = 1.5; // min velocity in a curve 2 m/s²

  // IDM related members
  double min_distance_to_vehicle_ahead = 10.0; // 10 meters minimum gap to vehicle in front
  double desired_time_headway          = 1.5;  // 1.5 seconds time headway
  double front_vehicle_velocity        = 0.0;  // temporary, TODO -> Get from traffic participants list
  double max_acceleration              = 2.0;  // Maximum acceleration 2.0 m/s²
  double max_deceleration              = 2.5;  // Maximum deceleration 2.5 m/s²
  double tau                           = 2.5;  // first order velocity profile
  double distance_to_goal              = 100.0;
  int    prediction_horizon            = 40; // 40 points for 4 seconds prediction horizon

  // Variables to store previous commands
  double               last_steering_angle = 0.0;
  double               last_acceleration   = 0.0;
  double               bad_counter         = 0;
  double steering_rate = 1.0;
  bool                 bad_condition       = false;
  dynamics::Trajectory previous_trajectory;

  // Variables to convert route to piecewise polynomial function
  adore::math::PiecewisePolynomial                  pp;
  adore::math::PiecewisePolynomial::PiecewiseStruct route_x;
  adore::math::PiecewisePolynomial::PiecewiseStruct route_y;
  adore::math::PiecewisePolynomial::PiecewiseStruct route_heading;

  // Variables for MPC solver configuration
  OptiNLC_Options options;

  // Helper function to define the dynamic model
  void setup_dynamic_model( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp );

  // Helper function to define the objective function
  void setup_objective_function( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp );

  // Helper function to set constraints
  void setup_constraints( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp );

  // Helper function to set up route to follow to piecewise polynomial
  void setup_reference_route( route_to_piecewise_polynomial& reference_route );

  // Helper function to get reference velocity
  void   setup_reference_velocity( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                                   const map::Map& latest_map, const dynamics::TrafficParticipantSet& traffic_participants );
  double calculate_idm_velocity( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                                 const map::Map& latest_map, const dynamics::TrafficParticipantSet& traffic_participants );

  // Helper function to set up the solver and solve the problem
  bool solve_mpc( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp,
                  VECTOR<double, state_size>& initial_state, VECTOR<double, input_size>& initial_input, std::vector<double>& delta_output,
                  std::vector<double>& acc_output, double current_time );

public:

  OptiNLCTrajectoryPlanner();
  dynamics::VehicleCommandLimits limits;

  // Public method to get the next vehicle command based on OptiNLCTrajectoryPlanner
  dynamics::Trajectory plan_trajectory( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                                        const map::Map& latest_map, const dynamics::TrafficParticipantSet& traffic_participants );

  void set_parameters( const std::map<std::string, double>& params );
};
} // namespace planner
} // namespace adore