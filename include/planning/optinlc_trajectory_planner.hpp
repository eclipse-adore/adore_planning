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
#include "planning/speed_profiles.hpp"

namespace adore
{
namespace planner
{

struct prediction_to_piecewise_polynomial
{
  adore::math::PiecewisePolynomial::PiecewiseStruct x;
  adore::math::PiecewisePolynomial::PiecewiseStruct y;
  adore::math::PiecewisePolynomial::PiecewiseStruct ax;
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
    std::vector<double> ax;
    std::vector<double> psi; // heading
    std::vector<double> width;
  } trajectory_to_follow;

  prediction_to_piecewise_polynomial setup_optimizer_parameters_using_prediction( const dynamics::TrafficParticipantSet& traffic_participants,
                                                                      const dynamics::VehicleStateDynamic& current_state );

  double lateral_weight            = 0.01;
  double heading_weight            = 0.06;
  double steering_weight           = 1.0;
  double dt                        = 0.1; // 10ms frequency of the node
  double wheelbase                 = 2.69;
  double max_forward_speed         = 13.6;
  double max_reverse_speed         = -5.0;
  double max_steering_velocity     = 0.5;
  double max_steering_acceleration = 0.5;
  double max_acceleration          = 2.0;  // Maximum acceleration 2.0 m/s²
  double max_deceleration          = 5.5;  // Maximum deceleration 2.5 m/s²

  double min_distance_in_route     = 0.1;
  double position_smoothing_factor = 0.9;
  double heading_smoothing_factor  = 0.9;
  double threshold_bad_output      = 5.0; // value of cost function above which is considered bad


  // Variables to store previous commands
  double  bad_counter         = 0;
  bool    bad_condition       = false;
  int     iteration           = 0;
  dynamics::Trajectory previous_trajectory;

  // Variables to convert route to piecewise polynomial function
  adore::math::PiecewisePolynomial                  pp_prediction;
  adore::math::PiecewisePolynomial::PiecewiseStruct ego_prediction_x;
  adore::math::PiecewisePolynomial::PiecewiseStruct ego_prediction_y;
  adore::math::PiecewisePolynomial::PiecewiseStruct ego_prediction_ax;
  adore::math::PiecewisePolynomial::PiecewiseStruct ego_prediction_heading;

  // Variables for MPC solver configuration
  OptiNLC_Options options;

  // Helper function to define the dynamic model
  void setup_dynamic_model( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp );

  // Helper function to define the objective function
  void setup_objective_function( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp );

  // Helper function to set constraints
  void setup_constraints( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp );

  // Helper function to set up route to follow to piecewise polynomial
  void setup_reference_trajectory( prediction_to_piecewise_polynomial& ego_prediction );

  // Helper function to set up the solver and solve the problem
  bool solve_mpc( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp,
                  VECTOR<double, state_size>& initial_state, VECTOR<double, input_size>& initial_input, std::vector<double>& delta_output,
                  std::vector<double>& acc_output, double current_time );

public:

  OptiNLCTrajectoryPlanner();
  SpeedProfile speed_profile;
  dynamics::VehicleCommandLimits limits;

  // Public method to get the next vehicle command based on OptiNLCTrajectoryPlanner
  dynamics::Trajectory plan_trajectory( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                                        const map::Map& latest_map, const dynamics::TrafficParticipantSet& traffic_participants );

  void set_parameters( const std::map<std::string, double>& params );
};
} // namespace planner
} // namespace adore
