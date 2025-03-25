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

#include "adore_map/route.hpp"
#include "adore_math/PiecewisePolynomial.h"
#include "adore_math/angles.h"
#include "adore_math/distance.h"
#include "adore_math/point.h"

#include "OptiNLC_Data.h"
#include "OptiNLC_OCP.h"
#include "OptiNLC_Options.h"
#include "OptiNLC_Solver.h"
#include "dynamics/trajectory.hpp"

namespace adore
{
namespace planner
{

class SafetyCorridorPlanner
{
public:

  enum STATES
  {
    X,
    Y,
    PSI,
    V,
    S,
    DELTA,
    dDELTA,
    L
  };

  enum CONTROLS
  {
    ddDELTA
  };

  static constexpr int    state_size       = 8;
  static constexpr int    input_size       = 1;
  static constexpr int    control_points   = 30;
  static constexpr double sim_time         = 3.0; // Simulation time for the MPC
  static constexpr int    constraints_size = 0;


private:

  double lateral_weight            = 0.4;
  double heading_weight            = 0.75;
  double steering_weight           = 1.0;
  double wheelbase                 = 2.69;
  double max_forward_speed         = 13.6;
  double max_reverse_speed         = -2.0;
  double max_steering_velocity     = 0.5;
  double max_steering_acceleration = 1.5;

  double reference_velocity = 3.0; // reference velocity of 3.0 m/s to start leaving the safety corridor
  double car_previous_x;
  double car_previous_y;
  double distance_moved = 0.0;
  bool   car_start      = true;

  double               bad_counter   = 0;
  bool                 bad_condition = false;
  dynamics::Trajectory previous_trajectory;

  // Variables to convert route to piecewise polynomial function
  adore::math::PiecewisePolynomial                  pp;
  adore::math::PiecewisePolynomial::PiecewiseStruct safety_corridor_x;
  adore::math::PiecewisePolynomial::PiecewiseStruct safety_corridor_y;
  adore::math::PiecewisePolynomial::PiecewiseStruct safety_corridor_heading;

  // Helper function to calculate position of car compared to border
  bool findIntersection( const adore::math::Point2d& p1, const adore::math::Point2d& p2, const adore::math::Point2d& carPos, double heading,
                         adore::math::Point2d& intersection );
  std::string getRelativePosition( const adore::math::Point2d& p1, const adore::math::Point2d& p2, const adore::math::Point2d& carPos );

  // Variables for MPC solver configuration
  OptiNLC_Options options;

  // Helper function to define the dynamic model
  void setup_dynamic_model( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp );

  // Helper function to define the objective function
  void setup_objective_function( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp );

  // Helper function to set constraints
  void setup_constraints( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp );

  // Helper function to set up the solver and solve the problem
  bool solve_mpc( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp,
                  VECTOR<double, state_size>& initial_state, VECTOR<double, input_size>& initial_input, std::vector<double>& delta_output,
                  std::vector<double>& acc_output, double current_time );

public:

  SafetyCorridorPlanner();
  dynamics::VehicleCommandLimits limits;

  // Public method to get the next vehicle command based on SafetyCorridorPlanner
  dynamics::Trajectory plan_trajectory( std::vector<adore::math::Point2d> border, const dynamics::VehicleStateDynamic& current_state );

  void set_parameters( const std::map<std::string, double>& params );
};
} // namespace planner
} // namespace adore
