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

#include "adore_math/angles.h"
#include "adore_math/distance.h"
#include "adore_math/PiecewisePolynomial.h"

#include "OptiNLC_Data.h"
#include "OptiNLC_OCP.h"
#include "OptiNLC_Options.h"
#include "OptiNLC_Solver.h"
#include "dynamics/trajectory.hpp"

namespace adore
{
namespace planner
{
  struct reference_to_piecewise_polynomial
  {
    adore::math::PiecewisePolynomial::PiecewiseStruct x;
    adore::math::PiecewisePolynomial::PiecewiseStruct y;
    adore::math::PiecewisePolynomial::PiecewiseStruct v;
    adore::math::PiecewisePolynomial::PiecewiseStruct heading;
  };

class OptiNLCTrajectoryOptimizer
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
  static constexpr double sim_time         = 3.0; // Simulation time for the MPC
  static constexpr int    constraints_size = 0;


private:

  double longitudinal_weight = 0.0;
  double lateral_weight      = 0.1;
  double velocity_weight     = 0.25;
  double heading_weight      = 0.25;
  double steering_weight     = 0.0;
  double acceleration_weight = 0.0;
  double wheelbase                 = 2.69;
  double max_forward_speed         = 13.6;
  double max_reverse_speed         = -2.0;
  double max_steering_velocity     = 0.25;
  double max_steering_acceleration = 0.5;
  double reference_velocity = 4.0;

  // Variables to store previous commands
  double last_steering_angle = 0.0;
  double last_acceleration   = 0.0;
  double bad_counter         = 0;
  int    counter             = 0;
  double tau                 = 2.5;  // first order velocity profile

  // Variables for MPC solver configuration
  OptiNLC_Options options;
  bool bad_condition       = false;
  dynamics::Trajectory previous_trajectory;
  double threshold_bad_output      = 20.0; // value of cost function above which is considered bad

  struct Path
  {
    std::vector<double> s; // progress
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> v;
    std::vector<double> psi; // heading
    std::vector<double> width;
  } reference_to_follow;

  // Variables to convert route to piecewise polynomial function
  adore::math::PiecewisePolynomial                  pp;
  adore::math::PiecewisePolynomial::PiecewiseStruct reference_trajectory_x;
  adore::math::PiecewisePolynomial::PiecewiseStruct reference_trajectory_y;
  adore::math::PiecewisePolynomial::PiecewiseStruct reference_trajectory_v;
  adore::math::PiecewisePolynomial::PiecewiseStruct reference_trajectory_heading;

  // Helper function to define the dynamic model
  void setup_dynamic_model( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp );

  // Helper function to define the objective function
  void setup_objective_function( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp );

  // Helper function to set constraints
  void setup_constraints( OptiNLC_OCP<double, input_size, state_size, constraints_size, control_points>& ocp );
                                
  // Helper function to set up route to follow to piecewise polynomial
  void setup_reference_trajectory( reference_to_piecewise_polynomial& latest_reference_trajectory );

  reference_to_piecewise_polynomial setup_optimizer_parameters_using_reference( const dynamics::Trajectory&          latest_reference_trajectory,
                                                                                const dynamics::VehicleStateDynamic& current_state );

  // Helper function to set up the solver and solve the problem
  bool solve_mpc( OptiNLC_OCP<double, OptiNLCTrajectoryOptimizer::input_size, OptiNLCTrajectoryOptimizer::state_size, 0,
                              OptiNLCTrajectoryOptimizer::control_points>& ocp,
                  VECTOR<double, state_size>& initial_state, VECTOR<double, input_size>& initial_input, std::vector<double>& delta_output,
                  std::vector<double>& acc_output, double current_time );

public:

  OptiNLCTrajectoryOptimizer();
  dynamics::VehicleCommandLimits limits;

  // Public method to get the next vehicle command based on OptiNLCTrajectoryOptimizer
  dynamics::Trajectory plan_trajectory( const dynamics::Trajectory&          reference_trajectory,
                                        const dynamics::VehicleStateDynamic& current_state );

  void set_parameters( const std::map<std::string, double>& params );
};
} // namespace planner
} // namespace adore
