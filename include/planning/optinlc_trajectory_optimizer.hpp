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

#include "OptiNLC_Data.h"
#include "OptiNLC_OCP.h"
#include "OptiNLC_Options.h"
#include "OptiNLC_Solver.h"
#include "dynamics/trajectory.hpp"

namespace adore
{
namespace planner
{

class OptiNLCTrajectoryOptimizer
{
public:

  enum STATES
  {
    X,
    Y,
    PSI,
    V,
    L
  };

  enum CONTROLS
  {
    DELTA,
    ACC
  };

  static constexpr int    state_size       = 5;
  static constexpr int    input_size       = 2;
  static constexpr int    control_points   = 30;
  static constexpr double sim_time         = 3.0; // Simulation time for the MPC
  static constexpr int    constraints_size = 0;


private:

  double longitudinal_weight = 1.0;
  double lateral_weight      = 1.0;
  double velocity_weight     = 1.0;
  double heading_weight      = 1.0;
  double steering_weight     = 1.0;
  double acceleration_weight = 1.0;

  // Variables to store previous commands
  double last_steering_angle = 0.0;
  double last_acceleration   = 0.0;
  double bad_counter         = 0;
  int    counter             = 0;

  // Variables for MPC solver configuration
  OptiNLC_Options options;

  // Helper function to define the dynamic model
  void setup_dynamic_model( OptiNLC_OCP<double, OptiNLCTrajectoryOptimizer::input_size, OptiNLCTrajectoryOptimizer::state_size, 0,
                                        OptiNLCTrajectoryOptimizer::control_points>& ocp,
                            const dynamics::Trajectory&                              trajectory );

  // Helper function to define the objective function
  void setup_objective_function( OptiNLC_OCP<double, OptiNLCTrajectoryOptimizer::input_size, OptiNLCTrajectoryOptimizer::state_size, 0,
                                             OptiNLCTrajectoryOptimizer::control_points>& ocp );

  // Helper function to set constraints
  void setup_constraints( OptiNLC_OCP<double, OptiNLCTrajectoryOptimizer::input_size, OptiNLCTrajectoryOptimizer::state_size, 0,
                                      OptiNLCTrajectoryOptimizer::control_points>& ocp );

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
