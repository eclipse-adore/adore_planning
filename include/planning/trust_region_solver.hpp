// /********************************************************************************
//  * Copyright (C) 2017-2025 German Aerospace Center (DLR).
//  * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
//  *
//  * This program and the accompanying materials are made available under the
//  * terms of the Eclipse Public License 2.0 which is available at
//  * http://www.eclipse.org/legal/epl-2.0.
//  *
//  * SPDX-License-Identifier: EPL-2.0
//  *
//  * Contributors:
//  *    Giovanni Lucente
//  *    Marko Mizdrak
//  ********************************************************************************/
// #pragma once
// #include <iostream>
// #include <optional>

// #include "adore_map/map.hpp"

// #include "dynamics/integration.hpp"
// #include "dynamics/trajectory.hpp"

// namespace adore
// {
// namespace planner
// {

// class Agent
// {
// public:

// dynamics::Trajectory                  trajectory;     // The agent's trajectory
// std::vector<dynamics::VehicleCommand> controls;       // Control inputs at each time step
// dynamics::VehicleCommandLimits        command_limits; // Limits for the control inputs
// double                                cost;           // Total cost for the agent
// size_t                                id = 0;

// // Constructor
// Agent( dynamics::Trajectory& initial_trajectory, const dynamics::VehicleCommandLimits& limits ) :
//   trajectory( initial_trajectory ),
//   command_limits( limits ),
//   cost( 0.0 )
// {
//   initialize_controls_from_trajectory();
// }

// // Initialize controls from the trajectory
// void
// initialize_controls_from_trajectory()
// {
//   size_t num_states = trajectory.states.size();
//   controls.resize( num_states ); // One control per state

// for( size_t i = 0; i < num_states; ++i )
// {
//   controls[i].steering_angle = trajectory.states[i].steering_angle;
//   controls[i].acceleration   = trajectory.states[i].ax; // Assuming ax is longitudinal acceleration
// }
// }

// // Update trajectory based on current controls
// void
// update_trajectory_from_controls( double dt, double wheelbase )
// {
//   size_t num_states = trajectory.states.size();

// for( size_t i = 1; i < num_states; ++i )
// {
//   dynamics::VehicleStateDynamic& prev_state = trajectory.states[i - 1];
//   dynamics::VehicleStateDynamic& curr_state = trajectory.states[i];

// dynamics::VehicleCommand& command = controls[i];

// // Clamp the command within limits
// command.clamp_within_limits( command_limits );

// // Integrate vehicle dynamics using euler_step
// curr_state = dynamics::euler_step( prev_state, command, dt, wheelbase );
// }
// }
// };

// class TrustRegionSolver
// {
// public:

// TrustRegionSolver( std::vector<Agent>& agents, double dt, double wheelbase ) :
//   agents( agents ),
//   num_agents( agents.size() ),
//   dt( dt ),
//   wheelbase( wheelbase ),
//   convergence_tolerance( 1e-6 ),
//   max_iterations( 100 ),
//   delta_initial( 1.0 ),
//   eta( 0.1 ),
//   gamma( 1.05 ),
//   rho( 1e-3 )
// {
//   initialize();
// }

// void solve();

// private:

// std::vector<Agent>& agents;
// size_t              num_agents;
// double              dt;
// double              wheelbase;

// double convergence_tolerance;
// int    max_iterations;
// double delta_initial;
// double eta;
// double gamma;
// double rho;

// std::vector<Eigen::MatrixXd> hessians;      // Hessian approximations for each agent
// std::vector<Eigen::VectorXd> gradients;     // Gradients for each agent
// std::vector<Eigen::VectorXd> old_gradients; // Gradients for each agent
// std::vector<double>          trust_deltas;  // Trust region deltas for each agent

// std::optional<map::Map> local_map = std::nullopt;

// void            initialize();
// bool            convergence( const Eigen::VectorXd& gradient );
// void            compute_cost_and_gradient( Agent& agent, size_t agent_index );
// void            update_trust_region( Agent& agent, size_t agent_index, const Eigen::VectorXd& step, double cost_reduction_ratio );
// Eigen::VectorXd solve_quadratic_subproblem( const Eigen::MatrixXd& hessian, const Eigen::VectorXd& gradient, double delta );
// void            update_hessian( Agent& agent, size_t agent_index, const Eigen::VectorXd& s, const Eigen::VectorXd& y );
// };
// } // namespace planner
// } // namespace adore
