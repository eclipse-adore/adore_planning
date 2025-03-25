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
// #include "planning/trust_region_solver.hpp"

// namespace adore
// {
// namespace planner
// {


// void
// TrustRegionSolver::initialize()
// {
//   hessians.resize( num_agents );
//   gradients.resize( num_agents );
//   trust_deltas.resize( num_agents, delta_initial );

// for( size_t i = 0; i < num_agents; ++i )
// {
//   size_t control_size = agents[i].controls.size() * 2; // Each control has steering_angle and acceleration
//   hessians[i]         = Eigen::MatrixXd::Identity( control_size, control_size );
//   gradients[i]        = Eigen::VectorXd::Zero( control_size );
// }
// }

// bool
// TrustRegionSolver::convergence( const Eigen::VectorXd& gradient )
// {
//   return gradient.norm() < convergence_tolerance;
// }

// void
// TrustRegionSolver::compute_cost_and_gradient( Agent& agent, size_t agent_index )
// {
//   double control_effort_weight = 1.0;
//   double collision_weight      = 1000.0; // High penalty for collisions

// double cost         = 0.0;
// size_t num_states   = agent.trajectory.states.size();
// size_t control_size = num_states * 2; // Two control inputs per state

// // Initialize gradient
// Eigen::VectorXd gradient = Eigen::VectorXd::Zero( control_size );

// // For each time step
// for( size_t i = 0; i < num_states; ++i )
// {
//   dynamics::VehicleCommand& command = agent.controls[i];

// // Control effort term
// double steering_angle  = command.steering_angle;
// double acceleration    = command.acceleration;
// double control_effort  = control_effort_weight * ( steering_angle * steering_angle + acceleration * acceleration );
// cost                  += control_effort;

// // Gradient of control effort
// gradient( 2 * i )     += 2 * control_effort_weight * steering_angle;
// gradient( 2 * i + 1 ) += 2 * control_effort_weight * acceleration;

// // Collision avoidance
// dynamics::VehicleStateDynamic& curr_state = agent.trajectory.states[i];

// for( size_t j = 0; j < num_agents; ++j )
// {
//   if( j == agent_index )
//     continue;

// dynamics::VehicleStateDynamic& other_state = agents[j].trajectory.states[i];

// double dx               = curr_state.x - other_state.x;
// double dy               = curr_state.y - other_state.y;
// double distance_squared = dx * dx + dy * dy;
// double safe_distance    = 5.0; // Minimum safe distance

// if( distance_squared < safe_distance * safe_distance )
// {
//   // Penalize proximity
//   double collision_penalty  = collision_weight / ( distance_squared + 1e-6 );
//   cost                     += collision_penalty;

// // Approximate gradient calculation
// double grad_coeff = -2.0 * collision_weight / ( ( distance_squared + 1e-6 ) * ( distance_squared + 1e-6 ) );

// // Compute partial derivatives (simplified for demonstration)
// gradient( 2 * i )     += grad_coeff * dx;
// gradient( 2 * i + 1 ) += grad_coeff * dy;
// }
// }
// }

// // Set the agent's cost and gradient
// agent.cost             = cost;
// gradients[agent_index] = gradient;
// }

// void
// TrustRegionSolver::update_trust_region( Agent& agent, size_t agent_index, const Eigen::VectorXd& step, double cost_reduction_ratio )
// {
//   if( cost_reduction_ratio < 0.25 )
//   {
//     trust_deltas[agent_index] *= 0.5;
//   }
//   else if( cost_reduction_ratio > 0.75 && step.norm() > 0.8 * trust_deltas[agent_index] )
//   {
//     trust_deltas[agent_index] = std::min( 2.0 * trust_deltas[agent_index], delta_initial );
//   }
// }

// Eigen::VectorXd
// TrustRegionSolver::solve_quadratic_subproblem( const Eigen::MatrixXd& hessian, const Eigen::VectorXd& gradient, double delta )
// {
//   // Use the scaled gradient method for simplicity
//   Eigen::VectorXd p      = -gradient;
//   double          norm_p = p.norm();

// if( norm_p <= delta )
// {
//   return p;
// }
// else
// {
//   return -( delta / norm_p ) * gradient;
// }
// }

// void
// TrustRegionSolver::update_hessian( Agent& agent, size_t agent_index, const Eigen::VectorXd& s, const Eigen::VectorXd& y )
// {
//   Eigen::VectorXd Bs         = hessians[agent_index] * s;
//   Eigen::VectorXd y_minus_Bs = y - Bs;

// double denominator = y_minus_Bs.transpose() * s;
// if( std::abs( denominator ) > 1e-8 )
// {
//   hessians[agent_index] += ( y_minus_Bs * y_minus_Bs.transpose() ) / denominator;
// }
// }

// Eigen::VectorXd
// controls_to_eigen( const std::vector<dynamics::VehicleCommand>& controls )
// {
//   size_t          num_controls = controls.size();
//   Eigen::VectorXd control_vector( num_controls * 2 );

// for( size_t i = 0; i < num_controls; ++i )
// {
//   control_vector( 2 * i )     = controls[i].steering_angle;
//   control_vector( 2 * i + 1 ) = controls[i].acceleration;
// }
// return control_vector;
// }

// void
// eigen_to_controls( const Eigen::VectorXd& control_vector, std::vector<dynamics::VehicleCommand>& controls )
// {
//   size_t num_controls = controls.size();

// for( size_t i = 0; i < num_controls; ++i )
// {
//   controls[i].steering_angle = control_vector( 2 * i );
//   controls[i].acceleration   = control_vector( 2 * i + 1 );
// }
// }

// void
// TrustRegionSolver::solve()
// {
//   bool all_converged = false;
//   int  iteration     = 0;

// // Initialize old gradients vector
// old_gradients.resize( num_agents );

// while( !all_converged && iteration < max_iterations )
// {
//   all_converged = true;

// // First Loop: Compute proposed control updates independently for each agent
// std::vector<Eigen::VectorXd> steps( num_agents );
// std::vector<double>          old_costs( num_agents );
// std::vector<Eigen::VectorXd> old_control_vectors( num_agents );

// for( size_t i = 0; i < num_agents; ++i )
// {
//   Agent& agent = agents[i];

// // Compute cost and gradient
// compute_cost_and_gradient( agent, i );

// // Save old gradient for Hessian update later
// old_gradients[i] = gradients[i];

// // Check convergence
// if( !convergence( gradients[i] ) )
// {
//   all_converged = false;

// // Save old controls and cost
// old_control_vectors[i] = controls_to_eigen( agent.controls );
// old_costs[i]           = agent.cost;

// // Solve quadratic subproblem independently
// steps[i] = solve_quadratic_subproblem( hessians[i], gradients[i], trust_deltas[i] );
// }
// else
// {
// // Agent has converged; set step to zero
// steps[i] = Eigen::VectorXd::Zero( gradients[i].size() );
// }
// }

// // Second Loop: Apply updates, evaluate new cost, update Hessians
// for( size_t i = 0; i < num_agents; ++i )
// {
//   Agent& agent = agents[i];

// if( steps[i].norm() > 0 )
// {
//   // Update control vector
//   Eigen::VectorXd new_control_vector = old_control_vectors[i] + steps[i];

// // Update agent controls from Eigen vector
// eigen_to_controls( new_control_vector, agent.controls );

// // Enforce command limits
// for( auto& command : agent.controls )
// {
//   command.clamp_within_limits( agent.command_limits );
// }

// // Update trajectory
// agent.update_trajectory_from_controls( dt, wheelbase );

// // Compute new cost and gradient
// compute_cost_and_gradient( agent, i );

// // Actual reduction
// double actual_reduction = old_costs[i] - agent.cost;

// // Predicted reduction
// double predicted_reduction = -gradients[i].dot( steps[i] ) - 0.5 * steps[i].dot( hessians[i] * steps[i] );
// // Compute cost reduction ratio
// double cost_reduction_ratio = actual_reduction / ( predicted_reduction + 1e-8 ); // Add epsilon to avoid division by zero

// // Update trust region
// update_trust_region( agent, i, steps[i], cost_reduction_ratio );

// // Update Hessian approximation
// Eigen::VectorXd y = gradients[i] - old_gradients[i];
// update_hessian( agent, i, steps[i], y );
// }
// }

// ++iteration;
// }

// if( all_converged )
// {
//   std::cerr << "Converged in " << iteration << " iterations." << std::endl;
// }
// else
// {
//   std::cerr << "Reached maximum iterations without full convergence." << std::endl;
// }
// }

// } // namespace planner
// } // namespace adore
