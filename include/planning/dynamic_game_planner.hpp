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
 *    Giovanni Lucente
 ********************************************************************************/

#include <vector>
#include <Eigen/Dense>
#include <thread>
#include <iomanip>
#include <mutex>
#include "dynamics/dynamic_game_data.hpp"
#include "dynamics/traffic_participant.hpp"
#include "dynamics/trajectory.hpp"
#include "dynamics/vehicle_state.hpp"

namespace adore
{
namespace planner
{

class DynamicGamePlanner {

private:
    const double tau = 2.0;
    const double k = 5.0;
    const double r_safe = 2.5;
    const double r_lane = 3.5;
    const double eps = 1e-6;
    const double length = 5.0;
    const double cg_ratio = 0.5;
    const double pi = 3.1415;
    const double v_max = 10.0;

    constexpr static const int N = 20;                                  /** number of integration nodes */
    constexpr static const int nX = 6;                                  /** <X, Y, V, PSI, S, L> */
    constexpr static const int nU = 2;                                  /** <d, F> */
    constexpr static const double max_gradient_norm = 1e7;
    

public:
    static const int nx = nX * (N + 1);                                 /** size of the state trajectory X_i for each vehicle */
    static const int nu = nU * (N + 1);                                 /** size of the input trajectory U_i for each vehicle */
    int M;                                                              /** number of agents */ 
    int nC;                                                             /** total number of inequality constraints */
    int nC_i;                                                           /** inequality constraints for one vehicle */
    int nG;                                                             /** number of elements in the gradient G */
    int nX_;                                                            /** number of elements in the state vector X */
    int nU_;                                                            /** number of elements in the input vector U */
    int M_old;                                                          /** number of traffic participants in the previous iteration*/
    double dt = 0.3;                                                    /** integration time step */
    double d_up = 0.7;                                                  /** upper bound yaw rate */
    double d_low = -0.7;                                                /** lower bound yaw rate */
    double F_up = 2.0;                                                  /** upper bound force */
    double F_low = -3.0;                                                /** lower bound force */

    // Parameters:
    double qf = 1e-2;                                                   /** penalty for the final error in the lagrangian */
    double gamma = 1.0;                                                 /** increasing factor of the penalty weight */
    double rho = 1e-3;                                                  /** penalty weight */ 
    double weight_target_speed = 1e0;                                   /** weight for the maximum speed in the lagrangian */
    double weight_center_lane = 1e-1;                                   /** weight for the center lane in the lagrangian */
    double weight_heading = 1e2;                                        /** weight for the heading in the lagrangian */
    double weight_input = 0.0;                                          /** weight for the input in the lagrangian */
    
    std::vector<double> U_old;                                          /** solution in the previous iteration*/
    Eigen::MatrixXd ul;                                                 /** controls lower bound*/
    Eigen::MatrixXd uu;                                                 /** controls upper bound*/
    Eigen::MatrixXd time;                                               /** time vector */
    Eigen::MatrixXd lagrangian_multipliers;                             /** lagrangian multipliers*/
    
    
    enum STATES {x, y, v, psi, s, l};
    enum INPUTS {d, F};

    adore::dynamic_game::TrafficParticipants traffic;

    DynamicGamePlanner();  // Constructor
    ~DynamicGamePlanner(); // Destructor

    void plan_trajectories(dynamics::TrafficParticipantSet& traffic_participant_set,
                            const bool debug_mode_active = false);                  /** Main method to plan trajectories */
    void run( adore::dynamic_game::TrafficParticipants& traffic_state,
                const bool debug_mode_active = false );                             /** Main method to execute the planner */
    void setup();                                                                   /** Setup function */
    void initial_guess(double* X, double* U);                                       /** Set the initial guess */
    void trust_region_solver(double* U_);                                           /** solver of the dynamic game based on trust region */
    void integrate(double* X, const double* U);                                     /** Integration function */
    void dynamic_step(double* d_state, const double* state, const double* ref_state, 
                    const double* control);                                         /** Dynamic step function */
    void hessian_SR1_update( Eigen::MatrixXd & H_, const Eigen::MatrixXd & s_,            
                     const Eigen::MatrixXd & y_, const double r_ );                /** SR1 Hessian matrix update*/
    void increasing_schedule();                                                    /** function to increase rho = rho * gamma */
    void save_lagrangian_multipliers(double* lagrangian_multipliers_);             /** function to save the lagrangian multipliers */
    void compute_lagrangian_multipliers(double* lagrangian_multipliers_, 
                                        const double* constraints_);               /** computation of the lagrangian multipliers */
    
    void compute_constraints(double* constraints, const double* X_, 
                            const double* U_);                                     /** computation of the inequality constraints */
    void compute_constraints_vehicle_i(double* C_i, 
                            const double* X_, const double* U_, int i);            /** computation of the inequality constraints 
                                                                                        for vehicle i */
    void compute_squared_distances_vector(double* squared_distances_, const double* X_, 
                            int ego, int j);                                       /** computes a vector of the squared distance 
                                                                                        between the trajectory of vehicle i and j*/
    void compute_squared_lateral_distance_vector(double* squared_distances_, 
                            const double* X_, int i);                               /** computes a vector of the squared lateral distance 
                                                                                        between the i-th trajectory and the allowed center 
                                                                                        lines at each time step*/
    double compute_cost_vehicle_i(const double* X_, const double* U_, int i);         /** compute the cost for vehicle i */
    void compute_lagrangian(double* lagrangian, 
                            const double* X_, const double* U_);                    /** computes of the augmented lagrangian vector 
                                                                                    L = <L_1, ..., L_M> 
                                                                                    L_i = cost_i + lagrangian_multipliers * constraints */
    double compute_lagrangian_vehicle_i(double J_i, const double* C_i, int i);      /** computation of the augmented lagrangian for vehicle i: 
                                                                                lagrangian_i = cost_i + lagrangian_multipliers_i * constraints_i */
    void compute_gradient(double* gradient, const double* U_);                      /** computes the gradient of lagrangian_i with respect to 
                                                                                    U_i for each i */
    void quadratic_problem_solver(Eigen::MatrixXd & s_, 
                                const Eigen::MatrixXd & G_, 
                                const Eigen::MatrixXd & H_, double Delta);          /** it solves the quadratic problem 
                                                                                        (GT * s + 0.5 * sT * H * s) with solution included in the 
                                                                                        trust region ||s|| < Delta */
    void constraints_diagnostic(const double* constraints, bool print);             /** shows violated constraints */
    void print_trajectories(const double* X, const double* U);                      /** prints trajectories */
    adore::dynamic_game::TrafficParticipants set_prediction(const double* X_, 
                                                        const double* U_);         /** sets the prediction to the traffic structure */
    double compute_heading(const tk::spline & spline_x, 
                           const tk::spline & spline_y, double s);                  /** computes the heading on the spline x(s) and y(s) at parameter s */
    double gradient_norm(const double* gradient);                                               /** computes the norm of the gradient */
    void correctionU(double* U_);                                                    /** corrects U if outside the boundaries */
    void save_solution(const double* U_, const int M_);                             /** saves the solution in the general variable U_old */
    void set_solution(dynamics::TrafficParticipantSet& traffic_participant_set);    /** sets the solution in the original struct */
};

} // namespace planner
} // namespace adore
