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

#include "planning/dynamic_game_planner.hpp"
#include <iostream>

namespace adore
{
namespace planner
{

DynamicGamePlanner::DynamicGamePlanner() 
{
}

DynamicGamePlanner::~DynamicGamePlanner() {
    // Destructor implementation
    std::cout << "DynamicGamePlanner destroyed." << std::endl;
}

void DynamicGamePlanner::plan_trajectories(dynamics::TrafficParticipantSet& traffic_participant_set, const bool debug_mode_active) {
    // Initialize the traffic participants
    adore::dynamic_game::TrafficParticipants traffic_;
    
    for (const auto& [id, participant] : traffic_participant_set.participants) {    
        adore::dynamic_game::VehicleState vehicle;
        
        vehicle.x = participant.state.x;
        vehicle.y = participant.state.y;
        vehicle.psi = participant.state.yaw_angle;
        vehicle.v = participant.state.vx;
        vehicle.beta = 0.0;
        vehicle.v_target = 5.0;
        vehicle.id = id; 
        
        if (participant.route.has_value())
        {
            double current_s = participant.route->get_s(participant.state);
            std::vector<double> x_vals, y_vals, s_vals;
            for (const auto& [s, point] : participant.route->center_lane) {
                if (s > current_s)
                {
                    x_vals.push_back(point.x);
                    y_vals.push_back(point.y);
                    s_vals.push_back(s - current_s);
                }
            }
            // Initialize the center lane spline
            vehicle.centerlane.initialize_spline(x_vals, y_vals, s_vals);
            vehicle.centerlane.s_max = participant.route->get_length() - current_s;    
            if (vehicle.centerlane.s_max < 5.0 * (N + 1) * dt){
                vehicle.v_target = vehicle.centerlane.s_max / ((N + 1) * dt); // Adjust target speed based on available route length
            }   
        }else{
            std::cerr << "No route available for participant with ID: " << id << std::endl;
            std::vector<double> x_vals, y_vals, s_vals;
            for (size_t i = 0; i < N + 1; i++)
            {
                x_vals.push_back(participant.state.x + i * dt * participant.state.vx * std::cos(participant.state.yaw_angle));
                y_vals.push_back(participant.state.y + i * dt * participant.state.vx * std::sin(participant.state.yaw_angle));
                s_vals.push_back(i * dt * participant.state.vx);
            }
            // Initialize the center lane spline
            vehicle.centerlane.initialize_spline(x_vals, y_vals, s_vals);
            vehicle.centerlane.s_max = N * dt * participant.state.vx; // Approximate length based on initial state
            vehicle.v_target = 0.0; 
        }
        
        // Add the traffic participant to the traffic participants vector
        traffic_.push_back(vehicle);
    }

    // Run the planner with the initialized traffic participants
    run(traffic_);
    
    // Populate the original traffic participant set with the planned trajectories
    set_solution(traffic_participant_set);
}

void DynamicGamePlanner::run(adore::dynamic_game::TrafficParticipants& traffic_state, const bool debug_mode_active) {
    
    traffic = traffic_state;

    // Variables initialization and setup:
    setup();

    // definition of the control variable vector U and of the state vector X:
    double U[nU_];
    double X[nX_];
    double constraints[nC];

    initial_guess(X, U);
    trust_region_solver(U);
    integrate(X, U);
    if (debug_mode_active) {
        print_trajectories(X, U);
        compute_constraints(constraints, X, U);
        constraints_diagnostic(constraints, false);
    }
    traffic = set_prediction(X, U);
    save_solution(U, M);
}

void DynamicGamePlanner::setup() {
    
    // Setup number of traffic participants:
    M = traffic.size();
    
    // Setup number of inequality constraints for one vehicle:
    // 2 * nU * (N + 1) inequality constraints for inputs 
    // (N + 1) * (M - 1) collision avoidance constraints
    // (N + 1) constraints to remain in the lane
    nC_i = 2 * nU * (N + 1) + (N + 1) * (M - 1) + (N + 1) + 1;

    // Setup number of inequality constraints for all the traffic participants
    nC = nC_i * M;

    // Setup number of elements in the state vector X:
    // number of state variables * number of timesteps * number of traffic participants
    nX_ = nX * (N + 1) * M;

    // Setup number of elements in the control vector U:
    // number of control variables * number of timesteps * number of traffic participants
    nU_ = nU * (N + 1) * M;

    // Setup length of the gradient vector G:
    nG = nU_;

    // resize and initialize limits for control input
    ul.resize(nU * (N + 1), 1);
    uu.resize(nU * (N + 1), 1);
    for (int j = 0; j < N + 1; j++){
        ul(nU * j + d, 0) = d_low;
        uu(nU * j + d, 0) = d_up;
        ul(nU * j + F, 0) = F_low;
        uu(nU * j + F, 0) = F_up;
    }

    // resize time vector
    time.resize(N + 1, 1);

    // resize and initialize lagrangian multiplier vector
    lagrangian_multipliers.resize(nC, 1);
    lagrangian_multipliers = Eigen::MatrixXd::Zero(nC, 1);

}

/** Sets the intial guess of the game */
void DynamicGamePlanner::initial_guess(double* X_, double* U_)
{
    for (int i = 0; i < M; i++){
        for (int j = 0; j < N + 1; j++){
            U_[nU * (N + 1) * i + nU * j + d] = 0.0;
            U_[nU * (N + 1) * i + nU * j + F] = 0.3;
        }
    }
    integrate(X_, U_);
    if (U_old.size() == nU * (N + 1) * M){
        for (int i = 0; i < M; i++){
            for (int j = 0; j < N + 1; j++){
                U_[nU * (N + 1) * i + nU * j + d] = U_old[nU * (N + 1) * i + nU * j + d];
                U_[nU * (N + 1) * i + nU * j + F] = U_old[nU * (N + 1) * i + nU * j + F];
            }
        }
    }
}


/** integrates the input U to get the state X */
void DynamicGamePlanner::integrate(double* X_, const double* U_)
{
    int tu;
    int td;
    int ind;
    double s_ref;
    double t;
    double s_t0[nX];
    double sr_t0[nX];
    double u_t0[nU];
    double ds_t0[nX];

    for (int i = 0; i < M; i++){
        ind = 0;
        td = nx * i;
        t = 0.0;

        // Initial state:
        s_t0[x] = traffic[i].x;
        s_t0[y] = traffic[i].y;
        s_t0[v] = traffic[i].v;
        s_t0[psi] = traffic[i].psi;
        s_t0[s] = 0.0;
        s_t0[l] = 0.0;

        for (int j = 0; j < N + 1; j++){
            tu = nU * (N + 1) * i + nU * j;
            td = nX * (N + 1) * i + nX * j;

            // Reference point on the center lane:
            s_ref = s_t0[s];
            sr_t0[x] = traffic[i].centerlane.spline_x(s_ref);
            sr_t0[y] =traffic[i].centerlane.spline_y(s_ref);
            sr_t0[psi] = traffic[i].centerlane.compute_heading(s_ref);
            sr_t0[v] = traffic[i].v_target; 

            // Input control:
            u_t0[d] = U_[tu + d];
            u_t0[F] = U_[tu + F];
            
            // Dynamic step: 
            dynamic_step(ds_t0, s_t0, sr_t0, u_t0);

            // Integration to compute the new state: 
            s_t0[x] += dt * ds_t0[x];
            s_t0[y] += dt * ds_t0[y];
            s_t0[v] += dt * ds_t0[v];
            s_t0[psi] += dt * ds_t0[psi];
            s_t0[s] += dt * ds_t0[s];
            s_t0[l] += dt * ds_t0[l];

            if (s_t0[v] < 0.0){s_t0[v] = 0.0;}

            // Save the state in the trajectory
            X_[td + x] = s_t0[x];
            X_[td + y] = s_t0[y];
            X_[td + v] = s_t0[v];
            X_[td + psi] = s_t0[psi];
            X_[td + s] = s_t0[s];
            X_[td + l] = s_t0[l];

            t+= dt;
        }
    }
}

/** Dyanamic step */
void DynamicGamePlanner::dynamic_step(double* d_state, const double* state, const double* ref_state, const double* control)
{
    /* Derivatives computation:*/
    d_state[x] = state[v] * cos(state[psi] + cg_ratio * control[d]);
    d_state[y] = state[v] * sin(state[psi] + cg_ratio * control[d]);
    d_state[v] = (-1/tau) * state[v] + (k) * control[F];
    d_state[psi] = state[v] * tan(control[d]) * cos(cg_ratio * control[d])/ length;
    d_state[l] = weight_target_speed * (state[v] - ref_state[v]) * (state[v] - ref_state[v])
            + weight_center_lane * ((ref_state[x] - state[x]) * (ref_state[x] - state[x]) + (ref_state[y] - state[y]) * (ref_state[y] - state[y]))
            + weight_heading * ((std::cos(ref_state[psi]) - std::cos(state[psi]))*(std::cos(ref_state[psi]) - std::cos(state[psi]))
            +        (std::sin(ref_state[psi]) - std::sin(state[psi]))*(std::sin(ref_state[psi]) - std::sin(state[psi])))
            + weight_input * control[F] * control[F];
    d_state[s] = state[v];
}

/** SR1 Hessian matrix update*/
void DynamicGamePlanner::hessian_SR1_update(Eigen::MatrixXd & H_, const Eigen::MatrixXd & s_, const Eigen::MatrixXd & y_, double r_)
{
    if (abs((s_.transpose() * (y_ - H_ * s_))(0,0)) > 
                r_ * (s_.transpose() * s_)(0,0) * ((y_ - H_* s_).transpose() * (y_ - H_ * s_))(0,0))
    {
        H_ = H_ + ((y_ - H_ * s_) * (y_ - H_ * s_).transpose()) / ((y_ - H_ * s_).transpose() * s_)(0,0);
    }
}

/** function to increase rho = rho * gamma at each iteration */
void DynamicGamePlanner::increasing_schedule()
{
    rho = gamma * rho;
}

/** function to save the lagrangian multipliers in the general variable */
void DynamicGamePlanner::save_lagrangian_multipliers(double* lagrangian_multipliers_)
{
    for (int i = 0; i < nC; i++){
        lagrangian_multipliers(i,0) = lagrangian_multipliers_[i];
    }
}

/* computation of lambda (without update)*/
void DynamicGamePlanner::compute_lagrangian_multipliers(double* lagrangian_multipliers_, const double* constraints_)
{
    double l;
    for (int i = 0; i < nC; i++){
        l = lagrangian_multipliers(i,0) + rho * constraints_[i];
        lagrangian_multipliers_[i] = std::max(l, 0.0);
    }
}

/** computation of the inequality constraints (target: constraints < 0) */
void DynamicGamePlanner::compute_constraints(double* constraints, const double* X_, const double* U_)
{
    double constraints_i[nC_i];
    for (int i = 0; i < M; i++){
        compute_constraints_vehicle_i(constraints_i, X_, U_, i);
        for (int j = 0; j < nC_i; j++){
            constraints[nC_i * i + j] = constraints_i[j];
        }
    }
}

/** computation of the inequality constraints C for vehicle i (target: C < 0) */
void DynamicGamePlanner::compute_constraints_vehicle_i(double* constraints_i, const double* X_, const double* U_, int i)
{
    int ind = 0;
    int indCu;
    int indCl;
    int indU;
    int indCto;
    int indClk;
    int indCuk;
    int indf;
    int n1;
    int n2;
    double dist2t[N + 1];
    double rad2[N + 1];
    double latdist2t[N + 1];
    double r_lane_ = r_lane;

    // constraints for the inputs 
    indU = nU * (N + 1) * i;
    indCu = nU * (N + 1);
    indCl = indCu + nU * (N + 1);
    for (int k = 0; k < N + 1; k++){
        constraints_i[nU * k + d] = (U_[indU + nU * k + d] - uu(nU * k + d,0));
        constraints_i[nU * k + F] = (U_[indU + nU * k + F] - uu(nU * k + F,0));
        constraints_i[indCu + nU * k + d] = (ul(nU * k + d,0) - U_[indU + nU * k + d]);
        constraints_i[indCu + nU * k + F] = (ul(nU * k + F,0) - U_[indU + nU * k + F]);
    }

    // collision avoidance constraints  
    for (int k = 0; k < M; k++){
        if (k != i){
            indCto = indCl + (N + 1) * ind;
            compute_squared_distances_vector(dist2t, X_, i, k);
            for (int j = 0; j < N + 1; j++){
                constraints_i[indCto + j] = (r_safe * r_safe - dist2t[j]);
            }
            ind++;
        }
    }
    indCto = indCl + (N + 1) * (M - 1);

    // constraints to remain in the lane
    compute_squared_lateral_distance_vector(latdist2t, X_, i);
    for (int k = 0; k < N + 1; k++){
        constraints_i[indCto + k] = (latdist2t[k] - r_lane_ * r_lane_);
    }
    indf = indCto + (N + 1);

    // constraint to stop at the goal position
    constraints_i[indf] = (X_[nX * (N + 1) * i + nX * N + s] - traffic[i].centerlane.s_max);
}

/** computes a vector of the squared distance between the trajectory of vehicle i and j*/
void DynamicGamePlanner::compute_squared_distances_vector(double* squared_distances, const double* X_, int ego, int j)
{
    double x_ego;
    double y_ego;
    double x_j;
    double y_j;
    double distance;
    for (int k = 0; k < N + 1; k++){
        x_ego = X_[nx * ego + nX * k + x];
        y_ego = X_[nx * ego + nX * k + y];
        x_j = X_[nx * j + nX * k + x];
        y_j = X_[nx * j + nX * k + y];
        distance = (x_ego - x_j) * (x_ego - x_j) + (y_ego - y_j) * (y_ego - y_j);
        squared_distances[k] = distance;
    }
}

/** computes a vector of the squared lateral distance between the i-th trajectory and the allowed center lines at each time step*/
void DynamicGamePlanner::compute_squared_lateral_distance_vector(double* squared_distances_, const double* X_, int i)
{
    double s_;
    double x_;
    double y_;
    double x_c;
    double y_c;
    double x_r;
    double y_r;
    double x_l;
    double y_l;
    double psi_c;
    double psi_l;
    double psi_r;
    double dist_c;
    double dist_l = 1e3;
    double dist_r = 1e3;
    double dist_long_c;
    double dist_long_l;
    double dist_long_r;
    double dist2_c[N + 1];
    double dist2_l[N + 1];
    double dist2_r[N + 1];
    double dist2_rl_min;
    for (int j = 0; j < N + 1; j++){
        s_ = X_[nx * i + nX * j + s];
        x_ = X_[nx * i + nX * j + x];
        y_ = X_[nx * i + nX * j + y];
        dist2_c[j] = 1e3;
        dist2_l[j] = 1e3;
        dist2_r[j] = 1e3;
        if (s_ < traffic[i].centerlane.s_max){
            x_c = traffic[i].centerlane.spline_x(s_);
            y_c = traffic[i].centerlane.spline_y(s_);
            psi_c = traffic[i].centerlane.compute_heading(s_);
            dist_c = ((x_ - x_c) * (x_ - x_c) + (y_ - y_c) * (y_ - y_c));
            dist_long_c = ((x_ - x_c) * std::cos(psi_c) + (y_ - y_c) * std::sin(psi_c)) * ((x_ - x_c) * std::cos(psi_c) + (y_ - y_c) * std::sin(psi_c));
            dist2_c[j] = dist_c - dist_long_c;
        }
        if (traffic[i].leftlane.present == true && s_ < traffic[i].leftlane.s_max && traffic[i].leftlane.s_max > 10.0){
            x_l = traffic[i].leftlane.spline_x(s_);
            y_l = traffic[i].leftlane.spline_y(s_);
            psi_l = traffic[i].leftlane.compute_heading(s_);
            dist_l = ((x_ - x_l) * (x_ - x_l) + (y_ - y_l) * (y_ - y_l));
            dist_long_l = ((x_ - x_l) * std::cos(psi_l) + (y_ - y_l) * std::sin(psi_l)) * ((x_ - x_l) * std::cos(psi_l) + (y_ - y_l) * std::sin(psi_l));
            dist2_l[j] = dist_l - dist_long_l;
        }
        if (traffic[i].rightlane.present == true && s_ < traffic[i].rightlane.s_max && traffic[i].rightlane.s_max > 10.0){
            x_r = traffic[i].rightlane.spline_x(s_);
            y_r = traffic[i].rightlane.spline_y(s_);
            psi_r = traffic[i].rightlane.compute_heading(s_);
            dist_r = ((x_ - x_r) * (x_ - x_r) + (y_ - y_r) * (y_ - y_r));
            dist_long_r = ((x_ - x_r) * std::cos(psi_r) + (y_ - y_r) * std::sin(psi_r)) * ((x_ - x_r) * std::cos(psi_r) + (y_ - y_r) * std::sin(psi_r));
            dist2_r[j] = dist_r - dist_long_r;
        }
        dist2_rl_min = std::min(dist2_l[j], dist2_r[j]);
        squared_distances_[j] = std::min(dist2_rl_min, dist2_c[j]);
    }
}

/** compute the cost for vehicle i */
double DynamicGamePlanner::compute_cost_vehicle_i(const double* X_, const double* U_, int i)
{
    double final_lagrangian = X_[nx * i + nX * N + l];
    double cost = 0.5 * final_lagrangian * qf * final_lagrangian;
    return cost;
}

/** computes of the augmented lagrangian vector  L = <L_1, ..., L_M> L_i = cost_i + lagrangian_multipliers * constraints */
void DynamicGamePlanner::compute_lagrangian(double* lagrangian, const double* X_, const double* U_)
{
    double lagrangian_i;
    double cost_i;
    double constraints_i[nC_i];
    double lagrangian_multipliers_i[nC_i];
    for (int i = 0; i < M; i++){
        cost_i = compute_cost_vehicle_i( X_, U_, i);
        compute_constraints_vehicle_i(constraints_i, X_, U_, i);
        lagrangian_i = compute_lagrangian_vehicle_i( cost_i, constraints_i, i);
        lagrangian[i] = lagrangian_i;
    }
}

/** computation of the augmented lagrangian for vehicle i: lagrangian_i = cost_i + lagrangian_multipliers_i * constraints_i */
double DynamicGamePlanner::compute_lagrangian_vehicle_i(double cost_i, const double* constraints_i, int i)
{
    double lagrangian_i = cost_i;
    double constraints;
    for (int k = 0; k < nC_i; k++){
        constraints = std::max(0.0, constraints_i[k]);
        lagrangian_i += 0.5 * rho * constraints * constraints + lagrangian_multipliers(i * nC_i + k,0) * constraints_i[k];
    }
    return lagrangian_i;
}

/** computation of the gradient of lagrangian_i with respect to U_i for each i with parallelization on cpu*/
void DynamicGamePlanner::compute_gradient(double* gradient, const double* U_)
{
    const int num_threads = std::thread::hardware_concurrency();
    std::mutex mutex;
    std::vector<std::thread> threads(num_threads);

    // Definition of the work for each thread:
    auto computeGradient = [&](int start, int end) {
        double dU[nU_];
        double dX[nX_];
        double X_[nX_];
        double lagrangian[M];
        double lagrangian_i;
        double cost_i;
        double constraints_i[nC_i];
        double lagrangian_multipliers_i[nC_i];
        int index;
        for (int i = 0; i < nU_; i++){
            dU[i] = U_[i];
        }
        integrate(X_, U_);
        compute_lagrangian(lagrangian, X_, U_);
        for (int i = start; i < end; i++) {
            index = i / nu;
            dU[i] = U_[i] + eps;
            integrate(dX, dU);
            compute_constraints_vehicle_i(constraints_i, dX, dU, index);
            cost_i = compute_cost_vehicle_i( dX, dU, index);
            lagrangian_i = compute_lagrangian_vehicle_i( cost_i, constraints_i, index);
            {
                std::lock_guard<std::mutex> lock(mutex);
                gradient[i] = (lagrangian_i - lagrangian[index]) / eps;
            }
            dU[i] = U_[i];
        }
    };

    // Parallelize:
    int work_per_thread = nU_ / num_threads;
    int start_index = 0;
    int end_index = 0;
    for (int i = 0; i < num_threads; ++i) {
        start_index = i * work_per_thread;
        end_index = (i == num_threads - 1) ? nU_ : start_index + work_per_thread;
        threads[i] = std::thread(computeGradient, start_index, end_index);
    }
    for (auto& thread : threads) {
        thread.join();
    }
}

/** it solves the quadratic problem (GT * s + 0.5 * sT * H * s) with solution included in the trust region ||s|| < Delta */
void DynamicGamePlanner::quadratic_problem_solver(Eigen::MatrixXd & s_, const Eigen::MatrixXd & G_, const Eigen::MatrixXd & H_, double Delta)
{
    Eigen::MatrixXd ps(nG,1);
    double tau;
    double tau_c;
    double normG;
    double GTHG;
    GTHG = (G_.transpose() * H_ * G_)(0,0);
    normG = sqrt((G_.transpose() * G_)(0,0));
    ps = - Delta * (G_ /(normG));
    if ( GTHG <= 0.0){
        tau = 1.0;
    }else{
        tau_c = (normG * normG * normG)/(Delta * GTHG);
        tau = std::min(tau_c, 1.0);
    }
    s_ = tau * ps;
}

/** prints if some constraints are violated */
void DynamicGamePlanner::constraints_diagnostic(const double* constraints, bool print = false)
{
    bool flag0 = false;
    bool flag1 = false;
    bool flag2 = false;
    bool flag3 = false;
    for (int i = 0; i < M; i++){
        flag0 = false;
        flag1 = false;
        flag2 = false;
        flag3 = false;
        for (int j = 0; j < nC_i; j++){
            if (constraints[nC_i * i +j] > 0){
                if (j < (2 * nU * (N + 1))) {
                    std::cerr<<"vehicle "<<i<<" violates input constraints: "<<constraints[nC_i * i + j]<<"\n";
                    flag0 = true;
                }
                if (j < (2 * nU * (N + 1) + (N + 1) * (M - 1)) && j > (2 * nU * (N + 1))){
                    std::cerr<<"vehicle "<<i<<" violates collision avoidance constraints: "<<constraints[nC_i * i + j]<<"\n";
                    flag1 = true;
                }
                if (j > (2 * nU * (N + 1) + (N + 1) * (M - 1)) && j < (2 * nU * (N + 1) + (N + 1) * (M - 1) + (N + 1))){
                    std::cerr<<"vehicle "<<i<<" violates lane constraints: "<<constraints[nC_i * i + j]<<"\n";
                    flag2 = true;
                }
                if (j > (2 * nU * (N + 1) + (N + 1) * (M - 1) + (N + 1))){
                    std::cerr<<"vehicle "<<i<<" violates maximum allowed progress constraints: "<<constraints[nC_i * i + j]<<"\n";
                    flag3 = true;
                }
            }
        }
        if (print == true){
            std::cerr<<"vehicle "<<i<<"\n";
            std::cerr<<"input constraint: \n";
            for (int j = 0; j < 2 * nU * (N + 1); j++){
                std::cerr<<constraints[nC_i * i +j]<<"\t";
            }
            std::cerr<<"\ncollision avoidance constraint: \n";
            for (int j = 2 * nU * (N + 1); j < (2 * nU * (N + 1) + (N + 1) * (M - 1)); j++){
                std::cerr<<constraints[nC_i * i +j]<<"\t";
            }
            std::cerr<<"\nlane constraint: \n";
            for (int j = (2 * nU * (N + 1) + (N + 1) * (M - 1)); j < (2 * nU * (N + 1) + (N + 1) * (M - 1) + (N + 1)); j++){
                std::cerr<<constraints[nC_i * i +j]<<"\t";
            }
            std::cerr<<"\n";
        }
    }
}

void DynamicGamePlanner::print_trajectories(const double* X, const double* U)
{
    // Define column width
    const int col_width = 20;  // Adjust this value as needed

    for (int i = 0; i < M; i++){
        std::cerr << "Vehicle: (" << traffic[i].x << ", " << traffic[i].y << ") \t" << traffic[i].v << "\n";

        // Print table header with aligned columns
        std::cerr << std::left  // Align text to the left
                  << std::setw(col_width) << "X"
                  << std::setw(col_width) << "Y"
                  << std::setw(col_width) << "V"
                  << std::setw(col_width) << "PSI"
                  << std::setw(col_width) << "S"
                  << std::setw(col_width) << "L"
                  << std::setw(col_width) << "F"
                  << std::setw(col_width) << "d"
                  << "\n";

        // Print separator line
        std::cerr << std::string(col_width * 8, '-') << "\n";

        // Print trajectory values
        for (int j = 0; j < N + 1; j++){
            std::cerr << std::fixed << std::left
                      << std::setw(col_width) << X[nX * (N + 1) * i + nX * j + x]
                      << std::setw(col_width) << X[nX * (N + 1) * i + nX * j + y]
                      << std::setw(col_width) << X[nX * (N + 1) * i + nX * j + v]
                      << std::setw(col_width) << X[nX * (N + 1) * i + nX * j + psi]
                      << std::setw(col_width) << X[nX * (N + 1) * i + nX * j + s]
                      << std::setw(col_width) << X[nX * (N + 1) * i + nX * j + l]
                      << std::setw(col_width) << U[nU * (N + 1) * i + nU * j + F]
                      << std::setw(col_width) << U[nU * (N + 1) * i + nU * j + d]
                      << "\n";
        }
        std::cerr << "\n";
    }
}

/** sets the prediction to the traffic structure*/
adore::dynamic_game::TrafficParticipants DynamicGamePlanner::set_prediction(const double* X_, const double* U_)
{
    adore::dynamic_game::TrafficParticipants traffic_ = traffic;
    for (int i = 0; i < M; i++){
        adore::dynamic_game::Trajectory trajectory;
        adore::dynamic_game::Control control;
        double time = 0.0;

        for (int j = 0; j < N + 1; j++){
            adore::dynamic_game::TrajectoryPoint point;
            adore::dynamic_game::Input input;
            input.a = (-1/tau) * X_[ nx * i + nX * j + v] + (k) * U_[nu * i + nU * j + F];
            input.delta = U_[nu * i + nU * j + d];
            point.x = X_[ nx * i + nX * j + x];
            point.y = X_[ nx * i + nX * j + y];
            point.psi = X_[ nx * i + nX * j + psi];
            point.v = X_[ nx * i + nX * j + v];
            point.omega = point.v * tan(input.delta) * cos(cg_ratio * input.delta)/ length;
            point.beta = 0.5 * input.delta;
            point.t_start = time;
            point.t_end = time + dt;
            trajectory.push_back(point);
            control.push_back(input);
            time += dt;
        }
        traffic_[i].predicted_trajectory = trajectory;
        traffic_[i].predicted_control = control;
    }
    return traffic_;
}

/** computes the heading on the spline x(s) and y(s) at parameter s*/
double DynamicGamePlanner::compute_heading( const tk::spline & spline_x, const tk::spline & spline_y, double s)
{
    double psi;
    double dx = spline_x.deriv(1, s);
    double dy = spline_y.deriv(1, s);
    psi = atan2(dy, dx);
    if(psi < 0.0) {psi += 2*M_PI;}
    return psi;
}

/** computes the norm of the gradient */
double DynamicGamePlanner::gradient_norm(const double* gradient)
{
    double norm = 0.0;
    for (int j = 0; j < nG; j++){
        norm += gradient[j] * gradient[j];
    }
    return norm;
}

/** Trust-Region solver of the dynamic game*/
void DynamicGamePlanner::trust_region_solver(double* U_)
{
    bool convergence = false;

    // Parameters:
    double eta = 1e-4;
    double r_ = 1e-8;
    double threshold_gradient_norm = M * 1e-1;
    int iter = 1;
    int iter_lim = 50;

    // Variables definition:
    double gradient[nG];
    double dU[nU_]; 
    double dU_[nU_]; 
    double dX[nX_]; 
    double dX_[nX_];
    double d_gradient[nG];
    double d_lagrangian[M];
    double lagrangian[M];
    double constraints[nC];
    double lagrangian_multipliers[nC];

    double actual_reduction[M];
    double predicted_reduction[M];
    double delta[M];
    std::vector<Eigen::MatrixXd> H_(M);
    std::vector<Eigen::MatrixXd> g_(M);
    std::vector<Eigen::MatrixXd> p_(M);
    std::vector<Eigen::MatrixXd> s_(M);
    std::vector<Eigen::MatrixXd> y_(M);

    // Variables initialization:
    integrate(dX, U_);
    for (int i = 0; i < nU_; i++){
        dU[i] = U_[i];
        dU_[i] = U_[i];
    }
    for (int i = 0; i < nX_; i++){
        dX_[i] = dX[i];
    }
    for (int i = 0; i < M; i++){
        H_[i].resize(nu, nu);
        g_[i].resize(nu, 1);
        p_[i].resize(nu, 1);
        s_[i].resize(nu, 1);
        y_[i].resize(nu, 1);
        delta[i] = 1.0;
        H_[i] = Eigen::MatrixXd::Identity(nu, nu);
    }
    compute_gradient(gradient, dU_);

    // Check for convergence:
    if (gradient_norm(gradient) < threshold_gradient_norm){
        std::cerr<<"gradient norm is already below the threshold: "<<gradient_norm(gradient)<<"\n";
        convergence = true;
    }

    // Iteration loop:
    while (convergence == false && iter < iter_lim ){

        // Compute the grandient and the lagrangian
        integrate(dX_, dU_);
        compute_gradient(gradient, dU_);
        compute_lagrangian(lagrangian, dX_, dU_);

        // Solves the quadratic subproblem and compute the possible step dU:
        for (int i = 0; i < M; i++){
            for (int j = 0; j < N + 1; j++){
                g_[i](j * nU + d,0) = gradient[nu * i + j * nU + d];
                g_[i](j * nU + F,0) = gradient[nu * i + j * nU + F];
            }
            quadratic_problem_solver(s_[i], g_[i], H_[i], delta[i]);
            for (int j = 0; j < N + 1; j++){
                dU[nu * i + j * nU + d] = dU_[nu * i + j * nU + d] + s_[i](j * nU + d,0);
                dU[nu * i + j * nU + F] = dU_[nu * i + j * nU + F] + s_[i](j * nU + F,0);
            }
        }

        // Compute the new grandient and the new lagrangian with the possible step dU:
        integrate(dX, dU);
        compute_gradient(d_gradient, dU);
        compute_lagrangian(d_lagrangian, dX, dU);

        // Check for each agent if to accept the step or not:
        for (int i = 0; i < M; i++){
            
            // Compute the actual reduction and of the predicted reduction:
            actual_reduction[i] = lagrangian[i] - d_lagrangian[i];
            predicted_reduction[i] = - (g_[i].transpose() *  s_[i] + 0.5 * s_[i].transpose() * H_[i] * s_[i])(0,0);

            // In case of very low or negative actual reduction, reject the step:
            if ( actual_reduction[i] / predicted_reduction[i] < eta){ 
                for (int j = 0; j < N + 1; j++){
                    dU[nu * i + j * nU + d] = dU_[nu * i + j * nU + d];
                    dU[nu * i + j * nU + F] = dU_[nu * i + j * nU + F];
                }
            }

            // In case of great reduction, and solution close to the trust region, increase the trust region:
            if ( actual_reduction[i] / predicted_reduction[i] > 0.75){ 
                if (std::sqrt((s_[i].transpose() * s_[i])(0,0)) > 0.8 * delta[i]){
                    delta[i] = 2.0 * delta[i];
                }
            }

            // In case of low actual reduction, decrease the step:
            if ( actual_reduction[i] / predicted_reduction[i] < 0.1){
                delta[i] = 0.5 * delta[i];
            }

            // Compute the difference of the gradients, then the Hessian matrix update:
            for (int j = 0; j < N + 1; j++){
                y_[i](j * nU + d,0) = d_gradient[nu * i + j * nU + d] - gradient[nu * i + j * nU + d];
                y_[i](j * nU + F,0) = d_gradient[nu * i + j * nU + F] - gradient[nu * i + j * nU + F];
            }
            hessian_SR1_update(H_[i], s_[i], y_[i], r_);

            // Save the solution for the next iteration:
            for (int j = 0; j < N + 1; j++){
                dU_[nu * i + j * nU + d] = dU[nu * i + j * nU + d];
                dU_[nu * i + j * nU + F] = dU[nu * i + j * nU + F];
            }
        }
        //std::cerr<<"gradient norm: "<<gradient_norm(gradient)<<"\n";
        // Check for convergence:
        if (gradient_norm(gradient) < threshold_gradient_norm){
            convergence = true;
        }

        // Compute the new state: 
        integrate(dX_, dU_);

        // Compute the constraints with the new solution:
        compute_constraints(constraints, dX_, dU_);

        // Compute and save in the general variable the lagrangian multipliers with the new solution:
        compute_lagrangian_multipliers(lagrangian_multipliers, constraints);
        save_lagrangian_multipliers(lagrangian_multipliers);

        // Increase the weight of the constraints in the lagrangian multipliers:
        increasing_schedule();
        iter++;
    }

    std::cerr<<"Solution reached in "<<iter<<" iterations \n";

    //Correct the final solution:
    correctionU(dU_);

    // Save the solution:
    for(int k = 0; k < nU_; k++){
        U_[k] = dU_[k];
    }
}

void DynamicGamePlanner::correctionU(double* U_)
{
    for (int i = 0; i < M; i++){
        for (int j = 0; j < N + 1; j++){
            if (j == N){
                U_[nU * (N + 1) * i + nU * j + d] = U_[nU * (N + 1) * i + nU * (j - 1) + d];
                U_[nU * (N + 1) * i + nU * j + F] = U_[nU * (N + 1) * i + nU * (j - 1) + F];
            }
            if (U_[nU * (N + 1) * i + nU * j + d] > d_up){
                U_[nU * (N + 1) * i + nU * j + d] = d_up;
            }
            if (U_[nU * (N + 1) * i + nU * j + d] < d_low){
                U_[nU * (N + 1) * i + nU * j + d] = d_low;
            }
        }
    }
}

void DynamicGamePlanner::save_solution(const double* U_, const int M_)
{
    M_old = M;
    U_old.clear();
    U_old.resize(nU * (N + 1) * M);
    for (int i = 0; i < M; i++){
        for (int j = 0; j < N + 1; j++){
            U_old[nu * i + nU * j + d] = U_[nu * i + nU * j + d];
            U_old[nu * i + nU * j + F] = U_[nu * i + nU * j + F];
        }
    }
}

void DynamicGamePlanner::set_solution(dynamics::TrafficParticipantSet& traffic_participant_set)
{
    for (auto& [id, participant] : traffic_participant_set.participants)
    {
        if (participant.trajectory.has_value() == false){
            participant.trajectory = dynamics::Trajectory();
        }else{
            participant.trajectory.value().states.clear();
        }
        for (auto& vehicle : traffic) 
        {
            if (vehicle.id == id)
            {
                for (int j = 0; j < N + 1; j++)
                {
                    dynamics::VehicleStateDynamic state;
                    state.x = vehicle.predicted_trajectory[j].x;
                    state.y = vehicle.predicted_trajectory[j].y;
                    state.z = 0.0;
                    state.vx = vehicle.predicted_trajectory[j].v * cos(vehicle.predicted_trajectory[j].beta);
                    state.vy = vehicle.predicted_trajectory[j].v * sin(vehicle.predicted_trajectory[j].beta);
                    state.yaw_angle = vehicle.predicted_trajectory[j].psi;
                    state.yaw_rate = vehicle.predicted_trajectory[j].omega;
                    state.steering_angle = vehicle.predicted_control[j].delta;
                    state.steering_rate = 0.0;
                    state.ax = vehicle.predicted_control[j].a * cos(vehicle.predicted_trajectory[j].beta);
                    state.ay = vehicle.predicted_control[j].a * sin(vehicle.predicted_trajectory[j].beta);
                    state.time = vehicle.predicted_trajectory[j].t_start;
                    participant.trajectory.value().states.push_back(state);
                }
                break;
            }
        }
    }
}


} // namespace planner
} // namespace adore
