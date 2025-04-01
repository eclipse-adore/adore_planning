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
#include "adore_math/spline.h"  // Include the third-party spline library

namespace adore
{
namespace planner
{
struct TrajectoryPoint {
    double x;       /** x coordinate */
    double y;       /** y coordinate */
    double psi;     /** heading */
    double v;       /** speed */
    double omega;   /** yaw rate */
    double beta;    /** slip angle */
    double k;       /** curvature */
    double s;       /** progression */   
    double t_start; /** starting time of the point*/
    double t_end;   /** ending time of the point*/
};
typedef std::vector<TrajectoryPoint> Trajectory;

struct Input {
    double a;       /** acceleration */
    double delta;   /** steering angle */     
};
typedef std::vector<Input> Control;

/** Lane structure to store lane properties */
struct Lane {
    bool present;                   /** Indicates if the lane is present */
    double s_max;                   /** Maximum longitudinal position in the lane */
    tk::spline spline_x;            /** Spline for x-coordinates */
    tk::spline spline_y;            /** Spline for y-coordinates */

    Lane() : present(false), s_max(0.0) {}  // Default constructor

    void initialize_spline(const std::vector<double>& x, 
                          const std::vector<double>& y, 
                          const std::vector<double>& s);
    double compute_heading(double s);
    double compute_curvature(double s);
};

/** Vehicle state representation */
struct VehicleState {
    double x;                               /** X position */
    double y;                               /**  Y position */
    double psi;                             /** heading of the vehicle */
    double beta;                            /** slip angle*/
    double v;                               /** speed of the vehicle */
    double a;                               /** acceleration */
    double L;                               /** length of the i-th vehicle */
    double W;                               /** width of the i-th vehicle */
    double v_target;                        /** target speed of the i-th vehicle */

    Lane centerlane;                        /** Center lane */
    Lane leftlane;                          /** Left lane */
    Lane rightlane;                         /** Right lane */

    Trajectory predicted_trajectory;        /** predicted trajectory*/
    Control predicted_control;              /** predicted control*/

    VehicleState(double x_, double y_, double v_, double psi_, double beta_, double a_, double v_target_)
        : x(x_), y(y_), v(v_), psi(psi_), beta(beta_), a(a_), v_target(v_target_) {}
};

using TrafficParticipants = std::vector<VehicleState>;  // Alias for a list of vehicles

} // namespace planner
} // namespace adore