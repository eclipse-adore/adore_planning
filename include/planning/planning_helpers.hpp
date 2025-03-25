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
 *    Marko Mizdrak
 ********************************************************************************/
#pragma once

#include <cmath>

#include <deque>
#include <iostream>
#include <limits>
#include <type_traits>
#include <vector>

#include "adore_math/spline.h"

#include "dynamics/integration.hpp"
#include "dynamics/physical_vehicle_model.hpp"
#include "dynamics/vehicle_state.hpp"
#include <eigen3/Eigen/Dense>

namespace adore
{
namespace planner
{
// Checks if the point is to the right of the first segment in the line
template<typename Point, typename Line>
bool
is_point_to_right_of_line( const Point& point, const Line& line )
{
  if( line.size() < 2 )
  {
    throw std::invalid_argument( "Line must have at least two points to define a direction." );
  }

  // Define the start and end points of the first segment
  const auto& p1 = line[0];
  const auto& p2 = line[1];

  // Calculate the direction vector of the first line segment
  double line_dx = p2.x - p1.x;
  double line_dy = p2.y - p1.y;

  // Calculate the vector from p1 to the point in question
  double point_dx = point.x - p1.x;
  double point_dy = point.y - p1.y;

  // Calculate the cross product of the line direction and point vector
  double cross_product = line_dx * point_dy - line_dy * point_dx;

  // If cross_product is negative, the point is to the right of the first line segment
  return cross_product < 0;
}

template<typename Line, typename Pose>
Line
filter_points_in_front( const Line& line, const Pose& reference_pose )
{
  Line filtered_points;

  // Calculate unit vector for the reference orientation
  double ref_dx = std::cos( reference_pose.yaw_angle );
  double ref_dy = std::sin( reference_pose.yaw_angle );

  for( const auto& point : line )
  {
    // Vector from reference position to the current point
    double dx = point.x - reference_pose.x;
    double dy = point.y - reference_pose.y;

    // Dot product to determine if the point is in front of the reference pose
    double dot_product = dx * ref_dx + dy * ref_dy;
    if( dot_product > 0 )
    { // If dot product is positive, the point is in front
      filtered_points.push_back( point );
    }
  }

  return filtered_points;
}

template<typename Line>
Line
shift_points_right( const Line& points, double shift_distance )
{
  // Check if we have enough points to calculate headings
  if( points.size() < 2 )
    return points;

  Line shifted_points;

  for( size_t i = 0; i < points.size(); ++i )
  {
    double dx, dy;

    if( i == points.size() - 1 )
    {
      // For the last point, use the direction to the previous point
      dx = points[i].x - points[i - 1].x;
      dy = points[i].y - points[i - 1].y;
    }
    else
    {
      // For others, direction to next point
      dx = points[i + 1].x - points[i].x;
      dy = points[i + 1].y - points[i].y;
    }

    // Normalize direction vector
    double length  = std::sqrt( dx * dx + dy * dy );
    dx            /= length;
    dy            /= length;

    // Calculate perpendicular direction to the right
    double shift_dx = dy * shift_distance;
    double shift_dy = -dx * shift_distance;

    // Apply shift to the current point
    auto shifted_point  = points[i];
    shifted_point.x    += shift_dx;
    shifted_point.y    += shift_dy;

    // Add shifted point to the result deque
    shifted_points.emplace_back( shifted_point );
  }

  return shifted_points;
}

// Helper for waypoints_to_trajectory - Calculate the distance error and direction error
static std::pair<double, double>
calculate_errors( const dynamics::VehicleStateDynamic& state, double target_x, double target_y, double target_yaw )
{
  double dx            = target_x - state.x;
  double dy            = target_y - state.y;
  double lateral_error = dy * std::cos( state.yaw_angle ) - dx * std::sin( state.yaw_angle );

  double heading_error = target_yaw - state.yaw_angle;
  heading_error        = adore::math::normalize_angle( heading_error );
  return { lateral_error, heading_error };
}

static double
compute_idm_velocity( double obstacle_distance, double goal_distance, double obstacle_speed,
                      const dynamics::VehicleStateDynamic& current_state )
{
  double max_speed            = 2.0;
  double desired_acceleration = 2.0;
  double desired_deceleration = 2.0;
  double min_distance         = 8.0;
  double time_headway         = 3.0;

  double effective_distance = std::min( obstacle_distance, goal_distance );
  if( goal_distance < obstacle_distance )
    min_distance = 0.0;

  double s_star = min_distance + current_state.vx * time_headway
                + current_state.vx * ( current_state.vx - obstacle_speed )
                    / ( 2 * std::sqrt( desired_acceleration * desired_deceleration ) );

  return current_state.vx
       + desired_acceleration * ( 1 - std::pow( current_state.vx / max_speed, 4 ) - std::pow( s_star / effective_distance, 2 ) );
}

// compute the distance to the nearest object, if within a certain radius from the center of the waypoint lane, considering obstacle fixed
static double
get_distance_to_nearest_obstacle( const tk::spline& waypoint_spline_x, const tk::spline& waypoint_spline_y, const double waypoints_length,
                                  const dynamics::TrafficParticipantSet& traffic_participants )
{
  double ds                     = 0.1; // check step size
  int    number_of_steps        = (int) waypoints_length / ds;
  double min_distance_to_object = std::numeric_limits<double>::max();
  double treshold_within_lane   = 1.0;

  for( const auto& [id, participant] : traffic_participants.participants )
  {
    math::Point2d object_position;
    object_position.x = participant.state.x;
    object_position.y = participant.state.y;
    double s          = 0.0;

    for( int i = 0; i < number_of_steps; i++ )
    {
      math::Point2d waypoint_position;
      waypoint_position.x = waypoint_spline_x( s );
      waypoint_position.y = waypoint_spline_y( s );

      double object_distance = adore::math::distance_2d( object_position, waypoint_position );

      if( object_distance < treshold_within_lane && s < min_distance_to_object )
      {
        min_distance_to_object = s;
        break;
      }

      s = ( i + 1 ) * ds;
    }
  }
  return min_distance_to_object;
}

template<typename Line>
dynamics::Trajectory
waypoints_to_trajectory( const dynamics::VehicleStateDynamic& start_state, const Line& waypoints, double dt, double target_speed,
                         const dynamics::VehicleCommandLimits& limits, const dynamics::TrafficParticipantSet& traffic_participants,
                         const dynamics::PhysicalVehicleModel& model, double k_speed = 0.5, double k_lateral = 1.0, double k_heading = 2.0,
                         double cg_ratio = 0.5 )
{
  dynamics::Trajectory trajectory;
  trajectory.states.push_back( start_state );

  // Initialize splines for waypoints
  tk::spline          spline_x, spline_y;
  std::vector<double> s_vec, x_vec, y_vec;
  double              cumulative_dist = 0.0;

  for( size_t i = 0; i < waypoints.size(); ++i )
  {
    if( i > 0 )
    {
      double dist = adore::math::distance_2d( waypoints[i], waypoints[i - 1] );
      if( dist < 0.01 || std::isnan( dist ) )
        continue;

      cumulative_dist += dist;
    }
    s_vec.push_back( cumulative_dist );
    x_vec.push_back( waypoints[i].x );
    y_vec.push_back( waypoints[i].y );
  }

  spline_x.set_points( s_vec, x_vec );
  spline_y.set_points( s_vec, y_vec );

  dynamics::VehicleStateDynamic current_state = start_state;
  double                        s             = 0.0;

  for( double time = 0; time <= cumulative_dist / target_speed; time += dt )
  {
    // calculate the distance to closest object
    double closest_obstacle_distance = get_distance_to_nearest_obstacle( spline_x, spline_y, s_vec.back(), traffic_participants );

    // Calculate acceleration based on speed error
    double speed_error  = target_speed - current_state.vx;
    double acceleration = -k_speed
                        * ( current_state.vx - compute_idm_velocity( closest_obstacle_distance, s_vec.back(), 0.0, current_state ) );

    dynamics::VehicleCommand control;
    control.acceleration = acceleration;

    // Update distance `s` based on current speed and calculated acceleration
    s += current_state.vx * dt + 0.5 * acceleration * dt * dt;

    // Get target position and heading from the splines using `s`
    double target_x   = spline_x( s );
    double target_y   = spline_y( s );
    double target_yaw = std::atan2( spline_y.deriv( 1, s ), spline_x.deriv( 1, s ) );

    // Calculate errors
    auto [lateral_error, heading_error] = calculate_errors( current_state, target_x, target_y, target_yaw );

    // Set steering angle based on direction and lateral error
    control.steering_angle = heading_error * k_heading + lateral_error * k_lateral * current_state.vx;
    control.clamp_within_limits( limits );

    // Update vehicle state using Euler integration
    current_state                = dynamics::integrate_euler( current_state, control, dt, model.motion_model );
    current_state.ax             = control.acceleration;
    current_state.steering_angle = control.steering_angle;

    // Add the updated state to the trajectory
    trajectory.states.push_back( current_state );
  }

  return trajectory;
}

} // namespace planner

} // namespace adore
