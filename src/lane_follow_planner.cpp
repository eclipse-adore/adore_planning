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
 *    Marko Mizdrak
 ********************************************************************************/
#include "planning/lane_follow_planner.hpp"

#include "adore_math/curvature.hpp"
#include "adore_math/point.h"
#include "adore_math/spline.h"

namespace adore
{
namespace planner
{
LaneFollowPlanner::LaneFollowPlanner() {}

void
LaneFollowPlanner::set_parameters( const std::map<std::string, double>& params )
{
  for( const auto& [name, value] : params )
  {
    if( name == "wheelbase" )
      wheelbase = value;
    if( name == "max_speed" )
      max_speed = value;
    if( name == "desired_acceleration" )
      desired_acceleration = value;
    if( name == "desired_deceleration" )
      desired_deceleration = value;
    if( name == "max_lateral_acceleration" )
      max_lateral_acceleration = value;
    if( name == "spline_length_time" )
      spline_length_time = value;
  }
}

dynamics::Trajectory
LaneFollowPlanner::plan_trajectory( const dynamics::VehicleStateDynamic& current_state, const std::deque<map::MapPoint>& route_points,
                                    const map::Map& local_map, const dynamics::VehicleCommandLimits& limits )
{
  desired_acceleration            = std::min( desired_acceleration, limits.max_acceleration );
  desired_deceleration            = -std::min( -desired_deceleration, limits.min_acceleration );
  dynamics::Trajectory trajectory = generate_trajectory_from_route( current_state, local_map, route_points, current_state.vx );
  previous_state                  = current_state;
  return trajectory;
}

dynamics::Trajectory
LaneFollowPlanner::generate_trajectory_from_route( const dynamics::VehicleStateDynamic& current_state, const map::Map& local_map,
                                                   const std::deque<map::MapPoint>& route_points, double initial_speed )
{
  // Check if route_points is empty
  if( route_points.empty() )
  {
    std::cerr << "Error: route_points is empty." << std::endl;
    return dynamics::Trajectory();
  }

  std::deque<map::MapPoint> filtered_points = filter_close_points( route_points );

  // Check if filtering results in too few points
  if( filtered_points.size() < 2 )
  {
    std::cerr << "Error: Insufficient points after filtering. Size: " << filtered_points.size() << std::endl;
    return dynamics::Trajectory();
  }

  // Apply the spline into the start
  spline_into_start( current_state, filtered_points );

  if( filtered_points.size() < 2 )
  {
    std::cerr << "Error: Insufficient points after spline_into_start. Size: " << filtered_points.size() << std::endl;
    return dynamics::Trajectory();
  }

  // Compute cumulative distances
  std::vector<double> distances = compute_cumulative_distances( filtered_points );

  // Check if cumulative distances are valid
  if( distances.empty() || distances.size() < 2 )
  {
    std::cerr << "Error: Invalid distances computed. Size: " << distances.size() << std::endl;
    return dynamics::Trajectory();
  }

  // Compute curvatures
  std::vector<double> curvatures = compute_curvatures( filtered_points );

  // Check for NaNs or Infs in curvatures
  for( double curvature : curvatures )
  {
    if( !std::isfinite( curvature ) )
    {
      std::cerr << "Error: Non-finite value in curvatures." << std::endl;
      return dynamics::Trajectory();
    }
  }

  // Generate speed profile
  std::vector<double> speeds = generate_speed_profile( distances, filtered_points, curvatures, local_map, initial_speed );

  // Check if speeds are valid
  if( speeds.empty() || speeds.size() != distances.size() )
  {
    std::cerr << "Error: Invalid speeds computed. Speeds size: " << speeds.size() << ", Distances size: " << distances.size() << std::endl;
    return dynamics::Trajectory();
  }

  // Check for NaNs or Infs in speeds
  for( double speed : speeds )
  {
    if( !std::isfinite( speed ) )
    {
      std::cerr << "Error: Non-finite value in speeds." << std::endl;
      return dynamics::Trajectory();
    }
  }

  // Compute cumulative times
  std::vector<double> times = compute_cumulative_times( distances, speeds );

  // Check if times are valid
  if( times.empty() || times.size() != distances.size() )
  {
    std::cerr << "Error: Invalid times computed. Times size: " << times.size() << ", Distances size: " << distances.size() << std::endl;
    return dynamics::Trajectory();
  }

  // Resample trajectory
  dynamics::Trajectory trajectory = resample_trajectory( current_state, times, filtered_points, speeds, curvatures );

  // Check if trajectory is populated
  if( trajectory.states.empty() )
  {
    std::cerr << "Error: Empty trajectory generated." << std::endl;
    return trajectory;
  }

  trajectory.adjust_start_time( current_state.time );

  return trajectory;
}

std::deque<map::MapPoint>
LaneFollowPlanner::filter_close_points( const std::deque<map::MapPoint>& points )
{
  std::deque<map::MapPoint> filtered_points;
  if( points.empty() )
    return filtered_points;

  filtered_points.push_back( points.front() );
  map::MapPoint last_point = points.front();

  for( const auto& point : points )
  {
    double distance = adore::math::distance_2d( last_point, point );
    if( distance >= min_point_distance )
    {
      filtered_points.push_back( point );
      last_point = point;
    }
  }

  return filtered_points;
}

std::vector<double>
LaneFollowPlanner::compute_cumulative_distances( const std::deque<map::MapPoint>& points )
{
  size_t              n = points.size();
  std::vector<double> distances( n, 0.0 );
  for( size_t i = 1; i < n; ++i )
  {
    double ds    = adore::math::distance_2d( points[i - 1], points[i] );
    distances[i] = distances[i - 1] + ds;
  }
  return distances;
}

std::vector<double>
LaneFollowPlanner::compute_curvatures( const std::deque<map::MapPoint>& points )
{
  size_t              n = points.size();
  std::vector<double> curvatures( n, 0.0 );

  // Compute curvature using a larger window size for robustness against noise
  for( size_t i = 2; i < n - 2; ++i ) // Adjust the window size here
  {
    // Average positions in a larger window to reduce noise sensitivity
    double x1 = ( points[i - 2].x + points[i - 1].x ) / 2.0;
    double y1 = ( points[i - 2].y + points[i - 1].y ) / 2.0;
    double x2 = points[i].x;
    double y2 = points[i].y;
    double x3 = ( points[i + 1].x + points[i + 2].x ) / 2.0;
    double y3 = ( points[i + 1].y + points[i + 2].y ) / 2.0;

    double kappa  = adore::math::compute_curvature( x1, y1, x2, y2, x3, y3 );
    curvatures[i] = kappa;
  }

  // Handle boundary points by copying neighboring values
  if( n > 4 )
  {
    curvatures[0]     = curvatures[2];
    curvatures[1]     = curvatures[2];
    curvatures[n - 1] = curvatures[n - 3];
    curvatures[n - 2] = curvatures[n - 3];
  }
  else
  {
    curvatures[0] = 0.0;
    if( n > 1 )
      curvatures[1] = 0.0;
  }

  // Apply a simple moving average filter to smooth the computed curvatures
  std::vector<double> smoothed_curvatures( n, 0.0 );
  const int           smoothing_window = 10; // Adjust window size for desired smoothing level
  for( size_t i = 0; i < n; ++i )
  {
    double sum   = 0.0;
    int    count = 0;
    for( int j = -smoothing_window; j <= smoothing_window; ++j )
    {
      int index = i + j;
      if( index >= 0 && static_cast<size_t>( index ) < n )
      {
        sum += curvatures[index];
        ++count;
      }
    }
    smoothed_curvatures[i] = sum / count;
  }

  return smoothed_curvatures;
}

std::vector<double>
LaneFollowPlanner::generate_speed_profile( const std::vector<double>& distances, const std::deque<map::MapPoint>& route_points,
                                           const std::vector<double>& curvatures, const map::Map& local_map, double initial_speed )
{
  size_t              n = distances.size();
  std::vector<double> speeds( n, initial_speed );


  // Forward pass to set speed limits based on constraints
  speeds[0] = initial_speed;
  for( size_t i = 1; i < n; ++i )
  {
    double ds                = distances[i] - distances[i - 1];
    double max_speed_allowed = std::sqrt( speeds[i - 1] * speeds[i - 1] + 2 * desired_acceleration * ds );

    // Limit speed based on lateral acceleration
    double curvature = std::abs( curvatures[i] );
    if( curvature > 0.0 )
    {
      double speed_lateral = std::sqrt( max_lateral_acceleration / curvature );
      max_speed_allowed    = std::min( max_speed_allowed, speed_lateral );
    }

    // Check speed limit of the lane at the current route point
    size_t lane_id = route_points[i].parent_id;
    if( local_map.lanes.find( lane_id ) != local_map.lanes.end() )
    {
      double lane_speed_limit = local_map.get_lane_speed_limit( lane_id );
      max_speed_allowed       = std::min( max_speed_allowed, lane_speed_limit );
    }
    if( route_points[i].max_speed )
      max_speed_allowed = std::min( max_speed_allowed, route_points[i].max_speed.value() );

    speeds[i] = std::min( max_speed, max_speed_allowed );
    speeds[i] = std::max( speeds[i], std::sqrt( speeds[i - 1] * speeds[i - 1] - 2 * desired_deceleration * ds ) );
  }

  // Backward pass for deceleration
  speeds[n - 1] = 0; // Final speed is zero at the end point
  for( size_t i = n - 2; i < n; --i )
  {
    double ds                = distances[i + 1] - distances[i];
    double max_speed_allowed = std::sqrt( speeds[i + 1] * speeds[i + 1] + 2 * desired_deceleration * ds );
    speeds[i]                = std::min( speeds[i], max_speed_allowed );
  }

  return speeds;
}

std::vector<double>
LaneFollowPlanner::compute_cumulative_times( const std::vector<double>& distances, const std::vector<double>& speeds )
{
  size_t              n = distances.size();
  std::vector<double> times( n, 0.0 );
  for( size_t i = 1; i < n; ++i )
  {
    double ds        = distances[i] - distances[i - 1];
    double avg_speed = ( speeds[i - 1] + speeds[i] ) / 2.0;
    double dt        = ds / avg_speed;
    times[i]         = times[i - 1] + dt;
  }
  return times;
}

dynamics::Trajectory
LaneFollowPlanner::resample_trajectory( const dynamics::VehicleStateDynamic& current_state, const std::vector<double>& times,
                                        const std::deque<map::MapPoint>& points, const std::vector<double>& speeds,
                                        const std::vector<double>& curvatures )
{
  dynamics::Trajectory trajectory;
  double               total_time = times.back();
  size_t               n          = times.size();

  double prev_v              = speeds[0];
  double prev_steering_angle = current_state.steering_angle;

  for( double t = 0.0; t <= total_time; t += dt )
  {
    auto   it  = std::lower_bound( times.begin(), times.end(), t );
    size_t idx = std::distance( times.begin(), it );

    if( idx == 0 )
      idx = 1;
    if( idx >= n )
      break;

    double t0     = times[idx - 1];
    double t1     = times[idx];
    double factor = ( t - t0 ) / ( t1 - t0 );

    const auto& p0  = points[idx - 1];
    const auto& p1  = points[idx];
    double      x   = p0.x + factor * ( p1.x - p0.x );
    double      y   = p0.y + factor * ( p1.y - p0.y );
    double      v   = speeds[idx - 1] + factor * ( speeds[idx] - speeds[idx - 1] );
    double      yaw = std::atan2( p1.y - p0.y, p1.x - p0.x );

    // Axial acceleration (ax)
    double ax = ( v - prev_v ) / dt;
    prev_v    = v;

    // Yaw rate (rate of change of yaw angle)

    double curvature = curvatures[idx - 1] + factor * ( curvatures[idx] - curvatures[idx - 1] );
    double yaw_rate  = curvature * v;

    // Compute steering angle using single-track model
    double steering_angle = std::atan( wheelbase * curvature );

    // Smooth noisy steering rate changes
    double steering_rate = ( steering_angle - prev_steering_angle ) / dt;

    // Update previous values
    prev_steering_angle = steering_angle;

    dynamics::VehicleStateDynamic state;
    state.x              = x;
    state.y              = y;
    state.vx             = v;
    state.yaw_angle      = yaw;
    state.time           = t;
    state.ax             = ax;
    state.yaw_rate       = yaw_rate;
    state.steering_rate  = steering_rate;
    state.steering_angle = steering_angle;

    trajectory.states.push_back( state );
  }
  if( trajectory.states.empty() )
    return trajectory;

  if( trajectory.states.back().vx < 0.01 )
  {
    for( size_t i = 0; i < 10; i++ )
    {
      trajectory.states.push_back( trajectory.states.back() );
    }
  }
  return trajectory;
}

void
LaneFollowPlanner::spline_into_start( const dynamics::VehicleStateDynamic& current_state, std::deque<map::MapPoint>& route_points )
{
  // If we don't have enough points, just return
  if( route_points.size() < 2 )
  {
    return;
  }

  // Remove very close overlapping points from the front
  double ds = 0.0;
  while( ds < 0.01 && route_points.size() > 2 )
  {
    route_points.pop_front();
    ds = adore::math::distance_2d( route_points[0], route_points[1] );
  }
  if( route_points.size() <= 2 )
  {
    return;
  }

  map::MapPoint ref_point = route_points.front();

  double dx            = ref_point.x - current_state.x;
  double dy            = ref_point.y - current_state.y;
  double route_heading = std::atan2( route_points[1].y - route_points[0].y, route_points[1].x - route_points[0].x );

  // Yaw difference: how aligned we are with the route
  double yaw_error = adore::math::normalize_angle( route_heading - current_state.yaw_angle );

  // Transform dx, dy into route-aligned frame
  double cos_h = std::cos( route_heading );
  double sin_h = std::sin( route_heading );
  // Lateral error is along the "left/right" axis
  double lateral_error = ( -sin_h * dx + cos_h * dy );

  double base_spline_length_time = spline_length_time; // This is the original parameter
  double lateral_factor          = 3.0;                // tuning parameter
  double yaw_factor              = 2.0;                // tuning parameter
  double adaptive_scale          = 1.0 + lateral_factor * std::fabs( lateral_error ) + yaw_factor * std::fabs( yaw_error );

  double lookahead_dist = base_spline_length_time * ( current_state.vx + 1.0 ) * adaptive_scale + 0.05;
  // Keep a minimum lookahead distance
  lookahead_dist = std::max( lookahead_dist, 0.3 );
  if( previous_state )
    lookahead_dist -= adore::math::distance_2d( current_state, *previous_state );

  // Now, figure out how many points to replace based on new lookahead distance
  size_t              num_points = route_points.size();
  std::vector<double> cumulative_d;
  cumulative_d.resize( num_points, 0.0 );
  for( size_t i = 1; i < num_points; ++i )
  {
    double dd       = adore::math::distance_2d( route_points[i - 1], route_points[i] );
    cumulative_d[i] = cumulative_d[i - 1] + dd;
  }

  // Find how many route points lie within 'lookahead_dist'
  size_t num_point_to_replace = 0;
  for( size_t i = 1; i < num_points; ++i )
  {
    if( cumulative_d[i] > lookahead_dist )
    {
      num_point_to_replace = i;
      break;
    }
  }

  // If we didn't find a suitable cutoff (route too short), just return
  if( num_point_to_replace == 0 || num_point_to_replace >= route_points.size() )
  {
    return;
  }

  // Extract the end points for the spline
  map::MapPoint end_point = route_points[num_point_to_replace];
  if( num_point_to_replace + 1 >= route_points.size() )
  {
    return; // Not enough points to form spline
  }
  map::MapPoint end_point2 = route_points[num_point_to_replace + 1];

  // Remove replaced segment
  route_points.erase( route_points.begin(), route_points.begin() + num_point_to_replace );


  double virtual_point_distance = 0.01 * adaptive_scale; // Scale virtual point spacing as well
  double x0                     = current_state.x;
  double y0                     = current_state.y;

  // Two virtual points ahead of current position to set initial heading
  map::MapPoint start_virtual;
  start_virtual.x         = x0 + virtual_point_distance * std::cos( current_state.yaw_angle );
  start_virtual.y         = y0 + virtual_point_distance * std::sin( current_state.yaw_angle );
  start_virtual.parent_id = end_point.parent_id;
  start_virtual.max_speed = std::nullopt;

  map::MapPoint start_virtual_2;
  start_virtual_2.x         = x0 + 2 * virtual_point_distance * std::cos( current_state.yaw_angle );
  start_virtual_2.y         = y0 + 2 * virtual_point_distance * std::sin( current_state.yaw_angle );
  start_virtual_2.parent_id = end_point.parent_id;
  start_virtual_2.max_speed = std::nullopt;

  // Build spline points
  std::vector<double> spline_s;
  std::vector<double> spline_x;
  std::vector<double> spline_y;

  spline_s.push_back( 0.0 );
  spline_x.push_back( x0 );
  spline_y.push_back( y0 );

  double s1 = virtual_point_distance;
  spline_s.push_back( s1 );
  spline_x.push_back( start_virtual.x );
  spline_y.push_back( start_virtual.y );

  double s2 = 2 * virtual_point_distance;
  spline_s.push_back( s2 );
  spline_x.push_back( start_virtual_2.x );
  spline_y.push_back( start_virtual_2.y );

  // Distance from start_virtual_2 to end_point
  double dist_to_end = adore::math::distance_2d( adore::math::Point2d( spline_x.back(), spline_y.back() ), end_point );
  double s3          = s2 + dist_to_end;
  spline_s.push_back( s3 );
  spline_x.push_back( end_point.x );
  spline_y.push_back( end_point.y );

  // Distance from end_point to end_point2
  double dist_end_to_end2 = adore::math::distance_2d( end_point, end_point2 );
  double s4               = s3 + dist_end_to_end2;
  spline_s.push_back( s4 );
  spline_x.push_back( end_point2.x );
  spline_y.push_back( end_point2.y );

  tk::spline s_x, s_y;
  s_x.set_points( spline_s, spline_x, tk::spline::cspline );
  s_y.set_points( spline_s, spline_y, tk::spline::cspline );

  // Fill route_points with spline samples at intervals ds (the original ds from filtering)
  // But we might want to use a slightly smaller ds here for a smoother curve.
  double sample_ds = ds * 0.5; // oversampling for smoother transitions
  for( double s = s4; s >= 0.0; s -= sample_ds )
  {
    double new_x = s_x( s );
    double new_y = s_y( s );

    map::MapPoint spline_point;
    spline_point.x         = new_x;
    spline_point.y         = new_y;
    spline_point.parent_id = end_point.parent_id;
    spline_point.max_speed = std::nullopt;

    route_points.push_front( spline_point );
  }

  // Ensure start point at s=0.0
  {
    map::MapPoint start_point;
    start_point.x         = s_x( 0.0 );
    start_point.y         = s_y( 0.0 );
    start_point.parent_id = end_point.parent_id;
    start_point.max_speed = std::nullopt;
    route_points.push_front( start_point );
  }
}


} // namespace planner
} // namespace adore
