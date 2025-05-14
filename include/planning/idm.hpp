#pragma once
#include <cmath>

namespace adore
{
namespace planner
{
namespace idm
{

static double
calculate_idm_acc( double distance_to_goal, double distance_to_object, double max_speed, double desired_time_headway,
                   double distance_to_maintain, double current_speed, double max_acceleration, double vehicle_velocity )

{
  double distance_for_idm = std::min( distance_to_object, distance_to_goal );


  double s_star = distance_to_maintain + current_speed * desired_time_headway
                + current_speed * ( current_speed - vehicle_velocity ) / ( 2 * sqrt( max_acceleration ) );

  double velocity_ratio = current_speed / max_speed;

  return max_acceleration
       * ( 1 - velocity_ratio * velocity_ratio * velocity_ratio * velocity_ratio
           - ( s_star / distance_for_idm ) * ( s_star / distance_for_idm ) );
}
} // namespace idm
} // namespace planner
} // namespace adore
