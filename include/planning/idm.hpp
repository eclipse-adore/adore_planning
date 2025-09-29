/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#pragma once
#include <cmath>

namespace adore
{
namespace planner
{
namespace idm
{

static double
calculate_idm_acc( double dist_goal, double dist_object, double max_speed, double time_headway, double dist_headway, double current_speed,
                   double max_acc, double speed_object )

{
  double distance_for_idm = std::min( dist_object, dist_goal );


  double s_star = dist_headway + current_speed * time_headway + current_speed * ( current_speed - speed_object ) / ( 2 * sqrt( max_acc ) );

  double velocity_ratio = current_speed / max_speed;

  return max_acc
       * ( 1 - velocity_ratio * velocity_ratio * velocity_ratio * velocity_ratio
           - ( s_star / distance_for_idm ) * ( s_star / distance_for_idm ) );
}
} // namespace idm
} // namespace planner
} // namespace adore
