#pragma once

#include <cmath>

#include <chrono>
#include <iostream>
#include <limits>
#include <map>
#include <vector>

#include "adore_map/map.hpp"
#include "adore_map/route.hpp"
#include "adore_math/angles.h"
#include "adore_math/distance.h"

#include "dynamics/traffic_participant.hpp"
#include "dynamics/trajectory.hpp"
#include "multi_agent_solver/models/single_track_model.hpp"
#include "multi_agent_solver/multi_agent_solver.hpp"
#include "planning/speed_profiles.hpp"

namespace adore
{
namespace planner
{

class OSQPPlanner
{


public:

  SpeedProfile            speed_profile;
  std::optional<mas::OCP> last_problem = std::nullopt;


  dynamics::Trajectory plan_trajectory( const map::Route& latest_route, const dynamics::VehicleStateDynamic& current_state,
                                        const map::Map& latest_map, const dynamics::TrafficParticipantSet& traffic_participants );


private:
};

} // namespace planner
} // namespace adore
