#pragma once

#include <cmath>

#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"

#include "drake/system1/simulation_options.h"

namespace drake {
namespace ros {

/**
 * Adds a custom stop function that checks whether the simulation should
 * abort based on a call to ros::ok().
 */
void AddAbortFunction(drake::SimulationOptions* options) {
  options->should_stop = [](double sim_time) {
    return !::ros::ok();
  };
}

}  // namespace ros
}  // namespace drake
