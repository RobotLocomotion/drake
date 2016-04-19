
#include <iostream>

#include "ros/ros.h"

#include "drake/Path.h"
// #include "drake/util/Polynomial.h"
#include "drake/examples/Pendulum/Pendulum.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/plants/bot_visualizer_ros.h"
#include "drake/systems/ros_system.h"
#include "drake/systems/cascade_system.h"
#include "drake/util/drakeAppUtil.h"

using Drake::SimulationOptions;
using drake::systems::plants::BotVisualizerROS;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "PendulumDynamicsNode");

  auto p = make_shared<Pendulum>();

  auto v = make_shared<BotVisualizerROS<PendulumState> >(
      Drake::getDrakePath() + "/examples/Pendulum/Pendulum.urdf",
      DrakeJoint::FIXED);

  auto sys = Drake::cascade(p, v);

  PendulumState<double> x0;
  x0.theta = 1;
  x0.thetadot = 0;

  cout << "coords: [" << getCoordinateName(x0, 0) << ", "
       << getCoordinateName(x0, 1) << "]" << endl;

  SimulationOptions options;
  options.realtime_factor = 1.0;
  if (commandLineOptionExists(argv, argv + argc, "--non-realtime")) {
    options.warn_real_time_violation = true;
  }

  drake::systems::RunROS(sys, 0, 10, x0, options);
}
