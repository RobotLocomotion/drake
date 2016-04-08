#include <iostream>
#include "drake/examples/Pendulum/Pendulum.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/feedback_system.h"
#include "drake/util/drakeAppUtil.h"

using namespace std;
using namespace Drake;

/**
 * Runs the energy shaping controller as an LCM node or, with "-s",
 * as a standalone BotVisualizer-compatible app.
 */
int main(int argc, char* argv[]) {
  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();
  if (!lcm->good()) return 1;

  auto p = std::make_shared<Pendulum>();
  auto c = std::make_shared<PendulumEnergyShapingController>(*p);

  if (commandLineOptionExists(argv, argv + argc, "-h")) {
    std::cout << "Runs the energy shaping controller as an LCM node or,"
              << std::endl
              << "with '-s', as a standalone BotVisualizer-compatible app."
              << std::endl;
    exit(0);
  }

  if (!commandLineOptionExists(argv, argv + argc, "-s")) {
    // run as LCM node
    runLCM(c, lcm, 0, 10, NullVector<double>());
  } else {  // run a stand-alone simulation
    SimulationOptions options;
    options.realtime_factor = 1.0;
    if (commandLineOptionExists(argv, argv + argc, "--non-realtime")) {
      options.warn_real_time_violation = true;
    }

    Eigen::Vector2d x0;
    x0 << 0.1, 0.2;

    auto v = std::make_shared<BotVisualizer<PendulumState> >(
        lcm, getDrakePath() + "/examples/Pendulum/Pendulum.urdf",
        DrakeJoint::FIXED);
    auto sys = cascade(feedback(p, c), v);
    simulate(*sys, 0, 10, x0, options);
  }
}
