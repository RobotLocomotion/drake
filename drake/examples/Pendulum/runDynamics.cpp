
#include <iostream>
#include "drake/util/Polynomial.h"
#include "drake/examples/Pendulum/Pendulum.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/cascade_system.h"
#include "drake/util/drakeAppUtil.h"

using namespace std;
using namespace Drake;

int main(int argc, char* argv[]) {
  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();
  if (!lcm->good()) return 1;

  auto p = make_shared<Pendulum>();
  auto v = make_shared<BotVisualizer<PendulumState> >(
      lcm, getDrakePath() + "/examples/Pendulum/Pendulum.urdf",
      DrakeJoint::FIXED);

  PendulumState<double> x0;
  x0.theta = 1;
  x0.thetadot = 0;
  //  cout << "PendulumState::x0 = " << x0 << endl;

  auto sys = cascade(p, v);

  cout << "coords:" << getCoordinateName(x0, 0) << ", "
       << getCoordinateName(x0, 1) << endl;

  SimulationOptions options;
  options.realtime_factor = 1.0;
  if (commandLineOptionExists(argv, argv + argc, "--non-realtime")) {
    options.warn_real_time_violation = true;
  }

  //  simulate(*sys, 0, 10, x0, options);
  runLCM(sys, lcm, 0, 10, x0, options);
}
