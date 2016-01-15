#include <iostream>
#include "drake/examples/Pendulum/Pendulum.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/plants/BotVisualizer.h"

using namespace std;
using namespace Drake;

int main(int argc, char* argv[]) {
  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();
  if(!lcm->good())
    return 1;

  auto p = std::make_shared<Pendulum>();
  auto c = std::make_shared<PendulumEnergyShapingController>(*p);

  if (true) { // run as LCM node
    runLCM(c,lcm,0,10,NullVector<double>());
  } else { // run a stand-alone simulation
    SimulationOptions options;
    options.realtime_factor = 1.0;

    Eigen::Vector2d x0;
    x0 << 0.1, 0.2;

    auto v = std::make_shared<BotVisualizer < PendulumState> >
             (lcm, getDrakePath() + "/examples/Pendulum/Pendulum.urdf", DrakeJoint::FIXED);
    auto sys = cascade(feedback(p, c), v);
    simulate(*sys, 0, 10, x0, options);
  }
}

