
#include <iostream>
#include "Quadrotor.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/plants/BotVisualizer.h"

using namespace std;
using namespace Drake;

int main(int argc, char* argv[]) {
  shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;

  auto quad = make_shared<Quadrotor>();
  auto v = make_shared<BotVisualizer<QuadrotorState> >(lcm,getDrakePath()+"/examples/Quadrotor/quadrotor.urdf",DrakeJoint::ROLLPITCHYAW);

  auto sys = cascade(quad,v);

  SimulationOptions options = default_simulation_options;
  options.realtime_factor = 1.0;
  options.initial_step_size = 0.005;

  simulate(*sys, 0, 5, getInitialState(*sys), options);
}
