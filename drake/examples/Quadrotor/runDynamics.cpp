
#include <iostream>
#include "Quadrotor.h"
#include "Simulation.h"
#include "BotVisualizer.h"

using namespace std;
using namespace Drake;

int main(int argc, char* argv[]) {
  shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;

  static_assert(!SystemStructureTraits<Quadrotor>::isDirectFeedthrough,"Quadrotor should not be direct feedthrough");
  static_assert(!SystemStructureTraits<Quadrotor>::isTimeVarying,"Quadrotor should be time invariant");

  auto quad = make_shared<Quadrotor>();
  auto v = make_shared<BotVisualizer<QuadrotorState> >(lcm,getDrakePath()+"/examples/Quadrotor/quadrotor.urdf",DrakeJoint::ROLLPITCHYAW);

  auto sys = cascade(quad,v);

  SimulationOptions options = default_simulation_options;
  options.realtime_factor = 1.0;
  options.initial_step_size = 0.005;

  simulate(*sys, 0, 50, toEigen(getRandomVector<QuadrotorState>()), options);
}
