#include <iostream>
#include "Pendulum.h"
#include "Simulation.h"
#include "BotVisualizer.h"

using namespace std;
using namespace Drake;

int main(int argc, char* argv[]) {
  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();
  if(!lcm->good())
    return 1;

  cout << SystemOutputMethodTraits<Pendulum>::hasTimeArgument << endl;
  cout << SystemOutputMethodTraits<Pendulum>::hasStateArgument << endl;
  cout << SystemOutputMethodTraits<Pendulum>::hasInputArgument << endl;

  cout << SystemStructureTraits<Pendulum>::isDirectFeedthrough << endl;
/*
  auto p = std::make_shared<Pendulum>();
  auto c = std::make_shared<PendulumEnergyShapingController>(*p);
  auto v = std::make_shared<BotVisualizer<PendulumState> >(lcm,getDrakePath()+"/examples/Pendulum/Pendulum.urdf",DrakeJoint::FIXED);

  auto sys = cascade(feedback(p,c),v);

  SimulationOptions options;
  options.realtime_factor=1.0;

  Eigen::Vector2d x0; x0 << 0.1, 0.2;
  simulate(*sys,0,10,x0,options);
  */
}

