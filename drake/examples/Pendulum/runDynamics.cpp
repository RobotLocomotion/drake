
#include <iostream>
#include <Polynomial.h>
#include "Pendulum.h"
#include "Simulation.h"
#include "BotVisualizer.h"

using namespace std;
using namespace Drake;

int main(int argc, char* argv[]) {

  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();
  if(!lcm->good())
   return 1;

  auto p = make_shared<Pendulum>();
  auto v = make_shared<BotVisualizer<PendulumState> >(lcm,getDrakePath()+"/examples/Pendulum/Pendulum.urdf",DrakeJoint::FIXED);

  PendulumState<double> x0;
  x0.theta = 1;
  x0.thetadot = 0;
  cout << "PendulumState::x0 = " << x0 << endl;

  auto sys = cascade(p,v);

  SimulationOptions options;
  options.realtime_factor=1.0;

  simulate(*sys,0,10,x0,options);
}
