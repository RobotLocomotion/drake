
#include <iostream>
#include "util/Polynomial.h"
#include "examples/Pendulum/Pendulum.h"
#include "systems/Simulation.h"
#include "systems/plants/BotVisualizer.h"
#include "systems/LCMSystem.h"

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
//  cout << "PendulumState::x0 = " << x0 << endl;

  auto sys = cascade(p,v);

  cout << "coords:" << getCoordinateName(x0,0) << ", " << getCoordinateName(x0,1) << endl;

  SimulationOptions options;
  options.realtime_factor=1.0;

//  simulate(*sys,0,10,x0,options);
  runLCM(sys,lcm,0,10,x0,options);
}
