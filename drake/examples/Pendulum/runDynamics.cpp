
#include <iostream>
#include "Pendulum.h"
#include "Simulation.h"

using namespace std;
using namespace Drake;

int main(int argc, char* argv[]) {

  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();
  if(!lcm->good())
   return 1;

  auto p = make_shared<Pendulum>();
  auto v = make_shared<BotVisualizer<PendulumState> >(lcm,"Pendulum.urdf",DrakeJoint::FIXED);

  PendulumState<double> x0;
  x0.theta = 1;
  x0.thetadot = 0;
  cout << "PendulumState::x0 = " << x0 << endl;

  CascadeSystemType(Pendulum,BotVisualizer<PendulumState> ) sys(p,v);

  simulate(sys,0,.1,static_cast<Eigen::Vector2d>(x0));
}
