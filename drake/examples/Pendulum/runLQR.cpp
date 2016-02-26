#include <iostream>
#include "drake/examples/Pendulum/Pendulum.h"
#include "drake/systems/CascadeSystem.h"
#include "drake/systems/FeedbackSystem.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/controllers/LQR.h"
#include "drake/systems/plants/BotVisualizer.h"

using namespace std;
using namespace Drake;

int main(int argc, char* argv[]) {
  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();
  if(!lcm->good())
    return 1;

  auto p = std::make_shared<Pendulum>();

  Eigen::MatrixXd Q(2,2);  Q << 10, 0, 0, 1;
  Eigen::MatrixXd R(1,1);  R << 1;
  PendulumState<double> xG;
  xG.theta = M_PI;
  xG.thetadot = 0;
  PendulumInput<double> uG;
  uG.tau = 0;
  auto c = timeInvariantLQR(*p,xG,uG,Q,R);
  auto v = std::make_shared<BotVisualizer<PendulumState> >(lcm,getDrakePath()+"/examples/Pendulum/Pendulum.urdf",DrakeJoint::FIXED);

  auto sys = cascade(feedback(p,c),v);

  SimulationOptions options;
  options.realtime_factor=1.0;

  for (int i=0; i<5; i++) {
    Eigen::Vector2d x0 = toEigen(xG);
    x0 += Eigen::Vector2d::Random();
    simulate(*sys,0,5,x0,options);
  }
}

