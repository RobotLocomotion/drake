#include <iostream>
#include "Pendulum.h"
#include "Simulation.h"
#include "LQR.h"
#include "BotVisualizer.h"

using namespace std;

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

  auto v = std::make_shared<Drake::BotVisualizer<PendulumState> >(lcm,Drake::getDrakePath()+"/examples/Pendulum/Pendulum.urdf",DrakeJoint::FIXED);

  typedef Drake::AffineSystem<Drake::UnusedVector,PendulumState,PendulumInput,true> LQRType;
  auto sys = std::make_shared<FeedbackSystemType(Pendulum,LQRType)>(p,c);
  CascadeSystemType(FeedbackSystemType(Pendulum,LQRType), Drake::BotVisualizer<PendulumState>) sys_w_vis(sys,v);

  Drake::SimulationOptions options;
  options.realtime_factor=1.0;

  for (int i=0; i<5; i++) {
    Eigen::Vector2d x0 = xG;
    x0 += Eigen::Vector2d::Random();
    simulate(sys_w_vis,0,5,x0,options);
  }
}

