#include <iostream>
#include "Pendulum.h"
#include "LCMCoordinateFrame.h"
#include "Simulation.h"
#include "BotVisualizer.h"

using namespace std;

int main(int argc, char* argv[]) {
  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();
  if(!lcm->good())
    return 1;

  auto p = std::make_shared<Pendulum>();
  auto c = std::make_shared<PendulumEnergyShapingController>(*p);
  auto v = std::make_shared<Drake::BotVisualizer<PendulumState> >(lcm,Drake::getDrakePath()+"/examples/Pendulum/Pendulum.urdf",DrakeJoint::FIXED);

  auto sys = cascade(feedback(p,c),v);

  Drake::SimulationOptions options;
  options.realtime_factor=1.0;

  Eigen::Vector2d x0; x0 << 0.1, 0.2;
  simulate(*sys,0,10,x0,options);
}

