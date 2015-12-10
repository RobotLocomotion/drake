#include <iostream>
#include "Pendulum.h"
#include "LCMCoordinateFrame.h"
#include "Simulation.h"
#include "BotVisualizer.h"

using namespace std;

int main(int argc, char* argv[]) {
  // todo: move these to core/test
  Eigen::Vector3d abc;  abc << 1,2,3;
  {
    Drake::CombinedVector<double, PendulumState, PendulumInput> test(abc);
    cout << test << endl;
    test=2*abc;
    cout << test << endl;
  }
  {
    Drake::CombinedVectorBuilder<PendulumState,PendulumInput>::VecType<double> test(abc);
    cout << test << endl;
    test=2*abc;
    cout << test << endl;
  }
  // end core tests

  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();
  if(!lcm->good())
    return 1;

  auto p = std::make_shared<Pendulum>();
  auto c = std::make_shared<PendulumEnergyShaping>(*p);
  auto v = std::make_shared<Drake::BotVisualizer<PendulumState> >(lcm,Drake::getDrakePath()+"/examples/Pendulum/Pendulum.urdf",DrakeJoint::FIXED);

  auto sys = std::make_shared<FeedbackSystemType(Pendulum,PendulumEnergyShaping)>(p,c);
  CascadeSystemType(FeedbackSystemType(Pendulum,PendulumEnergyShaping), Drake::BotVisualizer<PendulumState>) sys_w_vis(sys,v);

  Drake::SimulationOptions options;
  options.realtime_factor=1.0;

  Eigen::Vector2d x0; x0 << 0.1, 0.2;
  simulate(sys_w_vis,0,10,x0,options);
}

