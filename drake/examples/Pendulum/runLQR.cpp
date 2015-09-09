#include <iostream>
#include "Pendulum.h"
#include "LCMCoordinateFrame.h"

using namespace std;

int main(int argc, char* argv[]) {
  shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;

  std::shared_ptr<PendulumWithBotVis> pendulum = make_shared<PendulumWithBotVis>(lcm);
  DrakeSystemPtr controller = pendulum->balanceLQR();

  DrakeSystemPtr sys = feedback(pendulum,controller);

  Eigen::VectorXd x0(2);
  x0 << M_PI, 0;

  DrakeSystem::SimulationOptions options = sys->default_simulation_options;
  options.realtime_factor = 1.0;

  for (int i=0; i<5; i++) {
    sys->simulate(0, 5, x0+pendulum->getRandomState(), options);
  }
}

