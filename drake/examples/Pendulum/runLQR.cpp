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

  for (int i=0; i<5; i++) {
    x0 << M_PI+0.05, 0;
    sys->simulate(0, 5, x0);
  }
}

