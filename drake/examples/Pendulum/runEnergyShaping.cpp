#include <iostream>
#include "Pendulum.h"
#include "LCMCoordinateFrame.h"

using namespace std;

int main(int argc, char* argv[]) {
  shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;

  Pendulum p(lcm);
  DrakeSystemPtr controller = make_shared<PendulumEnergyShaping>(p);

  runLCM(controller,*lcm,0,5,Eigen::VectorXd::Zero(0));
}

