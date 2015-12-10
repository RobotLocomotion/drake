#include <iostream>
#include "Quadrotor.h"

using namespace std;
using namespace Eigen;

int main(int argc, char* argv[]) {
  shared_ptr<lcm::LCM> lcm(new lcm::LCM);

  if(!lcm->good())
    return 1;

  shared_ptr<Quadrotor> quad(new Quadrotor(lcm));
  DrakeSystemPtr controller = quad->balanceLQR();
  DrakeSystemPtr sys = feedback(quad, controller);

  DrakeSystem::SimulationOptions options = sys->default_simulation_options;
  
  options.realtime_factor = 1.0;
  options.initial_step_size = 0.005;

  runLCM(sys, *lcm, 0, 10, sys->getRandomState(), &options);
}

