
#include <iostream>
#include "Quadrotor.h"

using namespace std;

int main(int argc, char* argv[]) {
  shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;

  DrakeSystemPtr quad(new Quadrotor(lcm));

  DrakeSystem::SimulationOptions options = quad->default_simulation_options;
  options.realtime_factor = 1.0;
  options.initial_step_size = 0.005;

  runLCM(quad,*lcm, 0, 50, quad->getRandomState(), &options);
}
