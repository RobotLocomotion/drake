
#include <iostream>
#include "Quadrotor.h"

using namespace std;

int main(int argc, char* argv[]) {
  shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
   return 1;

  DrakeSystemPtr quad(new Quadrotor(lcm));
  runLCM(quad,*lcm, 0, 50, quad->getRandomState());
}
