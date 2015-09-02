
#include <iostream>
#include "Pendulum.h"

using namespace std;

int main(int argc, char* argv[]) {
  shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
   return 1;

  DrakeSystemPtr p = make_shared<PendulumWithBotVis>(lcm);
  runLCM(p,*lcm,0,50,p->getRandomState());

  cout << "output frame: " << p->getOutputFrame() << endl;
}
