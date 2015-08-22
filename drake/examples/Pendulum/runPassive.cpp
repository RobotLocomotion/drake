
#include <iostream>
#include "Pendulum.h"
#include "LCMCoordinateFrame.h"

using namespace std;

int main(int argc, char* argv[]) {
  DrakeSystemPtr p(new Pendulum);
  shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
   return 1;

  p->output_frame = make_shared<LCMCoordinateFrame<drake::lcmt_drake_signal> >("PendulumStateFrame",p->output_frame->getCoordinateNames(),lcm);

  p->runLCM(0,5,p->getRandomState());

  cout << "output frame: " << p->getOutputFrame() << endl;
}
