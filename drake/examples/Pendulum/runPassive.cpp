
#include <iostream>
#include "Pendulum.h"
#include "LCMCoordinateFrame.h"

using namespace std;

int main(int argc, char* argv[]) {
  DrakeSystemPtr p(new Pendulum);
  shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
   return 1;

  p->input_frame = make_shared<LCMCoordinateFrame<drake::lcmt_drake_signal> >("PendulumInput",p->input_frame->getCoordinateNames(),lcm);
  p->output_frame = make_shared<LCMCoordinateFrame<drake::lcmt_drake_signal> >("PendulumState",p->output_frame->getCoordinateNames(),lcm);

  runLCM(lcm,p,0,5,p->getRandomState());

  cout << "output frame: " << p->getOutputFrame() << endl;
}
