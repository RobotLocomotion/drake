
#include <iostream>
#include "Pendulum.h"
#include "LCMCoordinateFrame.h"

using namespace std;

int main(int argc, char* argv[]) {
  DrakeSystemPtr p(new Pendulum);
  shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
   return 1;

  p->output_frame = shared_ptr<CoordinateFrame>(new LCMCoordinateFrame<lcmt_drake_signal>("PendulumOutput",{"theta"},lcm));

  cout << "output frame: " << p->getOutputFrame() << endl;
  p->runLCM(0,5,p->getRandomState());
}
