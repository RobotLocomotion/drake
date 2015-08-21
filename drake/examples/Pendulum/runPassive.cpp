
#include <iostream>
#include "Pendulum.h"
#include "LCMCoordinateFrame.h"

using namespace std;

int main(int argc, char* argv[]) {
  Pendulum p;
  shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  p.output_frame = shared_ptr<CoordinateFrame>(new LCMCoordinateFrame<lcmt_drake_signal>("PendulumOutput",{"theta"},lcm));

  cout << "output frame: " << p.output_frame << endl;
  p.runLCM(0,5,p.getRandomState());
}
