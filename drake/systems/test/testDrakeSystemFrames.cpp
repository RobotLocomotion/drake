
#include <iostream>
#include "DrakeSystem.h"

using namespace std;

int main(int argc, char* argv[]) {

  shared_ptr<DrakeSystem> test_system1( new DrakeSystem("MySystem",
          shared_ptr<CoordinateFrame>(new CoordinateFrame("state",vector<string>({"theta","thetadot"}))),
          nullptr,
          shared_ptr<CoordinateFrame>(new CoordinateFrame("input",vector<string>({"tau"}))),
          shared_ptr<CoordinateFrame>(new CoordinateFrame("output",vector<string>({"theta"})))
  ) );

  cout << "input frame: " << test_system1->getInputFrame() << endl;
  cout << "state frame: " << test_system1->getStateFrame() << endl;
  cout << "output frame: " << test_system1->getOutputFrame() << endl;


  shared_ptr<DrakeSystem> test_system2( new DrakeSystem("MySystem2",
                                                       shared_ptr<CoordinateFrame>(new CoordinateFrame("state2",vector<string>({"theta2","thetadot2"}))),
                                                       nullptr,
                                                       test_system1->output_frame,
                                                       shared_ptr<CoordinateFrame>(new CoordinateFrame("output2",vector<string>({"theta2"})))
  ) );

  cout << "input frame: " << test_system2->getInputFrame() << endl;
  cout << "state frame: " << test_system2->getStateFrame() << endl;
  cout << "output frame: " << test_system2->getOutputFrame() << endl;

  shared_ptr<DrakeSystem> test_system3( new CascadeSystem(test_system1,test_system2) );

  cout << "input frame: " << test_system3->getInputFrame() << endl;
  cout << "state frame: " << test_system3->getStateFrame() << endl;
  cout << "output frame: " << test_system3->getOutputFrame() << endl;

  return 0;
}