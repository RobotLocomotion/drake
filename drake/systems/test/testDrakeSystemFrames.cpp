
#include <iostream>
#include "DrakeSystem.h"

int main(int argc, char* argv[]) {

  DrakeSystem test_system(
          new CoordinateFrame("state",2,{"theta","thetadot"}),
          nullptr,
          new CoordinateFrame("input",1,{"tau"}),
          new CoordinateFrame("output",1,{"theta"})
    );

  cout << "input frame" << test_system.getInputFrame() << endl;
  cout << "state frame" << test_system.getStateFrame() << endl;
  cout << "output frame" << test_system.getOutputFrame() << endl;

  return 0;
}