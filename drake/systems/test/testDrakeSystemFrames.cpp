
#include <iostream>
#include "DrakeSystem.h"

using namespace std;

int main(int argc, char* argv[]) {

  DrakeSystem test_system("MySystem",
          shared_ptr<CoordinateFrame>(new CoordinateFrame("state",2,vector<string>({"theta","thetadot"}))),
          nullptr,
          shared_ptr<CoordinateFrame>(new CoordinateFrame("input",1,vector<string>({"tau"}))),
          shared_ptr<CoordinateFrame>(new CoordinateFrame("output",1,vector<string>({"theta"})))
  );

  cout << "input frame" << test_system.getInputFrame() << endl;
  cout << "state frame" << test_system.getStateFrame() << endl;
  cout << "output frame" << test_system.getOutputFrame() << endl;

  return 0;
}