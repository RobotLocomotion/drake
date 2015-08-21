
#include <iostream>
#include "DrakeSystem.h"

using namespace std;

int main(int argc, char* argv[]) {

  DrakeSystem test_system("MySystem",
          shared_ptr<CoordinateFrame>(new CoordinateFrame("state",vector<string>({"theta","thetadot"}))),
          nullptr,
          shared_ptr<CoordinateFrame>(new CoordinateFrame("input",vector<string>({"tau"}))),
          shared_ptr<CoordinateFrame>(new CoordinateFrame("output",vector<string>({"theta"})))
  );

  cout << "input frame: " << test_system.getInputFrame() << endl;
  cout << "state frame: " << test_system.getStateFrame() << endl;
  cout << "output frame: " << test_system.getOutputFrame() << endl;

  return 0;
}