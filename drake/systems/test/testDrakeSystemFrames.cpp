
#include <iostream>
#include "DrakeSystem.h"

using namespace std;

int main(int argc, char* argv[]) {

  DrakeSystemPtr test_system1 = make_shared<DrakeSystem>("MySystem",
          make_shared<CoordinateFrame>("state",vector<string>({"theta","thetadot"})),
          nullptr,
          make_shared<CoordinateFrame>("input",vector<string>({"tau"})),
          make_shared<CoordinateFrame>("output",vector<string>({"theta"})));

  cout << "input frame: " << test_system1->getInputFrame() << endl;
  cout << "state frame: " << test_system1->getStateFrame() << endl;
  cout << "output frame: " << test_system1->getOutputFrame() << endl;

  DrakeSystemPtr test_system2 = make_shared<DrakeSystem>("MySystem2",
                                                         make_shared<CoordinateFrame>("state2",vector<string>({"theta2","thetadot2"})),
                                                         nullptr,
                                                         test_system1->output_frame,
                                                         make_shared<CoordinateFrame>("output2",vector<string>({"theta2"})));

  cout << "input frame: " << test_system2->getInputFrame() << endl;
  cout << "state frame: " << test_system2->getStateFrame() << endl;
  cout << "output frame: " << test_system2->getOutputFrame() << endl;

  auto test_system3 = cascade(test_system1,test_system2);

  cout << "input frame: " << test_system3->getInputFrame() << endl;
  cout << "state frame: " << test_system3->getStateFrame() << endl;
  cout << "output frame: " << test_system3->getOutputFrame() << endl;

  if (test_system3->input_frame != test_system1->input_frame ||
      test_system3->output_frame != test_system2->output_frame)
    throw runtime_error("base cascade input/output frames");

  vector<string> answer = {"theta","thetadot","theta2","thetadot2"};
  if(test_system3->getStateFrame().getDim()!= answer.size() ||
    !std::equal(answer.begin(), answer.end(), test_system3->getStateFrame().getCoordinateNames().begin()))
    throw runtime_error("base cascade state frame");

  return 0;
}