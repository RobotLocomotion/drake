#include "QPLocomotionPlan.h"
#include <iostream>

using namespace std;

int main(int argc, char **argv) {
  RigidBodyManipulator robot("examples/Atlas/urdf/atlas_minimal_contact.urdf");
  QPLocomotionPlanSettings settings;
  string lcm_channel = "qp_controller_input";
  QPLocomotionPlan plan(robot, settings, lcm_channel);

  cout << "test passed" << endl;
  return 0;
}
