
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/testUtil.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " [options] full_path_to_robot1 full_path_to_robot2"
              << std::endl;
    return 1;
  }

  auto r1 = RigidBodySystem(argv[1], DrakeJoint::QUATERNION);
  auto r2 = RigidBodySystem(argv[2], DrakeJoint::QUATERNION);

  // for debugging:
  r1.getRigidBodyTree()->drawKinematicTree("/tmp/r1.dot");
  r2.getRigidBodyTree()->drawKinematicTree("/tmp/r2.dot");
  // I ran this at the console to see the output:
  // dot -Tpng -O /tmp/r1.dot; dot -Tpng -O /tmp/r2.dot; open /tmp/r1.dot.png
  // /tmp/r2.dot.png

  valuecheck(r1.getNumStates(), r2.getNumStates());
  valuecheck(r1.getNumInputs(), r2.getNumInputs());
  valuecheck(r1.getNumOutputs(), r2.getNumOutputs());

  for (int i = 0; i < 1000; i++) {
    double t = 0.0;
    VectorXd x = getInitialState(r1);
    VectorXd u = VectorXd::Random(r1.getNumInputs());

    auto xdot1 = r1.dynamics(t, x, u);
    auto xdot2 = r2.dynamics(t, x, u);
    //    cout << "xdot = " << xdot.transpose() << endl;
    //    cout << "xdot_rb = " << xdot_rb.transpose() << endl;
    valuecheckMatrix(xdot1, xdot2, 1e-8);
  }
}
