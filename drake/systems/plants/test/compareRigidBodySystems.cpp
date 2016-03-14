
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/RigidBodyFrame.h"
#include "drake/util/testUtil.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " [options] full_path_to_robot1 full_path_to_robot2 x y z\n"
              << " The x y z parameters are optional and specify the position"
              << " of the second robot in the world, which is useful for URDF"
              << " models)"
              << std::endl;
    return 1;
  }

  std::shared_ptr<RigidBodyFrame> weld_to_frame;

  if (argc > 3) {
    weld_to_frame = allocate_shared<RigidBodyFrame>(
      aligned_allocator<RigidBodyFrame>(),
      "world",
      nullptr,  // not used since the robot is attached to the world
      Eigen::Vector3d(std::stod(argv[3]), std::stod(argv[4]), std::stod(argv[5])),  // xyz of the car's root link
      Eigen::Vector3d(0, 0, 0));       // rpy of the car's root link
  } else {
    weld_to_frame = allocate_shared<RigidBodyFrame>(
      aligned_allocator<RigidBodyFrame>(),
      "world",
      nullptr,
      Isometry3d::Identity());
  }

  auto r1 = make_shared<RigidBodySystem>();
  r1->addRobotFromFile(argv[1], DrakeJoint::QUATERNION);
  auto r2 = make_shared<RigidBodySystem>();
  r2->addRobotFromFile(argv[2], DrakeJoint::QUATERNION, weld_to_frame);

  // for debugging:
  r1->getRigidBodyTree()->drawKinematicTree("/tmp/r1.dot");
  r2->getRigidBodyTree()->drawKinematicTree("/tmp/r2.dot");
  // I ran this at the console to see the output:
  // dot -Tpng -O /tmp/r1.dot; dot -Tpng -O /tmp/r2.dot; open /tmp/r1.dot.png
  // /tmp/r2.dot.png


  valuecheck(r1->getNumStates(), r2->getNumStates());
  valuecheck(r1->getNumInputs(), r2->getNumInputs());
  valuecheck(r1->getNumOutputs(), r2->getNumOutputs());

  for (int i = 0; i < 1000; i++) {
    std::cout << "i = " << i << std::endl;
    double t = 0.0;
    VectorXd x = getInitialState(*r1);
    VectorXd u = VectorXd::Random(r1->getNumInputs());

    auto xdot1 = r1->dynamics(t, x, u);
    auto xdot2 = r2->dynamics(t, x, u);
    valuecheckMatrix(xdot1, xdot2, 1e-8);
  }
}
