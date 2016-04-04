#include <gtest/gtest.h>
#include <iostream>
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/eigen_matrix_compare.h"
#include "drake/util/testUtil.h"

using std::make_shared;
using Drake::RigidBodySystem;
using Eigen::VectorXd;
using drake::util::MatrixCompareType;

namespace drake {
namespace systems {
namespace plants {
namespace {

std::string modelFile1, modelFile2;

TEST(CompareRigidBodySystemsTest, TestAll) {
  auto r1 = make_shared<RigidBodySystem>();
  r1->addRobotFromFile(modelFile1, DrakeJoint::QUATERNION);

  auto r2 = make_shared<RigidBodySystem>();
  r2->addRobotFromFile(modelFile2, DrakeJoint::QUATERNION);

  // for debugging:
  // r1->getRigidBodyTree()->drawKinematicTree("/tmp/r1.dot");
  // r2->getRigidBodyTree()->drawKinematicTree("/tmp/r2.dot");
  // I ran this at the console to see the output:
  // dot -Tpng -O /tmp/r1.dot; dot -Tpng -O /tmp/r2.dot; open /tmp/r1.dot.png
  // /tmp/r2.dot.png

  valuecheck(r1->getNumStates(), r2->getNumStates());
  valuecheck(r1->getNumInputs(), r2->getNumInputs());
  valuecheck(r1->getNumOutputs(), r2->getNumOutputs());

  for (int i = 0; i < 1000; i++) {
    double t = 0.0;
    VectorXd x = getInitialState(*r1);
    VectorXd u = VectorXd::Random(r1->getNumInputs());
    auto xdot1 = r1->dynamics(t, x, u);
    auto xdot2 = r2->dynamics(t, x, u);
    EXPECT_TRUE(
        CompareMatrices(xdot1, xdot2, 1e-8, MatrixCompareType::absolute));
  }
}

}  // namespace
}  // namespace plants
}  // namespace systems
}  // namespace drake

int main(int argc, char **argv) {
  std::cout << "Running main() from compareRigidBodySystems.cpp" << std::endl;
  // ::testing::GTEST_FLAG(output) = "xml:hello.xml";

  std::cout << "Calling testing::InitGoogleTest(...)" << std::endl;
  testing::InitGoogleTest(&argc, argv);

  std::cout << "argc = " << argc << std::endl;
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " [options] full_path_to_robot1 full_path_to_robot2"
              << std::endl;
    return 1;
  }

  drake::systems::plants::modelFile1 = argv[1];
  drake::systems::plants::modelFile2 = argv[2];

  return RUN_ALL_TESTS();
}
