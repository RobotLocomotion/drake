#include "drake/systems/plants/RigidBodySystem.h"

#include <iostream>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/plants/RigidBodyFrame.h"
#include "drake/systems/plants/joints/floating_base_types.h"

namespace drake {
namespace systems {
namespace plants {
namespace {

using std::make_shared;

using Eigen::VectorXd;

using drake::RigidBodySystem;
using drake::systems::plants::joints::kQuaternion;

char* model_file_1 = nullptr;
char* model_file_2 = nullptr;

GTEST_TEST(CompareRigidBodySystemsTest, TestAll) {
  // Creates a rigid body system using the first model.
  auto r1 = make_shared<RigidBodySystem>();
  r1->AddModelInstanceFromFile(model_file_1, kQuaternion);

  // Creates a rigid body system using the second model.
  auto r2 = make_shared<RigidBodySystem>();
  r2->AddModelInstanceFromFile(model_file_2, kQuaternion);

  // for debugging:
  // r1->getRigidBodyTree()->drawKinematicTree("/tmp/r1.dot");
  // r2->getRigidBodyTree()->drawKinematicTree("/tmp/r2.dot");
  // I ran this at the console to see the output:
  // dot -Tpng -O /tmp/r1.dot; dot -Tpng -O /tmp/r2.dot; open /tmp/r1.dot.png
  // /tmp/r2.dot.png

  // Verfies that the two rigid body systems are equal. This is done by first
  // comparing the number of states, inputs, and outputs of the two rigid body
  // systems and then feeding in random state values into the two rigid body
  // systems and verifying that their resulting xdot states are the same.
  EXPECT_EQ(r1->getNumStates(), r2->getNumStates());
  EXPECT_EQ(r1->getNumInputs(), r2->getNumInputs());
  EXPECT_EQ(r1->getNumOutputs(), r2->getNumOutputs());

  for (int i = 0; i < 1000; i++) {
    double t = 0.0;
    VectorXd x = getInitialState(*r1);
    VectorXd u = VectorXd::Random(r1->getNumInputs());

    auto xdot1 = r1->dynamics(t, x, u);
    auto xdot2 = r2->dynamics(t, x, u);

    std::string explanation;
    EXPECT_TRUE(CompareMatrices(
        xdot1, xdot2, 1e-8, MatrixCompareType::relative, &explanation))
        << "Model mismatch!" << std::endl
        << "  - initial state:" << std::endl
        << x << std::endl
        << "  - inputs (joint torques?):" << std::endl
        << u << std::endl
        << "  - xdot1:" << std::endl
        << xdot1.transpose() << std::endl
        << "  - xdot2:" << std::endl
        << xdot2.transpose() << std::endl
        << "  - error message:" << std::endl
        << explanation;
  }
}

}  // namespace
}  // namespace plants
}  // namespace systems
}  // namespace drake

int main(int argc, char** argv) {
  // Initializes the Google Test infrastructure.
  testing::InitGoogleTest(&argc, argv);

  // Checks if the minimum number of command line parameters exists. If not,
  // prints a message describing the command line options and then abort with
  // an error code.
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " [options] full_path_to_robot1 full_path_to_robot2"
              << std::endl;
    return 1;
  }

  // Obtains the names of the two models to compare.
  drake::systems::plants::model_file_1 = argv[1];
  drake::systems::plants::model_file_2 = argv[2];

  return RUN_ALL_TESTS();
}
