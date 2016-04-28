#include "drake/systems/plants/RigidBodySystem.h"

#include <iostream>

#include <gtest/gtest.h>

#include "drake/systems/plants/RigidBodyFrame.h"
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

std::string model_file_1, model_file_2;
std::shared_ptr<RigidBodyFrame> model_pose_in_world;

TEST(CompareRigidBodySystemsTest, TestAll) {
  auto r1 = make_shared<RigidBodySystem>();
  r1->addRobotFromFile(model_file_1, DrakeJoint::QUATERNION);

  auto r2 = make_shared<RigidBodySystem>();
  r2->addRobotFromFile(model_file_2, DrakeJoint::QUATERNION, model_pose_in_world);

  // for debugging:
  // r1->getRigidBodyTree()->drawKinematicTree("/tmp/r1.dot");
  // r2->getRigidBodyTree()->drawKinematicTree("/tmp/r2.dot");
  // I ran this at the console to see the output:
  // dot -Tpng -O /tmp/r1.dot; dot -Tpng -O /tmp/r2.dot; open /tmp/r1.dot.png
  // /tmp/r2.dot.png

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
    EXPECT_TRUE(drake::util::CompareMatrices(
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

int main(int argc, char **argv) {
  std::cout << "Running main() from compareRigidBodySystems.cpp" << std::endl;
  // ::testing::GTEST_FLAG(output) = "xml:hello.xml";

  std::cout << "Calling testing::InitGoogleTest(...)" << std::endl;
  testing::InitGoogleTest(&argc, argv);

  std::cout << "argc = " << argc << std::endl;
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " [options] full_path_to_robot1 full_path_to_robot2 x y z\n"
              << " The x y z parameters are optional and specify the position"
              << " of robot2 in the world, which is useful for URDF"
              << " models)" << std::endl;
    return 1;
  }

  drake::systems::plants::model_file_1 = argv[1];
  drake::systems::plants::model_file_2 = argv[2];

  if (argc > 3) {
    drake::systems::plants::model_pose_in_world =
        std::allocate_shared<RigidBodyFrame>(
            Eigen::aligned_allocator<RigidBodyFrame>(), "world",
            nullptr,  // not used since the robot is attached to the world
            Eigen::Vector3d(std::stod(argv[3]), std::stod(argv[4]),
                            std::stod(argv[5])),  // xyz of the car's root link
            Eigen::Vector3d(0, 0, 0));            // rpy of the car's root link
  } else {
    drake::systems::plants::model_pose_in_world =
        std::allocate_shared<RigidBodyFrame>(
            Eigen::aligned_allocator<RigidBodyFrame>(), "world", nullptr,
            Eigen::Isometry3d::Identity());
  }

  return RUN_ALL_TESTS();
}
