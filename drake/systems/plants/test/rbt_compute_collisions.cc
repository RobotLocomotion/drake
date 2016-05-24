#include <memory>
#include <stdexcept>
#include <string>

#include "drake/Path.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/util/testUtil.h"

#include "gtest/gtest.h"
#include "gmock/gmock.h"

namespace drake {
namespace {

class RBTCollisionTest: public ::testing::Test {
 protected:
  void SetUp() override {
    tree.addRobotFromSDF(
        Drake::getDrakePath() + "/systems/plants/test/collision_test.sdf",
        DrakeJoint::QUATERNION);
  }

  RigidBodyTree tree;
};

TEST_F(RBTCollisionTest, FindAndComputeContactPoints) {
  std::cout << "Hello test" << std::endl;
}

} // namespace
} // namespace drake