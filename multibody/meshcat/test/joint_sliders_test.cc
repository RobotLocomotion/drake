#include "drake/multibody/meshcat/joint_sliders.h"

#include <gtest/gtest.h>

// #include "drake/common/find_resource.h"
// #include "drake/multibody/parsing/parser.h"
// #include "drake/multibody/plant/multibody_plant.h"
// #include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace meshcat {
namespace {

using geometry::Meshcat;

class JointSlidersTest : public ::testing::Test {
 protected:
  JointSlidersTest() : meshcat_(std::make_shared<Meshcat>()) {}

  std::shared_ptr<Meshcat> meshcat_;
};

TEST_F(JointSlidersTest, Publish) {
}

}  // namespace
}  // namespace meshcat
}  // namespace multibody
}  // namespace drake
