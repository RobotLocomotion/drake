#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsers/urdf_parser.h"

namespace drake {
namespace examples {
namespace pr2 {

// Tests if the model from pr2_simplified.urdf loads into a RigidBodyTree with
// no errors.
GTEST_TEST(LoadPr2SimplifiedTest, TestIfPr2SimplifiedLoads) {
  auto tree_ = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      FindResourceOrThrow("drake/examples/pr2/models/pr2_description/urdf/"
                          "pr2_simplified.urdf"),
      multibody::joints::
          kFixed /* our PR2 model moves with actuators, not a floating base */,
      nullptr /* weld to frame */, tree_.get());
  EXPECT_EQ(tree_->get_num_actuators(), 28);
  EXPECT_EQ(tree_->get_num_positions(), 28);
  EXPECT_EQ(tree_->bodies.size(), 86);
}

}  // namespace pr2
}  // namespace examples
}  // namespace drake
