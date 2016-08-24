#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/math/autodiff.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/common/drake_path.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace systems {
namespace plants {
namespace test {
namespace {

using drake::parsers::ModelInstanceIdTable;

class RigidBodyTreeInverseDynamicsTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    tree.reset(new RigidBodyTree());
    std::string kAtlasUrdf =
        drake::GetDrakePath() + "/examples/Atlas/urdf/atlas_convex_hull.urdf";
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        kAtlasUrdf, DrakeJoint::ROLLPITCHYAW,
        nullptr /* weld_to_frame */, tree.get());
  }

 public:
  // TODO(amcastro-tri): A stack object here (preferable to a pointer)
  // generates build issues on Windows platforms. See git-hub issue #1854.
  std::unique_ptr<RigidBodyTree> tree;
};

TEST_F(RigidBodyTreeInverseDynamicsTest, TestAccelerationJacobianIsMassMatrix) {
  drake::math::initializeAutoDiff(q);
}

}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
