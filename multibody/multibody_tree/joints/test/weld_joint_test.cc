#include "drake/multibody/multibody_tree/joints/weld_joint.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;
using systems::Context;

class WeldJointTest : public ::testing::Test {
 public:
  // Creates a simple model consisting of a single body with a weld joint
  // with the sole purpose of testing the WeldJoint user facing API.
  void SetUp() override {
    // Spatial inertia for adding body. The actual value is not important for
    // these tests and therefore we do not initialize it.
    const SpatialInertia<double> M_B;

    // Add a body so we can add joint to it.
    body_ = &model_.AddBody<RigidBody>(M_B);

    // Add a prismatic joint between the world and the body.
    joint_ = &model_.AddJoint<WeldJoint>(
        "Welder",
        model_.world_body(), {},  // X_PF
        *body_, {},               // X_BM
        X_FM_);                   // X_FM

    // We are done adding modeling elements. Finalize the model:
    model_.Finalize();
  }

 protected:
  MultibodyTree<double> model_;
  const RigidBody<double>* body_{nullptr};
  const WeldJoint<double>* joint_{nullptr};
  const Isometry3d X_FM_{Translation3d(0, 0.5, 0)};
};

// Verify the expected number of dofs.
TEST_F(WeldJointTest, NumDOFs) {
  EXPECT_EQ(model_.num_positions(), 0);
  EXPECT_EQ(model_.num_velocities(), 0);
  EXPECT_EQ(joint_->num_dofs(), 0);
}

// Verify we can retrieve the fixed posed between the welded frames.
TEST_F(WeldJointTest, GetX_PC) {
  EXPECT_EQ(joint_->X_PC().matrix(), X_FM_.matrix());
}

}  // namespace
}  // namespace multibody
}  // namespace drake
