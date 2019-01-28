#include "drake/multibody/tree/weld_joint.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/rigid_body.h"
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

    // Create an empty model.
    auto model = std::make_unique<internal::MultibodyTree<double>>();

    // Add a body so we can add joint to it.
    body_ = &model->AddBody<RigidBody>(M_B);

    // Add a prismatic joint between the world and the body.
    joint_ = &model->AddJoint<WeldJoint>(
        "Welder",
        model->world_body(), {},  // X_PF
        *body_, {},               // X_BM
        X_FM_);                   // X_FM

    // We are done adding modeling elements. Transfer tree to system for
    // computation.
    system_ = std::make_unique<internal::MultibodyTreeSystem<double>>(
        std::move(model));
  }

  const internal::MultibodyTree<double>& tree() const {
    return internal::GetInternalTree(*system_);
  }

 protected:
  std::unique_ptr<internal::MultibodyTreeSystem<double>> system_;

  const RigidBody<double>* body_{nullptr};
  const WeldJoint<double>* joint_{nullptr};
  const Isometry3d X_FM_{Translation3d(0, 0.5, 0)};
};

// Verify the expected number of dofs.
TEST_F(WeldJointTest, NumDOFs) {
  EXPECT_EQ(tree().num_positions(), 0);
  EXPECT_EQ(tree().num_velocities(), 0);
  EXPECT_EQ(joint_->num_positions(), 0);
  EXPECT_EQ(joint_->num_velocities(), 0);
  // We just verify we can call these methods. However their return value is
  // irrelevant since joints of type WeldJoint have no state.
  EXPECT_NO_THROW(joint_->position_start());
  EXPECT_NO_THROW(joint_->velocity_start());
}

// Verify we can retrieve the fixed posed between the welded frames.
TEST_F(WeldJointTest, GetX_PC) {
  EXPECT_EQ(joint_->X_PC().matrix(), X_FM_.matrix());
}

TEST_F(WeldJointTest, GetJointLimits) {
  EXPECT_EQ(joint_->position_lower_limits().size(), 0);
  EXPECT_EQ(joint_->position_upper_limits().size(), 0);
  EXPECT_EQ(joint_->velocity_lower_limits().size(), 0);
  EXPECT_EQ(joint_->velocity_upper_limits().size(), 0);
  EXPECT_EQ(joint_->acceleration_lower_limits().size(), 0);
  EXPECT_EQ(joint_->acceleration_upper_limits().size(), 0);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
