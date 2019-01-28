#include "drake/multibody/tree/prismatic_joint.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

const double kEpsilon = std::numeric_limits<double>::epsilon();

using Eigen::Vector3d;
using systems::Context;

constexpr double kPositionLowerLimit = -1.0;
constexpr double kPositionUpperLimit = 1.5;
constexpr double kVelocityLowerLimit = -1.1;
constexpr double kVelocityUpperLimit = 1.6;
constexpr double kAccelerationLowerLimit = -1.2;
constexpr double kAccelerationUpperLimit = 1.7;
constexpr double kDamping = 3;

class PrismaticJointTest : public ::testing::Test {
 public:
  // Creates a simple model consisting of a single body with a prismatic joint
  // with the sole purpose of testing the PrismaticJoint user facing API.
  void SetUp() override {
    // Spatial inertia for adding body. The actual value is not important for
    // these tests and therefore we do not initialize it.
    const SpatialInertia<double> M_B;  // Default construction is ok for this.

    // Create an empty model.
    auto model = std::make_unique<internal::MultibodyTree<double>>();

    // Add a body so we can add joint to it.
    body1_ = &model->AddBody<RigidBody>(M_B);

    // Add a prismatic joint between the world and body1:
    joint1_ = &model->AddJoint<PrismaticJoint>(
        "Joint1", model->world_body(), {}, *body1_, {}, Vector3d::UnitZ(),
        kPositionLowerLimit, kPositionUpperLimit, kDamping);
    Joint<double>& mutable_joint = model->get_mutable_joint(joint1_->index());
    mutable_joint.set_velocity_limits(
        Vector1<double>::Constant(kVelocityLowerLimit),
        Vector1<double>::Constant(kVelocityUpperLimit));
    mutable_joint.set_acceleration_limits(
        Vector1<double>::Constant(kAccelerationLowerLimit),
        Vector1<double>::Constant(kAccelerationUpperLimit));

    // We are done adding modeling elements. Transfer tree to system and get
    // a Context.
    system_ = std::make_unique<internal::MultibodyTreeSystem<double>>(
        std::move(model));
    context_ = system_->CreateDefaultContext();
  }

  const internal::MultibodyTree<double>& tree() const {
    return internal::GetInternalTree(*system_);
  }

 protected:
  std::unique_ptr<internal::MultibodyTreeSystem<double>> system_;
  std::unique_ptr<Context<double>> context_;

  const RigidBody<double>* body1_{nullptr};
  const PrismaticJoint<double>* joint1_{nullptr};
};

// Verify the expected number of dofs.
TEST_F(PrismaticJointTest, NumDOFs) {
  EXPECT_EQ(tree().num_positions(), 1);
  EXPECT_EQ(tree().num_velocities(), 1);
  EXPECT_EQ(joint1_->num_positions(), 1);
  EXPECT_EQ(joint1_->num_velocities(), 1);
  EXPECT_EQ(joint1_->position_start(), 0);
  EXPECT_EQ(joint1_->velocity_start(), 0);
}

// Default axis accessor.
TEST_F(PrismaticJointTest, GetAxis) {
  EXPECT_EQ(joint1_->translation_axis(), Vector3d::UnitZ());
}

TEST_F(PrismaticJointTest, GetJointLimits) {
  EXPECT_EQ(joint1_->position_lower_limits().size(), 1);
  EXPECT_EQ(joint1_->position_upper_limits().size(), 1);
  EXPECT_EQ(joint1_->velocity_lower_limits().size(), 1);
  EXPECT_EQ(joint1_->velocity_upper_limits().size(), 1);
  EXPECT_EQ(joint1_->acceleration_lower_limits().size(), 1);
  EXPECT_EQ(joint1_->acceleration_upper_limits().size(), 1);

  EXPECT_EQ(joint1_->position_lower_limit(), kPositionLowerLimit);
  EXPECT_EQ(joint1_->position_upper_limit(), kPositionUpperLimit);
  EXPECT_EQ(joint1_->velocity_lower_limit(), kVelocityLowerLimit);
  EXPECT_EQ(joint1_->velocity_upper_limit(), kVelocityUpperLimit);
  EXPECT_EQ(joint1_->acceleration_lower_limit(), kAccelerationLowerLimit);
  EXPECT_EQ(joint1_->acceleration_upper_limit(), kAccelerationUpperLimit);
  EXPECT_EQ(joint1_->damping(), kDamping);
}

// Context-dependent value access.
TEST_F(PrismaticJointTest, ContextDependentAccess) {
  const double some_value = 1.5;
  // Translation access:
  joint1_->set_translation(context_.get(), some_value);
  EXPECT_EQ(joint1_->get_translation(*context_), some_value);

  // Translation rate access:
  joint1_->set_translation_rate(context_.get(), some_value);
  EXPECT_EQ(joint1_->get_translation_rate(*context_), some_value);
}

// Tests API to apply torques to a joint.
TEST_F(PrismaticJointTest, AddInForces) {
  const double some_value = 1.5;
  // Default initialized to zero forces:
  MultibodyForces<double> forces1(tree());

  // Add value twice:
  joint1_->AddInForce(*context_, some_value, &forces1);
  joint1_->AddInForce(*context_, some_value, &forces1);

  MultibodyForces<double> forces2(tree());
  // Add value only once:
  joint1_->AddInForce(*context_, some_value, &forces2);
  // Add forces2 into itself (same as adding torque twice):
  forces2.AddInForces(forces2);

  // forces1 and forces2 should be equal:
  EXPECT_EQ(forces1.generalized_forces(), forces2.generalized_forces());
  auto F2 = forces2.body_forces().cbegin();
  for (auto& F1 : forces1.body_forces())
    EXPECT_TRUE(F1.IsApprox(*F2++, kEpsilon));
}

TEST_F(PrismaticJointTest, Clone) {
  auto model_clone = tree().CloneToScalar<AutoDiffXd>();
  const auto& joint1_clone = model_clone->get_variant(*joint1_);

  EXPECT_EQ(joint1_clone.name(), joint1_->name());
  EXPECT_EQ(joint1_clone.frame_on_parent().index(),
            joint1_->frame_on_parent().index());
  EXPECT_EQ(joint1_clone.frame_on_child().index(),
            joint1_->frame_on_child().index());
  EXPECT_EQ(joint1_clone.translation_axis(), joint1_->translation_axis());
  EXPECT_EQ(joint1_clone.position_lower_limits(),
            joint1_->position_lower_limits());
  EXPECT_EQ(joint1_clone.position_upper_limits(),
            joint1_->position_upper_limits());
  EXPECT_EQ(joint1_clone.velocity_lower_limits(),
            joint1_->velocity_lower_limits());
  EXPECT_EQ(joint1_clone.velocity_upper_limits(),
            joint1_->velocity_upper_limits());
  EXPECT_EQ(joint1_clone.acceleration_lower_limits(),
            joint1_->acceleration_lower_limits());
  EXPECT_EQ(joint1_clone.acceleration_upper_limits(),
            joint1_->acceleration_upper_limits());
  EXPECT_EQ(joint1_clone.damping(), joint1_->damping());
}

}  // namespace
}  // namespace multibody
}  // namespace drake
