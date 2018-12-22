// clang-format: off
#include "drake/multibody/tree/multibody_tree.h"
// clang-format: on

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

const double kEpsilon = std::numeric_limits<double>::epsilon();

using Eigen::Vector3d;
using systems::Context;

class RevoluteJointTest : public ::testing::Test {
 public:
  // Creates a DoublePendulumModel class with an underlying MultibodyTree model
  // of a double pendulum.
  void SetUp() override {
    // Spatial inertia for adding bodies. The actual value is not important for
    // these tests and therefore we do not initialize it.
    const SpatialInertia<double> M_B;  // Default construction is ok for this.

    // Create an empty model.
    auto model = std::make_unique<internal::MultibodyTree<double>>();

    // Add some bodies so we can add joints between them:
    body1_ = &model->AddBody<RigidBody>(M_B);

    // Add a revolute joint between the world and body1:
    const double lower_limit = -1.0;
    const double upper_limit = 1.5;
    const double damping = 3.0;
    joint1_ = &model->AddJoint<RevoluteJoint>(
        "Joint1",
        model->world_body(), {}, *body1_, {},
        Vector3d::UnitZ(), lower_limit, upper_limit, damping);

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
  const RevoluteJoint<double>* joint1_{nullptr};
};

// Verify the expected number of dofs.
TEST_F(RevoluteJointTest, NumDOFs) {
  EXPECT_EQ(tree().num_positions(), 1);
  EXPECT_EQ(tree().num_velocities(), 1);
  EXPECT_EQ(joint1_->num_positions(), 1);
  EXPECT_EQ(joint1_->num_velocities(), 1);
  EXPECT_EQ(joint1_->position_start(), 0);
  EXPECT_EQ(joint1_->velocity_start(), 0);
}

// Default axis accessor.
TEST_F(RevoluteJointTest, GetAxis) {
  EXPECT_EQ(joint1_->revolute_axis(), Vector3d::UnitZ());
}

TEST_F(RevoluteJointTest, GetJointLimits) {
  EXPECT_EQ(joint1_->lower_limits().size(), 1);
  EXPECT_EQ(joint1_->upper_limits().size(), 1);
  EXPECT_EQ(joint1_->lower_limits()[0], joint1_->lower_limit());
  EXPECT_EQ(joint1_->upper_limits()[0], joint1_->upper_limit());
}

// Context-dependent value access.
TEST_F(RevoluteJointTest, ContextDependentAccess) {
  const double some_value = M_PI_2;
  // Angle access:
  joint1_->set_angle(context_.get(), some_value);
  EXPECT_EQ(joint1_->get_angle(*context_), some_value);

  // Angular rate access:
  joint1_->set_angular_rate(context_.get(), some_value);
  EXPECT_EQ(joint1_->get_angular_rate(*context_), some_value);
}

// Tests API to apply torques to a joint.
TEST_F(RevoluteJointTest, AddInTorques) {
  const double some_value = M_PI_2;
  // Default initialized to zero forces:
  MultibodyForces<double> forces1(tree());

  // Add value twice:
  joint1_->AddInTorque(*context_, some_value, &forces1);
  joint1_->AddInTorque(*context_, some_value, &forces1);


  MultibodyForces<double> forces2(tree());
  // Add value only once:
  joint1_->AddInTorque(*context_, some_value, &forces2);
  // Add forces2 into itself (same as adding torque twice):
  forces2.AddInForces(forces2);

  // forces1 and forces2 should be equal:
  EXPECT_EQ(forces1.generalized_forces(), forces2.generalized_forces());
  auto F2 = forces2.body_forces().cbegin();
  for (auto& F1 : forces1.body_forces())
    EXPECT_TRUE(F1.IsApprox(*F2++, kEpsilon));
}

TEST_F(RevoluteJointTest, Clone) {
  auto model_clone = tree().CloneToScalar<AutoDiffXd>();
  const auto& joint1_clone = model_clone->get_variant(*joint1_);

  EXPECT_EQ(joint1_clone.name(), joint1_->name());
  EXPECT_EQ(joint1_clone.frame_on_parent().index(),
            joint1_->frame_on_parent().index());
  EXPECT_EQ(joint1_clone.frame_on_child().index(),
            joint1_->frame_on_child().index());
  EXPECT_EQ(joint1_clone.revolute_axis(), joint1_->revolute_axis());
  EXPECT_EQ(joint1_clone.lower_limits(), joint1_->lower_limits());
  EXPECT_EQ(joint1_clone.upper_limits(), joint1_->upper_limits());
  EXPECT_EQ(joint1_clone.lower_limit(), joint1_->lower_limit());
  EXPECT_EQ(joint1_clone.upper_limit(), joint1_->upper_limit());
  EXPECT_EQ(joint1_clone.damping(), joint1_->damping());
}

}  // namespace
}  // namespace multibody
}  // namespace drake
