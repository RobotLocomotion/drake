// clang-format: off
#include "drake/multibody/multibody_tree/multibody_tree.h"
// clang-format: on

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
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

    // Add some bodies so we can add joints between them:
    body1_ = &model_.AddBody<RigidBody>(M_B);

    // Add a revolute joint between the world and body1:
    joint1_ = &model_.AddJoint<RevoluteJoint>(
        "Joint1",
        model_.get_world_body(), {}, *body1_, {}, Vector3d::UnitZ());

    // We are done adding modeling elements. Finalize the model:
    model_.Finalize();

    // Create a context to store the state for this model:
    context_ = model_.CreateDefaultContext();
  }

 protected:
  MultibodyTree<double> model_;
  const RigidBody<double>* body1_{nullptr};
  const RevoluteJoint<double>* joint1_{nullptr};
  std::unique_ptr<Context<double>> context_;
};

// Verify the expected number of dofs.
TEST_F(RevoluteJointTest, NumDOFs) {
  EXPECT_EQ(joint1_->num_dofs(), 1);
}

// Default axis accessor.
TEST_F(RevoluteJointTest, GetAxis) {
  EXPECT_EQ(joint1_->get_revolute_axis(), Vector3d::UnitZ());
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
  MultibodyForces<double> forces1(model_);

  // Add value twice:
  joint1_->AddInTorque(*context_, some_value, &forces1);
  joint1_->AddInTorque(*context_, some_value, &forces1);


  MultibodyForces<double> forces2(model_);
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


}  // namespace
}  // namespace multibody
}  // namespace drake
