#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

const double kEpsilon = std::numeric_limits<double>::epsilon();

using Eigen::Vector3d;
using systems::Context;

class PrismaticJointTest : public ::testing::Test {
 public:
  // Creates a simple model consisting of a single body with a prismatic joint
  // with the sole purpose of testing the PrismaticJoint user facing API.
  void SetUp() override {
    // Spatial inertia for adding body. The actual value is not important for
    // these tests and therefore we do not initialize it.
    const SpatialInertia<double> M_B;  // Default construction is ok for this.

    // Add a body so we can add joint to it.
    body1_ = &model_.AddBody<RigidBody>(M_B);

    // Add a prismatic joint between the world and body1:
    joint1_ = &model_.AddJoint<PrismaticJoint>(
        "Joint1",
        model_.world_body(), {}, *body1_, {}, Vector3d::UnitZ());

    // We are done adding modeling elements. Finalize the model:
    model_.Finalize();

    // Create a context to store the state for this model:
    context_ = model_.CreateDefaultContext();
  }

 protected:
  MultibodyTree<double> model_;
  const RigidBody<double>* body1_{nullptr};
  const PrismaticJoint<double>* joint1_{nullptr};
  std::unique_ptr<Context<double>> context_;
};

// Verify the expected number of dofs.
TEST_F(PrismaticJointTest, NumDOFs) {
  EXPECT_EQ(joint1_->num_dofs(), 1);
}

// Default axis accessor.
TEST_F(PrismaticJointTest, GetAxis) {
  EXPECT_EQ(joint1_->translation_axis(), Vector3d::UnitZ());
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
  MultibodyForces<double> forces1(model_);

  // Add value twice:
  joint1_->AddInForce(*context_, some_value, &forces1);
  joint1_->AddInForce(*context_, some_value, &forces1);

  MultibodyForces<double> forces2(model_);
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


}  // namespace
}  // namespace multibody
}  // namespace drake
