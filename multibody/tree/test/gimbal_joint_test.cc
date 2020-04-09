// clang-format: off
#include "drake/multibody/tree/multibody_tree-inl.h"
// clang-format: on

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/gimbal_joint.h"
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

class GimbalJointTest : public ::testing::Test {
 public:
  // Creates a MultibodyTree model of a spherical pendulum.
  void SetUp() override {
    // Spatial inertia for adding bodies. The actual value is not important for
    // these tests and therefore we do not initialize it.
    const SpatialInertia<double> M_B;  // Default construction is ok for this.

    // Create an empty model.
    auto model = std::make_unique<internal::MultibodyTree<double>>();

    // Add some bodies so we can add joints between them:
    body1_ = &model->AddBody<RigidBody>(M_B);

    // Add a revolute joint between the world and body1:
    joint1_ = &model->AddJoint<GimbalJoint>(
        "Joint1", model->world_body(), std::nullopt, *body1_, std::nullopt,
        Vector3d::Constant(kPositionLowerLimit),
        Vector3d::Constant(kPositionUpperLimit), kDamping);
    mutable_joint1_ = dynamic_cast<GimbalJoint<double>*>(
        &model->get_mutable_joint(joint1_->index()));
    DRAKE_DEMAND(mutable_joint1_);
    mutable_joint1_->set_velocity_limits(
        Vector3d::Constant(kVelocityLowerLimit),
        Vector3d::Constant(kVelocityUpperLimit));
    mutable_joint1_->set_acceleration_limits(
        Vector3d::Constant(kAccelerationLowerLimit),
        Vector3d::Constant(kAccelerationUpperLimit));

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
  const GimbalJoint<double>* joint1_{nullptr};
  GimbalJoint<double>* mutable_joint1_{nullptr};
};

TEST_F(GimbalJointTest, Type) {
  const Joint<double>& base = *joint1_;
  EXPECT_EQ(base.type_name(), GimbalJoint<double>::kTypeName);
}

// Verify the expected number of dofs.
TEST_F(GimbalJointTest, NumDOFs) {
  EXPECT_EQ(tree().num_positions(), 3);
  EXPECT_EQ(tree().num_velocities(), 3);
  EXPECT_EQ(joint1_->num_positions(), 3);
  EXPECT_EQ(joint1_->num_velocities(), 3);
  EXPECT_EQ(joint1_->position_start(), 0);
  EXPECT_EQ(joint1_->velocity_start(), 0);
}

TEST_F(GimbalJointTest, GetJointLimits) {
  EXPECT_EQ(joint1_->position_lower_limits().size(), 3);
  EXPECT_EQ(joint1_->position_upper_limits().size(), 3);
  EXPECT_EQ(joint1_->velocity_lower_limits().size(), 3);
  EXPECT_EQ(joint1_->velocity_upper_limits().size(), 3);
  EXPECT_EQ(joint1_->acceleration_lower_limits().size(), 3);
  EXPECT_EQ(joint1_->acceleration_upper_limits().size(), 3);

  EXPECT_EQ(joint1_->position_lower_limits(),
            Vector3d::Constant(kPositionLowerLimit));
  EXPECT_EQ(joint1_->position_upper_limits(),
            Vector3d::Constant(kPositionUpperLimit));
  EXPECT_EQ(joint1_->velocity_lower_limits(),
            Vector3d::Constant(kVelocityLowerLimit));
  EXPECT_EQ(joint1_->velocity_upper_limits(),
            Vector3d::Constant(kVelocityUpperLimit));
  EXPECT_EQ(joint1_->acceleration_lower_limits(),
            Vector3d::Constant(kAccelerationLowerLimit));
  EXPECT_EQ(joint1_->acceleration_upper_limits(),
            Vector3d::Constant(kAccelerationUpperLimit));
  EXPECT_EQ(joint1_->damping(), kDamping);
}

// Context-dependent value access.
TEST_F(GimbalJointTest, ContextDependentAccess) {
  const Vector3d some_value(M_PI_2, 0., 1.);
  // Angle access:
  joint1_->set_angles(context_.get(), some_value);
  EXPECT_EQ(joint1_->get_angles(*context_), some_value);

  // Angular rate access:
  joint1_->set_angular_velocity(context_.get(), some_value);
  EXPECT_EQ(joint1_->get_angular_velocity(*context_), some_value);
}

// Tests API to apply torques to a joint.
TEST_F(GimbalJointTest, AddInTorques) {
  const double some_value = M_PI_2;
  // Default initialized to zero forces:
  MultibodyForces<double> forces1(tree());
  MultibodyForces<double> forces2(tree());

  for (int ii = 0; ii < 3; ii++) {
    // Add value twice:
    joint1_->AddInOneForce(*context_, ii, some_value, &forces1);
    joint1_->AddInOneForce(*context_, ii, some_value, &forces1);

    // Add value only once:
    joint1_->AddInOneForce(*context_, ii, some_value, &forces2);
  }

  // Add forces2 into itself (same as adding torque twice):
  forces2.AddInForces(forces2);

  // forces1 and forces2 should be equal:
  EXPECT_EQ(forces1.generalized_forces(), forces2.generalized_forces());
  auto F2 = forces2.body_forces().cbegin();
  for (auto& F1 : forces1.body_forces())
    EXPECT_TRUE(F1.IsApprox(*F2++, kEpsilon));
}

TEST_F(GimbalJointTest, Clone) {
  auto model_clone = tree().CloneToScalar<AutoDiffXd>();
  const auto& joint1_clone = model_clone->get_variant(*joint1_);

  EXPECT_EQ(joint1_clone.name(), joint1_->name());
  EXPECT_EQ(joint1_clone.frame_on_parent().index(),
            joint1_->frame_on_parent().index());
  EXPECT_EQ(joint1_clone.frame_on_child().index(),
            joint1_->frame_on_child().index());
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

TEST_F(GimbalJointTest, SetVelocityAndAccelerationLimits) {
  const double new_lower = -0.2;
  const double new_upper = 0.2;
  // Check for velocity limits.
  mutable_joint1_->set_velocity_limits(Vector3d::Constant(new_lower),
                                       Vector3d::Constant(new_upper));
  EXPECT_EQ(joint1_->velocity_lower_limits(), Vector3d::Constant(new_lower));
  EXPECT_EQ(joint1_->velocity_upper_limits(), Vector3d::Constant(new_upper));
  // Does not match num_velocities().
  EXPECT_THROW(mutable_joint1_->set_velocity_limits(VectorX<double>(3),
                                                    VectorX<double>()),
               std::runtime_error);
  EXPECT_THROW(mutable_joint1_->set_velocity_limits(VectorX<double>(),
                                                    VectorX<double>(3)),
               std::runtime_error);
  // Lower limit is larger than upper limit.
  EXPECT_THROW(mutable_joint1_->set_velocity_limits(Vector3d::Constant(2),
                                                    Vector3d::Constant(0)),
               std::runtime_error);

  // Check for acceleration limits.
  mutable_joint1_->set_acceleration_limits(Vector3d::Constant(new_lower),
                                           Vector3d::Constant(new_upper));
  EXPECT_EQ(joint1_->acceleration_lower_limits(),
            Vector3d::Constant(new_lower));
  EXPECT_EQ(joint1_->acceleration_upper_limits(),
            Vector3d::Constant(new_upper));
  // Does not match num_velocities().
  EXPECT_THROW(mutable_joint1_->set_acceleration_limits(VectorX<double>(3),
                                                        VectorX<double>()),
               std::runtime_error);
  EXPECT_THROW(mutable_joint1_->set_acceleration_limits(VectorX<double>(),
                                                        VectorX<double>(3)),
               std::runtime_error);
  // Lower limit is larger than upper limit.
  EXPECT_THROW(mutable_joint1_->set_acceleration_limits(Vector3d::Constant(2),
                                                        Vector3d::Constant(0)),
               std::runtime_error);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
