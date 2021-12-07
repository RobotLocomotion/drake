// clang-format: off
#include "drake/multibody/tree/multibody_tree-inl.h"
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

constexpr double kPositionLowerLimit = -1.0;
constexpr double kPositionUpperLimit = 1.5;
constexpr double kVelocityLowerLimit = -1.1;
constexpr double kVelocityUpperLimit = 1.6;
constexpr double kAccelerationLowerLimit = -1.2;
constexpr double kAccelerationUpperLimit = 1.7;
constexpr double kDamping = 3;

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
    joint1_ = &model->AddJoint<RevoluteJoint>(
        "Joint1", model->world_body(), std::nullopt, *body1_, std::nullopt,
        Vector3d::UnitZ(), kPositionLowerLimit, kPositionUpperLimit, kDamping);
    mutable_joint1_ = dynamic_cast<RevoluteJoint<double>*>(
        &model->get_mutable_joint(joint1_->index()));
    DRAKE_DEMAND(mutable_joint1_ != nullptr);
    mutable_joint1_->set_velocity_limits(
        Vector1<double>::Constant(kVelocityLowerLimit),
        Vector1<double>::Constant(kVelocityUpperLimit));
    mutable_joint1_->set_acceleration_limits(
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
  const RevoluteJoint<double>* joint1_{nullptr};
  RevoluteJoint<double>* mutable_joint1_{nullptr};
};

TEST_F(RevoluteJointTest, Type) {
  const Joint<double>& base = *joint1_;
  EXPECT_EQ(base.type_name(), RevoluteJoint<double>::kTypeName);
}

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
  EXPECT_EQ(joint1_clone.get_default_angle(), joint1_->get_default_angle());
}

TEST_F(RevoluteJointTest, SetVelocityAndAccelerationLimits) {
  const double new_lower = -0.2;
  const double new_upper = 0.2;
  // Check for velocity limits.
  mutable_joint1_->set_velocity_limits(Vector1<double>(new_lower),
                                       Vector1<double>(new_upper));
  EXPECT_EQ(joint1_->velocity_lower_limit(), new_lower);
  EXPECT_EQ(joint1_->velocity_upper_limit(), new_upper);
  // Does not match num_velocities().
  EXPECT_THROW(mutable_joint1_->set_velocity_limits(VectorX<double>(1),
                                                    VectorX<double>()),
               std::runtime_error);
  EXPECT_THROW(mutable_joint1_->set_velocity_limits(VectorX<double>(),
                                                    VectorX<double>(1)),
               std::runtime_error);
  // Lower limit is larger than upper limit.
  EXPECT_THROW(mutable_joint1_->set_velocity_limits(Vector1<double>(2),
                                                    Vector1<double>(0)),
               std::runtime_error);

  // Check for acceleration limits.
  mutable_joint1_->set_acceleration_limits(Vector1<double>(new_lower),
                                           Vector1<double>(new_upper));
  EXPECT_EQ(joint1_->acceleration_lower_limit(), new_lower);
  EXPECT_EQ(joint1_->acceleration_upper_limit(), new_upper);
  // Does not match num_velocities().
  EXPECT_THROW(mutable_joint1_->set_acceleration_limits(VectorX<double>(1),
                                                        VectorX<double>()),
               std::runtime_error);
  EXPECT_THROW(mutable_joint1_->set_acceleration_limits(VectorX<double>(),
                                                        VectorX<double>(1)),
               std::runtime_error);
  // Lower limit is larger than upper limit.
  EXPECT_THROW(mutable_joint1_->set_acceleration_limits(Vector1<double>(2),
                                                        Vector1<double>(0)),
               std::runtime_error);
}

TEST_F(RevoluteJointTest, NameSuffix) {
  EXPECT_EQ(joint1_->position_suffix(0), "q");
  EXPECT_EQ(joint1_->velocity_suffix(0), "w");
}

TEST_F(RevoluteJointTest, DefaultAngle) {
  const double default_angle = 0.0;

  const double new_default_angle =
      0.5 * kPositionLowerLimit + 0.5 * kPositionUpperLimit;

  const double out_of_bounds_low_angle = kPositionLowerLimit - 1;
  const double out_of_bounds_high_angle = kPositionUpperLimit + 1;

  // Constructor should set the default angle to 0.0
  EXPECT_EQ(joint1_->get_default_angle(), default_angle);

  // Setting a new default angle should propagate so that `get_default_angle()`
  // remains correct.
  mutable_joint1_->set_default_angle(new_default_angle);
  EXPECT_EQ(joint1_->get_default_angle(), new_default_angle);

  // Setting the default angle out of the bounds of the position limits
  // should NOT throw an exception
  EXPECT_NO_THROW(mutable_joint1_->set_default_angle(out_of_bounds_low_angle));
  EXPECT_NO_THROW(mutable_joint1_->set_default_angle(out_of_bounds_high_angle));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
