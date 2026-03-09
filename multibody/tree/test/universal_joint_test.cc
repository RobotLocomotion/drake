// clang-format: off
#include "drake/multibody/tree/multibody_tree-inl.h"
// clang-format: on

#include <limits>
#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/universal_joint.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector2d;
using systems::Context;

constexpr double kPositionLowerLimit = -1.0;
constexpr double kPositionUpperLimit = 1.5;
constexpr double kVelocityLowerLimit = -1.1;
constexpr double kVelocityUpperLimit = 1.6;
constexpr double kAccelerationLowerLimit = -1.2;
constexpr double kAccelerationUpperLimit = 1.7;
constexpr double kDamping = 3;
constexpr double kPositionNonZeroDefault = 0.25;

class UniversalJointTest : public ::testing::Test {
 public:
  // Creates a MultibodyTree model with a body hanging from a universal joint.
  void SetUp() override {
    // Spatial inertia for adding bodies. The actual value is not important for
    // these tests and therefore we do not initialize it.
    const auto M_B = SpatialInertia<double>::NaN();

    // Create an empty model.
    auto model = std::make_unique<internal::MultibodyTree<double>>();

    // Add some bodies so we can add joints between them:
    body_ = &model->AddRigidBody("Body", M_B);

    // Add a universal joint between the world and body1:
    joint_ = &model->AddJoint<UniversalJoint>("Joint", model->world_body(),
                                              std::nullopt, *body_,
                                              std::nullopt, kDamping);
    mutable_joint_ = dynamic_cast<UniversalJoint<double>*>(
        &model->get_mutable_joint(joint_->index()));
    DRAKE_DEMAND(mutable_joint_ != nullptr);
    mutable_joint_->set_position_limits(
        Vector2d::Constant(kPositionLowerLimit),
        Vector2d::Constant(kPositionUpperLimit));
    mutable_joint_->set_velocity_limits(
        Vector2d::Constant(kVelocityLowerLimit),
        Vector2d::Constant(kVelocityUpperLimit));
    mutable_joint_->set_acceleration_limits(
        Vector2d::Constant(kAccelerationLowerLimit),
        Vector2d::Constant(kAccelerationUpperLimit));

    // We are done adding modeling elements. Transfer tree to system and get
    // a Context.
    system_ = std::make_unique<internal::MultibodyTreeSystem<double>>(
        std::move(model), true /* is_discrete */);
    context_ = system_->CreateDefaultContext();
  }

  const internal::MultibodyTree<double>& tree() const {
    return internal::GetInternalTree(*system_);
  }

 protected:
  std::unique_ptr<internal::MultibodyTreeSystem<double>> system_;
  std::unique_ptr<Context<double>> context_;

  const RigidBody<double>* body_{nullptr};
  const UniversalJoint<double>* joint_{nullptr};
  UniversalJoint<double>* mutable_joint_{nullptr};
};

TEST_F(UniversalJointTest, Type) {
  const Joint<double>& base = *joint_;
  EXPECT_EQ(base.type_name(), UniversalJoint<double>::kTypeName);
}

// Verify the expected number of dofs.
TEST_F(UniversalJointTest, NumDOFs) {
  EXPECT_EQ(tree().num_positions(), 2);
  EXPECT_EQ(tree().num_velocities(), 2);
  EXPECT_EQ(joint_->num_positions(), 2);
  EXPECT_EQ(joint_->num_velocities(), 2);
  EXPECT_EQ(joint_->position_start(), 0);
  EXPECT_EQ(joint_->velocity_start(), 0);
}

TEST_F(UniversalJointTest, GetJointLimits) {
  EXPECT_EQ(joint_->position_lower_limits().size(), 2);
  EXPECT_EQ(joint_->position_upper_limits().size(), 2);
  EXPECT_EQ(joint_->velocity_lower_limits().size(), 2);
  EXPECT_EQ(joint_->velocity_upper_limits().size(), 2);
  EXPECT_EQ(joint_->acceleration_lower_limits().size(), 2);
  EXPECT_EQ(joint_->acceleration_upper_limits().size(), 2);

  EXPECT_EQ(joint_->position_lower_limits(),
            Vector2d::Constant(kPositionLowerLimit));
  EXPECT_EQ(joint_->position_upper_limits(),
            Vector2d::Constant(kPositionUpperLimit));
  EXPECT_EQ(joint_->velocity_lower_limits(),
            Vector2d::Constant(kVelocityLowerLimit));
  EXPECT_EQ(joint_->velocity_upper_limits(),
            Vector2d::Constant(kVelocityUpperLimit));
  EXPECT_EQ(joint_->acceleration_lower_limits(),
            Vector2d::Constant(kAccelerationLowerLimit));
  EXPECT_EQ(joint_->acceleration_upper_limits(),
            Vector2d::Constant(kAccelerationUpperLimit));
}

TEST_F(UniversalJointTest, Damping) {
  EXPECT_EQ(joint_->default_damping(), kDamping);
  EXPECT_EQ(joint_->default_damping_vector(), Vector2d::Constant(kDamping));
}

// Context-dependent value access.
TEST_F(UniversalJointTest, ContextDependentAccess) {
  const Vector2d some_value(M_PI_2, 0.3);
  // Angle access:
  joint_->set_angles(context_.get(), some_value);
  EXPECT_EQ(joint_->get_angles(*context_), some_value);

  // Angular rate access:
  joint_->set_angular_rates(context_.get(), some_value);
  EXPECT_EQ(joint_->get_angular_rates(*context_), some_value);

  // Joint locking.
  joint_->Lock(context_.get());
  EXPECT_EQ(joint_->get_angular_rates(*context_), Vector2d(0., 0.));

  // Damping.
  EXPECT_EQ(joint_->GetDampingVector(*context_), Vector2d::Constant(kDamping));
  EXPECT_NO_THROW(joint_->SetDampingVector(context_.get(), some_value));
  EXPECT_EQ(joint_->GetDampingVector(*context_), some_value);

  // Expect to throw on invalid damping values.
  EXPECT_THROW(joint_->SetDampingVector(context_.get(), Vector2d::Constant(-1)),
               std::exception);
}

// Tests API to apply torques to individual dof of joint.
TEST_F(UniversalJointTest, AddInOneForce) {
  const double some_value = M_PI_2;
  const double kEpsilon = std::numeric_limits<double>::epsilon();
  MultibodyForces<double> forces1(tree());
  MultibodyForces<double> forces2(tree());

  for (int ii = 0; ii < joint_->num_positions(); ii++) {
    // Add value twice:
    joint_->AddInOneForce(*context_, ii, some_value, &forces1);
    joint_->AddInOneForce(*context_, ii, some_value, &forces1);
    // Add value only once:
    joint_->AddInOneForce(*context_, ii, some_value, &forces2);
  }
  // Add forces2 into itself (same as adding torque twice):
  forces2.AddInForces(forces2);

  // forces1 and forces2 should be equal:
  EXPECT_EQ(forces1.generalized_forces(), forces2.generalized_forces());
  auto F2 = forces2.body_forces().cbegin();
  for (auto& F1 : forces1.body_forces())
    EXPECT_TRUE(F1.IsApprox(*F2++, kEpsilon));
}

// Tests API to add in damping forces.
TEST_F(UniversalJointTest, AddInDamping) {
  const Vector2d angular_rates(0.1, 0.2);
  const double damping = 3 * kDamping;

  const Vector2d damping_forces_expected = -damping * angular_rates;

  joint_->set_angular_rates(context_.get(), angular_rates);
  joint_->SetDampingVector(context_.get(), Vector2d::Constant(damping));

  MultibodyForces<double> forces(tree());
  joint_->AddInDamping(*context_, &forces);
  EXPECT_EQ(forces.generalized_forces(), damping_forces_expected);
}

TEST_F(UniversalJointTest, Clone) {
  auto model_clone = tree().CloneToScalar<AutoDiffXd>();
  const auto& joint_clone1 = dynamic_cast<const UniversalJoint<AutoDiffXd>&>(
      model_clone->get_variant(*joint_));

  const std::unique_ptr<Joint<AutoDiffXd>> shallow =
      joint_clone1.ShallowClone();
  const auto& joint_clone2 =
      dynamic_cast<const UniversalJoint<AutoDiffXd>&>(*shallow);

  for (const auto* clone : {&joint_clone1, &joint_clone2}) {
    EXPECT_EQ(clone->name(), joint_->name());
    EXPECT_EQ(clone->frame_on_parent().index(),
              joint_->frame_on_parent().index());
    EXPECT_EQ(clone->frame_on_child().index(),
              joint_->frame_on_child().index());
    EXPECT_EQ(clone->position_lower_limits(), joint_->position_lower_limits());
    EXPECT_EQ(clone->position_upper_limits(), joint_->position_upper_limits());
    EXPECT_EQ(clone->velocity_lower_limits(), joint_->velocity_lower_limits());
    EXPECT_EQ(clone->velocity_upper_limits(), joint_->velocity_upper_limits());
    EXPECT_EQ(clone->acceleration_lower_limits(),
              joint_->acceleration_lower_limits());
    EXPECT_EQ(clone->acceleration_upper_limits(),
              joint_->acceleration_upper_limits());
    EXPECT_EQ(clone->default_damping(), joint_->default_damping());
    EXPECT_EQ(clone->get_default_angles(), joint_->get_default_angles());
  }
}

TEST_F(UniversalJointTest, SetVelocityAndAccelerationLimits) {
  const double new_lower = -0.2;
  const double new_upper = 0.2;
  // Check for velocity limits.
  mutable_joint_->set_velocity_limits(Vector2d::Constant(new_lower),
                                      Vector2d::Constant(new_upper));
  EXPECT_EQ(joint_->velocity_lower_limits(), Vector2d::Constant(new_lower));
  EXPECT_EQ(joint_->velocity_upper_limits(), Vector2d::Constant(new_upper));
  // Does not match num_velocities().
  DRAKE_EXPECT_THROWS_MESSAGE(
      mutable_joint_->set_velocity_limits(VectorX<double>(2),
                                          VectorX<double>()),
      ".* 'lower_limits.size\\(\\) == upper_limits.size\\(\\)' failed.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      mutable_joint_->set_velocity_limits(VectorX<double>(),
                                          VectorX<double>(2)),
      ".* 'lower_limits.size\\(\\) == upper_limits.size\\(\\)' failed.");
  // Lower limit is larger than upper limit.
  DRAKE_EXPECT_THROWS_MESSAGE(mutable_joint_->set_velocity_limits(
                                  Vector2d::Constant(2), Vector2d::Constant(0)),
                              ".* '\\(lower_limits.array\\(\\) <= "
                              "upper_limits.array\\(\\)\\).all\\(\\)' failed.");

  // Check for acceleration limits.
  mutable_joint_->set_acceleration_limits(Vector2d::Constant(new_lower),
                                          Vector2d::Constant(new_upper));
  EXPECT_EQ(joint_->acceleration_lower_limits(), Vector2d::Constant(new_lower));
  EXPECT_EQ(joint_->acceleration_upper_limits(), Vector2d::Constant(new_upper));
  // Does not match num_velocities().
  DRAKE_EXPECT_THROWS_MESSAGE(
      mutable_joint_->set_acceleration_limits(VectorX<double>(2),
                                              VectorX<double>()),
      ".* 'lower_limits.size\\(\\) == upper_limits.size\\(\\)' failed.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      mutable_joint_->set_acceleration_limits(VectorX<double>(),
                                              VectorX<double>(2)),
      ".* 'lower_limits.size\\(\\) == upper_limits.size\\(\\)' failed.");
  // Lower limit is larger than upper limit.
  DRAKE_EXPECT_THROWS_MESSAGE(mutable_joint_->set_acceleration_limits(
                                  Vector2d::Constant(2), Vector2d::Constant(0)),
                              ".* '\\(lower_limits.array\\(\\) <= "
                              "upper_limits.array\\(\\)\\).all\\(\\)' failed.");
}

TEST_F(UniversalJointTest, CanRotateOrTranslate) {
  EXPECT_TRUE(joint_->can_rotate());
  EXPECT_FALSE(joint_->can_translate());
}

TEST_F(UniversalJointTest, NameSuffix) {
  EXPECT_EQ(joint_->position_suffix(0), "qx");
  EXPECT_EQ(joint_->position_suffix(1), "qy");
  EXPECT_EQ(joint_->velocity_suffix(0), "wx");
  EXPECT_EQ(joint_->velocity_suffix(1), "wy");
}

TEST_F(UniversalJointTest, DefaultAngles) {
  const Vector2d lower_limit_angles = Vector2d::Constant(kPositionLowerLimit);
  const Vector2d upper_limit_angles = Vector2d::Constant(kPositionUpperLimit);

  const Vector2d default_angles = Vector2d::Zero();

  const Vector2d new_default_angles =
      Vector2d::Constant(kPositionNonZeroDefault);

  const Vector2d out_of_bounds_low_angles =
      lower_limit_angles - Vector2d::Constant(1);
  const Vector2d out_of_bounds_high_angles =
      upper_limit_angles + Vector2d::Constant(1);

  // Constructor should set the default angle to Vector2d::Zero()
  EXPECT_EQ(joint_->get_default_angles(), default_angles);

  // Setting a new default angle should propagate so that `get_default_angle()`
  // remains correct.
  mutable_joint_->set_default_angles(new_default_angles);
  EXPECT_EQ(joint_->get_default_angles(), new_default_angles);

  // Setting the default angle out of the bounds of the position limits
  // should NOT throw an exception.
  EXPECT_NO_THROW(mutable_joint_->set_default_angles(out_of_bounds_low_angles));
  EXPECT_NO_THROW(
      mutable_joint_->set_default_angles(out_of_bounds_high_angles));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
