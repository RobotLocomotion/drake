// clang-format: off
#include "drake/multibody/tree/multibody_tree-inl.h"
// clang-format: on

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector2d;
using Eigen::Vector3d;
using systems::Context;

constexpr double kPositionLowerLimit = -1.0;
constexpr double kPositionUpperLimit = 1.5;
constexpr double kVelocityLowerLimit = -1.1;
constexpr double kVelocityUpperLimit = 1.6;
constexpr double kAccelerationLowerLimit = -1.2;
constexpr double kAccelerationUpperLimit = 1.7;
constexpr double kDamping = 3;
constexpr double kPositionNonZeroDefault = 0.25;

class PlanarJointTest : public ::testing::Test {
 public:
  // Creates a MultibodyTree model with a body attached via a planar joint.
  void SetUp() override {
    // Spatial inertia for adding bodies. The actual value is not important for
    // these tests and therefore we do not initialize it.
    const SpatialInertia<double> M_B;  // Default construction is ok for this.

    // Create an empty model.
    auto model = std::make_unique<internal::MultibodyTree<double>>();

    // Add some bodies so we can add joints between them:
    body_ = &model->AddBody<RigidBody>(M_B);

    // Add a universal joint between the world and body1:
    joint_ = &model->AddJoint<PlanarJoint>("Joint", model->world_body(),
                                           std::nullopt, *body_, std::nullopt,
                                           Vector3d::Constant(kDamping));
    mutable_joint_ = dynamic_cast<PlanarJoint<double>*>(
        &model->get_mutable_joint(joint_->index()));
    DRAKE_DEMAND(mutable_joint_);
    mutable_joint_->set_position_limits(
        Vector3d::Constant(kPositionLowerLimit),
        Vector3d::Constant(kPositionUpperLimit));
    mutable_joint_->set_velocity_limits(
        Vector3d::Constant(kVelocityLowerLimit),
        Vector3d::Constant(kVelocityUpperLimit));
    mutable_joint_->set_acceleration_limits(
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

  const RigidBody<double>* body_{nullptr};
  const PlanarJoint<double>* joint_{nullptr};
  PlanarJoint<double>* mutable_joint_{nullptr};
};

TEST_F(PlanarJointTest, Type) {
  const Joint<double>& base = *joint_;
  EXPECT_EQ(base.type_name(), PlanarJoint<double>::kTypeName);
}

// Verify the expected number of dofs.
TEST_F(PlanarJointTest, NumDOFs) {
  EXPECT_EQ(tree().num_positions(), 3);
  EXPECT_EQ(tree().num_velocities(), 3);
  EXPECT_EQ(joint_->num_positions(), 3);
  EXPECT_EQ(joint_->num_velocities(), 3);
  EXPECT_EQ(joint_->position_start(), 0);
  EXPECT_EQ(joint_->velocity_start(), 0);
}

TEST_F(PlanarJointTest, GetJointLimits) {
  EXPECT_EQ(joint_->position_lower_limits().size(), 3);
  EXPECT_EQ(joint_->position_upper_limits().size(), 3);
  EXPECT_EQ(joint_->velocity_lower_limits().size(), 3);
  EXPECT_EQ(joint_->velocity_upper_limits().size(), 3);
  EXPECT_EQ(joint_->acceleration_lower_limits().size(), 3);
  EXPECT_EQ(joint_->acceleration_upper_limits().size(), 3);

  EXPECT_EQ(joint_->position_lower_limits(),
            Vector3d::Constant(kPositionLowerLimit));
  EXPECT_EQ(joint_->position_upper_limits(),
            Vector3d::Constant(kPositionUpperLimit));
  EXPECT_EQ(joint_->velocity_lower_limits(),
            Vector3d::Constant(kVelocityLowerLimit));
  EXPECT_EQ(joint_->velocity_upper_limits(),
            Vector3d::Constant(kVelocityUpperLimit));
  EXPECT_EQ(joint_->acceleration_lower_limits(),
            Vector3d::Constant(kAccelerationLowerLimit));
  EXPECT_EQ(joint_->acceleration_upper_limits(),
            Vector3d::Constant(kAccelerationUpperLimit));
  EXPECT_EQ(joint_->damping(), Vector3d::Constant(kDamping));
}

// Context-dependent value access.
TEST_F(PlanarJointTest, ContextDependentAccess) {
  const Vector2d some_value(1, 0.3);
  const double some_value2 = 1.5;
  const Vector2d some_value3(2, 2.3);
  const double some_value4 = 3;
  // Position access:
  joint_->set_position(context_.get(), some_value);
  EXPECT_EQ(joint_->get_position(*context_), some_value);
  joint_->set_angle(context_.get(), some_value2);
  EXPECT_EQ(joint_->get_angle(*context_), some_value2);
  joint_->set_pose(context_.get(), some_value3, some_value4);
  EXPECT_EQ(joint_->get_position(*context_), some_value3);
  EXPECT_EQ(joint_->get_angle(*context_), some_value4);

  // Velocity access:
  joint_->set_translational_velocity(context_.get(), some_value);
  EXPECT_EQ(joint_->get_translational_velocity(*context_), some_value);
  joint_->set_angular_velocity(context_.get(), some_value2);
  EXPECT_EQ(joint_->get_angular_velocity(*context_), some_value2);
}

// Tests API to apply torques to individual dof of joint.
TEST_F(PlanarJointTest, AddInOneForce) {
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

TEST_F(PlanarJointTest, Clone) {
  auto model_clone = tree().CloneToScalar<AutoDiffXd>();
  const auto& joint_clone = model_clone->get_variant(*joint_);

  EXPECT_EQ(joint_clone.name(), joint_->name());
  EXPECT_EQ(joint_clone.frame_on_parent().index(),
            joint_->frame_on_parent().index());
  EXPECT_EQ(joint_clone.frame_on_child().index(),
            joint_->frame_on_child().index());
  EXPECT_EQ(joint_clone.position_lower_limits(),
            joint_->position_lower_limits());
  EXPECT_EQ(joint_clone.position_upper_limits(),
            joint_->position_upper_limits());
  EXPECT_EQ(joint_clone.velocity_lower_limits(),
            joint_->velocity_lower_limits());
  EXPECT_EQ(joint_clone.velocity_upper_limits(),
            joint_->velocity_upper_limits());
  EXPECT_EQ(joint_clone.acceleration_lower_limits(),
            joint_->acceleration_lower_limits());
  EXPECT_EQ(joint_clone.acceleration_upper_limits(),
            joint_->acceleration_upper_limits());
  EXPECT_EQ(joint_clone.damping(), joint_->damping());
}

TEST_F(PlanarJointTest, SetVelocityAndAccelerationLimits) {
  const double new_lower = -0.2;
  const double new_upper = 0.2;
  // Check for velocity limits.
  mutable_joint_->set_velocity_limits(Vector3d::Constant(new_lower),
                                      Vector3d::Constant(new_upper));
  EXPECT_EQ(joint_->velocity_lower_limits(), Vector3d::Constant(new_lower));
  EXPECT_EQ(joint_->velocity_upper_limits(), Vector3d::Constant(new_upper));
  // Does not match num_velocities().
  DRAKE_EXPECT_THROWS_MESSAGE(
      mutable_joint_->set_velocity_limits(VectorX<double>(2),
                                          VectorX<double>()),
      std::runtime_error,
      ".* 'lower_limits.size\\(\\) == upper_limits.size\\(\\)' failed.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      mutable_joint_->set_velocity_limits(VectorX<double>(),
                                          VectorX<double>(2)),
      std::runtime_error,
      ".* 'lower_limits.size\\(\\) == upper_limits.size\\(\\)' failed.");
  // Lower limit is larger than upper limit.
  DRAKE_EXPECT_THROWS_MESSAGE(mutable_joint_->set_velocity_limits(
                                  Vector3d::Constant(2), Vector3d::Constant(0)),
                              std::runtime_error,
                              ".* '\\(lower_limits.array\\(\\) <= "
                              "upper_limits.array\\(\\)\\).all\\(\\)' failed.");

  // Check for acceleration limits.
  mutable_joint_->set_acceleration_limits(Vector3d::Constant(new_lower),
                                          Vector3d::Constant(new_upper));
  EXPECT_EQ(joint_->acceleration_lower_limits(), Vector3d::Constant(new_lower));
  EXPECT_EQ(joint_->acceleration_upper_limits(), Vector3d::Constant(new_upper));
  // Does not match num_velocities().
  DRAKE_EXPECT_THROWS_MESSAGE(
      mutable_joint_->set_acceleration_limits(VectorX<double>(3),
                                              VectorX<double>()),
      std::runtime_error,
      ".* 'lower_limits.size\\(\\) == upper_limits.size\\(\\)' failed.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      mutable_joint_->set_acceleration_limits(VectorX<double>(),
                                              VectorX<double>(3)),
      std::runtime_error,
      ".* 'lower_limits.size\\(\\) == upper_limits.size\\(\\)' failed.");
  // Lower limit is larger than upper limit.
  DRAKE_EXPECT_THROWS_MESSAGE(mutable_joint_->set_acceleration_limits(
                                  Vector3d::Constant(3), Vector3d::Constant(0)),
                              std::runtime_error,
                              ".* '\\(lower_limits.array\\(\\) <= "
                              "upper_limits.array\\(\\)\\).all\\(\\)' failed.");
}

TEST_F(PlanarJointTest, DefaultState) {
  const Vector2d new_default_position =
      Vector2d::Constant(kPositionNonZeroDefault);
  const Vector2d out_of_bounds_low_position =
      Vector2d::Constant(kPositionLowerLimit - 1);
  const Vector2d out_of_bounds_high_position =
      Vector2d::Constant(kPositionUpperLimit + 1);

  const double new_default_angle = kPositionNonZeroDefault;
  const double out_of_bounds_low_angle = kPositionLowerLimit - 1;
  const double out_of_bounds_high_angle = kPositionUpperLimit + 1;

  // Constructor should set the default position to Vector2d::Zero() and angle
  // to zero.
  EXPECT_EQ(joint_->get_default_position(), Vector2d::Zero());
  EXPECT_EQ(joint_->get_default_angle(), 0);

  // Setting a new default position should propagate so that
  // `get_default_position()` remains correct.
  mutable_joint_->set_default_position(new_default_position);
  EXPECT_EQ(joint_->get_default_position(), new_default_position);

  // Setting a new default angle should propagate so that `get_default_angle()`
  // remains correct.
  mutable_joint_->set_default_angle(new_default_angle);
  EXPECT_EQ(joint_->get_default_angle(), new_default_angle);

  // Setting the default position or angle out of the bounds of the position
  // limits should NOT throw an exception.
  EXPECT_NO_THROW(
      mutable_joint_->set_default_position(out_of_bounds_low_position));
  EXPECT_NO_THROW(
      mutable_joint_->set_default_position(out_of_bounds_high_position));
  EXPECT_NO_THROW(
      mutable_joint_->set_default_angle(out_of_bounds_low_angle));
  EXPECT_NO_THROW(
      mutable_joint_->set_default_angle(out_of_bounds_high_angle));

  // Setting a new default pose should propagate so that
  // `get_default_position()` and `get_default_angle()` remain correct.
  mutable_joint_->set_default_pose(new_default_position, new_default_angle);
  EXPECT_EQ(joint_->get_default_position(), new_default_position);
  EXPECT_EQ(joint_->get_default_angle(), new_default_angle);
}

TEST_F(PlanarJointTest, RandomState) {
  RandomGenerator generator;
  std::uniform_real_distribution<symbolic::Expression> uniform;

  tree().SetRandomState(*context_, &context_->get_mutable_state(), &generator);

  EXPECT_EQ(joint_->get_position(*context_), Vector2d::Zero());
  EXPECT_EQ(joint_->get_angle(*context_), 0);

  mutable_joint_->set_random_pose_distribution(
      Vector2<symbolic::Expression>(uniform(generator) + 1.0,
                                    uniform(generator) + 2.0),
      uniform(generator) + 3.0);
  tree().SetRandomState(*context_, &context_->get_mutable_state(), &generator);
  EXPECT_GE(mutable_joint_->get_position(*context_)[0], 1.0);
  EXPECT_LE(mutable_joint_->get_position(*context_)[0], 2.0);
  EXPECT_GE(mutable_joint_->get_position(*context_)[1], 2.0);
  EXPECT_LE(mutable_joint_->get_position(*context_)[1], 3.0);
  EXPECT_GE(mutable_joint_->get_angle(*context_), 3.0);
  EXPECT_LE(mutable_joint_->get_angle(*context_), 4.0);

  // Check that they change on a second draw from the distribution.
  const Vector2d last_position = mutable_joint_->get_position(*context_);
  const double last_angle = mutable_joint_->get_angle(*context_);
  tree().SetRandomState(*context_, &context_->get_mutable_state(), &generator);
  EXPECT_NE(mutable_joint_->get_position(*context_), last_position);
  EXPECT_NE(mutable_joint_->get_angle(*context_), last_angle);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
