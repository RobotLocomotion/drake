// clang-format: off
#include "drake/multibody/tree/multibody_tree-inl.h"
// clang-format: on

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector3d;
using systems::Context;

constexpr double kPositionLowerLimit = -1.0;
constexpr double kPositionUpperLimit = 1.5;
constexpr double kVelocityLowerLimit = -1.1;
constexpr double kVelocityUpperLimit = 1.6;
constexpr double kAccelerationLowerLimit = -1.2;
constexpr double kAccelerationUpperLimit = 1.7;
constexpr double kDamping = 3;
constexpr double kPositionNonZeroDefault =
    (kPositionLowerLimit + kPositionUpperLimit) / 2;

class BallRpyJointTest : public ::testing::Test {
 public:
  // Creates a MultibodyTree model of a spherical pendulum.
  void SetUp() override {
    // Spatial inertia for adding bodies. The actual value is not important for
    // these tests and therefore we do not initialize it.
    const SpatialInertia<double> M_B;  // Default construction is ok for this.

    // Create an empty model.
    auto model = std::make_unique<internal::MultibodyTree<double>>();

    // Add some bodies so we can add joints between them:
    body_ = &model->AddRigidBody("Body", M_B);

    // Add a ball rpy joint between the world and body:
    joint_ = &model->AddJoint<BallRpyJoint>("Joint", model->world_body(),
                                            std::nullopt, *body_, std::nullopt,
                                            kDamping);
    mutable_joint_ = dynamic_cast<BallRpyJoint<double>*>(
        &model->get_mutable_joint(joint_->index()));
    DRAKE_DEMAND(mutable_joint_ != nullptr);
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
        std::move(model), true/* is_discrete */);
    context_ = system_->CreateDefaultContext();
  }

  const internal::MultibodyTree<double>& tree() const {
    return internal::GetInternalTree(*system_);
  }

 protected:
  std::unique_ptr<internal::MultibodyTreeSystem<double>> system_;
  std::unique_ptr<Context<double>> context_;

  const RigidBody<double>* body_{nullptr};
  const BallRpyJoint<double>* joint_{nullptr};
  BallRpyJoint<double>* mutable_joint_{nullptr};
};

TEST_F(BallRpyJointTest, Type) {
  const Joint<double>& base = *joint_;
  EXPECT_EQ(base.type_name(), BallRpyJoint<double>::kTypeName);
}

// Verify the expected number of dofs.
TEST_F(BallRpyJointTest, NumDOFs) {
  EXPECT_EQ(tree().num_positions(), 3);
  EXPECT_EQ(tree().num_velocities(), 3);
  EXPECT_EQ(joint_->num_positions(), 3);
  EXPECT_EQ(joint_->num_velocities(), 3);
  EXPECT_EQ(joint_->position_start(), 0);
  EXPECT_EQ(joint_->velocity_start(), 0);
}

TEST_F(BallRpyJointTest, GetJointLimits) {
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
}

TEST_F(BallRpyJointTest, Damping) {
  EXPECT_EQ(joint_->default_damping(), kDamping);
  EXPECT_EQ(joint_->default_damping_vector(), Vector3d::Constant(kDamping));

  // Ensure the deprecated versions are correct until removal.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  EXPECT_EQ(joint_->damping(), kDamping);
  EXPECT_EQ(joint_->damping_vector(), Vector3d::Constant(kDamping));
#pragma GCC diagnostic pop
}

// Context-dependent value access.
TEST_F(BallRpyJointTest, ContextDependentAccess) {
  const Vector3d some_value(M_PI_2, 0., 1.);
  // Angle access:
  joint_->set_angles(context_.get(), some_value);
  EXPECT_EQ(joint_->get_angles(*context_), some_value);

  // Angular velocity access:
  joint_->set_angular_velocity(context_.get(), some_value);
  EXPECT_EQ(joint_->get_angular_velocity(*context_), some_value);

  // Joint locking.
  joint_->Lock(context_.get());
  EXPECT_EQ(joint_->get_angular_velocity(*context_), Vector3d(0., 0., 0.));

  // Damping.
  EXPECT_EQ(joint_->GetDampingVector(*context_), Vector3d::Constant(kDamping));
  EXPECT_NO_THROW(joint_->SetDampingVector(context_.get(), some_value));
  EXPECT_EQ(joint_->GetDampingVector(*context_), some_value);

  // Expect to throw on invalid damping values.
  EXPECT_THROW(joint_->SetDampingVector(context_.get(), Vector3d::Constant(-1)),
               std::exception);
}

// Tests API to apply torques to joint.
TEST_F(BallRpyJointTest, AddInOneForce) {
  const double some_value = M_PI_2;
  MultibodyForces<double> forces(tree());

  // Since adding forces to individual degrees of freedom of this joint does
  // not make physical sense, this method should throw.
  EXPECT_THROW(joint_->AddInOneForce(*context_, 0, some_value, &forces),
               std::logic_error);
}

// Tests API to add in damping forces.
TEST_F(BallRpyJointTest, AddInDampingForces) {
  const Vector3d angular_velocity(0.1, 0.2, 0.3);
  const double damping = 3 * kDamping;

  const Vector3d damping_forces_expected = -damping * angular_velocity;

  joint_->set_angular_velocity(context_.get(), angular_velocity);
  joint_->SetDampingVector(context_.get(), Vector3d::Constant(damping));

  MultibodyForces<double> forces(tree());
  joint_->AddInDamping(*context_, &forces);
  EXPECT_EQ(forces.generalized_forces(), damping_forces_expected);
}

TEST_F(BallRpyJointTest, Clone) {
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
  EXPECT_EQ(joint_clone.default_damping(), joint_->default_damping());
  EXPECT_EQ(joint_clone.get_default_angles(), joint_->get_default_angles());
}

TEST_F(BallRpyJointTest, SetVelocityAndAccelerationLimits) {
  const double new_lower = -0.2;
  const double new_upper = 0.2;
  // Check for velocity limits.
  mutable_joint_->set_velocity_limits(Vector3d::Constant(new_lower),
                                      Vector3d::Constant(new_upper));
  EXPECT_EQ(joint_->velocity_lower_limits(), Vector3d::Constant(new_lower));
  EXPECT_EQ(joint_->velocity_upper_limits(), Vector3d::Constant(new_upper));
  // Does not match num_velocities().
  EXPECT_THROW(mutable_joint_->set_velocity_limits(VectorX<double>(3),
                                                   VectorX<double>()),
               std::exception);
  EXPECT_THROW(mutable_joint_->set_velocity_limits(VectorX<double>(),
                                                   VectorX<double>(3)),
               std::exception);
  // Lower limit is larger than upper limit.
  EXPECT_THROW(mutable_joint_->set_velocity_limits(Vector3d::Constant(2),
                                                   Vector3d::Constant(0)),
               std::exception);

  // Check for acceleration limits.
  mutable_joint_->set_acceleration_limits(Vector3d::Constant(new_lower),
                                          Vector3d::Constant(new_upper));
  EXPECT_EQ(joint_->acceleration_lower_limits(), Vector3d::Constant(new_lower));
  EXPECT_EQ(joint_->acceleration_upper_limits(), Vector3d::Constant(new_upper));
  // Does not match num_velocities().
  EXPECT_THROW(mutable_joint_->set_acceleration_limits(VectorX<double>(3),
                                                       VectorX<double>()),
               std::exception);
  EXPECT_THROW(mutable_joint_->set_acceleration_limits(VectorX<double>(),
                                                       VectorX<double>(3)),
               std::exception);
  // Lower limit is larger than upper limit.
  EXPECT_THROW(mutable_joint_->set_acceleration_limits(Vector3d::Constant(2),
                                                       Vector3d::Constant(0)),
               std::exception);
}

TEST_F(BallRpyJointTest, CanRotateOrTranslate) {
  EXPECT_TRUE(joint_->can_rotate());
  EXPECT_FALSE(joint_->can_translate());
}

TEST_F(BallRpyJointTest, NameSuffix) {
  EXPECT_EQ(joint_->position_suffix(0), "qx");
  EXPECT_EQ(joint_->position_suffix(1), "qy");
  EXPECT_EQ(joint_->position_suffix(2), "qz");
  EXPECT_EQ(joint_->velocity_suffix(0), "wx");
  EXPECT_EQ(joint_->velocity_suffix(1), "wy");
  EXPECT_EQ(joint_->velocity_suffix(2), "wz");
}

TEST_F(BallRpyJointTest, DefaultAngles) {
  const Vector3d lower_limit_angles = Vector3d::Constant(kPositionLowerLimit);
  const Vector3d upper_limit_angles = Vector3d::Constant(kPositionUpperLimit);

  const Vector3d default_angles = Vector3d::Zero();

  const Vector3d new_default_angles =
      Vector3d::Constant(kPositionNonZeroDefault);

  const Vector3d out_of_bounds_low_angles =
      lower_limit_angles - Vector3d::Constant(1);
  const Vector3d out_of_bounds_high_angles =
      upper_limit_angles + Vector3d::Constant(1);

  // Constructor should set the default angle to Vector3d::Zero()
  EXPECT_EQ(joint_->get_default_angles(), default_angles);

  // Setting a new default angle should propagate so that `get_default_angle()`
  // remains correct.
  mutable_joint_->set_default_angles(new_default_angles);
  EXPECT_EQ(joint_->get_default_angles(), new_default_angles);

  // Setting the default angle out of the bounds of the position limits
  // should NOT throw an exception
  EXPECT_NO_THROW(
      mutable_joint_->set_default_angles(out_of_bounds_low_angles));
  EXPECT_NO_THROW(
      mutable_joint_->set_default_angles(out_of_bounds_high_angles));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
