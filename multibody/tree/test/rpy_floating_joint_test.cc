// clang-format: off
#include "drake/multibody/tree/multibody_tree-inl.h"
// clang-format: on

#include <limits>
#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/rpy_floating_joint.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

const double kTolerance = 4 * std::numeric_limits<double>::epsilon();

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;
using systems::Context;

constexpr double kPositionLowerLimit = -1.0;
constexpr double kPositionUpperLimit = 1.5;
constexpr double kVelocityLowerLimit = -1.1;
constexpr double kVelocityUpperLimit = 1.6;
constexpr double kAccelerationLowerLimit = -1.2;
constexpr double kAccelerationUpperLimit = 1.7;
constexpr double kAngularDamping = 3;
constexpr double kTranslationalDamping = 4;
constexpr double kPositionNonZeroDefault =
    (kPositionLowerLimit + kPositionUpperLimit) / 2;

class RpyFloatingJointTest : public ::testing::Test {
 public:
  // Creates a MultibodyTree model of a spherical pendulum.
  void SetUp() override {
    // Spatial inertia for adding bodies. The actual value is not important for
    // these tests and therefore we do not initialize it.
    const auto M_B = SpatialInertia<double>::NaN();

    // Create an empty model.
    auto model = std::make_unique<internal::MultibodyTree<double>>();

    // Add some bodies so we can add joints between them:
    body_ = &model->AddRigidBody("Body", M_B);

    // Add a ball rpy joint between the world and body:
    joint_ = &model->AddJoint<RpyFloatingJoint>(
        "Joint", model->world_body(), std::nullopt, *body_, std::nullopt,
        kAngularDamping, kTranslationalDamping);
    mutable_joint_ = dynamic_cast<RpyFloatingJoint<double>*>(
        &model->get_mutable_joint(joint_->index()));
    DRAKE_DEMAND(mutable_joint_ != nullptr);
    mutable_joint_->set_position_limits(
        Vector6d::Constant(kPositionLowerLimit),
        Vector6d::Constant(kPositionUpperLimit));
    mutable_joint_->set_velocity_limits(
        Vector6d::Constant(kVelocityLowerLimit),
        Vector6d::Constant(kVelocityUpperLimit));
    mutable_joint_->set_acceleration_limits(
        Vector6d::Constant(kAccelerationLowerLimit),
        Vector6d::Constant(kAccelerationUpperLimit));

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
  const RpyFloatingJoint<double>* joint_{nullptr};
  RpyFloatingJoint<double>* mutable_joint_{nullptr};
};

TEST_F(RpyFloatingJointTest, Type) {
  const Joint<double>& base = *joint_;
  EXPECT_EQ(base.type_name(), RpyFloatingJoint<double>::kTypeName);
}

// Verify the expected number of dofs.
TEST_F(RpyFloatingJointTest, NumDOFs) {
  EXPECT_EQ(tree().num_positions(), 6);
  EXPECT_EQ(tree().num_velocities(), 6);
  EXPECT_EQ(joint_->num_positions(), 6);
  EXPECT_EQ(joint_->num_velocities(), 6);
  EXPECT_EQ(joint_->position_start(), 0);
  EXPECT_EQ(joint_->velocity_start(), 0);
}

TEST_F(RpyFloatingJointTest, GetJointLimits) {
  EXPECT_EQ(joint_->position_lower_limits().size(), 6);
  EXPECT_EQ(joint_->position_upper_limits().size(), 6);
  EXPECT_EQ(joint_->velocity_lower_limits().size(), 6);
  EXPECT_EQ(joint_->velocity_upper_limits().size(), 6);
  EXPECT_EQ(joint_->acceleration_lower_limits().size(), 6);
  EXPECT_EQ(joint_->acceleration_upper_limits().size(), 6);

  EXPECT_EQ(joint_->position_lower_limits(),
            Vector6d::Constant(kPositionLowerLimit));
  EXPECT_EQ(joint_->position_upper_limits(),
            Vector6d::Constant(kPositionUpperLimit));
  EXPECT_EQ(joint_->velocity_lower_limits(),
            Vector6d::Constant(kVelocityLowerLimit));
  EXPECT_EQ(joint_->velocity_upper_limits(),
            Vector6d::Constant(kVelocityUpperLimit));
  EXPECT_EQ(joint_->acceleration_lower_limits(),
            Vector6d::Constant(kAccelerationLowerLimit));
  EXPECT_EQ(joint_->acceleration_upper_limits(),
            Vector6d::Constant(kAccelerationUpperLimit));
}

TEST_F(RpyFloatingJointTest, Damping) {
  EXPECT_EQ(joint_->default_angular_damping(), kAngularDamping);
  EXPECT_EQ(joint_->default_translational_damping(), kTranslationalDamping);
  EXPECT_EQ(
      joint_->default_damping_vector(),
      (Vector6d() << kAngularDamping, kAngularDamping, kAngularDamping,
       kTranslationalDamping, kTranslationalDamping, kTranslationalDamping)
          .finished());
}

// Context-dependent value access.
TEST_F(RpyFloatingJointTest, ContextDependentAccess) {
  const Vector3d angles_A(M_PI_2, 0., 1.);
  const Vector3d angles_B(0.25, 0.5, M_PI_2);
  const Vector3d translation(1., 2., 3.);
  const Vector3d angular_velocity(0.5, 0.5, 0.5);
  const Vector3d translational_velocity(0.1, 0.2, 0.3);
  const RotationMatrixd rotation_matrix_A =
      RotationMatrixd(RollPitchYawd(angles_A));
  const RotationMatrixd rotation_matrix_B =
      RotationMatrixd(RollPitchYawd(angles_B));
  const RigidTransformd transform_A(rotation_matrix_A, translation);

  // Pose access:
  joint_->set_angles(context_.get(), angles_A);
  EXPECT_EQ(joint_->get_angles(*context_), angles_A);

  joint_->SetOrientation(context_.get(), rotation_matrix_B);
  EXPECT_TRUE(
      CompareMatrices(joint_->get_angles(*context_), angles_B, kTolerance));

  joint_->SetTranslation(context_.get(), translation);
  EXPECT_EQ(joint_->get_translation(*context_), translation);

  joint_->set_angles(context_.get(), Vector3d::Zero());  // Zero out pose.
  joint_->SetTranslation(context_.get(), Vector3d::Zero());
  joint_->SetPose(context_.get(), transform_A);
  // We expect a bit of roundoff error due to transforming between rpy
  // and rotation matrix representations.
  EXPECT_TRUE(
      joint_->GetPose(*context_).IsNearlyEqualTo(transform_A, kTolerance));

  // Angular velocity access:
  joint_->set_angular_velocity(context_.get(), angular_velocity);
  EXPECT_EQ(joint_->get_angular_velocity(*context_), angular_velocity);
  joint_->set_translational_velocity(context_.get(), translational_velocity);
  EXPECT_EQ(joint_->get_translational_velocity(*context_),
            translational_velocity);

  // Joint locking.
  joint_->Lock(context_.get());
  EXPECT_EQ(joint_->get_angular_velocity(*context_), Vector3d(0., 0., 0.));
  EXPECT_EQ(joint_->get_translational_velocity(*context_), Vector3d::Zero());

  // Damping.
  const Vector6d damping =
      (Vector6d() << kAngularDamping, kAngularDamping, kAngularDamping,
       kTranslationalDamping, kTranslationalDamping, kTranslationalDamping)
          .finished();
  const Vector6d different_damping =
      (Vector6d() << 2.3, 2.3, 2.3, 4.5, 4.5, 4.5).finished();
  EXPECT_EQ(joint_->GetDampingVector(*context_), damping);
  EXPECT_NO_THROW(joint_->SetDampingVector(context_.get(), different_damping));
  EXPECT_EQ(joint_->GetDampingVector(*context_), different_damping);

  // Expect to throw on invalid damping values.
  EXPECT_THROW(joint_->SetDampingVector(context_.get(), Vector6d::Constant(-1)),
               std::exception);
}

// Tests API to apply torques to joint.
TEST_F(RpyFloatingJointTest, AddInOneForce) {
  const double some_value = M_PI_2;
  MultibodyForces<double> forces(tree());

  // Since adding forces to individual degrees of freedom of this joint is
  // not supported, this method should throw.
  EXPECT_THROW(joint_->AddInOneForce(*context_, 0, some_value, &forces),
               std::exception);
}

// Tests API to add in damping forces.
TEST_F(RpyFloatingJointTest, AddInDampingForces) {
  const Vector3d angular_velocity(0.1, 0.2, 0.3);
  const Vector3d translational_veloctiy(0.4, 0.5, 0.6);
  const double angular_damping = 3 * kAngularDamping;
  const double translational_damping = 4 * kTranslationalDamping;

  const Vector6d damping_forces_expected =
      (Vector6d() << -angular_damping * angular_velocity,
       -translational_damping * translational_veloctiy)
          .finished();

  joint_->set_angular_velocity(context_.get(), angular_velocity);
  joint_->set_translational_velocity(context_.get(), translational_veloctiy);
  joint_->SetDampingVector(context_.get(),
                           (Vector6d() << Vector3d::Constant(angular_damping),
                            Vector3d::Constant(translational_damping))
                               .finished());

  MultibodyForces<double> forces(tree());
  joint_->AddInDamping(*context_, &forces);
  EXPECT_EQ(forces.generalized_forces(), damping_forces_expected);
}

TEST_F(RpyFloatingJointTest, Clone) {
  auto model_clone = tree().CloneToScalar<AutoDiffXd>();
  const auto& joint_clone1 = dynamic_cast<const RpyFloatingJoint<AutoDiffXd>&>(
      model_clone->get_variant(*joint_));

  const std::unique_ptr<Joint<AutoDiffXd>> shallow =
      joint_clone1.ShallowClone();
  const auto& joint_clone2 =
      dynamic_cast<const RpyFloatingJoint<AutoDiffXd>&>(*shallow);

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
    EXPECT_EQ(clone->default_angular_damping(),
              joint_->default_angular_damping());
    EXPECT_EQ(clone->default_translational_damping(),
              joint_->default_translational_damping());
    EXPECT_EQ(clone->get_default_angles(), joint_->get_default_angles());
    EXPECT_EQ(clone->get_default_translation(),
              joint_->get_default_translation());
  }
}

TEST_F(RpyFloatingJointTest, SetVelocityAndAccelerationLimits) {
  const double new_lower = -0.2;
  const double new_upper = 0.2;
  // Check for velocity limits.
  mutable_joint_->set_velocity_limits(Vector6d::Constant(new_lower),
                                      Vector6d::Constant(new_upper));
  EXPECT_EQ(joint_->velocity_lower_limits(), Vector6d::Constant(new_lower));
  EXPECT_EQ(joint_->velocity_upper_limits(), Vector6d::Constant(new_upper));
  // Does not match num_velocities().
  EXPECT_THROW(mutable_joint_->set_velocity_limits(VectorX<double>(6),
                                                   VectorX<double>()),
               std::exception);
  EXPECT_THROW(mutable_joint_->set_velocity_limits(VectorX<double>(),
                                                   VectorX<double>(6)),
               std::exception);
  // Lower limit is larger than upper limit.
  EXPECT_THROW(mutable_joint_->set_velocity_limits(Vector6d::Constant(2),
                                                   Vector6d::Constant(0)),
               std::exception);

  // Check for acceleration limits.
  mutable_joint_->set_acceleration_limits(Vector6d::Constant(new_lower),
                                          Vector6d::Constant(new_upper));
  EXPECT_EQ(joint_->acceleration_lower_limits(), Vector6d::Constant(new_lower));
  EXPECT_EQ(joint_->acceleration_upper_limits(), Vector6d::Constant(new_upper));
  // Does not match num_velocities().
  EXPECT_THROW(mutable_joint_->set_acceleration_limits(VectorX<double>(6),
                                                       VectorX<double>()),
               std::exception);
  EXPECT_THROW(mutable_joint_->set_acceleration_limits(VectorX<double>(),
                                                       VectorX<double>(6)),
               std::exception);
  // Lower limit is larger than upper limit.
  EXPECT_THROW(mutable_joint_->set_acceleration_limits(Vector6d::Constant(2),
                                                       Vector6d::Constant(0)),
               std::exception);
}

TEST_F(RpyFloatingJointTest, CanRotateOrTranslate) {
  EXPECT_TRUE(joint_->can_rotate());
  EXPECT_TRUE(joint_->can_translate());
}

TEST_F(RpyFloatingJointTest, NameSuffix) {
  EXPECT_EQ(joint_->position_suffix(0), "qx");
  EXPECT_EQ(joint_->position_suffix(1), "qy");
  EXPECT_EQ(joint_->position_suffix(2), "qz");
  EXPECT_EQ(joint_->position_suffix(3), "x");
  EXPECT_EQ(joint_->position_suffix(4), "y");
  EXPECT_EQ(joint_->position_suffix(5), "z");
  EXPECT_EQ(joint_->velocity_suffix(0), "wx");
  EXPECT_EQ(joint_->velocity_suffix(1), "wy");
  EXPECT_EQ(joint_->velocity_suffix(2), "wz");
  EXPECT_EQ(joint_->velocity_suffix(3), "vx");
  EXPECT_EQ(joint_->velocity_suffix(4), "vy");
  EXPECT_EQ(joint_->velocity_suffix(5), "vz");
}

TEST_F(RpyFloatingJointTest, DefaultAnglesAndTranslation) {
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
  EXPECT_NO_THROW(mutable_joint_->set_default_angles(out_of_bounds_low_angles));
  EXPECT_NO_THROW(
      mutable_joint_->set_default_angles(out_of_bounds_high_angles));

  EXPECT_EQ(joint_->get_default_translation(), Vector3d::Zero());
  mutable_joint_->set_default_translation(Vector3d(1.0, 2.0, 3.0));
  EXPECT_EQ(joint_->get_default_translation(), Vector3d(1.0, 2.0, 3.0));

  EXPECT_TRUE(joint_->GetDefaultPose().IsNearlyEqualTo(
      RigidTransformd(RollPitchYawd(joint_->get_default_angles()),
                      joint_->get_default_translation()),
      kTolerance));
}

TEST_F(RpyFloatingJointTest, RandomState) {
  RandomGenerator generator;
  std::uniform_real_distribution<symbolic::Expression> uniform;
  Eigen::Matrix<symbolic::Expression, 3, 1> zero_3(0.0, 0.0, 0.0);

  // Default behavior is to set to zero.
  tree().SetRandomState(*context_, &context_->get_mutable_state(), &generator);
  EXPECT_TRUE(joint_->GetPose(*context_).IsExactlyIdentity());
  // Set the angles & translation distribution to arbitrary values.
  Eigen::Matrix<symbolic::Expression, 3, 1> angles_distribution;
  Eigen::Matrix<symbolic::Expression, 3, 1> translation_distribution;
  for (int i = 0; i < 3; i++) {
    angles_distribution[i] = 0.0125 * (uniform(generator) + i + 1.0);
    translation_distribution[i] = uniform(generator) + i + 1.0;
  }

  mutable_joint_->set_random_angles_distribution(angles_distribution);
  mutable_joint_->set_random_translation_distribution(translation_distribution);
  tree().SetRandomState(*context_, &context_->get_mutable_state(), &generator);
  // We expect arbitrary non-zero values for the random state.
  EXPECT_FALSE(joint_->GetPose(*context_).IsExactlyIdentity());

  // Set position and quaternion distributions back to 0.
  mutable_joint_->set_random_angles_distribution(zero_3);
  mutable_joint_->set_random_translation_distribution(zero_3);
  tree().SetRandomState(*context_, &context_->get_mutable_state(), &generator);
  // We expect zero values for pose.
  EXPECT_TRUE(joint_->GetPose(*context_).IsExactlyIdentity());
}

}  // namespace
}  // namespace multibody
}  // namespace drake
