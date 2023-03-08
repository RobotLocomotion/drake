#include "drake/multibody/tree/space_xyz_floating_joint.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/context.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {
namespace multibody {
namespace {

const double kTolerance = std::numeric_limits<double>::epsilon();

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

class SpaceXYZFloatingJointTest : public ::testing::Test {
 public:
  // Creates a MultibodyTree model of a single free body.
  void SetUp() override {
    // Spatial inertia for adding bodies. The actual value is not important for
    // these tests and therefore we do not initialize it.
    const SpatialInertia<double> M_B;  // Default construction is ok for this.

    // Create an empty model.
    auto model = std::make_unique<internal::MultibodyTree<double>>();

    // Add a body so we can add a joint between world and body:
    body_ = &model->AddBody<RigidBody>("Body", M_B);

    // Add a quaternion floating joint between the world and body:
    joint_ = &model->AddJoint<SpaceXYZFloatingJoint>(
        "Joint", model->world_body(), std::nullopt, *body_, std::nullopt,
        kAngularDamping, kTranslationalDamping);
    mutable_joint_ = dynamic_cast<SpaceXYZFloatingJoint<double>*>(
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
  const SpaceXYZFloatingJoint<double>* joint_{nullptr};
  SpaceXYZFloatingJoint<double>* mutable_joint_{nullptr};
};

TEST_F(SpaceXYZFloatingJointTest, Type) {
  const Joint<double>& base = *joint_;
  EXPECT_EQ(base.type_name(), SpaceXYZFloatingJoint<double>::kTypeName);
}

// Verify the expected number of dofs.
TEST_F(SpaceXYZFloatingJointTest, NumDOFs) {
  EXPECT_EQ(tree().num_positions(), 6);
  EXPECT_EQ(tree().num_velocities(), 6);
  EXPECT_EQ(joint_->num_positions(), 6);
  EXPECT_EQ(joint_->num_velocities(), 6);
  EXPECT_EQ(joint_->position_start(), 0);
  EXPECT_EQ(joint_->velocity_start(), 0);
}

TEST_F(SpaceXYZFloatingJointTest, GetJointLimits) {
  EXPECT_EQ(joint_->position_lower_limits().size(), 6);
  EXPECT_EQ(joint_->position_upper_limits().size(), 6);
  EXPECT_EQ(joint_->velocity_lower_limits().size(), 6);
  EXPECT_EQ(joint_->velocity_upper_limits().size(), 6);
  EXPECT_EQ(joint_->acceleration_lower_limits().size(), 6);
  EXPECT_EQ(joint_->acceleration_upper_limits().size(), 6);

  EXPECT_EQ(joint_->position_lower_limits(),
            (Vector6d::Constant(kPositionLowerLimit)));
  EXPECT_EQ(joint_->position_upper_limits(),
            (Vector6d::Constant(kPositionUpperLimit)));
  EXPECT_EQ(joint_->velocity_lower_limits(),
            Vector6d::Constant(kVelocityLowerLimit));
  EXPECT_EQ(joint_->velocity_upper_limits(),
            Vector6d::Constant(kVelocityUpperLimit));
  EXPECT_EQ(joint_->acceleration_lower_limits(),
            Vector6d::Constant(kAccelerationLowerLimit));
  EXPECT_EQ(joint_->acceleration_upper_limits(),
            Vector6d::Constant(kAccelerationUpperLimit));
}

TEST_F(SpaceXYZFloatingJointTest, Damping) {
  EXPECT_EQ(joint_->angular_damping(), kAngularDamping);
  EXPECT_EQ(joint_->translational_damping(), kTranslationalDamping);
  EXPECT_EQ(
      joint_->damping_vector(),
      (Vector6d() << kAngularDamping, kAngularDamping, kAngularDamping,
       kTranslationalDamping, kTranslationalDamping, kTranslationalDamping)
          .finished());
}

// Context-dependent value access.
TEST_F(SpaceXYZFloatingJointTest, ContextDependentAccess) {
  const Vector3d position(1., 2., 3.);
  const Vector3d angular_velocity(0.5, 0.5, 0.5);
  const Vector3d translational_veloctiy(0.1, 0.2, 0.3);
  const Vector3d rpy_A(1., 2., 3.);
  const Vector3d rpy_B(5., 6., 7.);
  const RigidTransformd transform_A(RollPitchYawd(rpy_A), position);
  const RotationMatrixd rotation_matrix_B =
      RotationMatrixd(RollPitchYawd(rpy_B));

  // Position access:
  joint_->set_angles(context_.get(), rpy_A);
  EXPECT_EQ(joint_->get_angles(*context_), rpy_A);

  joint_->SetFromRotationMatrix(context_.get(), rotation_matrix_B);
  // We define an RMS error to account for multiple entries in the rotations.
  const double rms_error =
      (RotationMatrixd(RollPitchYawd(joint_->get_angles(*context_))).matrix() -
       rotation_matrix_B.matrix())
          .norm() /
      3.0;
  EXPECT_LT(rms_error, kTolerance);

  joint_->set_position(context_.get(), position);
  EXPECT_EQ(joint_->get_position(*context_), position);

  joint_->set_pose(context_.get(), transform_A);
  // We expect a bit of roundoff error due to transforming between space-xyz
  // and rotation matrix representations.
  EXPECT_TRUE(
      joint_->get_pose(*context_).IsNearlyEqualTo(transform_A, kTolerance));

  // Angular velocity access:
  joint_->set_angular_velocity(context_.get(), angular_velocity);
  EXPECT_EQ(joint_->get_angular_velocity(*context_), angular_velocity);
  joint_->set_translational_velocity(context_.get(), translational_veloctiy);
  EXPECT_EQ(joint_->get_translational_velocity(*context_),
            translational_veloctiy);

  // Joint locking.
  joint_->Lock(context_.get());
  EXPECT_EQ(joint_->get_angular_velocity(*context_), Vector3d::Zero());
  EXPECT_EQ(joint_->get_translational_velocity(*context_), Vector3d::Zero());
}

// Tests API to apply torques to joint.
TEST_F(SpaceXYZFloatingJointTest, AddInOneForce) {
  const double some_value = M_PI_2;
  MultibodyForces<double> forces(tree());

  // Since adding forces to individual degrees of freedom of this joint does
  // not make physical sense, this method should throw.
  EXPECT_THROW(joint_->AddInOneForce(*context_, 0, some_value, &forces),
               std::logic_error);
}

TEST_F(SpaceXYZFloatingJointTest, Clone) {
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
  EXPECT_EQ(joint_clone.angular_damping(), joint_->angular_damping());
  EXPECT_EQ(joint_clone.translational_damping(),
            joint_->translational_damping());
  EXPECT_EQ(joint_clone.get_default_angles(), joint_->get_default_angles());
  EXPECT_EQ(joint_clone.get_default_position(), joint_->get_default_position());
}

#if 0

TEST_F(SpaceXYZFloatingJointTest, SetVelocityAndAccelerationLimits) {
  const double new_lower = -0.2;
  const double new_upper = 0.2;
  // Check for velocity limits.
  mutable_joint_->set_velocity_limits(Vector6d::Constant(new_lower),
                                      Vector6d::Constant(new_upper));
  EXPECT_EQ(joint_->velocity_lower_limits(), Vector6d::Constant(new_lower));
  EXPECT_EQ(joint_->velocity_upper_limits(), Vector6d::Constant(new_upper));
  // Does not match num_velocities().
  EXPECT_THROW(mutable_joint_->set_velocity_limits(VectorX<double>(3),
                                                   VectorX<double>()),
               std::runtime_error);
  EXPECT_THROW(mutable_joint_->set_velocity_limits(VectorX<double>(),
                                                   VectorX<double>(3)),
               std::runtime_error);
  // Lower limit is larger than upper limit.
  EXPECT_THROW(mutable_joint_->set_velocity_limits(Vector6d::Constant(2),
                                                   Vector6d::Constant(0)),
               std::runtime_error);

  // Check for acceleration limits.
  mutable_joint_->set_acceleration_limits(Vector6d::Constant(new_lower),
                                          Vector6d::Constant(new_upper));
  EXPECT_EQ(joint_->acceleration_lower_limits(), Vector6d::Constant(new_lower));
  EXPECT_EQ(joint_->acceleration_upper_limits(), Vector6d::Constant(new_upper));
  // Does not match num_velocities().
  EXPECT_THROW(mutable_joint_->set_acceleration_limits(VectorX<double>(3),
                                                       VectorX<double>()),
               std::runtime_error);
  EXPECT_THROW(mutable_joint_->set_acceleration_limits(VectorX<double>(),
                                                       VectorX<double>(3)),
               std::runtime_error);
  // Lower limit is larger than upper limit.
  EXPECT_THROW(mutable_joint_->set_acceleration_limits(Vector6d::Constant(2),
                                                       Vector6d::Constant(0)),
               std::runtime_error);
}

TEST_F(SpaceXYZFloatingJointTest, CanRotateOrTranslate) {
  EXPECT_TRUE(joint_->can_rotate());
  EXPECT_TRUE(joint_->can_translate());
}

TEST_F(SpaceXYZFloatingJointTest, NameSuffix) {
  EXPECT_EQ(joint_->position_suffix(0), "qw");
  EXPECT_EQ(joint_->position_suffix(1), "qx");
  EXPECT_EQ(joint_->position_suffix(2), "qy");
  EXPECT_EQ(joint_->position_suffix(3), "qz");
  EXPECT_EQ(joint_->position_suffix(4), "x");
  EXPECT_EQ(joint_->position_suffix(5), "y");
  EXPECT_EQ(joint_->position_suffix(6), "z");
  EXPECT_EQ(joint_->velocity_suffix(0), "wx");
  EXPECT_EQ(joint_->velocity_suffix(1), "wy");
  EXPECT_EQ(joint_->velocity_suffix(2), "wz");
  EXPECT_EQ(joint_->velocity_suffix(3), "vx");
  EXPECT_EQ(joint_->velocity_suffix(4), "vy");
  EXPECT_EQ(joint_->velocity_suffix(5), "vz");
}

TEST_F(SpaceXYZFloatingJointTest, DefaultAngles) {
  const Vector6d lower_limit_angles = Vector6d::Constant(kPositionLowerLimit);
  const Vector6d upper_limit_angles = Vector6d::Constant(kPositionUpperLimit);

  const Vector6d default_angles = Vector6d::Identity();

  const Vector6d new_default_angles =
      Vector6d::Constant(kPositionNonZeroDefault);

  const Vector6d out_of_bounds_low_angles =
      lower_limit_angles - Vector6d::Constant(1);
  const Vector6d out_of_bounds_high_angles =
      upper_limit_angles + Vector6d::Constant(1);

  // Constructor should set the default angle to Vector3d::Zero()
  EXPECT_EQ(joint_->default_positions(), default_angles);

  // Setting a new default angle should propagate so that `get_default_angle()`
  // remains correct.
  mutable_joint_->set_default_positions(new_default_angles);
  EXPECT_EQ(joint_->default_positions(), new_default_angles);

  // Setting the default angle out of the bounds of the position limits
  // should NOT throw an exception
  EXPECT_NO_THROW(
      mutable_joint_->set_default_positions(out_of_bounds_low_angles));
  EXPECT_NO_THROW(
      mutable_joint_->set_default_positions(out_of_bounds_high_angles));
}

TEST_F(SpaceXYZFloatingJointTest, RandomState) {
  RandomGenerator generator;
  std::uniform_real_distribution<symbolic::Expression> uniform;

  // Default behavior is to set to zero.
  tree().SetRandomState(*context_, &context_->get_mutable_state(),
                           &generator);
  EXPECT_TRUE(joint_->get_pose(*context_).IsExactlyIdentity());
  // Set the position distribution to arbitrary values.
  Eigen::Matrix<symbolic::Expression, 3, 1> position_distribution;
  for (int i = 0; i < 3; i++) {
    position_distribution[i] = uniform(generator) + i + 1.0;
  }

  mutable_joint_->set_random_quaternion_distribution(
      math::UniformlyRandomQuaternion<symbolic::Expression>(&generator));
  mutable_joint_->set_random_position_distribution(position_distribution);
  tree().SetRandomState(*context_, &context_->get_mutable_state(),
                           &generator);
  // We expect arbitrary non-zero values for the random state.
  EXPECT_FALSE(joint_->get_pose(*context_).IsExactlyIdentity());

  // Set position and quaternion distributions back to 0.
  mutable_joint_->set_random_quaternion_distribution(
      Eigen::Quaternion<symbolic::Expression>::Identity());
  mutable_joint_->set_random_position_distribution(
      Eigen::Matrix<symbolic::Expression, 3, 1>::Zero());
  tree().SetRandomState(*context_, &context_->get_mutable_state(),
                           &generator);
  // We expect zero values for pose.
  EXPECT_TRUE(joint_->get_pose(*context_).IsExactlyIdentity());

  // Set the quaternion distribution using built in uniform sampling.
  mutable_joint_->set_random_quaternion_distribution_to_uniform();
  tree().SetRandomState(*context_, &context_->get_mutable_state(),
                           &generator);
  // We expect arbitrary non-zero pose.
  EXPECT_FALSE(joint_->get_pose(*context_).IsExactlyIdentity());
}

#endif

}  // namespace
}  // namespace multibody
}  // namespace drake
