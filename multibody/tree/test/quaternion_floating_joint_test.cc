// clang-format: off
#include "drake/multibody/tree/multibody_tree-inl.h"
// clang-format: on

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

const double kTolerance = std::numeric_limits<double>::epsilon();

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;
using systems::Context;

using Vector7d = Vector<double, 7>;

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

class QuaternionFloatingJointTest : public ::testing::Test {
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
    joint_ = &model->AddJoint<QuaternionFloatingJoint>(
        "Joint", model->world_body(), std::nullopt, *body_, std::nullopt,
        kAngularDamping, kTranslationalDamping);
    mutable_joint_ = dynamic_cast<QuaternionFloatingJoint<double>*>(
        &model->get_mutable_joint(joint_->index()));
    DRAKE_DEMAND(mutable_joint_ != nullptr);
    mutable_joint_->set_position_limits(
        Vector7d::Constant(kPositionLowerLimit),
        Vector7d::Constant(kPositionUpperLimit));
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
  const QuaternionFloatingJoint<double>* joint_{nullptr};
  QuaternionFloatingJoint<double>* mutable_joint_{nullptr};
};

TEST_F(QuaternionFloatingJointTest, Type) {
  const Joint<double>& base = *joint_;
  EXPECT_EQ(base.type_name(), QuaternionFloatingJoint<double>::kTypeName);
}

// Verify the expected number of dofs.
TEST_F(QuaternionFloatingJointTest, NumDOFs) {
  EXPECT_EQ(tree().num_positions(), 7);
  EXPECT_EQ(tree().num_velocities(), 6);
  EXPECT_EQ(joint_->num_positions(), 7);
  EXPECT_EQ(joint_->num_velocities(), 6);
  EXPECT_EQ(joint_->position_start(), 0);
  EXPECT_EQ(joint_->velocity_start(), 0);
}

TEST_F(QuaternionFloatingJointTest, GetJointLimits) {
  EXPECT_EQ(joint_->position_lower_limits().size(), 7);
  EXPECT_EQ(joint_->position_upper_limits().size(), 7);
  EXPECT_EQ(joint_->velocity_lower_limits().size(), 6);
  EXPECT_EQ(joint_->velocity_upper_limits().size(), 6);
  EXPECT_EQ(joint_->acceleration_lower_limits().size(), 6);
  EXPECT_EQ(joint_->acceleration_upper_limits().size(), 6);

  EXPECT_EQ(joint_->position_lower_limits(),
            (Vector7d::Constant(kPositionLowerLimit)));
  EXPECT_EQ(joint_->position_upper_limits(),
            (Vector7d::Constant(kPositionUpperLimit)));
  EXPECT_EQ(joint_->velocity_lower_limits(),
            Vector6d::Constant(kVelocityLowerLimit));
  EXPECT_EQ(joint_->velocity_upper_limits(),
            Vector6d::Constant(kVelocityUpperLimit));
  EXPECT_EQ(joint_->acceleration_lower_limits(),
            Vector6d::Constant(kAccelerationLowerLimit));
  EXPECT_EQ(joint_->acceleration_upper_limits(),
            Vector6d::Constant(kAccelerationUpperLimit));
}

TEST_F(QuaternionFloatingJointTest, Damping) {
  EXPECT_EQ(joint_->angular_damping(), kAngularDamping);
  EXPECT_EQ(joint_->translational_damping(), kTranslationalDamping);
  EXPECT_EQ(
      joint_->damping_vector(),
      (Vector6d() << kAngularDamping, kAngularDamping, kAngularDamping,
       kTranslationalDamping, kTranslationalDamping, kTranslationalDamping)
          .finished());
}

// Context-dependent value access.
TEST_F(QuaternionFloatingJointTest, ContextDependentAccess) {
  const Vector3d position(1., 2., 3.);
  const Vector3d angular_velocity(0.5, 0.5, 0.5);
  const Vector3d translational_veloctiy(0.1, 0.2, 0.3);
  Quaternion<double> rotation_A(1., 2., 3., 4.);
  Quaternion<double> rotation_B(5., 6., 7., 8.);
  rotation_A.normalize();
  rotation_B.normalize();
  const RigidTransformd pose(rotation_A, position);
  const RotationMatrixd rotation_matrix(rotation_B);

  // Position access:
  joint_->set_quaternion(context_.get(), rotation_A);
  EXPECT_EQ(joint_->get_quaternion(*context_).coeffs(), rotation_A.coeffs());

  joint_->SetFromRotationMatrix(context_.get(), rotation_matrix);
  EXPECT_TRUE(math::AreQuaternionsEqualForOrientation(
      joint_->get_quaternion(*context_), rotation_B, kTolerance));

  joint_->set_position(context_.get(), position);
  EXPECT_EQ(joint_->get_position(*context_), position);

  joint_->set_position(context_.get(), Vector3d::Zero());  // Zero out pose.
  joint_->set_pose(context_.get(), pose);
  // We expect a bit of roundoff error due to transforming between quaternion
  // and rotation matrix representations.
  EXPECT_TRUE(joint_->get_pose(*context_).IsNearlyEqualTo(pose, kTolerance));

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
TEST_F(QuaternionFloatingJointTest, AddInOneForce) {
  const double some_value = M_PI_2;
  MultibodyForces<double> forces(tree());

  // Since adding forces to individual degrees of freedom of this joint does
  // not make physical sense, this method should throw.
  EXPECT_THROW(joint_->AddInOneForce(*context_, 0, some_value, &forces),
               std::logic_error);
}

TEST_F(QuaternionFloatingJointTest, Clone) {
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
  EXPECT_EQ(joint_clone.get_default_quaternion().coeffs(),
            joint_->get_default_quaternion().coeffs());
  EXPECT_EQ(joint_clone.get_default_position(), joint_->get_default_position());
}

TEST_F(QuaternionFloatingJointTest, SetVelocityAndAccelerationLimits) {
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

TEST_F(QuaternionFloatingJointTest, CanRotateOrTranslate) {
  EXPECT_TRUE(joint_->can_rotate());
  EXPECT_TRUE(joint_->can_translate());
}

TEST_F(QuaternionFloatingJointTest, NameSuffix) {
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

TEST_F(QuaternionFloatingJointTest, DefaultAngles) {
  const Vector7d lower_limit_angles = Vector7d::Constant(kPositionLowerLimit);
  const Vector7d upper_limit_angles = Vector7d::Constant(kPositionUpperLimit);

  const Vector7d default_angles = Vector7d::Zero();

  const Vector7d new_default_angles =
      Vector7d::Constant(kPositionNonZeroDefault);

  const Vector7d out_of_bounds_low_angles =
      lower_limit_angles - Vector7d::Constant(1);
  const Vector7d out_of_bounds_high_angles =
      upper_limit_angles + Vector7d::Constant(1);

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

}  // namespace
}  // namespace multibody
}  // namespace drake
