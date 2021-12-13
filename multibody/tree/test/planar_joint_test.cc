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
  // Creates a MultibodyTree model with a body attached to the world via a
  // planar joint.
  void SetUp() override {
    // Spatial inertia for adding bodies. The actual value is not important for
    // these tests and therefore we do not initialize it.
    const SpatialInertia<double> M_B;  // Default construction is ok for this.

    // Create an empty model.
    auto model = std::make_unique<internal::MultibodyTree<double>>();

    // Add a body so we can add a joint between it and the world:
    body_ = &model->AddBody<RigidBody>(M_B);

    // Add a planar joint between the world and body1:
    joint_ = &model->AddJoint<PlanarJoint>("Joint", model->world_body(),
                                           std::nullopt, *body_, std::nullopt,
                                           Vector3d::Constant(kDamping));
    mutable_joint_ = dynamic_cast<PlanarJoint<double>*>(
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
  const PlanarJoint<double>* joint_{nullptr};
  PlanarJoint<double>* mutable_joint_{nullptr};
};

TEST_F(PlanarJointTest, Type) {
  const Joint<double>& base = *joint_;
  EXPECT_EQ(base.type_name(), "planar");
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
  const Vector2d translation1(1, 0.3);
  const double angle1 = 1.5;
  const Vector2d translation2(2, 2.3);
  const double angle2 = 3;
  // Position access:
  joint_->set_translation(context_.get(), translation1);
  EXPECT_EQ(joint_->get_translation(*context_), translation1);
  joint_->set_rotation(context_.get(), angle1);
  EXPECT_EQ(joint_->get_rotation(*context_), angle1);
  joint_->set_pose(context_.get(), translation2, angle2);
  EXPECT_EQ(joint_->get_translation(*context_), translation2);
  EXPECT_EQ(joint_->get_rotation(*context_), angle2);

  // Velocity access:
  joint_->set_translational_velocity(context_.get(), translation1);
  EXPECT_EQ(joint_->get_translational_velocity(*context_), translation1);
  joint_->set_angular_velocity(context_.get(), angle1);
  EXPECT_EQ(joint_->get_angular_velocity(*context_), angle1);

  // Joint locking.
  joint_->Lock(context_.get());
  EXPECT_EQ(joint_->get_translational_velocity(*context_), Vector2d(0., 0.));
  EXPECT_EQ(joint_->get_angular_velocity(*context_), 0.);
}

// Tests API to apply torques to individual dof of joint. Ensures that adding
// forces individually is equivalent to adding forces collectively.
TEST_F(PlanarJointTest, AddInOneForce) {
  const double force_value = M_PI_2;
  const double kEpsilon = std::numeric_limits<double>::epsilon();
  MultibodyForces<double> forces1(tree());
  MultibodyForces<double> forces2(tree());

  for (int ii = 0; ii < joint_->num_positions(); ii++) {
    // Add value twice:
    joint_->AddInOneForce(*context_, ii, force_value, &forces1);
    joint_->AddInOneForce(*context_, ii, force_value, &forces1);
    // Add value only once:
    joint_->AddInOneForce(*context_, ii, force_value, &forces2);
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
  EXPECT_EQ(joint_clone.get_default_rotation(), joint_->get_default_rotation());
  EXPECT_EQ(joint_clone.get_default_translation(),
            joint_->get_default_translation());
}

TEST_F(PlanarJointTest, NameSuffix) {
  EXPECT_EQ(joint_->position_suffix(0), "x");
  EXPECT_EQ(joint_->position_suffix(1), "y");
  EXPECT_EQ(joint_->position_suffix(2), "qz");
  EXPECT_EQ(joint_->velocity_suffix(0), "vx");
  EXPECT_EQ(joint_->velocity_suffix(1), "vy");
  EXPECT_EQ(joint_->velocity_suffix(2), "wz");
}

TEST_F(PlanarJointTest, DefaultState) {
  const Vector2d new_default_translation =
      Vector2d::Constant(kPositionNonZeroDefault);
  const Vector2d out_of_bounds_low_translation =
      Vector2d::Constant(kPositionLowerLimit - 1);
  const Vector2d out_of_bounds_high_translation =
      Vector2d::Constant(kPositionUpperLimit + 1);

  const double new_default_rotation = kPositionNonZeroDefault;
  const double out_of_bounds_low_rotation = kPositionLowerLimit - 1;
  const double out_of_bounds_high_rotation = kPositionUpperLimit + 1;

  // Constructor should set the default translation to Vector2d::Zero() and
  // rotation to zero.
  EXPECT_EQ(joint_->get_default_translation(), Vector2d::Zero());
  EXPECT_EQ(joint_->get_default_rotation(), 0);

  // Setting a new default translation should propagate so that
  // `get_default_translation()` remains correct.
  mutable_joint_->set_default_translation(new_default_translation);
  EXPECT_EQ(joint_->get_default_translation(), new_default_translation);

  // Setting a new default rotation should propagate so that
  // `get_default_rotation()` remains correct.
  mutable_joint_->set_default_rotation(new_default_rotation);
  EXPECT_EQ(joint_->get_default_rotation(), new_default_rotation);

  // Setting the default translation or rotation out of the bounds of the limits
  // should NOT throw an exception.
  EXPECT_NO_THROW(
      mutable_joint_->set_default_translation(out_of_bounds_low_translation));
  EXPECT_NO_THROW(
      mutable_joint_->set_default_translation(out_of_bounds_high_translation));
  EXPECT_NO_THROW(
      mutable_joint_->set_default_rotation(out_of_bounds_low_rotation));
  EXPECT_NO_THROW(
      mutable_joint_->set_default_rotation(out_of_bounds_high_rotation));

  // Setting a new default pose should propagate so that
  // `get_default_translation()` and `get_default_rotation()` remain correct.
  mutable_joint_->set_default_pose(new_default_translation,
                                   new_default_rotation);
  EXPECT_EQ(joint_->get_default_translation(), new_default_translation);
  EXPECT_EQ(joint_->get_default_rotation(), new_default_rotation);
}

TEST_F(PlanarJointTest, RandomState) {
  RandomGenerator generator;
  std::uniform_real_distribution<symbolic::Expression> uniform;

  tree().SetRandomState(*context_, &context_->get_mutable_state(), &generator);

  EXPECT_EQ(joint_->get_translation(*context_), Vector2d::Zero());
  EXPECT_EQ(joint_->get_rotation(*context_), 0);

  mutable_joint_->set_random_pose_distribution(
      Vector2<symbolic::Expression>(uniform(generator) + 1.0,
                                    uniform(generator) + 2.0),
      uniform(generator) + 3.0);
  tree().SetRandomState(*context_, &context_->get_mutable_state(), &generator);
  EXPECT_GE(mutable_joint_->get_translation(*context_)[0], 1.0);
  EXPECT_LE(mutable_joint_->get_translation(*context_)[0], 2.0);
  EXPECT_GE(mutable_joint_->get_translation(*context_)[1], 2.0);
  EXPECT_LE(mutable_joint_->get_translation(*context_)[1], 3.0);
  EXPECT_GE(mutable_joint_->get_rotation(*context_), 3.0);
  EXPECT_LE(mutable_joint_->get_rotation(*context_), 4.0);

  // Check that they change on a second draw from the distribution.
  const Vector2d last_translation = mutable_joint_->get_translation(*context_);
  const double last_rotation = mutable_joint_->get_rotation(*context_);
  tree().SetRandomState(*context_, &context_->get_mutable_state(), &generator);
  EXPECT_NE(mutable_joint_->get_translation(*context_), last_translation);
  EXPECT_NE(mutable_joint_->get_rotation(*context_), last_rotation);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
