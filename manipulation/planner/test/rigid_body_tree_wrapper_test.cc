#include "drake/manipulation/planner/rigid_body_tree_wrapper.h"

#include <limits>
#include <memory>
#include <random>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/manipulation/planner/kinematic_tree.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/math/transform.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/joints/roll_pitch_yaw_floating_joint.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/constraint.h"

using drake::math::Transform;
using drake::solvers::Constraint;
using drake::manipulation::util::WorldSimTreeBuilder;
using drake::multibody::joints::FloatingBaseType;
using testing::Return;
using testing::_;

namespace drake {
namespace manipulation {
namespace planner {
namespace {

std::unique_ptr<RigidBodyTree<double>> MakeRigidBodyTreeForTesting() {
  const std::string table_model_path = drake::FindResourceOrThrow(
      "drake/examples/kuka_iiwa_arm/models/table/extra_heavy_duty_table.sdf");
  WorldSimTreeBuilder<double> tree_builder;
  tree_builder.StoreDrakeModel(
      "iiwa",
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_spheres_collision.urdf");
  tree_builder.StoreDrakeModel(
      "table",
      "drake/examples/kuka_iiwa_arm/models/table/extra_heavy_duty_table.sdf");
  tree_builder.AddFixedModelInstance("iiwa", {0, 0, 0});
  tree_builder.AddFixedModelInstance("table", {0, -0.75, -0.3});
  return tree_builder.Build();
}

// Test fixture for RigidBodyTreeWrapper.
class RigidBodyTreeWrapperTests : public testing::TestWithParam<bool> {
 public:
  RigidBodyTreeWrapperTests() {
    tree_ = MakeRigidBodyTreeForTesting();
    bool owns_tree = GetParam();
    if (owns_tree) {
      wrapper_ =
          std::make_unique<RigidBodyTreeWrapper>(MakeRigidBodyTreeForTesting());
    } else {
      wrapper_ = std::make_unique<RigidBodyTreeWrapper>(tree_.get());
    }
  }

 protected:
  std::unique_ptr<RigidBodyTree<double>> tree_;
  std::unique_ptr<RigidBodyTreeWrapper> wrapper_;
};

TEST_P(RigidBodyTreeWrapperTests, NumPositionsTest) {
  EXPECT_EQ(this->wrapper_->num_positions(), this->tree_->get_num_positions());
}

TEST_P(RigidBodyTreeWrapperTests, NumVelocitiesTest) {
  EXPECT_EQ(this->wrapper_->num_velocities(),
            this->tree_->get_num_velocities());
}

TEST_P(RigidBodyTreeWrapperTests, PositionStartIndexForBody) {
  for (const auto& body : this->tree_->get_bodies()) {
    EXPECT_EQ(this->wrapper_->PositionStartIndexForBody(body->get_name()),
              body->get_position_start_index());
  }
}

// Verify that the default values of the joint position lower limits match those
// in the RigidBodyTree.
TEST_P(RigidBodyTreeWrapperTests, joint_position_lower_limit_default) {
  EXPECT_TRUE(CompareMatrices(this->wrapper_->joint_position_lower_limit(),
                              this->tree_->joint_limit_min));
}

// Verify that the default values of the joint position upper limits match those
// in the RigidBodyTree.
TEST_P(RigidBodyTreeWrapperTests, joint_position_upper_limit_default) {
  EXPECT_TRUE(CompareMatrices(this->wrapper_->joint_position_upper_limit(),
                              this->tree_->joint_limit_max));
}

// Verify that the default values of the joint velocity lower limits are 0.
TEST_P(RigidBodyTreeWrapperTests, joint_velocity_lower_limit_default) {
  for (int i = 0; i < this->wrapper_->num_velocities(); ++i) {
    EXPECT_EQ(this->wrapper_->joint_velocity_lower_limit()(i), 0);
  }
}

// Verify that the default values of the joint velocity upper limits are 0.
TEST_P(RigidBodyTreeWrapperTests, joint_velocity_upper_limit_default) {
  for (int i = 0; i < this->wrapper_->num_velocities(); ++i) {
    EXPECT_EQ(this->wrapper_->joint_velocity_upper_limit()(i), 0);
  }
}

TEST_P(RigidBodyTreeWrapperTests, GetZeroConfigurationTest) {
  EXPECT_TRUE(CompareMatrices(this->wrapper_->GetZeroConfiguration(),
                              this->tree_->getZeroConfiguration()));
}

// Verify that, with default position limits, GetRandomConfiguration() returns
// the same result as the corresponding method of the wrapped object when called
// with an equivalent random number generator.
TEST_P(RigidBodyTreeWrapperTests, GetRandomConfigurationDefaultTest) {
  std::default_random_engine generator0{1234};
  std::default_random_engine generator1{1234};
  EXPECT_TRUE(
      CompareMatrices(this->wrapper_->GetRandomConfiguration(&generator0),
                      this->tree_->getRandomConfiguration(generator1)));
}

// Verify that GetRandomConfiguration() returns a different result than that
// returned by the corresponding method of the wrapped object when the joint
// position limits have been modified.
TEST_P(RigidBodyTreeWrapperTests, GetRandomConfigurationModifiedLimitsTest) {
  std::default_random_engine generator0{1234};
  std::default_random_engine generator1{1234};

  // Change the joint position limits for one joint.
  this->wrapper_->SetJointPositionLimits(0, 0, 0);

  VectorX<double> random_configuration =
      this->wrapper_->GetRandomConfiguration(&generator0);
  EXPECT_FALSE(CompareMatrices(
      random_configuration, this->tree_->getRandomConfiguration(generator1)));

  // Check that random_configuration respects the modified limits.
  for (int i = 0; i < this->wrapper_->num_positions(); ++i) {
    EXPECT_GE(random_configuration(i),
              this->wrapper_->joint_position_lower_limit()(i));
    EXPECT_LE(random_configuration(i),
              this->wrapper_->joint_position_upper_limit()(i));
  }
}

// Verify that the wrapper computes the same relative transforms as the wrapped
// object for all pairs of frames.
TEST_P(RigidBodyTreeWrapperTests, CalcRelativeTransformTest) {
  std::default_random_engine generator{1234};
  VectorX<double> q = this->wrapper_->GetRandomConfiguration(&generator);
  drake::log()->debug("q = [{}]", q.transpose());
  auto cache = this->tree_->doKinematics(q);
  for (const auto& frame_A : this->tree_->get_frames()) {
    for (const auto& frame_B : this->tree_->get_frames()) {
      drake::log()->debug("Frame A: {}\tFrame B: {}", frame_A->get_name(),
                          frame_B->get_name());
      Transform<double> X_WA_expected(
          this->tree_->CalcFramePoseInWorldFrame(cache, *frame_A));
      Transform<double> X_WB_expected(
          this->tree_->CalcFramePoseInWorldFrame(cache, *frame_B));
      Transform<double> X_AB_expected = X_WA_expected.inverse() * X_WB_expected;
      Transform<double> X_AB_actual = this->wrapper_->CalcRelativeTransform(
          q, frame_A->get_name(), frame_B->get_name());
      EXPECT_TRUE(X_AB_actual
                      .IsNearlyEqualTo(X_AB_expected,
                                       std::numeric_limits<double>::epsilon())
                      .value());
    }
  }
}

// Verify that the collision avoidance constraint returned by the wrapper has
// qualitatively correct behavior.
TEST_P(RigidBodyTreeWrapperTests, MakeCollisionAvoidanceConstraintTest) {
  std::shared_ptr<Constraint> collision_avoidance_constraint =
      this->wrapper_->MakeCollisionAvoidanceConstraint(
          0.02 /* collision_avoidance_threshold */);
  // Check that the constraint is satisfied in a configuration known to be
  // collision free.
  EXPECT_TRUE(collision_avoidance_constraint->CheckSatisfied(
      this->wrapper_->GetZeroConfiguration()));

  // Check that the constraint is not satisfied in a configuration known to
  // cause a collision.
  VectorX<double> q_collision = this->wrapper_->GetZeroConfiguration();
  q_collision(0) = -M_PI_2;
  q_collision(1) = M_PI_2;

  EXPECT_FALSE(collision_avoidance_constraint->CheckSatisfied(q_collision));
}

// Verify that relative pose constraints returned by the wrapper have
// qualitatively correct behavior.
TEST_P(RigidBodyTreeWrapperTests, MakeRelativePoseConstraintTest) {
  std::default_random_engine generator{1234};
  VectorX<double> q_0 = this->wrapper_->GetRandomConfiguration(&generator);
  VectorX<double> q_1 = this->wrapper_->GetRandomConfiguration(&generator);
  for (const auto& frame_A : this->tree_->get_frames()) {
    for (const auto& frame_B : this->tree_->get_frames()) {
      Transform<double> X_AB_0 = this->wrapper_->CalcRelativeTransform(
          q_0, frame_A->get_name(), frame_B->get_name());
      Transform<double> X_AB_1 = this->wrapper_->CalcRelativeTransform(
          q_1, frame_A->get_name(), frame_B->get_name());
      std::shared_ptr<Constraint> relative_pose_constraint =
          this->wrapper_->MakeRelativePoseConstraint(
              frame_A->get_name(), frame_B->get_name(), X_AB_0);
      drake::log()->debug("Frame A: {}\tFrame B: {}", frame_A->get_name(),
                          frame_B->get_name());
      drake::log()->debug("X_AB_0:\n{}", X_AB_0.GetAsMatrix4());
      drake::log()->debug("X_AB_1:\n{}", X_AB_1.GetAsMatrix4());
      EXPECT_TRUE(relative_pose_constraint->CheckSatisfied(q_0));
      if (X_AB_1
              .IsNearlyEqualTo(X_AB_0,
                               10 * std::numeric_limits<double>::epsilon())
              .value()) {
        EXPECT_TRUE(relative_pose_constraint->CheckSatisfied(q_1));
      } else {
        EXPECT_FALSE(relative_pose_constraint->CheckSatisfied(q_1));
      }
    }
  }
}

std::unique_ptr<RigidBodyTree<double>> MakeUncompiledRobotWithFloatingBody() {
  std::unique_ptr<RigidBodyTree<double>> tree =
      std::make_unique<RigidBodyTree<double>>();
  auto body = std::make_unique<RigidBody<double>>();
  body->set_spatial_inertia(drake::SquareTwistMatrix<double>::Identity());
  body->set_name("floating_body");
  Transform<double> X_WF{};
  body->add_joint(&tree->world(), std::make_unique<RollPitchYawFloatingJoint>(
                                      "floating_joint", X_WF.GetAsIsometry3()));
  tree->add_rigid_body(std::move(body));
  return tree;
}

// Verify that a std::logic error is thrown if an uninitialized RigidBodyTree is
// passed to the constructor.
TEST_P(RigidBodyTreeWrapperTests, ThrowOnUninitializedRigidBodyTreeTest) {
  std::unique_ptr<RigidBodyTree<double>> tree =
      MakeUncompiledRobotWithFloatingBody();
  bool owns_tree = GetParam();
  if (owns_tree) {
    EXPECT_THROW(RigidBodyTreeWrapper(std::move(tree)), std::logic_error);
  } else {
    EXPECT_THROW(RigidBodyTreeWrapper(tree.get()), std::logic_error);
  }
}

// Verify that a std::logic error is thrown if a RigidBodyTree with non-finite
// joint limits is passed to the constructor.
TEST_P(RigidBodyTreeWrapperTests, ThrowOnNonFiniteJointLimitsTest) {
  std::unique_ptr<RigidBodyTree<double>> tree =
      MakeUncompiledRobotWithFloatingBody();
  tree->compile();
  bool owns_tree = GetParam();
  if (owns_tree) {
    EXPECT_THROW(RigidBodyTreeWrapper(std::move(tree)), std::logic_error);
  } else {
    EXPECT_THROW(RigidBodyTreeWrapper(tree.get()), std::logic_error);
  }
}

INSTANTIATE_TEST_CASE_P(OwningAndNonOwningTests, RigidBodyTreeWrapperTests,
                        testing::Bool());

}  // namespace
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
