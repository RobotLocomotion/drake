#include "drake/systems/rendering/pose_aggregator.h"

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_vector.h"

using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Vector3d;

namespace drake {
namespace systems {
namespace rendering {
namespace {

const int kNumRigidBodyPoses = 1;
const int kNumBundlePoses = 2;
const int kNumSinglePoses = 2;

class PoseAggregatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Glue together a robot with one link and one degree of freedom.
    SquareTwistMatrix<double> I = SquareTwistMatrix<double>::Zero();
    I.block<3, 3>(3, 3, 3, 3) << Eigen::Matrix3d::Identity();

    // Attach the link to the origin with a pin joint.
    auto rb1 = std::make_unique<RigidBody<double>>();
    rb1->set_model_name("robot");
    rb1->set_name("link1");
    rb1->set_model_instance_id(tree_.add_model_instance());
    rb1->set_spatial_inertia(I);

    rb1->add_joint(&tree_.world(),
                   std::make_unique<RevoluteJoint>(
                       "joint", Isometry3d::Identity(), axis_));

    tree_.add_rigid_body(std::move(rb1));
    tree_.compile();

    // Allocate an input for the tree we just assembled.
    aggregator_.AddRigidBodyPlantInput(tree_);
    // Allocate another input for a PoseBundle.
    aggregator_.AddBundleInput("bundle", kNumBundlePoses);
    // Allocate a third and fourth input for a PoseVector and FrameVelocity.
    const int kRandomModelInstanceId0 = 42;
    aggregator_.AddSinglePoseAndVelocityInput("single_xv",
                                              kRandomModelInstanceId0);
    // Allocate a fifth input for a PoseVector, without velocity.
    const int kRandomModelInstanceId1 = 43;
    aggregator_.AddSingleInput("single_x", kRandomModelInstanceId1);

    context_ = aggregator_.CreateDefaultContext();
    output_ = aggregator_.AllocateOutput(*context_);
  }

  PoseAggregator<double> aggregator_;
  RigidBodyTree<double> tree_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;

  // The axis of the single DoF in the RigidBodyTree.
  const Vector3d axis_{Vector3d::UnitX()};
};

// Tests that PoseAggregator aggregates poses from a RigidBodyTree input, two
// PoseVector inputs (one with velocity and one without), and a PoseBundle
// input.
TEST_F(PoseAggregatorTest, HeterogeneousAggregation) {
  // Set the rigid body state input so that the revolute joint has position
  // pi/2, which results in the link "link1" pointing in the +y direction.
  auto rbt_input = BasicVector<double>::Make({M_PI, 0.0});
  context_->FixInputPort(0, std::move(rbt_input));

  // Set some arbitrary translations in the PoseBundle input, and a velocity
  // for one of the poses.
  PoseBundle<double> generic_input(2);
  Eigen::Translation3d translation_0(0, 1, 0);
  Eigen::Translation3d translation_1(0, 1, 1);
  const int kSherlockModelInstanceId = 17;
  generic_input.set_name(0, "Sherlock");
  generic_input.set_pose(0, Isometry3d(translation_0));
  generic_input.set_model_instance_id(0, kSherlockModelInstanceId);
  FrameVelocity<double> sherlock_velocity;
  sherlock_velocity.get_mutable_value() << 2.5, 5.0, 7.5, 10.0, 12.5, 15.0;
  generic_input.set_velocity(0, sherlock_velocity);

  const int kMycroftModelInstanceId = 13;
  generic_input.set_name(1, "Mycroft");
  generic_input.set_pose(1, Isometry3d(translation_1));
  generic_input.set_model_instance_id(1, kMycroftModelInstanceId);
  context_->FixInputPort(1, AbstractValue::Make(generic_input));

  // Set an arbitrary translation in the first PoseVector input.
  auto pose_vec1 = std::make_unique<PoseVector<double>>();
  pose_vec1->set_translation(Eigen::Translation<double, 3>(0.5, 1.5, 2.5));
  context_->FixInputPort(2, std::move(pose_vec1));

  // Set some numbers in the corresponding FrameVelocity input.
  auto frame_vel1 = std::make_unique<FrameVelocity<double>>();
  frame_vel1->get_mutable_value() << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  context_->FixInputPort(3, std::move(frame_vel1));

  // Set an arbitrary rotation in the second PoseVector input.
  auto pose_vec2 = std::make_unique<PoseVector<double>>();
  pose_vec2->set_rotation(Eigen::Quaternion<double>(0.5, 0.5, 0.5, 0.5));
  context_->FixInputPort(4, std::move(pose_vec2));

  aggregator_.CalcOutput(*context_, output_.get());

  // Extract the output PoseBundle.
  const PoseBundle<double>& bundle =
      output_->get_data(0)->GetValueOrThrow<PoseBundle<double>>();
  ASSERT_EQ(kNumBundlePoses + kNumRigidBodyPoses + kNumSinglePoses,
            bundle.get_num_poses());

  // Check that the rigid body poses, as determined by the kinematics,
  // appear in the output.
  EXPECT_EQ("robot::link1", bundle.get_name(0));
  EXPECT_EQ(0, bundle.get_model_instance_id(0));
  const Isometry3d& link1_pose = bundle.get_pose(0);
  EXPECT_TRUE(CompareMatrices(Isometry3d(AngleAxisd(M_PI, axis_)).matrix(),
                              link1_pose.matrix()));

  // Check that the PoseBundle poses and velocities are passed through to the
  // output.
  EXPECT_EQ("bundle::Sherlock", bundle.get_name(1));
  EXPECT_EQ(kSherlockModelInstanceId, bundle.get_model_instance_id(1));
  const Isometry3d& generic_pose_0 = bundle.get_pose(1);
  EXPECT_TRUE(CompareMatrices(Isometry3d(translation_0).matrix(),
                              generic_pose_0.matrix()));
  EXPECT_TRUE(CompareMatrices(sherlock_velocity.get_value(),
                              bundle.get_velocity(1).get_value()));

  EXPECT_EQ("bundle::Mycroft", bundle.get_name(2));
  EXPECT_EQ(kMycroftModelInstanceId, bundle.get_model_instance_id(2));
  const Isometry3d& generic_pose_1 = bundle.get_pose(2);
  EXPECT_TRUE(CompareMatrices(Isometry3d(translation_1).matrix(),
                              generic_pose_1.matrix()));

  // Check that the PoseVector pose with velocity is passed through to the
  // output.
  EXPECT_EQ("single_xv", bundle.get_name(3));
  EXPECT_EQ(42, bundle.get_model_instance_id(3));
  const Isometry3d& xv_pose = bundle.get_pose(3);
  EXPECT_TRUE(CompareMatrices(
      Isometry3d(Eigen::Translation<double, 3>(0.5, 1.5, 2.5)).matrix(),
      xv_pose.matrix()));
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(1.0 + i, bundle.get_velocity(3)[i]);
  }

  // Check that the PoseVector pose without velocity is passed through to
  // the output.
  EXPECT_EQ("single_x", bundle.get_name(4));
  EXPECT_EQ(43, bundle.get_model_instance_id(4));
  const Isometry3d& x_pose = bundle.get_pose(4);
  EXPECT_TRUE(CompareMatrices(
      Isometry3d(Eigen::Quaternion<double>(0.5, 0.5, 0.5, 0.5)).matrix(),
      x_pose.matrix()));
}

// Tests that PoseAggregator allocates no state variables in the context_.
TEST_F(PoseAggregatorTest, Stateless) {
  EXPECT_TRUE(context_->is_stateless());
}

// Tests that AddSinglePoseAndVelocityInput returns descriptors for both
// the new ports.
TEST_F(PoseAggregatorTest, AddSinglePoseAndVelocityPorts) {
  auto ports = aggregator_.AddSinglePoseAndVelocityInput("test", 100);
  const InputPortDescriptor<double>& pose_port = ports.first;
  const InputPortDescriptor<double>& velocity_port = ports.second;
  EXPECT_EQ(PoseVector<double>::kSize, pose_port.size());
  EXPECT_EQ(FrameVelocity<double>::kSize, velocity_port.size());
}

}  // namespace
}  // namespace rendering
}  // namespace systems
}  // namespace drake
