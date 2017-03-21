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
const int kNumGenericPoses = 2;

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
    aggregator_.AddBundleInput("bundle", kNumGenericPoses);
    // Allocate a third input for a PoseVector.
    const int kRandomModelInstanceId = 42;
    aggregator_.AddSingleInput("vector", kRandomModelInstanceId);

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

// Tests that PoseAggregator aggregates poses from a RigidBodyTree input, a
// PoseVector input, and a PoseBundle input.
TEST_F(PoseAggregatorTest, HeterogeneousAggregation) {
  // Set the rigid body state input so that the revolute joint has position
  // pi/2, which results in the link "link1" pointing in the +y direction.
  auto rbt_input = BasicVector<double>::Make({M_PI, 0.0});
  context_->FixInputPort(0, std::move(rbt_input));

  // Set some arbitrary translations in the PoseBundle input.
  PoseBundle<double> generic_input(2);
  Eigen::Translation3d translation_0(0, 1, 0);
  Eigen::Translation3d translation_1(0, 1, 1);
  const int kSherlockModelInstanceId = 17;
  generic_input.set_name(0, "Sherlock");
  generic_input.set_pose(0, Isometry3d(translation_0));
  generic_input.set_model_instance_id(0, kSherlockModelInstanceId);

  const int kMycroftModelInstanceId = 13;
  generic_input.set_name(1, "Mycroft");
  generic_input.set_pose(1, Isometry3d(translation_1));
  generic_input.set_model_instance_id(1, kMycroftModelInstanceId);
  context_->FixInputPort(1, AbstractValue::Make(generic_input));

  // Set an arbitrary rotation in the PoseVector input.
  auto pose_vec = std::make_unique<PoseVector<double>>();
  pose_vec->set_rotation(Eigen::Quaternion<double>(0.5, 0.5, 0.5, 0.5));
  context_->FixInputPort(2, std::move(pose_vec));

  aggregator_.CalcOutput(*context_, output_.get());

  // Extract the output PoseBundle.
  const PoseBundle<double>& bundle =
      output_->get_data(0)->GetValueOrThrow<PoseBundle<double>>();
  ASSERT_EQ(kNumGenericPoses + kNumRigidBodyPoses + 1, bundle.get_num_poses());

  // Check that the rigid body poses, as determined by the kinematics,
  // appear in the output.
  EXPECT_EQ("robot::link1", bundle.get_name(0));
  EXPECT_EQ(0, bundle.get_model_instance_id(0));
  const Isometry3d& link1_pose = bundle.get_pose(0);
  EXPECT_TRUE(CompareMatrices(Isometry3d(AngleAxisd(M_PI, axis_)).matrix(),
                              link1_pose.matrix()));

  // Check that the generic poses are passed through to the output.
  EXPECT_EQ("bundle::Sherlock", bundle.get_name(1));
  EXPECT_EQ(kSherlockModelInstanceId, bundle.get_model_instance_id(1));
  const Isometry3d& generic_pose_0 = bundle.get_pose(1);
  EXPECT_EQ("bundle::Mycroft", bundle.get_name(2));
  EXPECT_EQ(kMycroftModelInstanceId, bundle.get_model_instance_id(2));
  const Isometry3d& generic_pose_1 = bundle.get_pose(2);

  EXPECT_TRUE(CompareMatrices(Isometry3d(translation_0).matrix(),
                              generic_pose_0.matrix()));
  EXPECT_TRUE(CompareMatrices(Isometry3d(translation_1).matrix(),
                              generic_pose_1.matrix()));

  EXPECT_EQ("vector", bundle.get_name(3));
  const Isometry3d& vector_pose = bundle.get_pose(3);
  EXPECT_EQ(42, bundle.get_model_instance_id(3));
  EXPECT_TRUE(CompareMatrices(
      Isometry3d(Eigen::Quaternion<double>(0.5, 0.5, 0.5, 0.5)).matrix(),
      vector_pose.matrix()));
}

// Tests that PoseAggregator allocates no state variables in the context_.
TEST_F(PoseAggregatorTest, Stateless) {
  EXPECT_TRUE(context_->is_stateless());
}

}  // namespace
}  // namespace rendering
}  // namespace systems
}  // namespace drake
