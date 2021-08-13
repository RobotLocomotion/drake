#include "drake/systems/rendering/pose_aggregator.h"

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_vector.h"

using drake::math::RigidTransform;
using drake::math::RigidTransformd;
using Eigen::AngleAxisd;
using Eigen::Vector3d;

namespace drake {
namespace systems {
namespace rendering {
namespace {

const int kNumBundlePoses = 2;
const int kNumSinglePoses = 2;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

class PoseAggregatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Glue together a robot with one link and one degree of freedom.
    SquareTwistMatrix<double> I = SquareTwistMatrix<double>::Zero();
    I.block<3, 3>(3, 3, 3, 3) << Eigen::Matrix3d::Identity();

    // Allocate an input for a PoseBundle.
    aggregator_.AddBundleInput("bundle", kNumBundlePoses);
    // Allocate a second and third input for a PoseVector and FrameVelocity.
    const int kRandomModelInstanceId0 = 42;
    aggregator_.AddSinglePoseAndVelocityInput("single_xv",
                                              kRandomModelInstanceId0);
    // Allocate a fourth input for a PoseVector, without velocity.
    const int kRandomModelInstanceId1 = 43;
    aggregator_.AddSingleInput("single_x", kRandomModelInstanceId1);

    context_ = aggregator_.CreateDefaultContext();
    output_ = aggregator_.AllocateOutput();
  }

  PoseAggregator<double> aggregator_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

namespace {
PoseBundle<AutoDiffXd> ToAutoDiffXd(const PoseBundle<double>& original) {
  // TODO(sherm1): Consider moving value transmogrification into AbstractValue,
  // as discussed in https://github.com/RobotLocomotion/drake/issues/5454
  PoseBundle<AutoDiffXd> result(original.get_num_poses());
  for (int i = 0; i < original.get_num_poses(); i++) {
    RigidTransform<AutoDiffXd> pose(
        original.get_transform(i).cast<AutoDiffXd>());
    result.set_transform(i, pose);
    FrameVelocity<AutoDiffXd> velocity;
    velocity.set_velocity(multibody::SpatialVelocity<AutoDiffXd>(
        original.get_velocity(i).get_velocity().get_coeffs()));
    result.set_velocity(i, velocity);
    result.set_name(i, original.get_name(i));
    result.set_model_instance_id(i, original.get_model_instance_id(i));
  }
  return result;
}
}  // namespace

// Tests that PoseAggregator aggregates poses from two PoseVector inputs (one
// with velocity and one without), and a PoseBundle input.
TEST_F(PoseAggregatorTest, CompositeAggregation) {
  // Set some arbitrary translations in the PoseBundle input, and a velocity
  // for one of the poses.
  PoseBundle<double> generic_input(2);
  Vector3d translation_0{0, 1, 0};
  Vector3d translation_1{0, 1, 1};
  const int kSherlockModelInstanceId = 17;
  generic_input.set_name(0, "Sherlock");
  generic_input.set_transform(0, RigidTransformd(translation_0));
  generic_input.set_model_instance_id(0, kSherlockModelInstanceId);
  FrameVelocity<double> sherlock_velocity;
  sherlock_velocity.get_mutable_value() << 2.5, 5.0, 7.5, 10.0, 12.5, 15.0;
  generic_input.set_velocity(0, sherlock_velocity);

  const int kMycroftModelInstanceId = 13;
  generic_input.set_name(1, "Mycroft");
  generic_input.set_transform(1, RigidTransformd(translation_1));
  generic_input.set_model_instance_id(1, kMycroftModelInstanceId);
  aggregator_.get_input_port(0).FixValue(context_.get(), generic_input);

  // Set an arbitrary translation in the first PoseVector input.
  PoseVector<double> pose_vec1 = PoseVector<double>();
  pose_vec1.set_translation(Eigen::Translation<double, 3>(0.5, 1.5, 2.5));
  // TODO(russt): Consider implementing ToAutoDiffXd for BasicVector<double>?
  PoseVector<AutoDiffXd> autodiff_pose_vec1 = PoseVector<AutoDiffXd>();
  autodiff_pose_vec1.set_value(
      pose_vec1.get_value().template cast<AutoDiffXd>());
  aggregator_.get_input_port(1).FixValue(context_.get(), pose_vec1);

  // Set some numbers in the corresponding FrameVelocity input.
  FrameVelocity<double> frame_vel1 = FrameVelocity<double>();
  frame_vel1.get_mutable_value() << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  FrameVelocity<AutoDiffXd> autodiff_frame_vel1 = FrameVelocity<AutoDiffXd>();
  autodiff_frame_vel1.set_value(
      frame_vel1.get_value().template cast<AutoDiffXd>());
  aggregator_.get_input_port(2).FixValue(context_.get(), frame_vel1);

  // Set an arbitrary rotation in the second PoseVector input.
  PoseVector<double> pose_vec2 = PoseVector<double>();
  pose_vec2.set_rotation(Eigen::Quaternion<double>(0.5, 0.5, 0.5, 0.5));
  PoseVector<AutoDiffXd> autodiff_pose_vec2 = PoseVector<AutoDiffXd>();
  autodiff_pose_vec2.set_value(
      pose_vec2.get_value().template cast<AutoDiffXd>());
  aggregator_.get_input_port(3).FixValue(context_.get(), pose_vec2);

  aggregator_.CalcOutput(*context_, output_.get());

  // Extract the output PoseBundle.
  const PoseBundle<double>& bundle =
      output_->get_data(0)->get_value<PoseBundle<double>>();
  ASSERT_EQ(kNumBundlePoses + kNumSinglePoses, bundle.get_num_poses());

  // Check that the PoseBundle poses and velocities are passed through to the
  // output.
  EXPECT_EQ("bundle::Sherlock", bundle.get_name(0));
  EXPECT_EQ(kSherlockModelInstanceId, bundle.get_model_instance_id(0));
  const RigidTransformd& generic_pose_0 = bundle.get_transform(0);
  EXPECT_TRUE(CompareMatrices(RigidTransformd(translation_0).GetAsMatrix4(),
                              generic_pose_0.GetAsMatrix4()));
  EXPECT_TRUE(CompareMatrices(sherlock_velocity.get_value(),
                              bundle.get_velocity(0).get_value()));

  EXPECT_EQ("bundle::Mycroft", bundle.get_name(1));
  EXPECT_EQ(kMycroftModelInstanceId, bundle.get_model_instance_id(1));
  const RigidTransformd& generic_pose_1 = bundle.get_transform(1);
  EXPECT_TRUE(CompareMatrices(RigidTransformd(translation_1).GetAsMatrix4(),
                              generic_pose_1.GetAsMatrix4()));

  // Check that the PoseVector pose with velocity is passed through to the
  // output.
  EXPECT_EQ("single_xv", bundle.get_name(2));
  EXPECT_EQ(42, bundle.get_model_instance_id(2));
  const RigidTransformd& xv_pose = bundle.get_transform(2);
  EXPECT_TRUE(CompareMatrices(
      RigidTransformd(Vector3d(0.5, 1.5, 2.5)).GetAsMatrix4(),
      xv_pose.GetAsMatrix4()));
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(1.0 + i, bundle.get_velocity(2)[i]);
  }

  // Check that the PoseVector pose without velocity is passed through to
  // the output.
  EXPECT_EQ("single_x", bundle.get_name(3));
  EXPECT_EQ(43, bundle.get_model_instance_id(3));
  const RigidTransformd& x_pose = bundle.get_transform(3);
  EXPECT_TRUE(CompareMatrices(
      RigidTransformd(Eigen::Quaternion<double>(0.5, 0.5, 0.5, 0.5),
                      Vector3d{0, 0, 0})
          .GetAsMatrix4(),
      x_pose.GetAsMatrix4()));

  // The sequel checks the AutoDiffXd conversion.
  auto autodiff_aggregator = aggregator_.ToAutoDiffXd();
  auto autodiff_context = autodiff_aggregator->CreateDefaultContext();
  autodiff_context->SetTimeStateAndParametersFrom(*context_);

  autodiff_aggregator->get_input_port(0).FixValue(autodiff_context.get(),
                                                  ToAutoDiffXd(generic_input));
  autodiff_aggregator->get_input_port(1).FixValue(autodiff_context.get(),
                                                  autodiff_pose_vec1);
  autodiff_aggregator->get_input_port(2).FixValue(autodiff_context.get(),
                                                  autodiff_frame_vel1);
  autodiff_aggregator->get_input_port(3).FixValue(autodiff_context.get(),
                                                  autodiff_pose_vec2);

  auto autodiff_output = autodiff_aggregator->AllocateOutput();
  autodiff_aggregator->CalcOutput(*autodiff_context, autodiff_output.get());
  const PoseBundle<AutoDiffXd>& autodiff_bundle =
      autodiff_output->get_data(0)->get_value<PoseBundle<AutoDiffXd>>();
  ASSERT_EQ(bundle.get_num_poses(), autodiff_bundle.get_num_poses());
  for (int i = 0; i < bundle.get_num_poses(); i++) {
    EXPECT_TRUE(
        CompareMatrices(bundle.get_transform(i).GetAsMatrix4(),
                        math::ExtractValue(
                            autodiff_bundle.get_transform(i).GetAsMatrix4())));
    EXPECT_TRUE(
        CompareMatrices(bundle.get_velocity(i).get_value(),
                        math::ExtractValue(
                            autodiff_bundle.get_velocity(i).get_value())));
    EXPECT_EQ(bundle.get_name(i), autodiff_bundle.get_name(i));
    EXPECT_EQ(bundle.get_model_instance_id(i),
              autodiff_bundle.get_model_instance_id(i));
  }
}

// Tests that PoseAggregator allocates no state variables in the context_.
TEST_F(PoseAggregatorTest, Stateless) { EXPECT_TRUE(context_->is_stateless()); }

// Tests that AddSinglePoseAndVelocityInput returns input ports for both
// the new ports.
TEST_F(PoseAggregatorTest, AddSinglePoseAndVelocityPorts) {
  auto ports = aggregator_.AddSinglePoseAndVelocityInput("test", 100);
  const InputPort<double>& pose_port = ports.pose_input_port;
  const InputPort<double>& velocity_port = ports.velocity_input_port;
  EXPECT_EQ(PoseVector<double>::kSize, pose_port.size());
  EXPECT_EQ(FrameVelocity<double>::kSize, velocity_port.size());
}

// Tests that PoseBundle supports Symbolic form.
GTEST_TEST(PoseBundleTest, Symbolic) {
  PoseBundle<symbolic::Expression> dut{1};
  multibody::SpatialVelocity<symbolic::Expression> msv =
    dut.get_velocity(0).get_velocity();
  // Just sanity check that some element got zero-initialized.
  const symbolic::Expression& x = msv[0];
  EXPECT_TRUE(x.EqualTo(0.0));
}

#pragma GCC diagnostic pop

}  // namespace
}  // namespace rendering
}  // namespace systems
}  // namespace drake
