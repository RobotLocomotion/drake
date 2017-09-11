#include "drake/manipulation/perception/pose_smoother.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_geometry_compare.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/manipulation/util/moving_average_filter.h"
#include "drake/manipulation/util/perception_utils.h"
#include "drake/math/quaternion.h"

namespace drake {
namespace manipulation {
namespace perception {
namespace {

using Eigen::Isometry3d;
using Eigen::Quaterniond;
using Eigen::AngleAxisd;
using util::VectorToIsometry3d;
using util::Isometry3dToVector;

struct CombinedState {
  CombinedState(const Isometry3d& pose = Isometry3d::Identity(),
                const Vector6<double>& velocity = Vector6<double>::Zero()) {
    pose_ = pose;
    velocity_ = velocity;
  }
  Isometry3d pose_;
  Vector6<double> velocity_;
};

const int kMovingAverageWindowSize = 3;
const double kOptitrackLcmStatusPeriod = 0.01;
const double kPoseComparisonTolerance = 1e-6;

class PoseSmootherTest : public ::testing::Test {
 public:
  void Initialize(double max_linear_velocity, double max_angular_velocity,
                  int filter_window_size = 0) {
    if (filter_window_size == 0) {
      // Pose Smoother with only outlier rejection.
      dut_ = std::make_unique<PoseSmoother>(
          max_linear_velocity, max_angular_velocity,
          kOptitrackLcmStatusPeriod /* optitrack_lcm_status_period */);
    } else {
      // Pose Smoother with both outlier rejection and smoothing.
      dut_ = std::make_unique<PoseSmoother>(
          max_linear_velocity, max_angular_velocity, filter_window_size,
          kOptitrackLcmStatusPeriod /* optitrack_lcm_status_period */);
    }

    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);

    EXPECT_EQ(dut_->get_num_input_ports(), 1);
    EXPECT_EQ(dut_->get_num_output_ports(), 2);
  }

  CombinedState UpdateStateCalcOutput(const Isometry3d input_pose) {
    std::unique_ptr<systems::AbstractValue> input(
        systems::AbstractValue::Make(Isometry3<double>::Identity()));
    input->SetValue(input_pose);
    context_->FixInputPort(0 /* input port ID*/, std::move(input));

    dut_->CalcUnrestrictedUpdate(*context_, context_->get_mutable_state());
    dut_->CalcOutput(*context_, output_.get());

    auto output_pose_value =
        output_->get_data(dut_->get_smoothed_pose_output_port().get_index());
    auto output_velocity_value = output_->get_data(
        dut_->get_smoothed_velocity_output_port().get_index());

    CombinedState return_state(
        output_pose_value->GetValue<Isometry3d>(),
        output_velocity_value->GetValue<Vector6<double>>());

    return return_state;
  }

 private:
  std::unique_ptr<PoseSmoother> dut_;
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
};

TEST_F(PoseSmootherTest, OutlierRejectionTest) {
  Initialize(1.0, 0.5 * M_PI);

  Isometry3d input_pose_0 = Isometry3d::Identity();
  input_pose_0.linear() =
      AngleAxisd(0.25 * M_PI, Eigen::Vector3d::UnitX()).matrix();
  input_pose_0.translation() << 0.01, -5.0, 10.10;

  CombinedState output_state_0;
  EXPECT_NO_THROW(output_state_0 = UpdateStateCalcOutput(input_pose_0));
  EXPECT_TRUE(CompareTransforms(output_state_0.pose_, input_pose_0,
                                kPoseComparisonTolerance));

  EXPECT_TRUE(CompareMatrices(output_state_0.velocity_, Vector6<double>::Zero(),
                              1e-3, MatrixCompareType::absolute));

  // Input which will be rejected due to large rotational velocity.
  Isometry3d input_pose_1 = Isometry3d::Identity();
  input_pose_1.linear() =
      AngleAxisd(0.35 * M_PI, Eigen::Vector3d::UnitX()).matrix();
  input_pose_1.translation() << 0.01, -5.0, 10.10;
  CombinedState output_state_1;
  EXPECT_NO_THROW(output_state_1 = UpdateStateCalcOutput(input_pose_1));
  // Since input_pose_1 is expected to be rejected, the posesmoother output does
  // not match its input.
  EXPECT_FALSE(CompareTransforms(output_state_1.pose_, input_pose_1,
                                 kPoseComparisonTolerance));
  // Since the current input was rejected, the posesmoother is expected to
  // output
  // the previous input instead.
  EXPECT_TRUE(CompareTransforms(output_state_1.pose_, input_pose_0,
                                kPoseComparisonTolerance));

  // Input which will be rejected due to large translational velocity.
  Isometry3d input_pose_2;
  input_pose_1.linear() =
      Eigen::AngleAxisd(0.25 * M_PI, Eigen::Vector3d::UnitX()).matrix();
  input_pose_1.translation() << 0.51, -5.0, 10.10;
  CombinedState output_state_2;
  EXPECT_NO_THROW(output_state_2 = UpdateStateCalcOutput(input_pose_2));
  // Since input_pose_2 is expected to be rejected, the posesmoother output does
  // not match its input.
  EXPECT_FALSE(CompareTransforms(output_state_2.pose_, input_pose_2,
                                 kPoseComparisonTolerance));
  // Since the current input was rejected, the posesmoother is expected to
  // output
  // the previous accepted input instead.
  EXPECT_TRUE(CompareTransforms(output_state_2.pose_, input_pose_0,
                                kPoseComparisonTolerance));
}

TEST_F(PoseSmootherTest, SmootherTest) {
  // Initializing with large outlier thresholds (this test only checks
  // the output of smoothing).
  Initialize(100.0, 10.0 * M_PI, kMovingAverageWindowSize);

  // Test smoothing with an initial pose.
  Isometry3d input_pose_0 = Isometry3d::Identity();
  input_pose_0.linear() =
      Eigen::AngleAxisd(0.25 * M_PI, Eigen::Vector3d::UnitX()).matrix();
  input_pose_0.translation() << 0.01, -5.0, 10.10;

  CombinedState output_state_0;
  EXPECT_NO_THROW(output_state_0 = UpdateStateCalcOutput(input_pose_0));
  CombinedState expected_output_state_0;
  // Since filter has just been initialised, expected output is that of the
  // input.
  expected_output_state_0.pose_ = input_pose_0;
  expected_output_state_0.velocity_ = Vector6<double>::Zero();

  EXPECT_TRUE(CompareTransforms(output_state_0.pose_,
                                expected_output_state_0.pose_,
                                kPoseComparisonTolerance));

  EXPECT_TRUE(CompareMatrices(output_state_0.velocity_,
                              expected_output_state_0.velocity_, 1e-3,
                              MatrixCompareType::absolute));

  // A small additional rotation applied in the next input.
  Isometry3d input_pose_1;
  input_pose_1.linear() =
      AngleAxisd(0.28 * M_PI, Eigen::Vector3d::UnitX()).matrix();
  input_pose_1.translation() << 0.01, -5.0, 10.10;

  CombinedState output_state_1;
  EXPECT_NO_THROW(output_state_1 = UpdateStateCalcOutput(input_pose_1));
  CombinedState expected_output_state_1 = Isometry3d::Identity();

  expected_output_state_1.pose_.translation() << 0.01, -5, 10.1;
  expected_output_state_1.pose_.linear() =
      (Eigen::MatrixXd(3, 3) << 1, 0, 0, 0, 0.67319401200771101,
       -0.73922055347988902, 0, 0.73922055347988902, 0.67319401200771101)
          .finished();

  expected_output_state_1.velocity_ << 0, 0, 0, 4.670903542103451, 0, 0;
  EXPECT_TRUE(CompareTransforms(output_state_1.pose_,
                                expected_output_state_1.pose_,
                                kPoseComparisonTolerance));

  EXPECT_TRUE(CompareMatrices(output_state_1.velocity_,
                              expected_output_state_1.velocity_, 1e-3,
                              MatrixCompareType::absolute));

  // Test 2 - identical pose as test 1 to ensure rotational velocities drop
  // averaged
  // by window size. Resulting output pose is averaged across previous 3.
  CombinedState output_state_2;
  EXPECT_NO_THROW(output_state_2 = UpdateStateCalcOutput(input_pose_1));
  CombinedState expected_output_state_2 = Isometry3d::Identity();

  expected_output_state_2.pose_.translation() << 0.01, -5, 10.1;
  expected_output_state_2.pose_.linear() =
      (MatrixX<double>(3, 3) << 1, 0, 0, 0, 0.66147703270805791,
       -0.74974268135601596, 0, 0.74974268135601596, 0.66147703270805791)
          .finished();

  expected_output_state_2.velocity_ << 0, 0, 0, 1.5751117688894509, 0, 0;
  EXPECT_TRUE(CompareTransforms(output_state_2.pose_,
                                expected_output_state_2.pose_,
                                kPoseComparisonTolerance));

  EXPECT_TRUE(CompareMatrices(output_state_2.velocity_,
                              expected_output_state_2.velocity_, 1e-3,
                              MatrixCompareType::absolute));

  // Test 3 - Small change in translation
  Isometry3d input_pose_3 = Isometry3d::Identity();
  input_pose_3.linear() =
      AngleAxisd(0.28 * M_PI, Eigen::Vector3d::UnitX()).matrix();
  input_pose_3.translation() << 0.04, -5.0, 10.10;

  // A small additional translation applied in the next input.
  CombinedState output_state_3;
  EXPECT_NO_THROW(output_state_3 = UpdateStateCalcOutput(input_pose_3));
  CombinedState expected_output_state_3;
  expected_output_state_3 = CombinedState();
  expected_output_state_3.pose_.translation() << 0.02, -5, 10.1;
  expected_output_state_3.pose_.linear() =
      (MatrixX<double>(3, 3) << 1, 0, 0, 0, 0.63742398974868997,
       -0.77051324277578903, 0, 0.77051324277578903, 0.63742398974868997)
          .finished();
  expected_output_state_3.pose_.makeAffine();
  expected_output_state_3.velocity_ << 1.0, 0, 0, 3.1786156325620212, 0, 0;

  EXPECT_TRUE(
      (CompareTransforms(output_state_3.pose_, expected_output_state_3.pose_,
                         kPoseComparisonTolerance)));

  EXPECT_TRUE(CompareMatrices(output_state_3.velocity_,
                              expected_output_state_3.velocity_, 1e-3,
                              MatrixCompareType::absolute));

  // A small additional rotation applied with a sign-inverted quaternion.
Quaterniond test_pose_quaternion{
      AngleAxisd(0.28 * M_PI, Eigen::Vector3d::UnitX())};
  Quaterniond sign_inverted_pose_quaternion(
      -test_pose_quaternion.w(), -test_pose_quaternion.x(),
      -test_pose_quaternion.y(), -test_pose_quaternion.z());
  Isometry3d input_pose_4 = Isometry3d::Identity();
  input_pose_4.linear() = sign_inverted_pose_quaternion.matrix();
  input_pose_4.translation() << 0.04, -5.0, 10.10;

  CombinedState output_state_4;
  EXPECT_NO_THROW(output_state_4 = UpdateStateCalcOutput(input_pose_4));
  CombinedState expected_output_state_4 = CombinedState();
expected_output_state_4.pose_.linear() <<
  (MatrixX<double>(3, 3) << 1, 0, 0, 0, 0.63742398974868986,
-0.77051324277578914, 0, 0.77051324277578914, 0.63742398974868986).finished();
  expected_output_state_4.pose_.translation() << 0.03, -5, 10.1;
  expected_output_state_4.velocity_ <<1.0, 0, 0, 0, 0, 0;

  EXPECT_TRUE(
  (CompareTransforms(output_state_4.pose_, expected_output_state_4.pose_,
                     kPoseComparisonTolerance)));
  EXPECT_TRUE(CompareMatrices(output_state_4.velocity_,
                              expected_output_state_4.velocity_, 1e-3,
                              MatrixCompareType::absolute));
}

}  // namespace
}  // namespace perception
}  // namespace manipulation
}  // namespace drake
