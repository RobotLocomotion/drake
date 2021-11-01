#include "drake/manipulation/perception/pose_smoother.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_geometry_compare.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/manipulation/util/moving_average_filter.h"
#include "drake/math/quaternion.h"

namespace drake {
namespace manipulation {
namespace perception {
namespace {

using Eigen::Isometry3d;
using Eigen::Quaterniond;
using Eigen::AngleAxisd;

// Provide a warning-free wrapper over a deprecated function.  In case
// we still need this function when eigen_geometry_compare.h is removed,
// we can just copy its source code here directly.
[[nodiscard]] ::testing::AssertionResult CompareTransforms(
    const Eigen::Isometry3d& X_expected, const Eigen::Isometry3d& X_actual,
    double tolerance) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  return drake::CompareTransforms(X_expected, X_actual, tolerance);
#pragma GCC diagnostic pop
}

struct CombinedState {
  CombinedState(
      const Isometry3d& default_pose = Isometry3d::Identity(),
      const Vector6<double>& default_velocity = Vector6<double>::Zero()) {
    pose = default_pose;
    velocity = default_velocity;
  }
  Isometry3d pose;
  Vector6<double> velocity;
};

const int kMovingAverageWindowSize = 3;
const double kPoseSmootherPeriod = 0.01;
const double kPoseComparisonTolerance = 1e-6;

class PoseSmootherTest : public ::testing::Test {
 public:
  void Initialize(double max_linear_velocity, double max_angular_velocity,
                  int filter_window_size = 0) {
      dut_ = std::make_unique<PoseSmoother>(
          max_linear_velocity, max_angular_velocity,
          kPoseSmootherPeriod, filter_window_size);

    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput();

    EXPECT_EQ(dut_->num_input_ports(), 1);
    EXPECT_EQ(dut_->num_output_ports(), 2);
  }

  CombinedState UpdateStateCalcOutput(
      const Isometry3d& input_pose, double input_time) {
    dut_->get_input_port(0).FixValue(context_.get(), input_pose);
    context_->SetTime(input_time);

    dut_->CalcUnrestrictedUpdate(*context_, &context_->get_mutable_state());
    dut_->CalcOutput(*context_, output_.get());

    auto output_pose_value =
        output_->get_data(dut_->get_smoothed_pose_output_port().get_index());
    auto output_velocity_value = output_->get_data(
        dut_->get_smoothed_velocity_output_port().get_index());

    CombinedState return_state(
        output_pose_value->get_value<Isometry3d>(),
        output_velocity_value->get_value<Vector6<double>>());

    return return_state;
  }

 private:
  std::unique_ptr<PoseSmoother> dut_;
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
};

TEST_F(PoseSmootherTest, OutlierRejectionTest) {
  Initialize(1.0, 0.5 * M_PI);

  double test_time = kPoseSmootherPeriod;

  Isometry3d input_pose_0 = Isometry3d::Identity();
  input_pose_0.linear() =
      AngleAxisd(0.25 * M_PI, Eigen::Vector3d::UnitX()).matrix();
  input_pose_0.translation() << 0.01, -5.0, 10.10;

  CombinedState output_state_0;
  output_state_0 = UpdateStateCalcOutput(input_pose_0, test_time);
  test_time += kPoseSmootherPeriod;
  EXPECT_TRUE(CompareTransforms(output_state_0.pose, input_pose_0,
                                kPoseComparisonTolerance));

  EXPECT_TRUE(CompareMatrices(output_state_0.velocity, Vector6<double>::Zero(),
                              1e-3, MatrixCompareType::absolute));

  // Input which will be rejected due to large rotational velocity.
  Isometry3d input_pose_1 = Isometry3d::Identity();
  input_pose_1.linear() =
      AngleAxisd(0.35 * M_PI, Eigen::Vector3d::UnitX()).matrix();
  input_pose_1.translation() << 0.01, -5.0, 10.10;
  CombinedState output_state_1;
  output_state_1 = UpdateStateCalcOutput(input_pose_1, test_time);
  test_time += kPoseSmootherPeriod;
  // Since input_pose_1 is expected to be rejected, the posesmoother output
  // does not match its input.
  EXPECT_FALSE(CompareTransforms(output_state_1.pose, input_pose_1,
                                 kPoseComparisonTolerance));
  // Since the current input was rejected, the posesmoother is expected to
  // output
  // the previous input instead.
  EXPECT_TRUE(CompareTransforms(output_state_1.pose, input_pose_0,
                                kPoseComparisonTolerance));

  // Input which will be rejected due to large translational velocity.
  Isometry3d input_pose_2 = Isometry3d::Identity();
  input_pose_2.linear() =
      Eigen::AngleAxisd(0.25 * M_PI, Eigen::Vector3d::UnitX()).matrix();
  input_pose_2.translation() << 0.51, -5.0, 10.10;
  CombinedState output_state_2;
  output_state_2 = UpdateStateCalcOutput(input_pose_2, test_time);
  // Since input_pose_2 is expected to be rejected, the posesmoother output does
  // not match its input.
  EXPECT_FALSE(CompareTransforms(output_state_2.pose, input_pose_2,
                                 kPoseComparisonTolerance));
  // Since the current input was rejected, the posesmoother is expected to
  // output
  // the previous accepted input instead.
  EXPECT_TRUE(CompareTransforms(output_state_2.pose, input_pose_0,
                                kPoseComparisonTolerance));
}

TEST_F(PoseSmootherTest, SmootherTest) {
  // Initializing with large outlier thresholds (this test only checks
  // the output of smoothing).
  Initialize(100.0, 10.0 * M_PI, kMovingAverageWindowSize);
  double test_time = kPoseSmootherPeriod;
  // Test smoothing with an initial pose.
  Isometry3d input_pose_0 = Isometry3d::Identity();
  input_pose_0.linear() =
      Eigen::AngleAxisd(0.25 * M_PI, Eigen::Vector3d::UnitX()).matrix();
  input_pose_0.translation() << 0.01, -5.0, 10.10;

  CombinedState output_state_0;

  output_state_0 = UpdateStateCalcOutput(input_pose_0, test_time);
  test_time += kPoseSmootherPeriod;

  CombinedState expected_output_state_0;
  // Since filter has just been initialised, expected output is that of the
  // input.
  expected_output_state_0.pose = input_pose_0;
  expected_output_state_0.velocity = Vector6<double>::Zero();

  EXPECT_TRUE(CompareTransforms(output_state_0.pose,
                                expected_output_state_0.pose,
                                kPoseComparisonTolerance));

  EXPECT_TRUE(CompareMatrices(output_state_0.velocity,
                              expected_output_state_0.velocity, 1e-3,
                              MatrixCompareType::absolute));

  // A small additional rotation applied in the next input.
  Isometry3d input_pose_1;
  input_pose_1.linear() =
      AngleAxisd(0.28 * M_PI, Eigen::Vector3d::UnitX()).matrix();
  input_pose_1.translation() << 0.01, -5.0, 10.10;

  CombinedState output_state_1;
  output_state_1 = UpdateStateCalcOutput(input_pose_1, test_time);

  test_time += kPoseSmootherPeriod;
  CombinedState expected_output_state_1 = Isometry3d::Identity();

  expected_output_state_1.pose.translation() << 0.01, -5, 10.1;
  expected_output_state_1.pose.linear() =
      (Eigen::MatrixXd(3, 3) <<
        1, 0, 0,
        0,  0.67301251350977331, -0.73963109497860968,
        0,  0.73963109497860968,  0.67301251350977331).finished();

  expected_output_state_1.velocity << 0, 0, 0, 4.7123889803846719, 0, 0;
  EXPECT_TRUE(CompareTransforms(output_state_1.pose,
                                expected_output_state_1.pose,
                                kPoseComparisonTolerance));

  EXPECT_TRUE(CompareMatrices(output_state_1.velocity,
                              expected_output_state_1.velocity, 1e-3,
                              MatrixCompareType::absolute));

  // Test 2 - identical pose as test 1 to ensure rotational velocities drop
  // averaged
  // by window size. Resulting output pose is averaged across previous 3.
  CombinedState output_state_2;
  output_state_2 = UpdateStateCalcOutput(input_pose_1, test_time);
  test_time += kPoseSmootherPeriod;
  CombinedState expected_output_state_2 = Isometry3d::Identity();

  expected_output_state_2.pose.translation() << 0.01, -5, 10.1;
  expected_output_state_2.pose.linear() =
      (MatrixX<double>(3, 3) <<
        1,  0,  0,
        0,  0.66130992678343126,  -0.75011277867910831,
        0,  0.75011277867910831,   0.66130992678343126).finished();

  expected_output_state_2.velocity << 0, 0, 0, 1.571054760257719, 0, 0;
  EXPECT_TRUE(CompareTransforms(output_state_2.pose,
                                expected_output_state_2.pose,
                                kPoseComparisonTolerance));

  EXPECT_TRUE(CompareMatrices(output_state_2.velocity,
                              expected_output_state_2.velocity, 1e-3,
                              MatrixCompareType::absolute));

  // Test 3 - Small change in translation
  Isometry3d input_pose_3 = Isometry3d::Identity();
  input_pose_3.linear() =
      AngleAxisd(0.28 * M_PI, Eigen::Vector3d::UnitX()).matrix();
  input_pose_3.translation() << 0.04, -5.0, 10.10;

  // A small additional translation applied in the next input.
  CombinedState output_state_3;
  output_state_3 = UpdateStateCalcOutput(input_pose_3, test_time);
  test_time += kPoseSmootherPeriod;
  CombinedState expected_output_state_3;
  expected_output_state_3 = CombinedState();
  expected_output_state_3.pose.translation() << 0.02, -5, 10.1;
  expected_output_state_3.pose.linear() =
      (MatrixX<double>(3, 3) << 1, 0, 0, 0, 0.63742398974868997,
       -0.77051324277578903, 0, 0.77051324277578903, 0.63742398974868997)
          .finished();
  expected_output_state_3.pose.makeAffine();
  expected_output_state_3.velocity << 1.0, 0, 0, 3.1413342201269292, 0, 0;

  EXPECT_TRUE(
      (CompareTransforms(output_state_3.pose, expected_output_state_3.pose,
                         kPoseComparisonTolerance)));

  EXPECT_TRUE(CompareMatrices(output_state_3.velocity,
                              expected_output_state_3.velocity, 1e-3,
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
  output_state_4 = UpdateStateCalcOutput(input_pose_4, test_time);
  test_time += kPoseSmootherPeriod;
  CombinedState expected_output_state_4 = CombinedState();
  expected_output_state_4.pose.linear() << (
      MatrixX<double>(3, 3) << 1, 0, 0, 0, 0.63742398974868986,
          -0.77051324277578914, 0, 0.77051324277578914,
          0.63742398974868986).finished();
  expected_output_state_4.pose.translation() << 0.03, -5, 10.1;
  expected_output_state_4.velocity <<1.0, 0, 0, 0, 0, 0;

  EXPECT_TRUE(
  (CompareTransforms(output_state_4.pose, expected_output_state_4.pose,
                     kPoseComparisonTolerance)));
  EXPECT_TRUE(CompareMatrices(output_state_4.velocity,
                              expected_output_state_4.velocity, 1e-3,
                              MatrixCompareType::absolute));
}

}  // namespace
}  // namespace perception
}  // namespace manipulation
}  // namespace drake
