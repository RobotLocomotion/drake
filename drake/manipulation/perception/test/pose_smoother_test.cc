#include "drake/manipulation/perception/pose_smoother.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/manipulation/util/moving_average_filter.h"

namespace drake {
namespace manipulation {
namespace perception {
namespace {
using Eigen::Quaterniond;
using Eigen::AngleAxisd;

struct CombinedState {
  CombinedState(const Isometry3<double>& pose = Isometry3<double>::Identity(),
                const Vector6<double>& velocity = Vector6<double>::Zero()) {
    pose_ = pose;
    velocity_ = velocity;
  }
  Isometry3<double> pose_;
  Vector6<double> velocity_;
};

const int kMovingAverageWindowSize = 3;
const double kTranslationAccuracy = 1e-3;
const double kRotationAccuracy = 1e-3;
const double kOptitrackLcmStatusPeriod = 0.01;

::testing::AssertionResult ComparePoses(const Isometry3<double>& pose_1,
                                        const Isometry3<double>& pose_2) {
  ::testing::AssertionResult translation_comparison =
      CompareMatrices(pose_1.translation(), pose_2.translation(),
                      kTranslationAccuracy, MatrixCompareType::absolute);

  ::testing::AssertionResult rotation_comparison =
      CompareMatrices(Eigen::Quaterniond(pose_1.linear()).vec(),
                      Eigen::Quaterniond(pose_2.linear()).vec(),
                      kRotationAccuracy, MatrixCompareType::absolute);

  if (translation_comparison == ::testing::AssertionSuccess() &&
      rotation_comparison == ::testing::AssertionSuccess()) {
    return ::testing::AssertionSuccess();
  } else {
    return ::testing::AssertionFailure();
  }
}

class PoseSmootherTest : public ::testing::Test {
 public:
  void Initialize(double max_linear_velocity, double max_angular_velocity,
                  unsigned int filter_window_size = 0) {
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

  CombinedState UpdateStateCalcOutput(const Isometry3<double> input_pose) {
    std::unique_ptr<systems::AbstractValue> input(
        new systems::Value<Isometry3<double>>(Isometry3<double>::Identity()));
    input->SetValue(input_pose);
    context_->FixInputPort(0 /* input port ID*/, std::move(input));

    dut_->CalcUnrestrictedUpdate(*context_, context_->get_mutable_state());
    dut_->CalcOutput(*context_, output_.get());

    auto output_pose_value =
        output_->get_data(dut_->get_smoothed_pose_output_port().get_index());
    auto output_velocity_value = output_->get_data(
        dut_->get_smoothed_velocity_output_port().get_index());

    CombinedState return_state(
        output_pose_value->GetValue<Isometry3<double>>(),
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

  Isometry3<double> input_pose_0;
  input_pose_0.linear() =
      AngleAxisd(0.25 * M_PI, Eigen::Vector3d::UnitX()).matrix();
  input_pose_0.translation() << 0.01, -5.0, 10.10;

  CombinedState output_state_0;
  EXPECT_NO_THROW(output_state_0 = UpdateStateCalcOutput(input_pose_0));
  EXPECT_TRUE(ComparePoses(output_state_0.pose_, input_pose_0));

  EXPECT_TRUE(CompareMatrices(output_state_0.velocity_, Vector6<double>::Zero(),
                              1e-3, MatrixCompareType::absolute));

  // Input which will be rejected due to large rotational velocity.
  Isometry3<double> input_pose_1;
  input_pose_1.linear() =
      AngleAxisd(0.35 * M_PI, Eigen::Vector3d::UnitX()).matrix();
  input_pose_1.translation() << 0.01, -5.0, 10.10;
  CombinedState output_state_1;
  EXPECT_NO_THROW(output_state_1 = UpdateStateCalcOutput(input_pose_1));
  // Since input_pose_1 is expected to be rejected, the posesmoother output does
  // not match its input.
  EXPECT_FALSE(ComparePoses(output_state_1.pose_, input_pose_1));
  // Since the current input was rejected, the posesmoother is expected to
  // output
  // the previous input instead.
  EXPECT_TRUE(ComparePoses(output_state_1.pose_, input_pose_0));

  // Input which will be rejected due to large translational velocity.
  Isometry3<double> input_pose_2;
  input_pose_1.linear() =
      Eigen::AngleAxisd(0.25 * M_PI, Eigen::Vector3d::UnitX()).matrix();
  input_pose_1.translation() << 0.51, -5.0, 10.10;
  CombinedState output_state_2;
  EXPECT_NO_THROW(output_state_2 = UpdateStateCalcOutput(input_pose_2));
  // Since input_pose_2 is expected to be rejected, the posesmoother output does
  // not match its input.
  EXPECT_FALSE(ComparePoses(output_state_2.pose_, input_pose_2));
  // Since the current input was rejected, the posesmoother is expected to
  // output
  // the previous accepted input instead.
  EXPECT_TRUE(ComparePoses(output_state_2.pose_, input_pose_0));
}

Isometry3<double> VectorToIsometry3(const VectorX<double>& pose_vector) {
  Isometry3<double> pose;
  pose.linear() = Quaterniond(pose_vector.segment<4>(3)).matrix();
  pose.translation() = pose_vector.segment<3>(0);
  return pose;
}

VectorX<double> Isometry3ToVector(const Isometry3<double>& pose) {
  VectorX<double> pose_vector = VectorX<double>::Zero(7);
  pose_vector.segment<3>(0) = pose.translation();
  pose_vector.segment<4>(3) = Quaterniond(pose.linear()).coeffs();
  return pose_vector;
}

CombinedState AveragePoseAndVelocities(const Isometry3<double>& input_pose) {
  static auto filter =
      std::make_unique<util::MovingAverageFilter<VectorX<double>>>(
          kMovingAverageWindowSize);
  static bool first_time = true;
  static Isometry3<double> previous_pose;
  Isometry3<double> fixed_input_pose = input_pose, output_pose;

  Vector6<double> velocities = Vector6<double>::Zero();

  if (first_time) {
    first_time = false;
    previous_pose = fixed_input_pose;
  } else {
    // drake::log()->info("!first time");
    Quaterniond q1 = Quaterniond(previous_pose.linear());
    Quaterniond q2 = Quaterniond(input_pose.linear());

    // Flip quaternion if needed.
    double dot = q1.dot(q2);
    if (dot < 0.0) {
      // Invert sign.
      q2.w() = -q2.w();
      q2.x() = -q2.x();
      q2.y() = -q2.y();
      q2.z() = -q2.z();
    }
    fixed_input_pose = input_pose;
    fixed_input_pose.linear() = q2.matrix();
  }
  output_pose =
      VectorToIsometry3(filter->Update(Isometry3ToVector(fixed_input_pose)));

  Eigen::Array3d translation_diff =
      output_pose.translation().array() - previous_pose.translation().array();

  velocities.segment<3>(0) =
      (translation_diff / kOptitrackLcmStatusPeriod).matrix();

  Eigen::AngleAxisd angle_axis_diff;

  angle_axis_diff = Eigen::AngleAxisd(output_pose.linear() *
                                      previous_pose.linear().inverse());
  velocities.segment<3>(3) = angle_axis_diff.axis() * angle_axis_diff.angle() /
                             kOptitrackLcmStatusPeriod;
  CombinedState output_state;
  output_state.pose_ = output_pose;
  output_state.velocity_ = velocities;
  previous_pose = output_pose;
  return output_state;
}

TEST_F(PoseSmootherTest, SmootherTest) {
  // Initializing with large outlier thresholds (this test only checks
  // the output of smoothing).
  Initialize(100.0, 10.0 * M_PI, kMovingAverageWindowSize);

  // Test smoothing with an initial pose.
  Isometry3<double> input_pose_0;
  input_pose_0.linear() =
      Eigen::AngleAxisd(0.25 * M_PI, Eigen::Vector3d::UnitX()).matrix();
  input_pose_0.translation() << 0.01, -5.0, 10.10;

  CombinedState output_state_0;
  EXPECT_NO_THROW(output_state_0 = UpdateStateCalcOutput(input_pose_0));
  CombinedState expected_output_state_0 =
      AveragePoseAndVelocities(input_pose_0);
  EXPECT_TRUE(
      ComparePoses(output_state_0.pose_, expected_output_state_0.pose_));

  EXPECT_TRUE(CompareMatrices(output_state_0.velocity_,
                              expected_output_state_0.velocity_, 1e-3,
                              MatrixCompareType::absolute));

  // A small additional rotation applied in the next input.
  Isometry3<double> input_pose_1;
  input_pose_1.linear() =
      AngleAxisd(0.28 * M_PI, Eigen::Vector3d::UnitX()).matrix();
  input_pose_1.translation() << 0.01, -5.0, 10.10;

  CombinedState output_state_1;
  EXPECT_NO_THROW(output_state_1 = UpdateStateCalcOutput(input_pose_1));
  CombinedState expected_output_state_1;
  expected_output_state_1 = AveragePoseAndVelocities(input_pose_1);
  drake::log()->info("velocities returned from test {}",
                     expected_output_state_1.velocity_.transpose());
  drake::log()->info("velocities returned from sys  {}",
                     output_state_1.velocity_.transpose());
  EXPECT_TRUE(
      ComparePoses(output_state_1.pose_, expected_output_state_1.pose_));

  EXPECT_TRUE(CompareMatrices(output_state_1.velocity_,
                              expected_output_state_1.velocity_, 1e-3,
                              MatrixCompareType::absolute));

  Isometry3<double> input_pose_2;
  input_pose_2.linear() =
      AngleAxisd(0.28 * M_PI, Eigen::Vector3d::UnitX()).matrix();
  input_pose_2.translation() << 0.04, -5.0, 10.10;

  // A small additional translation applied in the next input.
  CombinedState output_state_2;
  EXPECT_NO_THROW(output_state_2 = UpdateStateCalcOutput(input_pose_2));
  CombinedState expected_output_state_2 =
      AveragePoseAndVelocities(input_pose_2);
  EXPECT_TRUE(
      (ComparePoses(output_state_2.pose_, expected_output_state_2.pose_)));

  //  drake::log()->info("So far ok 2");
  EXPECT_TRUE(CompareMatrices(output_state_2.velocity_,
                              expected_output_state_2.velocity_, 1e-3,
                              MatrixCompareType::absolute));

  // A small additional rotation applied with a sign inverted quaternion.
  Isometry3<double> input_pose_3;
  Quaterniond test_pose_quaternion{
      AngleAxisd(0.28 * M_PI, Eigen::Vector3d::UnitX())};
  Quaterniond sign_inverted_pose_quaternion(
      -test_pose_quaternion.w(), -test_pose_quaternion.x(),
      -test_pose_quaternion.y(), -test_pose_quaternion.z());
  input_pose_3.linear() = sign_inverted_pose_quaternion.matrix();
  input_pose_3.translation() << 0.04, -5.0, 10.10;

  CombinedState output_state_3;
  EXPECT_NO_THROW(output_state_3 = UpdateStateCalcOutput(input_pose_3));
  CombinedState expected_output_state_3 =
      AveragePoseAndVelocities(input_pose_3);
  EXPECT_TRUE(
      (ComparePoses(output_state_3.pose_, expected_output_state_3.pose_)));

  EXPECT_TRUE(CompareMatrices(output_state_3.velocity_,
                              expected_output_state_3.velocity_, 1e-3,
                              MatrixCompareType::absolute));
}

}  // namespace
}  // namespace perception
}  // namespace manipulation
}  // namespace drake
