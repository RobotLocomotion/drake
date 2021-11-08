#include "drake/common/trajectories/piecewise_pose.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace trajectories {

using Eigen::Vector3d;
using math::RigidTransform;
using math::RigidTransformd;
using math::RotationMatrixd;
using trajectories::PiecewisePolynomial;
using trajectories::PiecewiseQuaternionSlerp;

class PiecewisePoseTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::vector<double> times = {1, 2};
    std::vector<AngleAxis<double>> rot_samples(times.size());
    std::vector<MatrixX<double>> pos_samples(times.size(),
                                             MatrixX<double>(3, 1));

    rot_samples[0] = AngleAxis<double>(0.3, Vector3<double>::UnitX());
    rot_samples[1] = AngleAxis<double>(-1, Vector3<double>::UnitY());

    pos_samples[0] << 0.3, 0, -0.5;
    pos_samples[1] << 1, -1, 3;

    std::vector<RigidTransform<double>> samples(times.size());
    for (size_t i = 0; i < times.size(); ++i) {
      samples[i] = RigidTransform<double>(rot_samples[i], pos_samples[i]);
    }

    Vector3<double> start_vel(Vector3<double>::Zero());
    Vector3<double> end_vel(Vector3<double>::Zero());

    dut_ = PiecewisePose<double>::MakeCubicLinearWithEndLinearVelocity(
        times, samples, start_vel, end_vel);

    test_times_ = {times.front() - 0.2, times.front(),
                   (times.front() + times.back()) / 2., times.back(),
                   times.back() + 0.3};

    position_ =
        PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
            times, pos_samples, start_vel, end_vel);
    orientation_ = PiecewiseQuaternionSlerp<double>(times, rot_samples);
  }

  PiecewisePose<double> dut_;
  std::vector<double> test_times_;

  PiecewisePolynomial<double> position_;
  PiecewiseQuaternionSlerp<double> orientation_;
};

TEST_F(PiecewisePoseTest, TestConstructor) {
  EXPECT_EQ(dut_.get_number_of_segments(), 1);
  EXPECT_EQ(dut_.get_segment_times()[1], 2);
}

// Tests linear velocity starts and ends at zero.
TEST_F(PiecewisePoseTest, TestEndLinearVelocity) {
  double t0 = dut_.get_position_trajectory().start_time();
  double t1 = dut_.get_position_trajectory().end_time();

  EXPECT_TRUE(drake::CompareMatrices(dut_.GetVelocity(t0).tail<3>(),
                                     Vector3<double>::Zero(), 1e-12,
                                     drake::MatrixCompareType::absolute));

  EXPECT_TRUE(drake::CompareMatrices(dut_.GetVelocity(t1).tail<3>(),
                                     Vector3<double>::Zero(), 1e-12,
                                     drake::MatrixCompareType::absolute));
}

// Tests pose matches that directly interpolated from PiecewiseQuaternionSlerp
// and PiecewisePolynomial.
TEST_F(PiecewisePoseTest, TestPose) {
  for (double time : test_times_) {
    math::RigidTransform<double> expected(orientation_.orientation(time),
                                          position_.value(time));
    EXPECT_TRUE(drake::CompareMatrices(dut_.GetPose(time).GetAsMatrix4(),
                                       expected.GetAsMatrix4(), 1e-12,
                                       drake::MatrixCompareType::absolute));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    EXPECT_TRUE(drake::CompareMatrices(dut_.get_pose(time).GetAsMatrix4(),
                                       expected.GetAsMatrix4(), 1e-12,
                                       drake::MatrixCompareType::absolute));
#pragma GCC diagnostic pop
    EXPECT_TRUE(drake::CompareMatrices(dut_.value(time),
                                       expected.GetAsMatrix4(), 1e-12,
                                       drake::MatrixCompareType::absolute));
  }
}

// Tests velocity matches that directly interpolated from
// PiecewiseQuaternionSlerp and PiecewisePolynomial.
TEST_F(PiecewisePoseTest, TestVelocity) {
  for (double time : test_times_) {
    Vector6<double> expected;
    expected.head<3>() = orientation_.angular_velocity(time);
    expected.tail<3>() = position_.derivative().value(time);

    if (!orientation_.is_time_in_range(time)) expected.head<3>().setZero();
    if (!position_.is_time_in_range(time)) expected.tail<3>().setZero();

    EXPECT_TRUE(drake::CompareMatrices(dut_.GetVelocity(time), expected, 1e-12,
                                       drake::MatrixCompareType::absolute));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    EXPECT_TRUE(drake::CompareMatrices(dut_.get_velocity(time), expected, 1e-12,
                                       drake::MatrixCompareType::absolute));
#pragma GCC diagnostic pop
  }
}

// Tests angular acceleration is always zero because of linear interpolation,
// and linear acceleration matches that interpolated from
// PiecewisePolynomial.
TEST_F(PiecewisePoseTest, TestAcceleration) {
  for (double time : test_times_) {
    Vector6<double> expected;
    expected.head<3>() = Vector3<double>::Zero();
    expected.tail<3>() = position_.derivative(2).value(time);

    if (!position_.is_time_in_range(time)) expected.tail<3>().setZero();

    EXPECT_TRUE(drake::CompareMatrices(dut_.GetAcceleration(time), expected,
                                       1e-12,
                                       drake::MatrixCompareType::absolute));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    EXPECT_TRUE(drake::CompareMatrices(dut_.get_acceleration(time), expected,
                                       1e-12,
                                       drake::MatrixCompareType::absolute));
#pragma GCC diagnostic pop
  }
}

// Tests getters.
TEST_F(PiecewisePoseTest, TestGetTrajectory) {
  EXPECT_TRUE(dut_.get_position_trajectory().isApprox(position_, 1e-12));
  EXPECT_TRUE(dut_.get_orientation_trajectory().is_approx(orientation_, 1e-12));
}

// Tests is_approx().
TEST_F(PiecewisePoseTest, TestIsApprox) {
  std::vector<double> times = {1, 2};
  std::vector<AngleAxis<double>> rot_samples(times.size());
  std::vector<MatrixX<double>> pos_samples(times.size(), MatrixX<double>(3, 1));
  pos_samples[0] << -3, 1, 0;
  pos_samples[1] << -2, -1, 5;

  rot_samples[0] = AngleAxis<double>(0.3, Vector3<double>::UnitX());
  rot_samples[1] = AngleAxis<double>(-1.2, Vector3<double>::UnitY());

  Vector3<double> start_vel(Vector3<double>::Zero());
  Vector3<double> end_vel(Vector3<double>::Zero());

  PiecewisePolynomial<double> new_pos_traj =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          times, pos_samples, start_vel, end_vel);
  PiecewiseQuaternionSlerp<double> new_rot_traj(times, rot_samples);

  PiecewisePose<double> diff_position(new_pos_traj, orientation_);
  EXPECT_TRUE(!diff_position.IsApprox(dut_, 1e-12));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  EXPECT_TRUE(!diff_position.is_approx(dut_, 1e-12));
#pragma GCC diagnostic pop

  PiecewisePose<double> diff_orientation(position_, new_rot_traj);
  EXPECT_TRUE(!diff_orientation.IsApprox(dut_, 1e-12));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  EXPECT_TRUE(!diff_orientation.is_approx(dut_, 1e-12));
#pragma GCC diagnostic pop
}

// Tests getters.
TEST_F(PiecewisePoseTest, TestTrajectoryOverrides) {
  EXPECT_EQ(dut_.rows(), 4);
  EXPECT_EQ(dut_.cols(), 4);
  EXPECT_TRUE(dut_.has_derivative());

  std::vector<double> truncated_test_times_ =
      std::vector<double>(test_times_.begin() + 1, test_times_.end() - 1);

  const auto zeroth_derivative = dut_.MakeDerivative(0);
  const auto first_derivative = dut_.MakeDerivative(1);
  const auto second_derivative = dut_.MakeDerivative(2);

  for (double time : truncated_test_times_) {
    EXPECT_TRUE(drake::CompareMatrices(dut_.value(time),
                                       zeroth_derivative->value(time), 1e-12,
                                       drake::MatrixCompareType::absolute));
    EXPECT_TRUE(drake::CompareMatrices(dut_.value(time),
                                       dut_.EvalDerivative(time, 0), 1e-12,
                                       drake::MatrixCompareType::absolute));

    EXPECT_TRUE(drake::CompareMatrices(dut_.GetVelocity(time),
                                       first_derivative->value(time), 1e-12,
                                       drake::MatrixCompareType::absolute));
    EXPECT_TRUE(drake::CompareMatrices(dut_.GetVelocity(time),
                                       dut_.EvalDerivative(time, 1), 1e-12,
                                       drake::MatrixCompareType::absolute));

    EXPECT_TRUE(drake::CompareMatrices(dut_.GetAcceleration(time),
                                       second_derivative->value(time), 1e-12,
                                       drake::MatrixCompareType::absolute));
    EXPECT_TRUE(drake::CompareMatrices(dut_.GetAcceleration(time),
                                       dut_.EvalDerivative(time, 2), 1e-12,
                                       drake::MatrixCompareType::absolute));
  }
}

GTEST_TEST(PiecewisePoseMakeTest, MakeLinear) {
  const std::vector<double> times = {1., 2., 3.};
  const std::vector<RigidTransformd> poses{
      RigidTransformd(),
      RigidTransformd(RotationMatrixd::MakeXRotation(M_PI / 2),
                      Vector3d{.1, .2, .3}),
      RigidTransformd(Vector3d{.4, .5, .6})};

  PiecewisePose<double> pose_traj =
      PiecewisePose<double>::MakeLinear(times, poses);

  EXPECT_EQ(pose_traj.start_time(), 1.);
  EXPECT_EQ(pose_traj.end_time(), 3.);
  EXPECT_EQ(pose_traj.get_number_of_segments(), 2);

  const auto& position = pose_traj.get_position_trajectory();
  EXPECT_EQ(position.start_time(), 1.);
  EXPECT_EQ(position.end_time(), 3.);
  EXPECT_EQ(position.get_number_of_segments(), 2);
  EXPECT_EQ(position.getSegmentPolynomialDegree(0), 1);
  EXPECT_EQ(position.getSegmentPolynomialDegree(1), 1);

  const auto& orientation = pose_traj.get_orientation_trajectory();
  EXPECT_EQ(orientation.start_time(), 1.);
  EXPECT_EQ(orientation.end_time(), 3.);
  EXPECT_EQ(orientation.get_number_of_segments(), 2);

  EXPECT_TRUE(pose_traj.GetPose(1.5).IsNearlyEqualTo(
      RigidTransformd(RotationMatrixd::MakeXRotation(M_PI / 4),
                      Vector3d{.05, .1, .15}),
      1e-14));
  EXPECT_TRUE(pose_traj.GetPose(2.5).IsNearlyEqualTo(
      RigidTransformd(RotationMatrixd::MakeXRotation(M_PI / 4),
                      Vector3d{.25, .35, .45}),
      1e-14));
}

template <typename T>
void TestScalarType() {
  std::vector<T> times = {1, 2};
  std::vector<AngleAxis<T>> rot_samples(times.size());
  std::vector<MatrixX<T>> pos_samples(times.size(), MatrixX<T>(3, 1));

  rot_samples[0] = AngleAxis<T>(0.3, Vector3<T>::UnitX());
  rot_samples[1] = AngleAxis<T>(-1, Vector3<T>::UnitY());

  pos_samples[0] << 0.3, 0, -0.5;
  pos_samples[1] << 1, -1, 3;

  Vector3<T> start_vel(Vector3<T>::Zero());
  Vector3<T> end_vel(Vector3<T>::Zero());

  PiecewisePolynomial<T> position_traj =
      PiecewisePolynomial<T>::CubicWithContinuousSecondDerivatives(
          times, pos_samples, start_vel, end_vel);
  PiecewiseQuaternionSlerp<T> orientation_traj(times, rot_samples);

  PiecewisePose<T> pose_traj(position_traj, orientation_traj);
}

GTEST_TEST(PiecewisePoseScalarTest, ScalarTypes) {
  TestScalarType<double>();
  TestScalarType<AutoDiffXd>();
  TestScalarType<symbolic::Expression>();
}

}  // namespace trajectories
}  // namespace drake
