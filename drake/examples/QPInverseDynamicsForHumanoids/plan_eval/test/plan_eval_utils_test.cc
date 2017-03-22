#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/plan_eval_utils.h"

#include <gtest/gtest.h>
#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

class PiecewiseCubicTrajectoryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::vector<double> times = {0, 2, 3, 4};
    std::vector<MatrixX<double>> knots(times.size(),
                                       MatrixX<double>::Zero(2, 1));
    knots[0] << 0, 1;
    knots[1] << 2, -3;
    knots[2] << 1.2, 5;
    knots[3] << -1, 6;

    test_times_ = {times.front() - 0.2, times.front(),
                   (times.front() + times.back()) / 2., times.back(),
                   times.back() + 0.3};

    pos_ = PiecewisePolynomial<double>::Cubic(times, knots);
    dut_ = PiecewiseCubicTrajectory<double>(pos_);
    vel_ = pos_.derivative();
    acc_ = vel_.derivative();
  }

  PiecewiseCubicTrajectory<double> dut_;
  std::vector<double> test_times_;

  PiecewisePolynomial<double> pos_;
  PiecewisePolynomial<double> vel_;
  PiecewisePolynomial<double> acc_;
};

// Tests get position matches PiecewisePolynomial.
TEST_F(PiecewiseCubicTrajectoryTest, GetPosition) {
  for (double time : test_times_) {
    EXPECT_TRUE(drake::CompareMatrices(pos_.value(time),
                                       dut_.get_position(time), 1e-12,
                                       drake::MatrixCompareType::absolute));
  }
}

// Tests get velocity matches PiecewisePolynomial when time is in range,
// and zero otherwise.
TEST_F(PiecewiseCubicTrajectoryTest, GetVelocity) {
  for (double time : test_times_) {
    VectorX<double> expected = vel_.value(time);
    if (!pos_.isTimeInRange(time)) expected.setZero();

    EXPECT_TRUE(drake::CompareMatrices(expected, dut_.get_velocity(time), 1e-12,
                                       drake::MatrixCompareType::absolute));
  }
}

// Tests get acceleration matches PiecewisePolynomial when time is in range,
// and zero otherwise.
TEST_F(PiecewiseCubicTrajectoryTest, GetAcceleration) {
  for (double time : test_times_) {
    VectorX<double> expected = acc_.value(time);
    if (!pos_.isTimeInRange(time)) expected.setZero();

    EXPECT_TRUE(drake::CompareMatrices(expected, dut_.get_acceleration(time),
                                       1e-12,
                                       drake::MatrixCompareType::absolute));
  }
}

class PiecewiseCartesianTrajectoryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::vector<double> times = {1, 2};
    eigen_aligned_std_vector<AngleAxis<double>> rot_knots(times.size());
    std::vector<Vector3<double>> pos_knots(times.size());

    rot_knots[0] = AngleAxis<double>(0.3, Vector3<double>::UnitX());
    rot_knots[1] = AngleAxis<double>(-1, Vector3<double>::UnitY());

    pos_knots[0] << 0.3, 0, -0.5;
    pos_knots[1] << 1, -1, 3;

    std::vector<Isometry3<double>> knots(times.size());
    for (size_t i = 0; i < times.size(); ++i) {
      knots[i].translation() = pos_knots[i];
      knots[i].linear() = rot_knots[i].toRotationMatrix();
      knots[i].makeAffine();
    }

    dut_ = PiecewiseCartesianTrajectory<
        double>::MakeCubicLinearWithEndLinearVelocity(times, knots,
                                                      Vector3<double>::Zero(),
                                                      Vector3<double>::Zero());

    test_times_ = {times.front() - 0.2, times.front(),
                   (times.front() + times.back()) / 2., times.back(),
                   times.back() + 0.3};

    orientation_ = PiecewiseQuaternionSlerp<double>(times, rot_knots);
  }

  PiecewiseCartesianTrajectory<double> dut_;
  std::vector<double> test_times_;

  // Not testing position part explicitly, assumes those are covered by
  // PiecewiseCubicTrajectoryTest.
  PiecewiseQuaternionSlerp<double> orientation_;
};

// Tests linear velocity starts and ends at zero.
TEST_F(PiecewiseCartesianTrajectoryTest, TestEndLinearVelocity) {
  double t0 = dut_.get_position_trajectory().get_start_time();
  double t1 = dut_.get_position_trajectory().get_end_time();

  EXPECT_TRUE(drake::CompareMatrices(dut_.get_velocity(t0).tail<3>(),
                                     Vector3<double>::Zero(), 1e-12,
                                     drake::MatrixCompareType::absolute));

  EXPECT_TRUE(drake::CompareMatrices(dut_.get_velocity(t1).tail<3>(),
                                     Vector3<double>::Zero(), 1e-12,
                                     drake::MatrixCompareType::absolute));
}

// Tests angular velocity matches PiecewiseQuaternionSlerp when time is in
// range, and zero otherwise.
TEST_F(PiecewiseCartesianTrajectoryTest, TestAngularVelocity) {
  for (double time : test_times_) {
    Vector3<double> expected = orientation_.angular_velocity(time);
    if (!orientation_.isTimeInRange(time)) expected.setZero();

    EXPECT_TRUE(drake::CompareMatrices(dut_.get_velocity(time).head<3>(),
                                       expected, 1e-12,
                                       drake::MatrixCompareType::absolute));
  }
}

// Tests angular acceleration is always zero because of first order
// interpolation.
TEST_F(PiecewiseCartesianTrajectoryTest, TestAngularAccelertaion) {
  for (double time : test_times_) {
    EXPECT_TRUE(drake::CompareMatrices(dut_.get_acceleration(time).head<3>(),
                                       Vector3<double>::Zero(), 1e-12,
                                       drake::MatrixCompareType::absolute));
  }
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
