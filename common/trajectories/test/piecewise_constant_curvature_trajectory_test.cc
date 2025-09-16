#include "drake/common/trajectories/piecewise_constant_curvature_trajectory.h"

#include <random>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

using drake::math::RigidTransform;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::multibody::SpatialAcceleration;
using drake::multibody::SpatialVelocity;
using Eigen::Vector3d;
using Eigen::VectorXd;

static double kTolerance = std::sqrt(std::numeric_limits<double>::epsilon());

namespace drake {
namespace trajectories {
namespace {

// Generates a vector of random orientation using randomized axis angles.
template <typename T>
std::vector<T> GenerateRandomTurningRates(const std::vector<double>& breaks,
                                          std::default_random_engine* generator,
                                          double max_angle = (10 * M_PI)) {
  const int size = breaks.size() - 1;
  DRAKE_DEMAND(size >= 0);
  std::vector<T> turning_rates(size);
  std::uniform_real_distribution<double> travel_angle(-max_angle, max_angle);
  for (int i = 0; i < size; ++i) {
    double duration = breaks[i + 1] - breaks[i];
    if (i % 2) {
      turning_rates[i] = T(0);
    } else {
      // Pick a turning rate so the angle change on the segment is in (-pi, pi).
      turning_rates[i] = T(travel_angle(*generator) / duration);
    }
  }
  return turning_rates;
}

template <typename Scalar>
Quaternion<Scalar> GenerateRandomQuaternion(
    std::default_random_engine* generator) {
  std::uniform_real_distribution<double> uniform(-10, 10);
  Quaternion<Scalar> ret;
  return Quaternion<Scalar>(AngleAxis<Scalar>(
      uniform(*generator),
      Vector3<Scalar>(uniform(*generator), uniform(*generator),
                      uniform(*generator))));
}

GTEST_TEST(TestPiecewiseConstantCurvatureTrajectory, TestAnalytical) {
  const double r = 2;
  const double kappa = 1 / r;
  const int num_segments = 3;
  std::vector<double> segment_breaks(num_segments + 1);
  std::vector<double> turning_rates(num_segments);
  segment_breaks[0] = 0;
  for (int i = 0; i < num_segments; ++i) {
    segment_breaks[i + 1] = 2 * M_PI * r * (i + 1.) / num_segments;
    turning_rates[i] = kappa;
  }
  // Construct trajectory consisting of the r-radius circle,
  // starting from (r, 0, 0) in the x-y plane.
  const Vector3d plane_normal = Vector3d::UnitZ();
  const Vector3d curve_tangent = Vector3d::UnitY();
  PiecewiseConstantCurvatureTrajectory<double> trajectory(
      segment_breaks, turning_rates, curve_tangent, plane_normal,
      r * Vector3d::UnitX());

  const RotationMatrixd expected_initial_rotation =
      RotationMatrixd::MakeZRotation(M_PI / 2);

  const double s_dot = 0.5;
  const double s_ddot = 0.25;
  const double ds = .01;
  for (double s = segment_breaks.front(); s < segment_breaks.back(); s += ds) {
    const double angle_change = s * kappa;
    const Vector3d expected_position =
        Vector3d(r * cos(angle_change), r * sin(angle_change), 0);
    const RotationMatrixd expected_rotation =
        expected_initial_rotation *
        RotationMatrixd::MakeZRotation(angle_change);

    const RigidTransformd expected_pose =
        RigidTransformd(expected_rotation, expected_position);
    // Normal to circle points to center (0, 0, 0) from position.
    const Vector3d expected_normal = -expected_position.normalized();
    // Tangent to circle is pi/2 radians clockwise from normal.
    const Vector3d expected_tangent =
        RotationMatrixd::MakeZRotation(-M_PI / 2) * expected_normal;

    const SpatialVelocity<double> expected_velocity_A(
        s_dot * kappa * Vector3d::UnitZ(), s_dot * expected_tangent);

    const SpatialVelocity<double> expected_velocity_M(
        Vector3d(0, 0, s_dot * kappa), Vector3d(s_dot, 0, 0));

    const SpatialAcceleration<double> expected_acceleration_A(
        s_ddot * kappa * Vector3d::UnitZ(),
        s_ddot * expected_tangent + s_dot * s_dot * kappa * expected_normal);

    const SpatialAcceleration<double> expected_acceleration_M(
        Vector3d(0, 0, s_ddot * kappa),
        Vector3d(s_ddot, s_dot * s_dot * kappa, 0));

    const RigidTransformd pose = trajectory.CalcPose(s);
    const SpatialVelocity<double> velocity_A =
        trajectory.CalcSpatialVelocity(s, s_dot);
    const SpatialVelocity<double> velocity_M =
        trajectory.CalcSpatialVelocityInM(s, s_dot);
    const SpatialAcceleration<double> acceleration_A =
        trajectory.CalcSpatialAcceleration(s, s_dot, s_ddot);
    const SpatialAcceleration<double> acceleration_M =
        trajectory.CalcSpatialAccelerationInM(s, s_dot, s_ddot);

    EXPECT_TRUE(pose.IsNearlyEqualTo(expected_pose, kTolerance));
    EXPECT_TRUE(velocity_A.IsApprox(expected_velocity_A, kTolerance));
    EXPECT_TRUE(velocity_M.IsApprox(expected_velocity_M, kTolerance));
    EXPECT_TRUE(acceleration_A.IsApprox(expected_acceleration_A, kTolerance));
    EXPECT_TRUE(acceleration_M.IsApprox(expected_acceleration_M, kTolerance));
  }
}

// Tests CheckSlerpInterpolation for PiecewiseQuaternionSlerp
// generated from random breaks and samples.
GTEST_TEST(TestPiecewiseConstantCurvatureTrajectory, TestRandomizedTrajectory) {
  std::default_random_engine generator(123);
  const int num_segments = 10;
  std::vector<double> breaks =
      PiecewiseTrajectory<double>::RandomSegmentTimes(num_segments, generator);
  const double start = breaks[0];
  for (double& s : breaks) {
    s -= start;
  }
  const std::vector<double> turning_rates =
      GenerateRandomTurningRates<double>(breaks, &generator);
  const VectorXd breaks_vector =
      Eigen::Map<const VectorXd>(breaks.data(), breaks.size());
  const VectorXd turning_rates_vector =
      Eigen::Map<const VectorXd>(turning_rates.data(), turning_rates.size());
  const VectorXd segment_durations =
      breaks_vector.tail(num_segments) - breaks_vector.head(num_segments);
  const VectorXd segment_angles =
      turning_rates_vector.cwiseProduct(segment_durations);
  std::vector<MatrixX<double>> cumulative_angles(num_segments + 1);
  cumulative_angles[0] = MatrixX<double>::Zero(1, 1);
  for (int i = 1; i < num_segments + 1; ++i) {
    cumulative_angles[i] =
        cumulative_angles[i - 1] + Vector1d(segment_angles[i - 1]);
  }
  const Quaternion<double> initial_quat =
      GenerateRandomQuaternion<double>(&generator);
  const RotationMatrixd initial_rotation(initial_quat);
  Eigen::Vector3d initial_plane_normal = initial_rotation.col(2);
  Eigen::Vector3d initial_curve_tangent = initial_rotation.col(0);

  const PiecewisePolynomial<double> expected_angle_trajectory =
      PiecewisePolynomial<double>::FirstOrderHold(breaks, cumulative_angles);
  const PiecewiseConstantCurvatureTrajectory<double> trajectory(
      breaks, turning_rates, initial_curve_tangent, initial_plane_normal,
      Vector3d::Zero());

  // Check that curvatures are reported properly.
  for (int i = 0; i < num_segments; ++i) {
    const double s = (breaks[i + 1] + breaks[i]) / 2;  // middle of segment
    EXPECT_EQ(trajectory.curvature(s), turning_rates[i]);
  }

  const Vector3d initial_position =
      trajectory.CalcPose(breaks.front()).translation();
  // Check dense interpolated quaternions.
  for (double l = breaks.front(); l < breaks.back(); l += 0.01) {
    const RigidTransformd pose = trajectory.CalcPose(l);
    const RotationMatrixd rotation = pose.rotation();
    const Vector3d position = pose.translation();

    const SpatialVelocity<double> spatial_velocity =
        trajectory.CalcSpatialVelocity(l, 1.);
    const Vector3d translational_velocity = spatial_velocity.translational();
    const Vector3d rotational_velocity = spatial_velocity.rotational();

    const SpatialAcceleration<double> spatial_acceleration =
        trajectory.CalcSpatialAcceleration(l, 1., 0.);
    const Vector3d translational_acceleration =
        spatial_acceleration.translational();
    const Vector3d rotational_acceleration = spatial_acceleration.rotational();

    // Rotation should just be cumulative angle rotation about plane axis.
    const RotationMatrixd expected_relative_rotation =
        RotationMatrixd(AngleAxis<double>(
            expected_angle_trajectory.value(l)(0, 0), initial_plane_normal));
    const RotationMatrixd expected_rotation =
        expected_relative_rotation * initial_rotation;
    EXPECT_TRUE(rotation.IsNearlyEqualTo(expected_rotation, kTolerance));

    // Displacement should be within plane.
    const Vector3d displacement = position - initial_position;
    EXPECT_NEAR(displacement.dot(initial_plane_normal), 0, kTolerance);

    // Translational velocity should be in-plane and unit norm.
    EXPECT_NEAR(translational_velocity.dot(initial_plane_normal), 0,
                kTolerance);
    EXPECT_NEAR(translational_velocity.norm(), 1, kTolerance);

    // Rotational velocity should be normal to plane.
    EXPECT_NEAR(std::abs(rotational_velocity.dot(initial_plane_normal)),
                rotational_velocity.norm(), kTolerance);

    // Translational acceleration should in-plane and orthogonal to velocity.
    EXPECT_NEAR(translational_acceleration.dot(initial_plane_normal), 0,
                kTolerance);
    EXPECT_NEAR(translational_acceleration.dot(translational_velocity), 0,
                kTolerance);

    // Rotational acceleration should be 0 almost everywhere.
    EXPECT_NEAR(rotational_acceleration.dot(initial_plane_normal), 0,
                kTolerance);
  }
}

GTEST_TEST(TestPiecewiseConstantCurvatureTrajectory, TestPeriodicity) {
  const double r = 2;
  const double length = 2 * M_PI * r;
  const double kappa = 1 / r;
  const int num_segments = 10;
  std::vector<double> segment_breaks_periodic(num_segments + 1);
  std::vector<double> segment_breaks_aperiodic(num_segments + 1);
  std::vector<double> turning_rates(num_segments);

  segment_breaks_periodic[0] = 0;
  segment_breaks_aperiodic[0] = 0;

  for (int i = 0; i < num_segments; ++i) {
    segment_breaks_periodic[i + 1] = 2 * M_PI * r * (i + 1.) / num_segments;
    segment_breaks_aperiodic[i + 1] = M_PI * r * (i + 1.) / num_segments;
    turning_rates[i] = kappa;
  }
  const Vector3d plane_normal = Vector3d::UnitZ();
  const Vector3d curve_tangent = Vector3d::UnitX();
  const PiecewiseConstantCurvatureTrajectory<double> periodic_trajectory(
      segment_breaks_periodic, turning_rates, curve_tangent, plane_normal,
      Vector3d::Zero(), true);

  const PiecewiseConstantCurvatureTrajectory<double> aperiodic_trajectory(
      segment_breaks_aperiodic, turning_rates, curve_tangent, plane_normal,
      Vector3d::Zero());

  EXPECT_TRUE(periodic_trajectory.is_periodic());
  EXPECT_TRUE(periodic_trajectory.EndpointsAreNearlyEqual(kTolerance));
  EXPECT_FALSE(aperiodic_trajectory.is_periodic());
  EXPECT_FALSE(aperiodic_trajectory.EndpointsAreNearlyEqual(kTolerance));

  // Test periodicity for at arbitrary value.
  const double s = 1.5;
  const RigidTransformd X_AF_s0 = periodic_trajectory.CalcPose(s);
  const RigidTransformd X_AF_s1 = periodic_trajectory.CalcPose(s + length);
  const RigidTransformd X_AF_s3 =
      periodic_trajectory.CalcPose(s + 3.0 * length);
  EXPECT_TRUE(X_AF_s0.IsNearlyEqualTo(X_AF_s1, kTolerance));
  EXPECT_TRUE(X_AF_s0.IsNearlyEqualTo(X_AF_s3, kTolerance));

  // Check that curvature for periodic trajectory is properly wrapped.
  for (int i = 0; i < num_segments; ++i) {
    // Should be at the middle of each segment after wrapping.
    const double s2 =
        2 * length +
        (segment_breaks_periodic[i + 1] + segment_breaks_periodic[i]) / 2;
    EXPECT_EQ(periodic_trajectory.curvature(s2), turning_rates[i]);
  }
}

GTEST_TEST(TestPiecewiseConstantCurvatureTrajectory, TestScalarConversion) {
  const std::vector<double> breaks{0, 1, 2, 3};
  const std::vector<double> turning_rates{-1, 0, 1};
  const Vector3d plane_normal = Vector3d::UnitZ();
  const Vector3d curve_tangent = Vector3d::UnitX();
  const PiecewiseConstantCurvatureTrajectory<double> double_trajectory(
      breaks, turning_rates, curve_tangent, plane_normal, Vector3d::Zero(),
      true);
  const PiecewiseConstantCurvatureTrajectory<AutoDiffXd> autodiff_trajectory(
      double_trajectory);
  const PiecewiseConstantCurvatureTrajectory<symbolic::Expression>
      expression_trajectory(double_trajectory);

  EXPECT_TRUE(autodiff_trajectory.is_periodic() ==
              double_trajectory.is_periodic());
  EXPECT_TRUE(expression_trajectory.is_periodic() ==
              double_trajectory.is_periodic());

  const double s_dot = 2.;
  const double s_ddot = 3.;
  const AutoDiffXd s_dot_ad(s_dot);
  const symbolic::Expression s_dot_exp(s_dot);
  const AutoDiffXd s_ddot_ad(s_ddot);
  const symbolic::Expression s_ddot_exp(s_ddot);

  for (double l = breaks.front(); l < breaks.back(); l += 0.01) {
    const RigidTransformd double_pose = double_trajectory.CalcPose(l);
    const RigidTransform<AutoDiffXd> autodiff_pose =
        double_pose.template cast<AutoDiffXd>();
    const RigidTransform<symbolic::Expression> expression_pose =
        double_pose.template cast<symbolic::Expression>();

    const SpatialVelocity<double> double_velocity =
        double_trajectory.CalcSpatialVelocity(l, s_dot);
    const SpatialVelocity<AutoDiffXd> autodiff_velocity(
        double_velocity.rotational().template cast<AutoDiffXd>(),
        double_velocity.translational().template cast<AutoDiffXd>());
    const SpatialVelocity<symbolic::Expression> expression_velocity(
        double_velocity.rotational().template cast<symbolic::Expression>(),
        double_velocity.translational().template cast<symbolic::Expression>());

    const SpatialAcceleration<double> double_acceleration =
        double_trajectory.CalcSpatialAcceleration(l, s_dot, s_ddot);
    const SpatialAcceleration<AutoDiffXd> autodiff_acceleration(
        double_acceleration.rotational().template cast<AutoDiffXd>(),
        double_acceleration.translational().template cast<AutoDiffXd>());
    const SpatialAcceleration<symbolic::Expression> expression_acceleration(
        double_acceleration.rotational().template cast<symbolic::Expression>(),
        double_acceleration.translational()
            .template cast<symbolic::Expression>());

    EXPECT_TRUE(autodiff_pose.IsNearlyEqualTo(autodiff_trajectory.CalcPose(l),
                                              kTolerance));
    EXPECT_TRUE(expression_pose.IsNearlyEqualTo(
        expression_trajectory.CalcPose(l), kTolerance));

    EXPECT_TRUE(autodiff_velocity.IsApprox(
        autodiff_trajectory.CalcSpatialVelocity(l, s_dot_ad), kTolerance));
    EXPECT_TRUE(expression_velocity.IsApprox(
        expression_trajectory.CalcSpatialVelocity(l, s_dot_exp), kTolerance));

    EXPECT_TRUE(autodiff_acceleration.IsApprox(
        autodiff_trajectory.CalcSpatialAcceleration(l, s_dot_ad, s_ddot_ad),
        kTolerance));
    EXPECT_TRUE(expression_acceleration.IsApprox(
        expression_trajectory.CalcSpatialAcceleration(l, s_dot_exp, s_ddot_exp),
        kTolerance));
  }
}

}  // namespace
}  // namespace trajectories
}  // namespace drake
