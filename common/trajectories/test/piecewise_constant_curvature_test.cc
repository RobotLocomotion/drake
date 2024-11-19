#include <random>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/trajectories/piecewise_constant_curvature_trajectory.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

using Eigen::Vector3d;
using Eigen::VectorXd;

static double kTolerance = std::sqrt(std::numeric_limits<double>::epsilon());

namespace drake {
namespace trajectories {
namespace {

// Generates a vector of random orientation using randomized axis angles.
template <typename T>
std::vector<T> GenerateRandomCurvatures(std::vector<double> times,
                                        std::default_random_engine* generator,
                                        double max_angle = (10 * M_PI)) {
  int size = times.size() - 1;
  DRAKE_DEMAND(size >= 0);
  std::vector<T> curavtures(size);
  std::uniform_real_distribution<double> travel_angle(-max_angle, max_angle);
  std::bernoulli_distribution is_linear(0.25);
  for (int i = 0; i < size; ++i) {
    double duration = times[i + 1] - times[i];
    if (is_linear(*generator)) {
      curavtures[i] = T(0);
    } else {
      // pick a curvature so the angle change on the segment is in (-pi, pi)
      curavtures[i] = T(travel_angle(*generator) / duration);
    }
  }
  return curavtures;
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

GTEST_TEST(TestPiecewiseQuaternionSlerp, TestAnalytical) {
  double r = 2;
  double kappa = 1 / r;
  int num_segments = 10000;
  std::vector<double> segment_breaks(num_segments + 1);
  std::vector<double> curvatures(num_segments);
  segment_breaks[0] = 0;
  for (int i = 0; i < num_segments; ++i) {
    segment_breaks[i + 1] = 2 * M_PI * r * (i + 1.) / num_segments;
    curvatures[i] = kappa;
  }
  Vector3d plane_normal = Vector3d::UnitZ();
  Vector3d curve_tangent = Vector3d::UnitX();
  PiecewiseConstantCurvatureTrajectory<double> trajectory(
      segment_breaks, curvatures, curve_tangent, plane_normal);

  for (double l = segment_breaks.front(); l < segment_breaks.back();
       l += 0.01) {
    math::RigidTransformd expected_pose = math::RigidTransformd(
        math::RotationMatrixd::MakeZRotation(l * kappa),
        Vector3d(r * sin(l * kappa), r * (1 - cos(l * kappa)), 0));
    multibody::SpatialVelocity<double> expected_velocity(
        kappa * Vector3d::UnitZ(), Vector3d(cos(l * kappa), sin(l * kappa), 0));
    multibody::SpatialAcceleration<double> expected_acceleration(
        Vector3d::Zero(),
        Vector3d(-kappa * sin(l * kappa), kappa * cos(l * kappa), 0));

    auto actual_pose = trajectory.GetPose(l);
    auto actual_velocity = trajectory.GetVelocity(l);
    auto actual_acceleration = trajectory.GetAcceleration(l);

    EXPECT_TRUE(actual_pose.IsNearlyEqualTo(expected_pose, kTolerance));
    EXPECT_TRUE(actual_velocity.IsApprox(expected_velocity, kTolerance));
    EXPECT_TRUE(
        actual_acceleration.IsApprox(expected_acceleration, kTolerance));
  }
}

// Tests CheckSlerpInterpolation for PiecewiseQuaternionSlerp
// generated from random breaks and samples.
GTEST_TEST(TestPiecewiseConstantCuravatureTrajectory,
           TestRandomizedTrajectory) {
  std::default_random_engine generator(123);
  int num_segments = 10000;
  std::vector<double> breaks =
      PiecewiseTrajectory<double>::RandomSegmentTimes(num_segments, generator);
  std::vector<double> curves =
      GenerateRandomCurvatures<double>(breaks, &generator);
  VectorXd segment_times_vector =
      Eigen::Map<VectorXd>(breaks.data(), breaks.size());
  VectorXd segment_curves_vector =
      Eigen::Map<VectorXd>(curves.data(), curves.size());
  VectorXd segment_durations = segment_times_vector.tail(num_segments) -
                               segment_times_vector.head(num_segments);
  VectorXd segment_angles =
      segment_curves_vector.cwiseProduct(segment_durations);
  std::vector<MatrixX<double>> cumulative_angles(num_segments + 1);
  cumulative_angles[0] = MatrixX<double>::Zero(1, 1);
  for (int i = 1; i < num_segments + 1; ++i) {
    cumulative_angles[i] =
        cumulative_angles[i - 1] + Vector1d(segment_angles[i - 1]);
  }
  auto initial_quat = GenerateRandomQuaternion<double>(&generator);
  auto initial_rotation = drake::math::RotationMatrixd(initial_quat);
  Eigen::Vector3d initial_plane_normal = initial_rotation.col(2);
  Eigen::Vector3d initial_curve_tangent = initial_rotation.col(0);

  auto angle_spline =
      PiecewisePolynomial<double>::FirstOrderHold(breaks, cumulative_angles);
  PiecewiseConstantCurvatureTrajectory<double> curve_spline(
      breaks, curves, initial_curve_tangent, initial_plane_normal);
  auto initial_position = curve_spline.GetPose(breaks.front()).translation();
  // Check dense interpolated quaternions.
  for (double l = breaks.front(); l < breaks.back(); l += 0.01) {
    auto curve_pose = curve_spline.GetPose(l);
    auto curve_rotation = curve_pose.rotation();
    auto curve_position = curve_pose.translation();

    auto curve_velocity = curve_spline.GetVelocity(l);
    auto translational_velocity = curve_velocity.translational();
    auto rotational_velocity = curve_velocity.rotational();

    auto curve_acceleration = curve_spline.GetAcceleration(l);
    auto translational_acceleration = curve_acceleration.translational();
    auto rotational_acceleration = curve_acceleration.rotational();

    // Rotation should just be cumulative angle rotation about plane axis
    auto relative_rotation_expected = drake::math::RotationMatrixd(
        AngleAxis<double>(angle_spline.value(l)(0, 0), initial_plane_normal));
    auto curve_rotation_expected =
        relative_rotation_expected * initial_rotation;
    EXPECT_TRUE(
        curve_rotation.IsNearlyEqualTo(curve_rotation_expected, kTolerance));

    // Displacement should be within plane
    auto displacement = curve_position - initial_position;
    EXPECT_NEAR(displacement.dot(initial_plane_normal), 0, kTolerance);

    // Translational velocity should be in-plane and unit norm
    EXPECT_NEAR(translational_velocity.dot(initial_plane_normal), 0,
                kTolerance);
    EXPECT_NEAR(translational_velocity.norm(), 1, kTolerance);

    // Rotational velocity should be normal to plane
    EXPECT_NEAR(std::abs(rotational_velocity.dot(initial_plane_normal)),
                rotational_velocity.norm(), kTolerance);

    // Translational acceleration should in-plane and orthogonal to velocity
    EXPECT_NEAR(translational_acceleration.dot(initial_plane_normal), 0,
                kTolerance);
    EXPECT_NEAR(translational_acceleration.dot(translational_velocity), 0,
                kTolerance);

    // Rotational acceleration should be 0 almost everywhere
    EXPECT_NEAR(rotational_acceleration.dot(initial_plane_normal), 0,
                kTolerance);
  }
}

GTEST_TEST(TestPiecewiseConstantCuravatureTrajectory, TestPeriodicity) {
  double r = 2;
  double kappa = 1 / r;
  int num_segments = 10000;
  std::vector<double> segment_breaks_periodic(num_segments + 1);
  std::vector<double> segment_breaks_aperiodic(num_segments + 1);
  std::vector<double> curvatures(num_segments);

  segment_breaks_periodic[0] = 0;
  segment_breaks_aperiodic[0] = 0;

  for (int i = 0; i < num_segments; ++i) {
    segment_breaks_periodic[i + 1] = 2 * M_PI * r * (i + 1.) / num_segments;
    segment_breaks_aperiodic[i + 1] = M_PI * r * (i + 1.) / num_segments;
    curvatures[i] = kappa;
  }
  Vector3d plane_normal = Vector3d::UnitZ();
  Vector3d curve_tangent = Vector3d::UnitX();
  PiecewiseConstantCurvatureTrajectory<double> periodic_trajectory(
      segment_breaks_periodic, curvatures, curve_tangent, plane_normal);

  PiecewiseConstantCurvatureTrajectory<double> aperiodic_trajectory(
      segment_breaks_aperiodic, curvatures, curve_tangent, plane_normal);

  EXPECT_TRUE(periodic_trajectory.IsNearlyPeriodic(kTolerance));
  EXPECT_FALSE(aperiodic_trajectory.IsNearlyPeriodic(kTolerance));
}

GTEST_TEST(TestPiecewiseConstantCuravatureTrajectory, TestScalarConversion) {
  std::vector<double> times{1, 2, 3, 4};
  std::vector<double> curavtures{-1, 0, 1};
  Vector3d plane_normal = Vector3d::UnitZ();
  Vector3d curve_tangent = Vector3d::UnitX();
  PiecewiseConstantCurvatureTrajectory<double> double_trajectory(
      times, curavtures, curve_tangent, plane_normal);
  PiecewiseConstantCurvatureTrajectory<AutoDiffXd> autodiff_trajectory =
      double_trajectory.template CloneToScalar<AutoDiffXd>();
  PiecewiseConstantCurvatureTrajectory<symbolic::Expression>
      expression_trajectory =
          double_trajectory.template CloneToScalar<symbolic::Expression>();

  for (double l = times.front(); l < times.back(); l += 0.01) {
    math::RigidTransform<double> double_pose = double_trajectory.GetPose(l);
    math::RigidTransform<AutoDiffXd> autodiff_pose =
        double_pose.template cast<AutoDiffXd>();
    math::RigidTransform<symbolic::Expression> expression_pose =
        double_pose.template cast<symbolic::Expression>();

    multibody::SpatialVelocity<double> double_velocity =
        double_trajectory.GetVelocity(l);
    multibody::SpatialVelocity<AutoDiffXd> autodiff_velocity(
        double_velocity.rotational().template cast<AutoDiffXd>(),
        double_velocity.translational().template cast<AutoDiffXd>());
    multibody::SpatialVelocity<symbolic::Expression> expression_velocity(
        double_velocity.rotational().template cast<symbolic::Expression>(),
        double_velocity.translational().template cast<symbolic::Expression>());

    multibody::SpatialAcceleration<double> double_acceleration =
        double_trajectory.GetAcceleration(l);
    multibody::SpatialAcceleration<AutoDiffXd> autodiff_acceleration(
        double_acceleration.rotational().template cast<AutoDiffXd>(),
        double_acceleration.translational().template cast<AutoDiffXd>());
    multibody::SpatialAcceleration<symbolic::Expression>
        expression_acceleration(double_acceleration.rotational()
                                    .template cast<symbolic::Expression>(),
                                double_acceleration.translational()
                                    .template cast<symbolic::Expression>());

    EXPECT_TRUE(autodiff_pose.IsNearlyEqualTo(autodiff_trajectory.GetPose(l),
                                              kTolerance));
    EXPECT_TRUE(expression_pose.IsNearlyEqualTo(
        expression_trajectory.GetPose(l), kTolerance));

    EXPECT_TRUE(autodiff_velocity.IsApprox(autodiff_trajectory.GetVelocity(l),
                                           kTolerance));
    EXPECT_TRUE(expression_velocity.IsApprox(
        expression_trajectory.GetVelocity(l), kTolerance));

    EXPECT_TRUE(autodiff_acceleration.IsApprox(
        autodiff_trajectory.GetAcceleration(l), kTolerance));
    EXPECT_TRUE(expression_acceleration.IsApprox(
        expression_trajectory.GetAcceleration(l), kTolerance));
  }
}

}  // namespace
}  // namespace trajectories
}  // namespace drake
