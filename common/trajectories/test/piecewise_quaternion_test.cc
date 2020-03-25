#include "drake/common/trajectories/piecewise_quaternion.h"

#include <random>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/wrap_to.h"

namespace drake {
namespace trajectories {
namespace {

// Returns q1 = quat(omega * dt) * q0
template <typename Scalar>
Quaternion<double> EulerIntegrateQuaternion(const Quaternion<Scalar>& q0,
                                            const Vector3<Scalar>& omega,
                                            double dt) {
  Vector3<Scalar> delta = omega * dt;
  Quaternion<Scalar> q1;

  if (delta.norm() < Eigen::NumTraits<Scalar>::epsilon()) {
    q1 = q0;
  } else {
    Quaternion<Scalar> q_delta =
        Quaternion<Scalar>(AngleAxis<Scalar>(delta.norm(), delta.normalized()));
    q1 = q_delta * q0;
  }
  q1.normalize();
  return q1;
}

// Checks q[n].dot(q[n-1]) >= 0
template <typename Scalar>
bool CheckClosest(
    const std::vector<Quaternion<Scalar>>& quaternions) {
  if (quaternions.size() <= 1) return true;
  for (size_t i = 1; i < quaternions.size(); ++i) {
    if (quaternions[i].dot(quaternions[i - 1]) < 0) return false;
  }
  return true;
}

// Let i be the head of the segment where t falls in.
// Checks spline.orientation(t) = quat(omega_i * (t - t_i)) * q_i
template <typename Scalar>
bool CheckSlerpInterpolation(const PiecewiseQuaternionSlerp<Scalar>& spline,
                             double t) {
  t = std::min(t, spline.end_time());
  t = std::max(t, spline.start_time());

  int i = spline.get_segment_index(t);
  double t0 = spline.start_time(i);
  double t1 = spline.end_time(i);
  t = std::min(t, t1);
  t = std::max(t, t0);
  double dt = t - t0;
  DRAKE_DEMAND(dt <= spline.duration(i));

  Quaternion<Scalar> q_spline = spline.orientation(t);

  Quaternion<Scalar> q0 = spline.orientation(t0);
  Vector3<Scalar> w0 = spline.angular_velocity(t0);
  Quaternion<Scalar> q1 = EulerIntegrateQuaternion(q0, w0, dt);

  if (q_spline.isApprox(q1, 1e-10))
    return true;
  else
    return false;
}

// Generates a vector of random orientation using randomized axis angles.
template <typename Scalar>
std::vector<Quaternion<Scalar>> GenerateRandomQuaternions(
    int size, std::default_random_engine* generator) {
  DRAKE_DEMAND(size >= 0);
  std::vector<Quaternion<Scalar>> ret(size);
  std::uniform_real_distribution<double> uniform(-10, 10);
  for (int i = 0; i < size; ++i) {
    ret[i] = Quaternion<Scalar>(AngleAxis<Scalar>(
        uniform(*generator),
        Vector3<Scalar>(uniform(*generator), uniform(*generator),
                        uniform(*generator))));
  }
  return ret;
}

// Tests CheckSlerpInterpolation and "closestness" for PiecewiseQuaternionSlerp
// generated from random breaks and samples.
GTEST_TEST(TestPiecewiseQuaternionSlerp,
           TestRandomizedPiecewiseQuaternionSlerp) {
  std::default_random_engine generator(123);
  int N = 10000;
  std::vector<double> time =
      PiecewiseTrajectory<double>::RandomSegmentTimes(N - 1, generator);
  std::vector<Quaternion<double>> quat =
      GenerateRandomQuaternions<double>(N, &generator);

  PiecewiseQuaternionSlerp<double> rot_spline(time, quat);

  // Check dense interpolated quaternions.
  for (double t = time.front(); t < time.back(); t += 0.01) {
    EXPECT_TRUE(CheckSlerpInterpolation(rot_spline, t));
  }

  EXPECT_TRUE(CheckClosest(rot_spline.get_quaternion_samples()));
}

// Tests when the given quaternions are not "closest" to the previous one.
// Also tests the returned angular velocity against known values.
GTEST_TEST(TestPiecewiseQuaternionSlerp,
           TestShortestQuaternionPiecewiseQuaternionSlerp) {
  std::vector<double> time = {0, 1.6, 2.32};
  std::vector<double> ang = {1, 2.4 - 2 * M_PI, 5.3};
  Vector3<double> axis = Vector3<double>(1, 2, 3).normalized();
  std::vector<Quaternion<double>> quat(ang.size());
  for (size_t i = 0; i < ang.size(); ++i) {
    quat[i] = Quaternion<double>(AngleAxis<double>(ang[i], axis));
  }

  PiecewiseQuaternionSlerp<double> rot_spline(time, quat);
  const std::vector<Quaternion<double>>& internal_quat =
      rot_spline.get_quaternion_samples();

  EXPECT_TRUE(CheckClosest(internal_quat));
  EXPECT_FALSE(CheckClosest(quat));

  for (size_t i = 0; i < time.size() - 1; ++i) {
    EXPECT_TRUE(CompareMatrices(rot_spline.orientation(time[i]).coeffs(),
                                internal_quat[i].coeffs(), 1e-10,
                                MatrixCompareType::absolute));
    double ang_diff = ang[i + 1] - ang[i];
    double time_diff = time[i + 1] - time[i];
    double omega = math::wrap_to(ang_diff, -M_PI, M_PI) / time_diff;
    EXPECT_TRUE(CompareMatrices(rot_spline.angular_velocity(time[i]),
                                omega * axis, 1e-10,
                                MatrixCompareType::absolute));
  }

  // Check dense interpolated quaternions.
  for (double t = time.front(); t < time.back(); t += 0.01) {
    EXPECT_TRUE(CheckSlerpInterpolation(rot_spline, t));
  }
}

GTEST_TEST(TestPiecewiseQuaternionSlerp,
           TestIdenticalPiecewiseQuaternionSlerp) {
  std::vector<double> time = {0, 1.6};
  std::vector<double> ang = {-1.3, -1.3 + 2 * M_PI};
  Vector3<double> axis = Vector3<double>(1, 2, 3).normalized();
  std::vector<Quaternion<double>> quat(ang.size());
  for (size_t i = 0; i < ang.size(); ++i) {
    quat[i] = Quaternion<double>(AngleAxis<double>(ang[i], axis));
  }
  PiecewiseQuaternionSlerp<double> rot_spline(time, quat);

  double t = 0.3 * time[0] + 0.7 * time[1];

  const std::vector<Quaternion<double>>& internal_quats =
      rot_spline.get_quaternion_samples();
  EXPECT_TRUE(CompareMatrices(
      rot_spline.orientation(t).coeffs(),
      internal_quats[0].coeffs(), 1e-10,
      MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      rot_spline.orientation(t).coeffs(),
      internal_quats[1].coeffs(), 1e-10,
      MatrixCompareType::absolute));
}

GTEST_TEST(TestPiecewiseQuaternionSlerp, TestIsApprox) {
  std::vector<double> time = {0, 1.6};
  std::vector<double> ang = {-1.3, 1};
  Vector3<double> axis = Vector3<double>(1, 2, 3).normalized();
  std::vector<Quaternion<double>> quat(ang.size());
  for (size_t i = 0; i < ang.size(); ++i) {
    quat[i] = Quaternion<double>(AngleAxis<double>(ang[i], axis));
  }
  PiecewiseQuaternionSlerp<double> traj0(time, quat);

  // -q represents the same orientation, so it should result in equality.
  std::vector<Quaternion<double>> quat_neg = quat;
  for (size_t i = 0; i < ang.size(); ++i) {
    quat_neg[i].coeffs() *= -1;
  }
  PiecewiseQuaternionSlerp<double> traj1(time, quat_neg);
  EXPECT_TRUE(traj0.is_approx(traj1, 1e-12));

  // Mutates a bit, should not be equal anymore.
  quat[0] = Quaternion<double>(AngleAxis<double>(ang[0] + 1e-6, axis));
  PiecewiseQuaternionSlerp<double> traj2(time, quat);
  EXPECT_FALSE(traj0.is_approx(traj2, 1e-7));
  EXPECT_TRUE(traj0.is_approx(traj2, 1e-5));
}

}  // namespace
}  // namespace trajectories
}  // namespace drake
