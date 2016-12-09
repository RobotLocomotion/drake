#include "drake/common/trajectories/piecewise_quaternion.h"

#include <random>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/util/drakeGeometryUtil.h"
#include "gtest/gtest.h"

namespace drake {
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
bool CheckClosest(const std::vector<Quaternion<Scalar>>& quaternions) {
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
  t = std::min(t, spline.getEndTime());
  t = std::max(t, spline.getStartTime());

  int i = spline.getSegmentIndex(t);
  double t0 = spline.getStartTime(i);
  double t1 = spline.getEndTime(i);
  t = std::min(t, t1);
  t = std::max(t, t0);
  double dt = t - t0;
  DRAKE_DEMAND(dt <= spline.getDuration(i));

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
  std::uniform_real_distribution<double> uniform(-1, 1);
  for (int i = 0; i < size; ++i) {
    ret[i] = Quaternion<Scalar>(AngleAxis<Scalar>(
        uniform(*generator),
        Vector3<Scalar>(uniform(*generator), uniform(*generator),
                        uniform(*generator))));
  }
  return ret;
}

// Tests CheckSlerpInterpolation and "closestness" for PiecewiseQuaternionSlerp
// generated from random breaks and knots.
GTEST_TEST(TestPiecewiseQuaternionSlerp,
           TestRandomizedPiecewiseQuaternionSlerp) {
  std::default_random_engine generator(123);
  int N = 10000;
  std::vector<double> time =
      PiecewiseFunction::randomSegmentTimes(N - 1, generator);
  std::vector<Quaternion<double>> quat =
      GenerateRandomQuaternions<double>(N, &generator);

  PiecewiseQuaternionSlerp<double> rot_spline(time, quat);

  for (double t = time.front(); t < time.back(); t += 0.01) {
    EXPECT_TRUE(CheckSlerpInterpolation(rot_spline, t));
  }

  EXPECT_TRUE(CheckClosest(rot_spline.get_quaternion_knots()));
}

// Tests when the given quaternions are not "closest" to the previous one.
// Also tests the returned angular velocity against known values.
GTEST_TEST(TestPiecewiseQuaternionSlerp,
           TestShortestQuaternionPiecewiseQuaternionSlerp) {
  std::vector<double> time = {0, 1.6, 2.32};
  std::vector<double> ang = {1, 2.4, 5.3};
  Vector3<double> axis = Vector3<double>(1, 2, 3).normalized();
  std::vector<Quaternion<double>> quat(ang.size());
  for (size_t i = 0; i < ang.size(); ++i) {
    quat[i] = Quaternion<double>(AngleAxis<double>(ang[i], axis));
  }

  PiecewiseQuaternionSlerp<double> rot_spline(time, quat);
  EXPECT_TRUE(CheckClosest(rot_spline.get_quaternion_knots()));
  EXPECT_TRUE(CheckClosest(quat));

  for (size_t i = 0; i < time.size() - 1; ++i) {
    EXPECT_TRUE(CompareMatrices(rot_spline.orientation(time[i]).coeffs(),
                                quat[i].coeffs(), 1e-10,
                                MatrixCompareType::absolute));
    double omega = angleDiff(ang[i], ang[i + 1]) / (time[i + 1] - time[i]);
    EXPECT_TRUE(CompareMatrices(rot_spline.angular_velocity(time[i]),
                                omega * axis, 1e-10,
                                MatrixCompareType::absolute));
  }

  // Now "flip" quat[1].
  ang[1] -= 2 * M_PI;
  quat[1] = Quaternion<double>(AngleAxis<double>(ang[1], axis));
  rot_spline = PiecewiseQuaternionSlerp<double>(time, quat);
  EXPECT_TRUE(CheckClosest(rot_spline.get_quaternion_knots()));
  EXPECT_FALSE(CheckClosest(quat));

  // Since quat[1] is flipped, the interpolated quaternion's coeffs won't be the
  // same.
  for (size_t i = 0; i < time.size() - 1; ++i) {
    double omega = angleDiff(ang[i], ang[i + 1]) / (time[i + 1] - time[i]);
    EXPECT_TRUE(CompareMatrices(rot_spline.angular_velocity(time[i]),
                                omega * axis, 1e-10,
                                MatrixCompareType::absolute));
  }
}

}  // namespace
}  // namespace drake
