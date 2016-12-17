#include "drake/common/trajectories/piecewise_quaternion.h"

#include <algorithm>

#include "drake/math/quaternion.h"

namespace drake {

template <typename Scalar>
void PiecewiseQuaternionSlerp<Scalar>::ComputeAngularVelocities() {
  if (quaternions_.empty()) return;

  angular_velocities_.resize(quaternions_.size() - 1);

  AngleAxis<Scalar> angle_axis_diff;
  // Computes q[t+1] = q_delta * q[t] first, turn q_delta into an axis, which
  // turns into an angular velocity.
  for (size_t i = 1; i < quaternions_.size(); ++i) {
    angle_axis_diff =
        AngleAxis<Scalar>(quaternions_[i] * quaternions_[i - 1].inverse());
    angular_velocities_[i - 1] =
        angle_axis_diff.axis() * angle_axis_diff.angle() / getDuration(i - 1);
  }
}

template <typename Scalar>
void PiecewiseQuaternionSlerp<Scalar>::Initialize(
    const std::vector<double>& breaks,
    const eigen_aligned_std_vector<Quaternion<Scalar>>& quaternions) {
  if (quaternions.size() != breaks.size()) {
    throw std::runtime_error("Quaternions and breaks length mismatch.");
  }
  if (quaternions.size() < 2) {
    throw std::runtime_error("Not enough quaternions for slerp.");
  }
  quaternions_.resize(breaks.size());

  // Set to the closest wrt to the previous, also normalize.
  for (size_t i = 0; i < quaternions.size(); ++i) {
    if (i == 0) {
      quaternions_[i] = quaternions[i].normalized();
    } else {
      quaternions_[i] =
          math::ClosestQuaternion(quaternions_[i - 1], quaternions[i]);
    }
  }
  ComputeAngularVelocities();
}

template <typename Scalar>
PiecewiseQuaternionSlerp<Scalar>::PiecewiseQuaternionSlerp(
    const std::vector<double>& breaks,
    const eigen_aligned_std_vector<Quaternion<Scalar>>& quaternions)
    : PiecewiseFunction(breaks) {
  Initialize(breaks, quaternions);
}

template <typename Scalar>
PiecewiseQuaternionSlerp<Scalar>::PiecewiseQuaternionSlerp(
    const std::vector<double>& breaks,
    const eigen_aligned_std_vector<Matrix3<Scalar>>& rot_matrices)
    : PiecewiseFunction(breaks) {
  eigen_aligned_std_vector<Quaternion<Scalar>> quaternions(rot_matrices.size());
  for (size_t i = 0; i < rot_matrices.size(); ++i) {
    quaternions[i] = Quaternion<Scalar>(rot_matrices[i]);
  }
  Initialize(breaks, quaternions);
}

template <typename Scalar>
PiecewiseQuaternionSlerp<Scalar>::PiecewiseQuaternionSlerp(
    const std::vector<double>& breaks,
    const eigen_aligned_std_vector<AngleAxis<Scalar>>& ang_axes)
    : PiecewiseFunction(breaks) {
  eigen_aligned_std_vector<Quaternion<Scalar>> quaternions(ang_axes.size());
  for (size_t i = 0; i < ang_axes.size(); ++i) {
    quaternions[i] = Quaternion<Scalar>(ang_axes[i]);
  }
  Initialize(breaks, quaternions);
}

template <typename Scalar>
double PiecewiseQuaternionSlerp<Scalar>::ComputeInterpTime(int segment_index,
                                                           double time) const {
  time = (time - getStartTime(segment_index)) / getDuration(segment_index);
  time = std::max(time, 0.0);
  time = std::min(time, 1.0);
  return time;
}

template <typename Scalar>
Quaternion<Scalar> PiecewiseQuaternionSlerp<Scalar>::orientation(
    double t) const {
  int segment_index = getSegmentIndex(t);
  t = ComputeInterpTime(segment_index, t);

  Quaternion<Scalar> q1 = quaternions_.at(segment_index)
                              .slerp(t, quaternions_.at(segment_index + 1));

  q1.normalize();

  return q1;
}

template <typename Scalar>
Vector3<Scalar> PiecewiseQuaternionSlerp<Scalar>::angular_velocity(
    double t) const {
  int segment_index = getSegmentIndex(t);

  return angular_velocities_.at(segment_index);
}

template <typename Scalar>
Vector3<Scalar> PiecewiseQuaternionSlerp<Scalar>::angular_acceleration(
    double t) const {
  return Vector3<Scalar>::Zero();
}

template class PiecewiseQuaternionSlerp<double>;

}  // namespace drake
