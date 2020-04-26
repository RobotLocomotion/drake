#include "drake/common/trajectories/piecewise_quaternion.h"

#include <algorithm>
#include <stdexcept>

#include "drake/math/quaternion.h"

namespace drake {
namespace trajectories {

template <typename T>
bool PiecewiseQuaternionSlerp<T>::is_approx(
    const PiecewiseQuaternionSlerp<T>& other, const T& tol) const {
  // Velocities are derived from the quaternions, and I don't want to
  // overload units for tol, so I am skipping the checks on velocities.
  if (!this->SegmentTimesEqual(other, tol))
    return false;

  if (quaternions_.size() != other.quaternions_.size())
    return false;

  for (size_t i = 0; i < quaternions_.size(); ++i) {
    // A quick reference:
    // Page "Metric on sphere of unit quaternions" from
    // http://www.cs.cmu.edu/afs/cs/academic/class/16741-s07/www/Lecture8.pdf
    T dot = std::abs(quaternions_[i].dot(other.quaternions_[i]));
    if (dot < std::cos(tol / 2)) {
      return false;
    }
  }

  return true;
}

template <typename T>
void PiecewiseQuaternionSlerp<T>::ComputeAngularVelocities() {
  if (quaternions_.empty()) return;

  angular_velocities_.resize(quaternions_.size() - 1);

  AngleAxis<T> angle_axis_diff;
  // Computes q[t+1] = q_delta * q[t] first, turn q_delta into an axis, which
  // turns into an angular velocity.
  for (size_t i = 1; i < quaternions_.size(); ++i) {
    angle_axis_diff =
        AngleAxis<T>(quaternions_[i] * quaternions_[i - 1].inverse());
    angular_velocities_[i - 1] = angle_axis_diff.axis() *
                                 angle_axis_diff.angle() /
                                 this->duration(i - 1);
  }
}

template <typename T>
void PiecewiseQuaternionSlerp<T>::Initialize(
    const std::vector<double>& breaks,
    const std::vector<Quaternion<T>>& quaternions) {
  if (quaternions.size() != breaks.size()) {
    throw std::logic_error("Quaternions and breaks length mismatch.");
  }
  if (quaternions.size() < 2) {
    throw std::logic_error("Not enough quaternions for slerp.");
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

template <typename T>
PiecewiseQuaternionSlerp<T>::PiecewiseQuaternionSlerp(
    const std::vector<double>& breaks,
    const std::vector<Quaternion<T>>& quaternions)
    : PiecewiseTrajectory<T>(breaks) {
  Initialize(breaks, quaternions);
}

template <typename T>
PiecewiseQuaternionSlerp<T>::PiecewiseQuaternionSlerp(
    const std::vector<double>& breaks,
    const std::vector<Matrix3<T>>& rot_matrices)
    : PiecewiseTrajectory<T>(breaks) {
  std::vector<Quaternion<T>> quaternions(rot_matrices.size());
  for (size_t i = 0; i < rot_matrices.size(); ++i) {
    quaternions[i] = Quaternion<T>(rot_matrices[i]);
  }
  Initialize(breaks, quaternions);
}

template <typename T>
PiecewiseQuaternionSlerp<T>::PiecewiseQuaternionSlerp(
    const std::vector<double>& breaks,
    const std::vector<AngleAxis<T>>& ang_axes)
    : PiecewiseTrajectory<T>(breaks) {
  std::vector<Quaternion<T>> quaternions(ang_axes.size());
  for (size_t i = 0; i < ang_axes.size(); ++i) {
    quaternions[i] = Quaternion<T>(ang_axes[i]);
  }
  Initialize(breaks, quaternions);
}

template <typename T>
std::unique_ptr<Trajectory<T>> PiecewiseQuaternionSlerp<T>::Clone() const {
  return std::make_unique<PiecewiseQuaternionSlerp>(*this);
}

template <typename T>
double PiecewiseQuaternionSlerp<T>::ComputeInterpTime(int segment_index,
                                                           double time) const {
  time =
      (time - this->start_time(segment_index)) / this->duration(segment_index);
  time = std::max(time, 0.0);
  time = std::min(time, 1.0);
  return time;
}

template <typename T>
Quaternion<T> PiecewiseQuaternionSlerp<T>::orientation(double t) const {
  int segment_index = this->get_segment_index(t);
  t = ComputeInterpTime(segment_index, t);

  Quaternion<T> q1 = quaternions_.at(segment_index)
                              .slerp(t, quaternions_.at(segment_index + 1));

  q1.normalize();

  return q1;
}

template <typename T>
Vector3<T> PiecewiseQuaternionSlerp<T>::angular_velocity(double t) const {
  int segment_index = this->get_segment_index(t);

  return angular_velocities_.at(segment_index);
}

template <typename T>
Vector3<T> PiecewiseQuaternionSlerp<T>::angular_acceleration(double) const {
  return Vector3<T>::Zero();
}

template class PiecewiseQuaternionSlerp<double>;

}  // namespace trajectories
}  // namespace drake
