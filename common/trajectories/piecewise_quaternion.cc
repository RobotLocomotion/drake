#include "drake/common/trajectories/piecewise_quaternion.h"

#include <algorithm>
#include <stdexcept>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/math/quaternion.h"

namespace drake {
namespace trajectories {

using std::abs;
using std::cos;
using std::max;
using std::min;

template <typename T>
bool PiecewiseQuaternionSlerp<T>::is_approx(
    const PiecewiseQuaternionSlerp<T>& other, double tol) const {
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
    double dot =
        abs(ExtractDoubleOrThrow(quaternions_[i].dot(other.quaternions_[i])));
    if (dot < cos(tol / 2)) {
      return false;
    }
  }

  return true;
}

namespace internal {

template <typename T>
Vector3<T> ComputeAngularVelocity(const T& duration, const Quaternion<T>& q,
                                  const Quaternion<T>& qnext) {
  // Computes qnext = q_delta * q first, turn q_delta into an axis, which
  // turns into an angular velocity.
  AngleAxis<T> angle_axis_diff(qnext * q.inverse());
  return angle_axis_diff.axis() * angle_axis_diff.angle() / duration;
}

}  // namespace internal

template <typename T>
void PiecewiseQuaternionSlerp<T>::Initialize(
    const std::vector<T>& breaks,
    const std::vector<Quaternion<T>>& quaternions) {
  if (quaternions.size() != breaks.size()) {
    throw std::logic_error("Quaternions and breaks length mismatch.");
  }
  if (quaternions.size() < 2) {
    throw std::logic_error("Not enough quaternions for slerp.");
  }
  quaternions_.resize(breaks.size());
  angular_velocities_.resize(breaks.size() - 1);

  // Set to the closest wrt to the previous, also normalize.
  for (size_t i = 0; i < quaternions.size(); ++i) {
    if (i == 0) {
      quaternions_[i] = quaternions[i].normalized();
    } else {
      quaternions_[i] =
          math::ClosestQuaternion(quaternions_[i - 1], quaternions[i]);
      angular_velocities_[i - 1] = internal::ComputeAngularVelocity(
          this->duration(i - 1), quaternions_[i - 1], quaternions[i]);
    }
  }
}

template <typename T>
PiecewiseQuaternionSlerp<T>::PiecewiseQuaternionSlerp(
    const std::vector<T>& breaks,
    const std::vector<Quaternion<T>>& quaternions)
    : PiecewiseTrajectory<T>(breaks) {
  Initialize(breaks, quaternions);
}

template <typename T>
PiecewiseQuaternionSlerp<T>::PiecewiseQuaternionSlerp(
    const std::vector<T>& breaks,
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
    const std::vector<T>& breaks,
    const std::vector<math::RotationMatrix<T>>& rot_matrices)
    : PiecewiseTrajectory<T>(breaks) {
  std::vector<Quaternion<T>> quaternions(rot_matrices.size());
  for (size_t i = 0; i < rot_matrices.size(); ++i) {
    quaternions[i] = rot_matrices[i].ToQuaternion();
  }
  Initialize(breaks, quaternions);
}

template <typename T>
PiecewiseQuaternionSlerp<T>::PiecewiseQuaternionSlerp(
    const std::vector<T>& breaks,
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
T PiecewiseQuaternionSlerp<T>::ComputeInterpTime(int segment_index,
                                                  const T& time) const {
  T interp_time =
      (time - this->start_time(segment_index)) / this->duration(segment_index);
  interp_time = max(interp_time, T(0.0));
  interp_time = min(interp_time, T(1.0));
  return interp_time;
}

template <typename T>
Quaternion<T> PiecewiseQuaternionSlerp<T>::orientation(const T& t) const {
  int segment_index = this->get_segment_index(t);
  T interp_t = ComputeInterpTime(segment_index, t);

  Quaternion<T> q1 = quaternions_.at(segment_index)
                         .slerp(interp_t, quaternions_.at(segment_index + 1));

  q1.normalize();

  return q1;
}

template <typename T>
Vector3<T> PiecewiseQuaternionSlerp<T>::angular_velocity(const T& t) const {
  int segment_index = this->get_segment_index(t);

  return angular_velocities_.at(segment_index);
}

template <typename T>
Vector3<T> PiecewiseQuaternionSlerp<T>::angular_acceleration(const T&) const {
  return Vector3<T>::Zero();
}

template <typename T>
void PiecewiseQuaternionSlerp<T>::Append(
    const T& time, const Quaternion<T>& quaternion) {
  DRAKE_DEMAND(this->breaks().empty() || time > this->breaks().back());
  if (quaternions_.empty()) {
    quaternions_.push_back(quaternion.normalized());
  } else {
    angular_velocities_.push_back(internal::ComputeAngularVelocity(
        time - this->breaks().back(), quaternions_.back(), quaternion));
    quaternions_.push_back(math::ClosestQuaternion(
        quaternions_.back(), quaternion));
  }
  this->get_mutable_breaks().push_back(time);
}

template <typename T>
void PiecewiseQuaternionSlerp<T>::Append(
    const T& time, const math::RotationMatrix<T>& rotation_matrix) {
  Append(time, rotation_matrix.ToQuaternion());
}

template <typename T>
void PiecewiseQuaternionSlerp<T>::Append(
    const T& time, const AngleAxis<T>& angle_axis) {
  Append(time, Quaternion<T>(angle_axis));
}

template <typename T>
bool PiecewiseQuaternionSlerp<T>::do_has_derivative() const {
  return true;
}

template <typename T>
MatrixX<T> PiecewiseQuaternionSlerp<T>::DoEvalDerivative(
  const T& t, int derivative_order) const {
  if (derivative_order == 0) {
    return value(t);
  } else if (derivative_order == 1) {
    return angular_velocity(t);
  }
  // All higher derivatives are zero.
  return Vector3<T>::Zero();
}

template <typename T>
std::unique_ptr<Trajectory<T>> PiecewiseQuaternionSlerp<T>::DoMakeDerivative(
      int derivative_order) const {
  if (derivative_order == 0) {
    return this->Clone();
  } else if (derivative_order == 1) {
    std::vector<MatrixX<T>> m(angular_velocities_.begin(),
                              angular_velocities_.end());
    m.push_back(Vector3<T>::Zero());
    return PiecewisePolynomial<T>::ZeroOrderHold(
      this->get_segment_times(), m).Clone();
  }
  // All higher derivatives are zero.
  return std::make_unique<PiecewisePolynomial<T>>(Vector3<T>::Zero());
}

}  // namespace trajectories
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::PiecewiseQuaternionSlerp)
