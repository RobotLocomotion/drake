#include "drake/common/trajectories/piecewise_pose.h"

#include "drake/common/pointer_cast.h"
#include "drake/common/polynomial.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace trajectories {

template <typename T>
PiecewisePose<T>::PiecewisePose(
    const PiecewisePolynomial<T>& position_trajectory,
    const PiecewiseQuaternionSlerp<T>& orientation_trajectory)
    : PiecewiseTrajectory<T>(position_trajectory.get_segment_times()) {
  DRAKE_DEMAND(position_trajectory.rows() == 3);
  DRAKE_DEMAND(position_trajectory.cols() == 1);
  DRAKE_DEMAND(this->SegmentTimesEqual(orientation_trajectory, 0));
  position_ = position_trajectory;
  velocity_ = position_.derivative();
  acceleration_ = velocity_.derivative();
  orientation_ = orientation_trajectory;
}

template <typename T>
PiecewisePose<T> PiecewisePose<T>::MakeLinear(
    const std::vector<T>& times,
    const std::vector<math::RigidTransform<T>>& poses) {
  std::vector<MatrixX<T>> pos_knots(poses.size());
  std::vector<math::RotationMatrix<T>> rot_knots(poses.size());
  for (size_t i = 0; i < poses.size(); ++i) {
    pos_knots[i] = poses[i].translation();
    rot_knots[i] = poses[i].rotation();
  }

  return PiecewisePose<T>(
      PiecewisePolynomial<T>::FirstOrderHold(times, pos_knots),
      PiecewiseQuaternionSlerp<T>(times, rot_knots));
}

template <typename T>
PiecewisePose<T> PiecewisePose<T>::MakeCubicLinearWithEndLinearVelocity(
    const std::vector<T>& times,
    const std::vector<math::RigidTransform<T>>& poses,
    const Vector3<T>& start_vel, const Vector3<T>& end_vel) {
  std::vector<MatrixX<T>> pos_knots(poses.size());
  std::vector<math::RotationMatrix<T>> rot_knots(poses.size());
  for (size_t i = 0; i < poses.size(); ++i) {
    pos_knots[i] = poses[i].translation();
    rot_knots[i] = poses[i].rotation();
  }

  return PiecewisePose<T>(
      PiecewisePolynomial<T>::CubicWithContinuousSecondDerivatives(
          times, pos_knots, start_vel, end_vel),
      PiecewiseQuaternionSlerp<T>(times, rot_knots));
}

template <typename T>
std::unique_ptr<Trajectory<T>> PiecewisePose<T>::Clone() const {
  return std::make_unique<PiecewisePose>(*this);
}

template <typename T>
math::RigidTransform<T> PiecewisePose<T>::GetPose(const T& time) const {
  return math::RigidTransform<T>(orientation_.orientation(time),
                                 position_.value(time));
}

template <typename T>
Vector6<T> PiecewisePose<T>::GetVelocity(const T& time) const {
  Vector6<T> velocity;
  if (orientation_.is_time_in_range(time)) {
    velocity.template head<3>() = orientation_.angular_velocity(time);
  } else {
    velocity.template head<3>().setZero();
  }
  if (position_.is_time_in_range(time)) {
    velocity.template tail<3>() = velocity_.value(time);
  } else {
    velocity.template tail<3>().setZero();
  }
  return velocity;
}

template <typename T>
Vector6<T> PiecewisePose<T>::GetAcceleration(const T& time) const {
  Vector6<T> acceleration;
  if (orientation_.is_time_in_range(time)) {
    acceleration.template head<3>() = orientation_.angular_acceleration(time);
  } else {
    acceleration.template head<3>().setZero();
  }
  if (position_.is_time_in_range(time)) {
    acceleration.template tail<3>() = acceleration_.value(time);
  } else {
    acceleration.template tail<3>().setZero();
  }
  return acceleration;
}

template <typename T>
bool PiecewisePose<T>::IsApprox(const PiecewisePose<T>& other,
                                 double tol) const {
  bool ret = position_.isApprox(other.position_, tol);
  ret &= orientation_.is_approx(other.orientation_, tol);
  return ret;
}

template <typename T>
bool PiecewisePose<T>::do_has_derivative() const {
  return true;
}

template <typename T>
MatrixX<T> PiecewisePose<T>::DoEvalDerivative(const T& t,
                                              int derivative_order) const {
  if (derivative_order == 0) {
    return value(t);
  }
  Vector6<T> derivative;
  derivative.template head<3>() =
      orientation_.EvalDerivative(t, derivative_order);
  derivative.template tail<3>() = position_.EvalDerivative(t, derivative_order);
  return derivative;
}

template <typename T>
std::unique_ptr<Trajectory<T>> PiecewisePose<T>::DoMakeDerivative(
    int derivative_order) const {
  if (derivative_order == 0) {
    return this->Clone();
  }
  std::unique_ptr<PiecewisePolynomial<T>> orientation_deriv =
      dynamic_pointer_cast<PiecewisePolynomial<T>>(
          orientation_.MakeDerivative(derivative_order));
  PiecewisePolynomial<T> position_deriv =
      position_.derivative(derivative_order);
  const std::vector<T>& breaks = position_deriv.get_segment_times();
  std::vector<MatrixX<Polynomial<T>>> derivative;
  for (size_t ii = 0; ii < breaks.size() - 1; ii++) {
    MatrixX<Polynomial<T>> segment_derivative(6, 1);
    segment_derivative.template topRows<3>() =
        orientation_deriv->getPolynomialMatrix(ii);
    segment_derivative.template bottomRows<3>() =
        position_deriv.getPolynomialMatrix(ii);
    derivative.push_back(segment_derivative);
  }
  return std::make_unique<PiecewisePolynomial<T>>(derivative, breaks);
}

}  // namespace trajectories
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::PiecewisePose)
