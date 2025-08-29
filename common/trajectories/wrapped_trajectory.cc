#include "drake/common/trajectories/wrapped_trajectory.h"

#include <utility>
namespace drake {
namespace trajectories {
namespace internal {

template <typename T>
WrappedTrajectory<T>::WrappedTrajectory(
    std::shared_ptr<const Trajectory<T>> trajectory)
    : trajectory_(std::move(trajectory)) {
  DRAKE_THROW_UNLESS(trajectory_ != nullptr);
}

template <typename T>
WrappedTrajectory<T>::~WrappedTrajectory() = default;

template <typename T>
const Trajectory<T>* WrappedTrajectory<T>::unwrap() const {
  return trajectory_.get();
}

template <typename T>
std::unique_ptr<Trajectory<T>> WrappedTrajectory<T>::DoClone() const {
  // N.B. It's important that we directly return the nested clone, instead of
  // wrapping it again in a WrappedTrajectory. If the class we're calling itself
  // returns a WrappedTrajectory internally, then the number of wrappers will
  // grow longer with each successive call to Clone(), possibly leading to
  // accidentally quadratic runtime behavior. (Possibly we could conditionally
  // re-wrap the result only in case it wasn't already wrapped, but why go to
  // all of that trouble?)
  return trajectory_->Clone();
}

template <typename T>
MatrixX<T> WrappedTrajectory<T>::do_value(const T& t) const {
  return trajectory_->value(t);
}

template <typename T>
bool WrappedTrajectory<T>::do_has_derivative() const {
  return trajectory_->has_derivative();
}

template <typename T>
MatrixX<T> WrappedTrajectory<T>::DoEvalDerivative(const T& t,
                                                  int derivative_order) const {
  return trajectory_->EvalDerivative(t, derivative_order);
}

template <typename T>
std::unique_ptr<Trajectory<T>> WrappedTrajectory<T>::DoMakeDerivative(
    int derivative_order) const {
  return trajectory_->MakeDerivative(derivative_order);
}

template <typename T>
Eigen::Index WrappedTrajectory<T>::do_rows() const {
  return trajectory_->rows();
}

template <typename T>
Eigen::Index WrappedTrajectory<T>::do_cols() const {
  return trajectory_->cols();
}

template <typename T>
T WrappedTrajectory<T>::do_start_time() const {
  return trajectory_->start_time();
}

template <typename T>
T WrappedTrajectory<T>::do_end_time() const {
  return trajectory_->end_time();
}

}  // namespace internal
}  // namespace trajectories
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::internal::WrappedTrajectory);
