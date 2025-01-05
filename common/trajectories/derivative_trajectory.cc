#include "drake/common/trajectories/derivative_trajectory.h"

namespace drake {
namespace trajectories {

template <typename T>
DerivativeTrajectory<T>::DerivativeTrajectory(const Trajectory<T>& nominal,
                                              int derivative_order)
    : nominal_{nominal.Clone()}, derivative_order_{derivative_order} {
  DRAKE_THROW_UNLESS(nominal.has_derivative());
  DRAKE_THROW_UNLESS(derivative_order >= 0);
  // Currently, we must evaluate the derivative once in order to know the sizes
  // of this new object. (See PiecewiseQuaternion, for example, whose nominal
  // value has 4 rows, but whose derivative has only 3).
  MatrixX<T> v = nominal.EvalDerivative(nominal.start_time(), derivative_order);
  rows_ = v.rows();
  cols_ = v.cols();
}

template <typename T>
DerivativeTrajectory<T>::~DerivativeTrajectory() = default;

template <typename T>
std::unique_ptr<Trajectory<T>> DerivativeTrajectory<T>::DoClone() const {
  return std::make_unique<DerivativeTrajectory>(*nominal_, derivative_order_);
}

template <typename T>
MatrixX<T> DerivativeTrajectory<T>::do_value(const T& t) const {
  return nominal_->EvalDerivative(t, derivative_order_);
}

template <typename T>
MatrixX<T> DerivativeTrajectory<T>::DoEvalDerivative(
    const T& t, int derivative_order) const {
  return nominal_->EvalDerivative(t, derivative_order_ + derivative_order);
}

template <typename T>
std::unique_ptr<Trajectory<T>> DerivativeTrajectory<T>::DoMakeDerivative(
    int derivative_order) const {
  return std::make_unique<DerivativeTrajectory<T>>(
      *nominal_, derivative_order_ + derivative_order);
}

template <typename T>
Eigen::Index DerivativeTrajectory<T>::do_rows() const {
  return rows_;
}

template <typename T>
Eigen::Index DerivativeTrajectory<T>::do_cols() const {
  return cols_;
}

template <typename T>
T DerivativeTrajectory<T>::do_start_time() const {
  return nominal_->start_time();
}

template <typename T>
T DerivativeTrajectory<T>::do_end_time() const {
  return nominal_->end_time();
}

}  // namespace trajectories
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::DerivativeTrajectory);
