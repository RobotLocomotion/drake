#include "drake/common/trajectories/trajectory.h"

#include "drake/common/unused.h"

namespace drake {
namespace trajectories {

template <typename T>
MatrixX<T> Trajectory<T>::vector_values(const std::vector<T>& t) const {
  if (cols() != 1 && rows() != 1) {
    throw std::runtime_error(
        "This method only supports vector-valued trajectories.");
  }
  if (cols() == 1) {
    MatrixX<T> values(rows(), t.size());
    for (int i = 0; i < static_cast<int>(t.size()); i++) {
      values.col(i) = value(t[i]);
    }
    return values;
  }
  MatrixX<T> values(t.size(), cols());
  for (int i = 0; i < static_cast<int>(t.size()); i++) {
    values.row(i) = value(t[i]);
  }
  return values;
}

template <typename T>
bool Trajectory<T>::has_derivative() const {
  return do_has_derivative();
}

template <typename T>
bool Trajectory<T>::do_has_derivative() const {
  return false;
}

template <typename T>
MatrixX<T> Trajectory<T>::EvalDerivative(const T& t,
                                         int derivative_order) const {
  return DoEvalDerivative(t, derivative_order);
}

template <typename T>
MatrixX<T> Trajectory<T>::DoEvalDerivative(const T& t,
                                           int derivative_order) const {
  unused(t);
  unused(derivative_order);
  if (has_derivative()) {
    throw std::logic_error(
        "Trajectory classes that promise derivatives via do_has_derivative() "
        "must implement DoEvalDerivative().");
  } else {
    throw std::logic_error(
        "You asked for derivatives from a class that does not support "
        "derivatives.");
  }
}

template <typename T>
std::unique_ptr<Trajectory<T>> Trajectory<T>::MakeDerivative(
    int derivative_order) const {
  return DoMakeDerivative(derivative_order);
}

template <typename T>
std::unique_ptr<Trajectory<T>> Trajectory<T>::DoMakeDerivative(
    int derivative_order) const {
  unused(derivative_order);
  if (has_derivative()) {
    throw std::logic_error(
        "Trajectory classes that promise derivatives via do_has_derivative() "
        "must implement DoMakeDerivative().");
  } else {
    throw std::logic_error(
        "You asked for derivatives from a class that does not support "
        "derivatives.");
  }
}

}  // namespace trajectories
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::Trajectory)
