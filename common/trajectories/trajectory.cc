#include "drake/common/trajectories/trajectory.h"

#include "drake/common/unused.h"

// Remove 2025-08-01 with deprecation.
#include "drake/common/nice_type_name.h"

namespace drake {
namespace trajectories {

template <typename T>
Trajectory<T>::~Trajectory() = default;

template <typename T>
std::unique_ptr<Trajectory<T>> Trajectory<T>::Clone() const {
  return DoClone();
}

template <typename T>
MatrixX<T> Trajectory<T>::vector_values(const std::vector<T>& t) const {
  return vector_values(Eigen::Map<const VectorX<T>>(t.data(), t.size()));
}

template <typename T>
MatrixX<T> Trajectory<T>::vector_values(
    const Eigen::Ref<const VectorX<T>>& t) const {
  if (cols() != 1 && rows() != 1) {
    throw std::runtime_error(
        "This method only supports vector-valued trajectories.");
  }
  if (cols() == 1) {
    MatrixX<T> values(rows(), t.size());
    for (int i = 0; i < static_cast<int>(t.size()); ++i) {
      values.col(i) = value(t[i]);
    }
    return values;
  }
  MatrixX<T> values(t.size(), cols());
  for (int i = 0; i < static_cast<int>(t.size()); ++i) {
    values.row(i) = value(t[i]);
  }
  return values;
}

// Switch to be pure virtual on 2025-08-01.
template <typename T>
std::unique_ptr<Trajectory<T>> Trajectory<T>::DoClone() const {
  throw std::logic_error(
      fmt::format("{} is implemented incorrectly: it failed to override {}",
                  drake::NiceTypeName::Get(*this), __func__));
}

// Switch to be pure virtual on 2025-08-01.
template <typename T>
MatrixX<T> Trajectory<T>::do_value(const T& t) const {
  unused(t);
  throw std::logic_error(
      fmt::format("{} is implemented incorrectly: it failed to override {}",
                  drake::NiceTypeName::Get(*this), __func__));
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
  DRAKE_THROW_UNLESS(derivative_order >= 0);
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
  DRAKE_THROW_UNLESS(derivative_order >= 0);
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

// Switch to be pure virtual on 2025-08-01.
template <typename T>
Eigen::Index Trajectory<T>::do_rows() const {
  throw std::logic_error(
      fmt::format("{} is implemented incorrectly: it failed to override {}",
                  drake::NiceTypeName::Get(*this), __func__));
}

// Switch to be pure virtual on 2025-08-01.
template <typename T>
Eigen::Index Trajectory<T>::do_cols() const {
  throw std::logic_error(
      fmt::format("{} is implemented incorrectly: it failed to override {}",
                  drake::NiceTypeName::Get(*this), __func__));
}

// Switch to be pure virtual on 2025-08-01.
template <typename T>
T Trajectory<T>::do_start_time() const {
  throw std::logic_error(
      fmt::format("{} is implemented incorrectly: it failed to override {}",
                  drake::NiceTypeName::Get(*this), __func__));
}

// Switch to be pure virtual on 2025-08-01.
template <typename T>
T Trajectory<T>::do_end_time() const {
  throw std::logic_error(
      fmt::format("{} is implemented incorrectly: it failed to override {}",
                  drake::NiceTypeName::Get(*this), __func__));
}

}  // namespace trajectories
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::Trajectory);
