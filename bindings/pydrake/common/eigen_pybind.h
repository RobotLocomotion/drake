#pragma once

#include <Eigen/Dense>
#include "pybind11/eigen.h"

#include "drake/common/eigen_types.h"

namespace drake {
namespace pydrake {

// TODO(eric.cousineau): Ensure that all C++ mutator call sites use `EigenPtr`.
/// Provides a mutable Ref<> for a pointer.
/// Meant to be used for decorating methods passed to `pybind11` (e.g. virtual
/// function dispatch).
template <typename Derived>
auto ToEigenRef(Eigen::VectorBlock<Derived>* derived) {
  return Eigen::Ref<Derived>(*derived);
}

/// Converts a raw array to a numpy array.
template <typename T>
py::object ToArray(T* ptr, int size, py::tuple shape) {
  // Create flat array to be reshaped in numpy.
  using Vector = VectorX<T>;
  Eigen::Map<Vector> data(ptr, size);
  return py::cast(Eigen::Ref<Vector>(data), py_reference)
      .attr("reshape")(shape);
}

/// Converts a raw array to a numpy array (`const` variant).
template <typename T>
py::object ToArray(const T* ptr, int size, py::tuple shape) {
  // Create flat array to be reshaped in numpy.
  using Vector = const VectorX<T>;
  Eigen::Map<Vector> data(ptr, size);
  return py::cast(Eigen::Ref<Vector>(data), py_reference)
      .attr("reshape")(shape);
}

}  // namespace pydrake
}  // namespace drake
