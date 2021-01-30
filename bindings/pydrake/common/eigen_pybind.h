#pragma once

#include "pybind11/eigen.h"
#include <Eigen/Dense>

#include "drake/common/eigen_types.h"

namespace drake {
namespace pydrake {

// TODO(eric.cousineau): Ensure that all C++ mutator call sites use `EigenPtr`.
/**
Provides a mutable Ref<> for a pointer.
Meant to be used for decorating methods passed to `pybind11` (e.g. virtual
function dispatch).
*/
template <typename Derived>
auto ToEigenRef(Eigen::VectorBlock<Derived>* derived) {
  return Eigen::Ref<Derived>(*derived);
}

/** Converts a raw array to a numpy array. */
template <typename T>
py::object ToArray(T* ptr, int size, py::tuple shape) {
  // Create flat array to be reshaped in numpy.
  using Vector = VectorX<T>;
  Eigen::Map<Vector> data(ptr, size);
  return py::cast(Eigen::Ref<Vector>(data), py_rvp::reference)
      .attr("reshape")(shape);
}

/** Converts a raw array to a numpy array (`const` variant). */
template <typename T>
py::object ToArray(const T* ptr, int size, py::tuple shape) {
  // Create flat array to be reshaped in numpy.
  using Vector = const VectorX<T>;
  Eigen::Map<Vector> data(ptr, size);
  return py::cast(Eigen::Ref<Vector>(data), py_rvp::reference)
      .attr("reshape")(shape);
}

/**
Wraps a overload instance method to reshape the output to be the same as a
given input argument. The input should be the first and only argument to
trigger reshaping.

This preserves the original docstrings so that they still indicate the shapes
of the input and output arrays.

Example:

@code
cls  // BR
  .def("multiply", [](const Class& self, const Class& other) { ... })
  .def("multiply", [](const Class& self, const Vector3<T>& p) { ... })
  .def("multiply", [](const Class& self, const Matrix3X<T>& plist) { ... });
cls.attr("multiply") = WrapToMatchInputShape(cls.attr("multiply"));
@endcode

@sa @ref PydrakeReturnVectorsOrMatrices
*/
inline py::object WrapToMatchInputShape(py::handle func) {
  py::handle wrap =
      py::module::import("pydrake.common").attr("_wrap_to_match_input_shape");
  return wrap(func);
}

}  // namespace pydrake
}  // namespace drake
