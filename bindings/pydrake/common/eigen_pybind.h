#pragma once

#include <utility>

#include <Eigen/Dense>

#include "drake/bindings/pydrake/pydrake_pybind.h"
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
py::object ToArray(T* ptr, int size, py::tuple shape,
    py::return_value_policy policy = py_rvp::reference,
    py::handle parent = py::handle()) {
  // Create flat array to be reshaped in numpy.
  using Vector = VectorX<T>;
  Eigen::Map<Vector> data(ptr, size);
  return py::cast(Eigen::Ref<Vector>(data), policy, parent)
      .attr("reshape")(shape);
}

/** Converts a raw array to a numpy array (`const` variant). */
template <typename T>
py::object ToArray(const T* ptr, int size, py::tuple shape,
    py::return_value_policy policy = py_rvp::reference,
    py::handle parent = py::handle()) {
  // Create flat array to be reshaped in numpy.
  using Vector = const VectorX<T>;
  Eigen::Map<Vector> data(ptr, size);
  return py::cast(Eigen::Ref<Vector>(data), policy, parent)
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

namespace pybind11 {
namespace detail {

/**
Provides pybind11 `type_caster`s for drake::EigenPtr.

Uses `type_caster<Eigen::Ref>` internally to avoid code duplication.
See http://pybind11.readthedocs.io/en/stable/advanced/cast/custom.html for
more details on custom type casters.

TODO(eric.cousineau): Place all logic inside of `drake` namespace once our
pybind11 fork includes PYBIND11_TYPE_CASTER macro w/ fully qualified symbols.
*/
template <typename T>
struct type_caster<drake::EigenPtr<T>> {
  using PtrType = drake::EigenPtr<T>;
  using RefType = Eigen::Ref<T>;
  using InnerCaster = type_caster<RefType>;

 public:
  PYBIND11_TYPE_CASTER(PtrType, _("Optional[") + InnerCaster::name + _("]"));

  bool load(handle src, bool convert) {
    value = PtrType(nullptr);

    if (src.ptr() == Py_None) {
      return true;
    }

    bool success = inner_caster.load(src, convert);

    if (success) {
      RefType& ref = inner_caster;
      value = PtrType(&ref);
    }

    return success;
  }

  static handle cast(PtrType src, return_value_policy policy, handle parent) {
    if (src == nullptr) {
      return Py_None;
    } else {
      RefType ref = *src;
      return InnerCaster::cast(ref, policy, parent);
    }
  }

 private:
  InnerCaster inner_caster;
};

}  // namespace detail
}  // namespace pybind11
