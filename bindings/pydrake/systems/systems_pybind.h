#pragma once

/// @file
/// Helpers for defining Python types within the Systems framework.

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/cpp_template_pybind.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace pydrake {
namespace pysystems {

/// Defines an instantiation of `pydrake.systems.framework.Value[...]`.
/// @prereq `T` must have already been exposed to `pybind11`.
/// @param scope Parent scope.
/// @tparam T Inner parameter of `Value<T>`.
/// @tparam Class Class to be bound. By default, `Value<T>` is used.
/// @returns `py::class_<>` object, for defining emplace constructors.
template <typename T, typename Class = systems::Value<T>>
auto AddValueInstantiation(py::module scope) {
  py::class_<Class, systems::AbstractValue> py_class(
      scope, TemporaryClassName<Class>().c_str());
  // Only use copy (clone) construction.
  // Ownership with `unique_ptr<T>` has some annoying caveats, and some are
  // simplified by always copying.
  // See warnings for `set_value` for presently unavoidable caveats.
  py_class.def(py::init<const T&>());
  // N.B. `reference_internal` for pybind POD types (int, str, etc.) does not
  // really do anything meaningful.
  // TODO(eric.cousineau): Add check to get rid of this.
  py_class
    .def("get_value", &Class::get_value, py_reference_internal)
    .def("get_mutable_value", &Class::get_mutable_value, py_reference_internal)
    // WARNING: This will destroy C++ objects, while potentially leaving a
    // dangling Python reference, since the `unique_ptr` will be reset
    // internally.
    // TODO(eric.cousineau): Resolve this, either by (a) using a snowflake
    // `unique_ptr_tracked` should be used in `Value<>`, (b) consider
    // `shared_ptr`, or (c) throw an error if this object's value is already
    // registered in `pybind` when calling `set_value`.
    // (c) ONLY works if `set_value` is ONLY called by Python, which is not
    // enforceable, as it could be called by C++.
    .def("set_value", &Class::set_value);
  py::module py_module = py::module::import("pydrake.systems.framework");
  AddTemplateClass(py_module, "Value", py_class, GetPyParam<T>());
  return py_class;
}

}  // namespace pysystems
}  // namespace pydrake
}  // namespace drake
