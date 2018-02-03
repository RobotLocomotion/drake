#pragma once

/// @file
/// Helpers for defining Python types within the Systems framework.

#include <string>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/cpp_template_pybind.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace pydrake {
namespace pysystems {

/// Defines an instantiation of `pydrake.systems.framework.Value[...]`. This is
/// only meant to bind `Value<T>` (or specializations thereof).
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
  // See docstring for `set_value` for presently unavoidable caveats.
  py_class.def(py::init<const T&>());
  // N.B. `reference_internal` for pybind POD types (int, str, etc.) does not
  // really do anything meaningful.
  // TODO(eric.cousineau): Add check to warn about this.
  py_class
    .def("get_value", &Class::get_value, py_reference_internal)
    .def("get_mutable_value", &Class::get_mutable_value, py_reference_internal);
  std::string set_value_docstring = "Replaces stored value with a new one.";
  if (!std::is_copy_constructible<T>::value) {
    set_value_docstring += R"""(

@note The value type for this class is non-copyable.
You should ensure that you do not have any dangling references to previous
values returned by `get_value` or `get_mutable_value`, because this memory will
be destroyed when it is replaced, since it is stored using `unique_ptr<>`.
)""";
  }
  py_class.def("set_value", &Class::set_value, set_value_docstring.c_str());
  // Register instantiation.
  py::module py_module = py::module::import("pydrake.systems.framework");
  AddTemplateClass(py_module, "Value", py_class, GetPyParam<T>());
  return py_class;
}

}  // namespace pysystems
}  // namespace pydrake
}  // namespace drake
