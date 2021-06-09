#pragma once

/// @file
/// Helpers for defining instantiations of drake::Value<>.

#include <string>

#include <fmt/format.h>

#include "drake/bindings/pydrake/common/cpp_param_pybind.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/drake_throw.h"
#include "drake/common/value.h"

namespace drake {
namespace pydrake {

/// Defines an instantiation of `pydrake.common.value.Value[...]`. This is only
/// meant to bind `Value<T>` (or specializations thereof).
/// @prereq `T` must have already been exposed to `pybind11`.
/// @param scope Parent scope.
/// @tparam T Inner parameter of `Value<T>`.
/// @tparam Class Class to be bound. By default, `Value<T>` is used.
/// @returns Reference to the registered Python type.
template <typename T, typename Class = drake::Value<T>>
py::class_<Class, drake::AbstractValue> AddValueInstantiation(
    py::module scope) {
  static_assert(!py::detail::is_pyobject<T>::value, "See docs for GetPyParam");
  py::module py_common = py::module::import("pydrake.common.value");
  py::class_<Class, drake::AbstractValue> py_class(
      scope, TemporaryClassName<Class>().c_str());
  // Register instantiation.
  py::tuple param = GetPyParam<T>();
  AddTemplateClass(py_common, "Value", py_class, param);
  // Only use copy (clone) construction.
  // Ownership with `unique_ptr<T>` has some annoying caveats, and some are
  // simplified by always copying.
  // See docstring for `set_value` for presently unavoidable caveats.
  py_class.def(py::init<const T&>());
  // Define emplace constructor.
  py::object py_T = param[0];
  py_class.def(py::init([py_T](py::args args, py::kwargs kwargs) {
    // Use Python constructor for the bound type.
    py::object py_v = py_T(*args, **kwargs);
    // TODO(eric.cousineau): Use `unique_ptr` for custom types if it's ever a
    // performance concern.
    // Use `type_caster` so that we are not forced to copy T, which is not
    // possible for non-movable types. This can be avoided if we bind a
    // `cpp_function` accepting a reference. However, that may cause the Python
    // instance to be double-initialized.
    py::detail::type_caster<T> caster;
    DRAKE_THROW_UNLESS(caster.load(py_v, false));
    const T& v = caster;  // Use implicit conversion from `type_caster<>`.
    return new Class(v);
  }));
  // If the type is registered via `py::class_`, or is of type `Object`
  // (`py::object`), then we can obtain a mutable view into the value.
  constexpr bool has_get_mutable_value =
      internal::is_generic_pybind_v<T> || std::is_same_v<T, Object>;
  if constexpr (has_get_mutable_value) {
    py::return_value_policy return_policy = py_rvp::reference_internal;
    if (std::is_same_v<T, Object>) {
      // N.B. This implies that `Object` will be copied by value; however, it
      // is only a shallow copy of the pointer, not a deep copy of the object.
      return_policy = py::return_value_policy::copy;
    }
    std::string set_value_docstring = "Replaces stored value with a new one.";
    if (!std::is_copy_constructible_v<T>) {
      set_value_docstring += R"""(

@note The value type for this class is non-copyable.
You should ensure that you do not have any dangling references to previous
values returned by `get_value` or `get_mutable_value`, because this memory
will
be destroyed when it is replaced, since it is stored using `unique_ptr<>`.
  )""";
    }
    py_class  // BR
        .def("get_value", &Class::get_value, return_policy)
        .def("get_mutable_value", &Class::get_mutable_value, return_policy)
        .def("set_value", &Class::set_value, set_value_docstring.c_str());
  } else {
    py_class  // BR
        .def("get_value", &Class::get_value)
        .def("get_mutable_value",
            [py_T](const Class&) {
              throw std::logic_error(
                  fmt::format("Cannot get mutable value (or reference) for a "
                              "type-conversion type: {}",
                      py::str(py_T).cast<std::string>()));
            })
        .def("set_value", &Class::set_value);
  }
  return py_class;
}

}  // namespace pydrake
}  // namespace drake
