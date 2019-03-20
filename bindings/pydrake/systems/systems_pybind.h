#pragma once

/// @file
/// Helpers for defining Python types within the Systems framework.

#include <string>

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/common/cpp_param_pybind.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/common/drake_throw.h"
#include "drake/common/value.h"

namespace drake {
namespace pydrake {
namespace pysystems {

/// Binds `Clone` and Pythonic `__copy__` and `__deepcopy__` for a class.
template <typename PyClass>
void DefClone(PyClass* ppy_class) {
  using Class = typename PyClass::type;
  PyClass& py_class = *ppy_class;
  py_class  // BR
      .def("Clone", &Class::Clone)
      .def("__copy__", &Class::Clone)
      .def("__deepcopy__",
          [](const Class* self, py::dict /* memo */) { return self->Clone(); });
}

/// Defines an instantiation of `pydrake.systems.framework.Value[...]`. This is
/// only meant to bind `Value<T>` (or specializations thereof).
/// @prereq `T` must have already been exposed to `pybind11`.
/// @param scope Parent scope.
/// @tparam T Inner parameter of `Value<T>`.
/// @tparam Class Class to be bound. By default, `Value<T>` is used.
/// @returns Reference to the registered Python type.
template <typename T, typename Class = drake::Value<T>>
py::class_<Class, drake::AbstractValue> AddValueInstantiation(
    py::module scope) {
  py::class_<Class, drake::AbstractValue> py_class(
      scope, TemporaryClassName<Class>().c_str());
  // Register instantiation.
  py::module py_framework = py::module::import("pydrake.systems.framework");
  AddTemplateClass(py_framework, "Value", py_class, GetPyParam<T>());
  // Only use copy (clone) construction.
  // Ownership with `unique_ptr<T>` has some annoying caveats, and some are
  // simplified by always copying.
  // See docstring for `set_value` for presently unavoidable caveats.
  py_class.def(py::init<const T&>());
  // Define emplace constructor.
  // TODO(eric.cousineau): This presently requires that `T` be aliased or
  // registered. For things like `std::vector`, this fails. Consider alternative
  // to retrieve Python type from T?
  py::object py_T = GetPyParam<T>()[0];
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
  // N.B. `reference_internal` for pybind POD types (int, str, etc.) does not
  // really do anything meaningful.
  // TODO(eric.cousineau): Add check to warn about this.
  py_class  // BR
      .def("get_value", &Class::get_value, py_reference_internal)
      .def("get_mutable_value", &Class::get_mutable_value,
          py_reference_internal);
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
  return py_class;
}

// N.B. This should be kept in sync with the `*_DEFAULT_SCALARS` macro in
// `default_scalars.h`.
/// Type pack defining common scalar types.
using CommonScalarPack = type_pack<  // BR
    double,                          //
    AutoDiffXd,                      //
    symbolic::Expression>;

// N.B. This should be kept in sync with the `*_DEFAULT_NONSYMBOLIC_SCALARS`
// macro in `default_scalars.h`.
/// Type pack for non-symbolic common scalar types.
using NonSymbolicScalarPack = type_pack<  // BR
    double,                               //
    AutoDiffXd>;

}  // namespace pysystems
}  // namespace pydrake
}  // namespace drake
