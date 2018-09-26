#pragma once

#include <string>

#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/type_safe_index.h"

namespace drake {
namespace pydrake {

/// Binds a TypeSafeIndex instantiation.
// TODO(jamiesnape): Find a way to add documentation to bindings that use this
// function.
template <typename Type>
auto BindTypeSafeIndex(py::module m, const std::string& name) {
  py::class_<Type> cls(m, name.c_str());
  cls
    .def(py::init<int>())
    .def("__int__", &Type::operator int)
    .def("__eq__", [](const Type* self, const Type* other) {
      return *self == *other;
    }, py::is_operator())
    .def("__eq__", [](const Type* self, int other) {
      return *self == other;
    }, py::is_operator())
    // TODO(eric.cousineau): Add more operators.
    .def("is_valid", &Type::is_valid);
  return cls;
}

}  // namespace pydrake
}  // namespace drake
