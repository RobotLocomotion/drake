#pragma once

#include <string>

#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/type_safe_index.h"

namespace drake {
namespace pydrake {

/// Binds a TypeSafeIndex instantiation.
template <typename Type>
auto BindTypeSafeIndex(py::module m, const std::string& name) {
  py::class_<Type> cls(m, name.c_str());
  cls
    .def(py::init<int>())
    .def("__int__", &Type::operator int)
    .def("is_valid", &Type::is_valid);
  return cls;
}

}  // namespace pydrake
}  // namespace drake
