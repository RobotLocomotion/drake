#pragma once

#include <string>

#include <pybind11/pybind11.h>

#include "drake/common/type_safe_index.h"

// This is a header-safe alias for use as part of `pydrake`.
namespace py = pybind11;

namespace drake {

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

}  // namespace drake
