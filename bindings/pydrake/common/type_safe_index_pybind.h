#pragma once

#include <string>

#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/type_safe_index.h"

namespace drake {
namespace pydrake {

/// Binds a TypeSafeIndex instantiation.
template <typename Type>
auto BindTypeSafeIndex(
    py::module m, const std::string& name, const std::string& class_doc = "") {
  py::class_<Type> cls(m, name.c_str(), class_doc.c_str());
  cls  // BR
      .def(py::init<int>(), pydrake_doc.drake.TypeSafeIndex.ctor.doc_0args)
      .def("__int__", &Type::operator int)
      .def("__eq__",
          [](const Type* self, const Type* other) { return *self == *other; },
          py::is_operator())
      .def("__eq__", [](const Type* self, int other) { return *self == other; },
          py::is_operator())
      // TODO(eric.cousineau): Add more operators.
      .def("is_valid", &Type::is_valid,
          pydrake_doc.drake.TypeSafeIndex.is_valid.doc);
  return cls;
}

}  // namespace pydrake
}  // namespace drake
