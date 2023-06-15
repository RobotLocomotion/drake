#pragma once

#include <string>

#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/type_safe_index.h"

namespace drake {
namespace pydrake {

/// Binds a TypeSafeIndex instantiation along with its Value[Class]
/// type-erasure wrapper.
template <typename Class>
auto BindTypeSafeIndex(
    py::module m, const std::string& name, const std::string& class_doc = "") {
  py::class_<Class> cls(m, name.c_str(), class_doc.c_str());
  cls  // BR
      .def(py::init<>(), pydrake_doc.drake.TypeSafeIndex.ctor.doc_0args)
      .def(
          py::init<int>(), pydrake_doc.drake.TypeSafeIndex.ctor.doc_1args_index)
      .def("__int__", &Class::operator int)
      .def("__index__", &Class::operator int)
      .def(py::self == py::self)
      .def(py::self == int{})
      .def(py::self < py::self)
      .def(py::hash(py::self))
      // TODO(eric.cousineau): Add more operators.
      .def("is_valid", &Class::is_valid,
          pydrake_doc.drake.TypeSafeIndex.is_valid.doc)
      .def("__repr__", [name](const Class& self) {
        return py::str("{}({})").format(name, static_cast<int>(self));
      });
  AddValueInstantiation<Class>(m);
  return cls;
}

}  // namespace pydrake
}  // namespace drake
