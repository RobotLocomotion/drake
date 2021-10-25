#pragma once

#include <string>

#include "pybind11/operators.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/identifier.h"

namespace drake {
namespace pydrake {

/// Binds an Identifier instantiation.
template <typename Class, typename ModuleOrClass>
void BindIdentifier(
    ModuleOrClass m, const std::string& name, const char* id_doc) {
  constexpr auto& cls_doc = pydrake_doc.drake.Identifier;

  py::class_<Class>(m, name.c_str(), id_doc)
      .def("get_value", &Class::get_value, cls_doc.get_value.doc)
      .def("is_valid", &Class::is_valid, cls_doc.is_valid.doc)
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def(py::self < py::self)
      .def(py::hash(py::self))
      .def_static("get_new_id", &Class::get_new_id, cls_doc.get_new_id.doc)
      .def("__repr__", [name](const Class& self) {
        return py::str("<{} value={}>").format(name, self.get_value());
      });
}

}  // namespace pydrake
}  // namespace drake
