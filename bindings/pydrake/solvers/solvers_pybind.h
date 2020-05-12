#pragma once

/// @file
/// Helper methods for binding solvers.

#include <memory>
#include <string>

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace pysolvers {

// Binds both `AcquireLicense` and the corresponding returned `License` class.
// This wraps the `License` class (see implementation) and permits explicit
// context management.
template <typename PyClass, typename Doc>
void BindAcquireLicense(PyClass* pcls, Doc&& cls_doc) {
  using Class = typename PyClass::type;
  using License = typename Class::License;
  auto& cls = *pcls;

  // Permit `pybind11` to bind the forward-declared `License` class (whose ctor
  // / dtor is type-erased by using `shared_ptr`).
  struct PyLicense {
    PyLicense* enter() { return this; }
    void exit(py::args, py::kwargs) {
      // Explicitly release lock.
      ptr = nullptr;
    }
    bool is_valid() const { return ptr != nullptr; }

    std::shared_ptr<License> ptr;
  };

  std::string license_doc =
      py::str("Context-manageable license from ``{}.AcquireLicense()``.")
          .format(cls.attr("__name__"));
  py::class_<PyLicense>(cls, "License", license_doc.c_str())
      .def("__enter__", &PyLicense::enter)
      .def("__exit__", &PyLicense::exit)
      .def("is_valid", &PyLicense::is_valid,
          "Indicates that this has a valid license that has not been "
          "released.");
  cls.def_static(
      "AcquireLicense", []() { return PyLicense{Class::AcquireLicense()}; },
      cls_doc.AcquireLicense.doc);
}

}  // namespace pysolvers
}  // namespace pydrake
}  // namespace drake
