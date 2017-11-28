#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_assertion_error.h"
#include "drake/common/drake_path.h"
#include "drake/common/find_resource.h"

namespace py = pybind11;

// This function is defined in drake/common/drake_assert_and_throw.cc.
extern "C" void drake_set_assertion_failure_to_throw_exception();

namespace {
void trigger_an_assertion_failure() {
  DRAKE_DEMAND(false);
}
}  // namespace

PYBIND11_MODULE(_pydrake_common, m) {
  m.doc() = "Bindings for //drake/common:common";

  // Turn DRAKE_ASSERT and DRAKE_DEMAND exceptions into native SystemExit.
  // Admittedly, it's unusual for a python library like pydrake to raise
  // SystemExit, but for now its better than C++ ::abort() taking down the
  // whole interpreter with a worse diagnostic message.
  py::register_exception_translator([](std::exception_ptr p) {
      try {
        if (p) { std::rethrow_exception(p); }
      } catch (const drake::detail::assertion_error& e) {
        PyErr_SetString(PyExc_SystemExit, e.what());
      }
    });
  // Convenient wrapper to add a resource search path.
  m.def("AddResourceSearchPath", &drake::AddResourceSearchPath,
        "Adds a path in which to search for resource files. "
        "The path refers to the relative path within the Drake repository, ",
        py::arg("search_path"));
  // Convenient wrapper to get the list of resource search paths.
  m.def("GetResourceSearchPaths", &drake::GetResourceSearchPaths,
        "Gets a copy of the list of paths set programmatically in which "
        "resource files are searched.");
  // Convenient wrapper for querying FindResource(resource_path).
  m.def("FindResourceOrThrow", &drake::FindResourceOrThrow,
        "Attempts to locate a Drake resource named by the given path string. "
        "The path refers to the relative path within the Drake repository, "
        "e.g., drake/examples/pendulum/Pendulum.urdf. Raises an exception "
        "if the resource was not found.",
        py::arg("resource_path"));
  // Returns the fully-qualified path to the root of the `drake` source tree.
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  m.def("GetDrakePath", &drake::GetDrakePath,
        "Get Drake path");
  #pragma GCC diagnostic pop  // pop -Wdeprecated-declarations
  // These are meant to be called internally by pydrake; not by users.
  m.def("set_assertion_failure_to_throw_exception",
        &drake_set_assertion_failure_to_throw_exception,
        "Set Drake's assertion failure mechanism to be exceptions");
  m.def("trigger_an_assertion_failure", &trigger_an_assertion_failure,
        "Trigger a Drake C++ assertion failure");
}
