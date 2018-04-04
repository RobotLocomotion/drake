#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_assertion_error.h"
#include "drake/common/drake_path.h"
#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"

namespace drake {
namespace pydrake {

// This function is defined in drake/common/drake_assert_and_throw.cc.
extern "C" void drake_set_assertion_failure_to_throw_exception();

namespace {
void trigger_an_assertion_failure() {
  DRAKE_DEMAND(false);
}
}  // namespace

PYBIND11_MODULE(_common_py, m) {
  m.doc() = "Bindings for //common:common";

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
  m.def("AddResourceSearchPath", &AddResourceSearchPath,
        "Adds a path in which to search for resource files. "
        "The path refers to the relative path within the Drake repository, ",
        py::arg("search_path"));
  // Convenient wrapper to get the list of resource search paths.
  m.def("GetResourceSearchPaths", &GetResourceSearchPaths,
        "Gets a copy of the list of paths set programmatically in which "
        "resource files are searched.");
  // Convenient wrapper for querying FindResource(resource_path).
  m.def("FindResourceOrThrow", &FindResourceOrThrow,
        "Attempts to locate a Drake resource named by the given path string. "
        "The path refers to the relative path within the Drake repository, "
        "e.g., drake/examples/pendulum/Pendulum.urdf. Raises an exception "
        "if the resource was not found.",
        py::arg("resource_path"));
  m.def("temp_directory", &temp_directory,
        "Returns a directory location suitable for temporary files that is "
        "the value of the environment variable TEST_TMPDIR if defined or "
        "otherwise ${TMPDIR:-/tmp}/robotlocomotion_drake_XXXXXX where each X "
        "is replaced by a character from the portable filename character set. "
        "Any trailing / will be stripped from the output.");
  // Returns the fully-qualified path to the root of the `drake` source tree.
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  m.def("GetDrakePath", &GetDrakePath,
        "Get Drake path");
  #pragma GCC diagnostic pop  // pop -Wdeprecated-declarations
  // These are meant to be called internally by pydrake; not by users.
  m.def("set_assertion_failure_to_throw_exception",
        &drake_set_assertion_failure_to_throw_exception,
        "Set Drake's assertion failure mechanism to be exceptions");
  m.def("trigger_an_assertion_failure", &trigger_an_assertion_failure,
        "Trigger a Drake C++ assertion failure");
}

}  // namespace pydrake
}  // namespace drake
