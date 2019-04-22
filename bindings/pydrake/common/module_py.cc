#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_assertion_error.h"
#include "drake/common/drake_path.h"
#include "drake/common/find_resource.h"
#include "drake/common/random.h"
#include "drake/common/temp_directory.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace pydrake {

// This function is defined in drake/common/drake_assert_and_throw.cc.
extern "C" void drake_set_assertion_failure_to_throw_exception();

namespace {
void trigger_an_assertion_failure() {
  DRAKE_DEMAND(false);
}
}  // namespace

PYBIND11_MODULE(_module_py, m) {
  m.doc() = "Bindings for //common:common";

  constexpr auto& doc = pydrake_doc.drake;
  m.attr("_HAVE_SPDLOG") = logging::kHaveSpdlog;

  // TODO(eric.cousineau): Provide a Pythonic spdlog sink that connects to
  // Python's `logging` module; possibly use `pyspdlog`.
  m.def("set_log_level", &logging::set_log_level, py::arg("level"),
      doc.logging.set_log_level.doc);

  py::enum_<drake::RandomDistribution>(
      m, "RandomDistribution", doc.RandomDistribution.doc)
      .value("kUniform", drake::RandomDistribution::kUniform,
          doc.RandomDistribution.kUniform.doc)
      .value("kGaussian", drake::RandomDistribution::kGaussian,
          doc.RandomDistribution.kGaussian.doc)
      .value("kExponential", drake::RandomDistribution::kExponential,
          doc.RandomDistribution.kExponential.doc);

  // Adds a binding for drake::RandomGenerator.
  py::class_<RandomGenerator> random_generator_cls(
      m, "RandomGenerator", doc.RandomGenerator.doc);
  random_generator_cls
      .def(py::init<>(),
          "Default constructor. Seeds the engine with the default_seed.")
      .def(py::init<RandomGenerator::result_type>(),
          "Constructs the engine and initializes the state with a given "
          "value.")
      .def("__call__", [](RandomGenerator& self) { return self(); },
          "Generates a pseudo-random value.");

  // Turn DRAKE_ASSERT and DRAKE_DEMAND exceptions into native SystemExit.
  // Admittedly, it's unusual for a python library like pydrake to raise
  // SystemExit, but for now its better than C++ ::abort() taking down the
  // whole interpreter with a worse diagnostic message.
  py::register_exception_translator([](std::exception_ptr p) {
    try {
      if (p) {
        std::rethrow_exception(p);
      }
    } catch (const drake::detail::assertion_error& e) {
      PyErr_SetString(PyExc_SystemExit, e.what());
    }
  });
  // Convenient wrapper to add a resource search path.
  m.def("AddResourceSearchPath",
      [](const std::string& x) {
        WarnDeprecated("See API docs for deprecation notice.");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        AddResourceSearchPath(x);
#pragma GCC diagnostic pop
      },
      py::arg("search_path"), doc.AddResourceSearchPath.doc_deprecated);
  // Convenient wrapper to get the list of resource search paths.
  m.def("GetResourceSearchPaths",
      []() {
        WarnDeprecated("See API docs for deprecation notice.");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        return GetResourceSearchPaths();
#pragma GCC diagnostic pop
      },
      doc.GetResourceSearchPaths.doc_deprecated);
  // Convenient wrapper for querying FindResource(resource_path).
  m.def("FindResourceOrThrow", &FindResourceOrThrow,
      "Attempts to locate a Drake resource named by the given path string. "
      "The path refers to the relative path within the Drake repository, "
      "e.g., drake/examples/pendulum/Pendulum.urdf. Raises an exception "
      "if the resource was not found.",
      py::arg("resource_path"), doc.FindResourceOrThrow.doc);
  m.def("temp_directory", &temp_directory,
      "Returns a directory location suitable for temporary files that is "
      "the value of the environment variable TEST_TMPDIR if defined or "
      "otherwise ${TMPDIR:-/tmp}/robotlocomotion_drake_XXXXXX where each X "
      "is replaced by a character from the portable filename character set. "
      "Any trailing / will be stripped from the output.",
      doc.temp_directory.doc);
// Returns the fully-qualified path to the root of the `drake` source tree.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  m.def("GetDrakePath", &GetDrakePath, "Get Drake path",
      doc.GetDrakePath.doc_deprecated);
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
