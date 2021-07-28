#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/text_logging_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/constants.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_assertion_error.h"
#include "drake/common/drake_path.h"
#include "drake/common/find_resource.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/nice_type_name_override.h"
#include "drake/common/random.h"
#include "drake/common/temp_directory.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace pydrake {

using drake::internal::type_erased_ptr;

// This function is defined in drake/common/drake_assert_and_throw.cc.
extern "C" void drake_set_assertion_failure_to_throw_exception();

namespace {

void trigger_an_assertion_failure() {
  DRAKE_DEMAND(false);
}

// Resolves to a Python handle given a type erased pointer. If the instance or
// lowest-level RTTI type are unregistered, returns an empty handle.
py::handle ResolvePyObject(const type_erased_ptr& ptr) {
  auto py_type_info = py::detail::get_type_info(ptr.info);
  return py::detail::get_object_handle(ptr.raw, py_type_info);
}

// Gets a class's fully-qualified name.
std::string GetPyClassName(py::handle obj) {
  DRAKE_DEMAND(bool{obj});
  py::handle cls = obj.get_type();
  return py::str("{}.{}").format(
      cls.attr("__module__"), cls.attr("__qualname__"));
}

// Override for SetNiceTypeNamePtrOverride, to ensure that instances that are
// registered (along with their types) can use their Python class's name.
std::string PyNiceTypeNamePtrOverride(const type_erased_ptr& ptr) {
  DRAKE_DEMAND(ptr.raw != nullptr);
  const std::string cc_name = NiceTypeName::Get(ptr.info);
  if (cc_name.find("pydrake::") != std::string::npos) {
    py::handle obj = ResolvePyObject(ptr);
    if (obj) {
      return GetPyClassName(obj);
    }
  }
  return cc_name;
}

namespace testing {
// Registered type. Also a base class for UnregisteredDerivedType.
class RegisteredType {
 public:
  virtual ~RegisteredType() {}
};
// Completely unregistered type.
class UnregisteredType {};
// Unregistered type, but with a registered base.
class UnregisteredDerivedType : public RegisteredType {};

void def_testing(py::module m) {
  py::class_<RegisteredType>(m, "RegisteredType").def(py::init());
  // See comments in `module_test.py`.
  m.def("get_nice_type_name_cc_registered_instance",
      [](const RegisteredType& obj) { return NiceTypeName::Get(obj); });
  m.def("get_nice_type_name_cc_unregistered_instance",
      []() { return NiceTypeName::Get(RegisteredType()); });
  m.def("get_nice_type_name_cc_typeid",
      [](const RegisteredType& obj) { return NiceTypeName::Get(typeid(obj)); });
  m.def("get_nice_type_name_cc_unregistered_type",
      []() { return NiceTypeName::Get(UnregisteredType()); });
  m.def("make_cc_unregistered_derived_type", []() {
    return std::unique_ptr<RegisteredType>(new UnregisteredDerivedType());
  });
}
}  // namespace testing

PYBIND11_MODULE(_module_py, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
  m.doc() = "Bindings for //common:common";
  constexpr auto& doc = pydrake_doc.drake;

  // WARNING: Deprecations for this module can be *weird* because of stupid
  // cyclic dependencies (#7912). If you need functions that immediately import
  // `pydrake.common.deprecation` (e.g. DeprecateAttribute, WrapDeprecated),
  // please ping @EricCousineau-TRI.

  m.attr("_HAVE_SPDLOG") = logging::kHaveSpdlog;

  m.def("set_log_level", &logging::set_log_level, py::arg("level"),
      doc.logging.set_log_level.doc);

  internal::RedirectPythonLogging();

  py::enum_<drake::ToleranceType>(m, "ToleranceType", doc.ToleranceType.doc)
      .value("kAbsolute", drake::ToleranceType::kAbsolute,
          doc.ToleranceType.kAbsolute.doc)
      .value("kRelative", drake::ToleranceType::kRelative,
          doc.ToleranceType.kRelative.doc);

  py::enum_<drake::RandomDistribution>(
      m, "RandomDistribution", doc.RandomDistribution.doc)
      .value("kUniform", drake::RandomDistribution::kUniform,
          doc.RandomDistribution.kUniform.doc)
      .value("kGaussian", drake::RandomDistribution::kGaussian,
          doc.RandomDistribution.kGaussian.doc)
      .value("kExponential", drake::RandomDistribution::kExponential,
          doc.RandomDistribution.kExponential.doc);

  m.def("CalcProbabilityDensity", &CalcProbabilityDensity<double>,
       py::arg("distribution"), py::arg("x"), doc.CalcProbabilityDensity.doc)
      .def("CalcProbabilityDensity", &CalcProbabilityDensity<AutoDiffXd>,
          py::arg("distribution"), py::arg("x"),
          doc.CalcProbabilityDensity.doc);

  // Adds a binding for drake::RandomGenerator.
  py::class_<RandomGenerator> random_generator_cls(m, "RandomGenerator",
      (std::string(doc.RandomGenerator.doc) + R"""(

Note: For many workflows in drake, we aim to have computations that are fully
deterministic given a single random seed.  This is accomplished by passing a
RandomGenerator object.  To generate random numbers in numpy that are
deterministic given the C++ random seed (see drake issue #12632 for the
discussion), use e.g.

.. code-block:: python
    
   generator = pydrake.common.RandomGenerator()
   random_state = numpy.random.RandomState(generator())
   
   my_random_value = random_state.uniform()
   ...

)""")
          .c_str());
  random_generator_cls
      .def(py::init<>(),
          "Default constructor. Seeds the engine with the default_seed.")
      .def(py::init<RandomGenerator::result_type>(),
          "Constructs the engine and initializes the state with a given "
          "value.")
      .def(
          "__call__", [](RandomGenerator& self) { return self(); },
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
    } catch (const drake::internal::assertion_error& e) {
      PyErr_SetString(PyExc_SystemExit, e.what());
    }
  });
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
  // The pydrake function named GetDrakePath is residue from when there used to
  // be a C++ method named drake::GetDrakePath(). For backward compatibility,
  // we'll keep the pydrake function name intact even though there's no
  // matching C++ method anymore.
  m.def(
      "GetDrakePath",
      []() {
        py::object result;
        if (auto optional_result = MaybeGetDrakePath()) {
          result = py::str(*optional_result);
        }
        return result;
      },
      doc.MaybeGetDrakePath.doc);
  // These are meant to be called internally by pydrake; not by users.
  m.def("set_assertion_failure_to_throw_exception",
      &drake_set_assertion_failure_to_throw_exception,
      "Set Drake's assertion failure mechanism to be exceptions");
  m.def("trigger_an_assertion_failure", &trigger_an_assertion_failure,
      "Trigger a Drake C++ assertion failure");

  m.attr("kDrakeAssertIsArmed") = kDrakeAssertIsArmed;

  // Make nice_type_name use Python type info when available.
  drake::internal::SetNiceTypeNamePtrOverride(PyNiceTypeNamePtrOverride);

  // Define testing.
  py::module m_testing = m.def_submodule("_testing");
  testing::def_testing(m_testing);
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
