#include <memory>
#include <string>

#include "pybind11/eval.h"

#include "drake/bindings/generated_docstrings/common.h"
#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/autodiffutils/autodiffutils_py.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/submodules_py.h"
#include "drake/bindings/pydrake/common/text_logging_pybind.h"
#include "drake/bindings/pydrake/math/math_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic/symbolic_py.h"
#include "drake/common/constants.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_assertion_error.h"
#include "drake/common/drake_path.h"
#include "drake/common/find_resource.h"
#include "drake/common/memory_file.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/nice_type_name_override.h"
#include "drake/common/parallelism.h"
#include "drake/common/random.h"
#include "drake/common/sha256.h"
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

// Override for SetNiceTypeNamePtrOverride, to ensure that instances that are
// registered (along with their types) can use their Python class's name.
std::string PyNiceTypeNamePtrOverride(const type_erased_ptr& ptr) {
  DRAKE_DEMAND(ptr.raw != nullptr);
  const std::string cc_name = NiceTypeName::Get(ptr.info);
  if (cc_name.find("pydrake::") != std::string::npos) {
    py::handle obj = ResolvePyObject(ptr);
    if (obj) {
      py::handle cls = obj.get_type();
      const bool use_qualname = true;
      return py::str("{}.{}").format(
          cls.attr("__module__"), internal::PrettyClassName(cls, use_qualname));
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

namespace {
// We put the work of PYBIND11_MODULE into a function so that we can easily
// catch exceptions.
void InitLowLevelModules(py::module m) {
  m.doc() = "Bindings for //common:common";
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
  constexpr auto& doc = pydrake_doc_common.drake;

  // Morph any DRAKE_ASSERT and DRAKE_DEMAND failures into SystemExit exceptions
  // instead of process aborts.  See RobotLocomotion/drake#5268.
  drake_set_assertion_failure_to_throw_exception();

  // TODO(jwnimmer-tri) Split out the bindings for functions and classes into
  // their own separate files, so that this file is only an orchestrator.

  // WARNING: Deprecations for this module can be *weird* because of stupid
  // cyclic dependencies (#7912). If you need functions that immediately import
  // `pydrake.common.deprecation` (e.g. DeprecateAttribute, WrapDeprecated),
  // please ping @EricCousineau-TRI.

  m.attr("_HAVE_SPDLOG") = logging::kHaveSpdlog;

  // Python users should not touch the C++ level; thus, we bind this privately.
  m.def("_set_log_level", &logging::set_log_level, py::arg("level"),
      doc.logging.set_log_level.doc);

  internal::MaybeRedirectPythonLogging();
  m.def("_use_native_cpp_logging", &internal::UseNativeCppLogging);

  {
    using Class = Parallelism;
    constexpr auto& cls_doc = doc.Parallelism;
    py::class_<Class> cls(m, "Parallelism", cls_doc.doc);
    py::implicitly_convertible<bool, Class>();
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<bool>(), py::arg("parallelize").noconvert(),
            cls_doc.ctor.doc_1args_parallelize)
        .def(py::init<int>(), py::arg("num_threads").noconvert(),
            cls_doc.ctor.doc_1args_num_threads)
        // Note that we can't bind Parallelism::None(), because `None`
        // is a reserved word in Python.
        .def_static("Max", &Class::Max, cls_doc.Max.doc)
        .def("num_threads", &Class::num_threads, cls_doc.num_threads.doc)
        .def("__repr__", [](const Class& self) {
          const int num_threads = self.num_threads();
          return fmt::format("Parallelism(num_threads={})", num_threads);
        });
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = Sha256;
    constexpr auto& cls_doc = doc.Sha256;
    py::class_<Class> cls(m, "Sha256", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc)
        .def_static("Checksum",
            py::overload_cast<std::string_view>(&Class::Checksum),
            cls_doc.Checksum.doc_1args_data)
        .def_static("Parse", &Class::Parse, cls_doc.Parse.doc)
        .def("to_string", &Class::to_string, cls_doc.to_string.doc)
        .def(py::self == py::self)
        .def(py::self != py::self)
        .def(py::self < py::self)
        .def(py::pickle([](const Sha256& self) { return self.to_string(); },
            [](const std::string& ascii_hash) {
              std::optional<Sha256> sha_maybe = Sha256::Parse(ascii_hash);
              DRAKE_THROW_UNLESS(sha_maybe.has_value());
              return *sha_maybe;
            }));
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = MemoryFile;
    constexpr auto& cls_doc = doc.MemoryFile;
    py::class_<Class> cls(m, "MemoryFile", cls_doc.doc);
    py::object ctor = m.attr("MemoryFile");
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<std::string, std::string, std::string>(),
            py::arg("contents"), py::arg("extension"), py::arg("filename_hint"),
            cls_doc.ctor.doc_3args)
        .def(
            "contents",
            [](const Class& self) { return py::bytes(self.contents()); },
            cls_doc.contents.doc)
        .def("extension", &Class::extension, py_rvp::reference_internal,
            cls_doc.extension.doc)
        .def("sha256", &Class::sha256, py_rvp::reference_internal,
            cls_doc.sha256.doc)
        .def("filename_hint", &Class::filename_hint, py_rvp::reference_internal,
            cls_doc.filename_hint.doc)
        .def_static("Make", &Class::Make, py::arg("path"), cls_doc.Make.doc)
        .def(py::pickle(
            [](const MemoryFile& self) {
              return py::dict(py::arg("contents") = self.contents(),
                  py::arg("extension") = self.extension(),
                  py::arg("filename_hint") = self.filename_hint());
            },
            [ctor](const py::dict& kwargs) {
              return ctor(**kwargs).cast<MemoryFile>();
            }));
    // Note: __repr__ is defined in _common_extra.py.
    DefCopyAndDeepCopy(&cls);
    // Add the same __fields__ that DefAttributesUsingSerialize would have.
    cls.def_property_readonly_static("__fields__", [](py::object /* cls */) {
      auto str_ctor = py::eval("str");
      auto bytes_ctor = py::eval("bytes");
      auto make_namespace = py::module::import("types").attr("SimpleNamespace");
      auto contents = make_namespace();
      py::setattr(contents, "name", py::str("contents"));
      py::setattr(contents, "type", bytes_ctor);
      auto extension = make_namespace();
      py::setattr(extension, "name", py::str("extension"));
      py::setattr(extension, "type", str_ctor);
      auto filename_hint = make_namespace();
      py::setattr(filename_hint, "name", py::str("filename_hint"));
      py::setattr(filename_hint, "type", str_ctor);
      py::list fields;
      fields.append(contents);
      fields.append(extension);
      fields.append(filename_hint);
      return py::tuple(fields);
    });
    // Use _rewrite_yaml_dump_attr_name to instruct yaml_dump_typed to access
    // the "_foo" property instead of "foo". Override __setattr__ to instruct
    // yaml_load_type to do the same.
    cls.def_static("_rewrite_yaml_dump_attr_name",
        [](std::string_view name) { return fmt::format("_{}", name); });
    cls.def("__setattr__", [](Class& self, py::str name, py::object value) {
      const std::string name_str{name};
      if (name_str == "contents" || name_str == "extension" ||
          name_str == "filename_hint") {
        name = py::str(fmt::format("_{}", name_str));
      }
      py::eval("object.__setattr__")(self, name, value);
    });
    // Provide properties for use by yaml_{dump,load}_typed.
    cls.def_property(
        "_contents",
        [](const Class& self) -> py::bytes { return self.contents(); },
        [](Class& self, const py::bytes& contents) {
          self = MemoryFile{
              std::string{contents}, self.extension(), self.filename_hint()};
        });
    cls.def_property(
        "_extension", [](const Class& self) { return self.extension(); },
        [](Class& self, const std::string& extension) {
          self = MemoryFile{self.contents(), extension, self.filename_hint()};
        });
    cls.def_property(
        "_filename_hint",
        [](const Class& self) { return self.filename_hint(); },
        [](Class& self, const std::string& filename_hint) {
          self = MemoryFile{self.contents(), self.extension(), filename_hint};
        });
  }

  ExecuteExtraPythonCode(m, true);

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
      py::arg("distribution"), py::arg("x"), doc.CalcProbabilityDensity.doc);
  // N.B. The AutoDiffXd overload is bound later on in this function.

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
  random_generator_cls  // BR
      .def(py::init<>(), doc.RandomGenerator.ctor.doc_0args)
      .def(py::init<RandomGenerator::result_type>(), py::arg("seed"),
          doc.RandomGenerator.ctor.doc_1args)
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
  m.def("trigger_an_assertion_failure", &trigger_an_assertion_failure,
      "Trigger a Drake C++ assertion failure");

  m.attr("kDrakeAssertIsArmed") = kDrakeAssertIsArmed;

  // Make nice_type_name use Python type info when available.
  drake::internal::SetNiceTypeNamePtrOverride(PyNiceTypeNamePtrOverride);

  // =========================================================================
  // Now we'll starting defining other modules.
  // The following needs to proceed in topological dependency order.
  // =========================================================================

  // Define `_testing` submodule.
  py::module pydrake_top = py::eval("sys.modules['pydrake']");
  py::module pydrake_common = py::eval("sys.modules['pydrake.common']");

  py::module testing = pydrake_common.def_submodule("_testing");
  testing::def_testing(testing);

  // Install NumPy warning filters.
  // N.B. This may interfere with other code, but until that is a confirmed
  // issue, we should aggressively try to avoid these warnings.
  py::module::import("pydrake.common.deprecation")
      .attr("install_numpy_warning_filters")();

  // Install NumPy formatters patch.
  py::module::import("pydrake.common.compatibility")
      .attr("maybe_patch_numpy_formatters")();

  // Define `autodiffutils` top-level module.
  py::module autodiffutils = pydrake_top.def_submodule("autodiffutils");
  autodiffutils.doc() = "Bindings for Eigen AutoDiff Scalars";
  internal::DefineAutodiffutils(autodiffutils);
  ExecuteExtraPythonCode(autodiffutils, true);

  // Define overloads in `pydrake.common` that require AutoDiffXd.
  m.def("CalcProbabilityDensity", &CalcProbabilityDensity<AutoDiffXd>,
      py::arg("distribution"), py::arg("x"), doc.CalcProbabilityDensity.doc);

  // Define `symbolic` top-level module.
  py::module symbolic = pydrake_top.def_submodule("symbolic");
  symbolic.doc() =
      "Symbolic variable, variables, monomial, expression, polynomial, and "
      "formula";
  internal::DefineSymbolicMonolith(symbolic);
  ExecuteExtraPythonCode(symbolic, true);

  // Define `value` submodule.
  py::module value = pydrake_common.def_submodule("value");
  value.doc() = "Bindings for //common:value";
  internal::DefineModuleValue(value);

  // Define `eigen_geometry` submodule.
  py::module eigen_geometry = pydrake_common.def_submodule("eigen_geometry");
  eigen_geometry.doc() = "Bindings for Eigen geometric types.";
  internal::DefineModuleEigenGeometry(eigen_geometry);
  ExecuteExtraPythonCode(eigen_geometry, false);

  // Define `math` top-level module.
  py::module math = pydrake_top.def_submodule("math");
  // N.B. Docstring contained in `_math_extra.py`.
  internal::DefineMathOperators(math);
  internal::DefineMathMatmul(math);
  internal::DefineMathMonolith(math);
  ExecuteExtraPythonCode(math, true);

  // Define `schema` submodule.
  py::module schema = pydrake_common.def_submodule("schema");
  schema.doc() = "Bindings for the common.schema package.";
  internal::DefineModuleSchema(schema);
}
}  // namespace

PYBIND11_MODULE(common, m) {
  try {
    InitLowLevelModules(m);
  } catch (const std::exception& e) {
    drake::log()->critical("Could not initialize pydrake: {}", e.what());
    throw;
  }
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
