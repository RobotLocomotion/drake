#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/drake_deprecated.h"

namespace drake {
namespace pydrake {
namespace {

class ExampleCppClass {
 public:
  ExampleCppClass() {}

  // Deprecated constructors.
  explicit ExampleCppClass(int) {}

  void DeprecatedMethod() {}
  int deprecated_prop{};

  // Good overload.
  void overload() {}

  // Deprecated overload.
  // N.B. We add an actual C++ deprecation to show an example of how to
  // suppress the error when calling the method.
  DRAKE_DEPRECATED("2038-01-19", "Example message for overload")
  void overload(int) {}
};

PYBIND11_MODULE(cc_module, m) {
  m.def("emit_deprecation", []() {
    // N.B. This is only for coverage. You generally do not need this.
    WarnDeprecated("Example emitting of deprecation");
  });

  py::class_<ExampleCppClass> cls(m, "ExampleCppClass");
  cls  // BR
      .def(py::init())
      .def(py_init_deprecated<ExampleCppClass, int>("Example message for ctor"))
      .def(py_init_deprecated("Example message for factory ctor",
          [](double) { return ExampleCppClass(); }))
      .def("overload", py::overload_cast<>(&ExampleCppClass::overload));

  // Add deprecated method.
  cls.def("DeprecatedMethod", &ExampleCppClass::DeprecatedMethod);
  DeprecateAttribute(cls, "DeprecatedMethod", "Example message for method");

  // Add deprecated property.
  cls.def_readwrite("deprecated_prop", &ExampleCppClass::deprecated_prop);
  DeprecateAttribute(cls, "deprecated_prop", "Example message for property");

  // Add deprecated overload.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  cls.def("overload", WrapDeprecated("Example message for overload",
                          py::overload_cast<int>(&ExampleCppClass::overload)));
#pragma GCC diagnostic pop
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
