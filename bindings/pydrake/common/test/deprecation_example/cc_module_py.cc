#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace {

class ExampleCppClass {
 public:
  void DeprecatedMethod() {}
  int deprecated_prop{};

  // Good overload.
  void overload() {}

  // Deprecated overload.
  void overload(int) {}
};

PYBIND11_MODULE(cc_module, m) {
  // Add nominal bindings.
  py::class_<ExampleCppClass> cls(m, "ExampleCppClass");
  cls  // BR
      .def(py::init())
      .def("overload", py::overload_cast<>(&ExampleCppClass::overload));

  // Add deprecated method.
  cls.def("DeprecatedMethod", &ExampleCppClass::DeprecatedMethod);
  DeprecateAttribute(cls, "DeprecatedMethod", "Deprecated method");

  // Add deprecated property.
  cls.def_readwrite("deprecated_prop", &ExampleCppClass::deprecated_prop);
  DeprecateAttribute(cls, "deprecated_prop", "Deprecated property");

  // Add deprecated overload.
  cls.def("overload", [](ExampleCppClass* self, int value) {
    // This deprecates the specific overload.
    WarnDeprecated("overload(int) is deprecated");
    self->overload(value);
  });
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
