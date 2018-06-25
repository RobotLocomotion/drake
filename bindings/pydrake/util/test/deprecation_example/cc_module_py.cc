#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/deprecation_pybind.h"

namespace drake {
namespace pydrake {
namespace {

class ExampleCppClass {
 public:
  void DeprecatedMethod() {}
  int deprecated_prop{};

  // Good overload.
  void overload() {}

  // Bad overload.
  void overload(int) {}
};

PYBIND11_MODULE(cc_module, m) {
  py::class_<ExampleCppClass> cls(m, "ExampleCppClass");
  py::handle cls_handle = cls;
  // Store messages for testing.
  cls.attr("message_overload") = "overload(int) is deprecated";
  cls.attr("message_method") = "Deprecated method";
  cls.attr("message_prop") = "Deprecated property";
  // Add deprecated members.
  cls
      .def(py::init())
      .def("DeprecatedMethod", &ExampleCppClass::DeprecatedMethod)
      .def_readwrite("deprecated_prop", &ExampleCppClass::deprecated_prop)
      .def("overload", py::overload_cast<>(&ExampleCppClass::overload))
      .def("overload", [cls_handle](ExampleCppClass* self, int value) {
        WarnDeprecated(cls_handle.attr("message_overload"));
        self->overload(value);
      });
  DeprecateAttribute(cls, "DeprecatedMethod", cls.attr("message_method"));
  DeprecateAttribute(cls, "deprecated_prop", cls.attr("message_prop"));
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
