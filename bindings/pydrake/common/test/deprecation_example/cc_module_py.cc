#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/test/deprecation_example/example_class.h"  // NOLINT
#include "drake/bindings/pydrake/common/test/deprecation_example/example_class_documentation.h"  // NOLINT
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/drake_deprecated.h"

namespace drake {
namespace pydrake {
namespace {

PYBIND11_MODULE(cc_module, m) {
  constexpr auto& doc = pydrake_doc.drake.example_class;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::example_class;

  m.def("emit_deprecation", []() {
    // N.B. This is only for coverage. You generally do not need this.
    WarnDeprecated("Example emitting of deprecation");
  });

  {
    using Class = ExampleCppClass;
    constexpr auto& cls_doc = doc.ExampleCppClass;
    py::class_<Class> cls(m, "ExampleCppClass", cls_doc.doc);
    cls  // BR
        .def(py::init(), cls_doc.ctor.doc_0args)
        .def("overload", py::overload_cast<>(&Class::overload),
            cls_doc.overload.doc)
        .def_readwrite("prop", &Class::prop, cls_doc.prop.doc);

    // Bind deprecated constructor overload.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def(py_init_deprecated<Class, int>(
                cls_doc.ctor.doc_deprecated_deprecated_1args_int),
        cls_doc.ctor.doc_deprecated_deprecated_1args_int);
#pragma GCC diagnostic pop

    // Bind deprecated factory-style constructor.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def(
        py_init_deprecated(cls_doc.ctor.doc_deprecated_deprecated_1args_double,
            [](double arg) { return Class(arg); }),
        cls_doc.ctor.doc_deprecated_deprecated_1args_double);
#pragma GCC diagnostic pop

    // Bind deprecated overload.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def("overload",
        WrapDeprecated(cls_doc.overload.doc_deprecated,
            py::overload_cast<int>(&ExampleCppClass::overload)),
        cls_doc.overload.doc_deprecated);
#pragma GCC diagnostic pop

    // Bind entirely deprecated method.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def("DeprecatedMethod", &ExampleCppClass::DeprecatedMethod,
        cls_doc.DeprecatedMethod.doc_deprecated);
#pragma GCC diagnostic pop
    DeprecateAttribute(
        cls, "DeprecatedMethod", cls_doc.DeprecatedMethod.doc_deprecated);

    // Bind deprecated property (which forwards to original property).
    constexpr char deprecated_prop_doc[] =
        "Do not use deprecated_prop. This will be removed on or after "
        "2038-01-19.";
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def_property("deprecated_prop",
        [](const Class* self) { return self->prop; },
        [](Class* self, int value) { self->prop = value; },
        deprecated_prop_doc);
#pragma GCC diagnostic pop
    DeprecateAttribute(cls, "deprecated_prop", deprecated_prop_doc);
  }
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
