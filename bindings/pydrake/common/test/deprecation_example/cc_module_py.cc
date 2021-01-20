/*
This file provides a set of examples for deprecating methods, overloads,
classes, etc.

Please review this file and the corresponding test,
`deprecation_example_test.py`.
*/

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
    WarnDeprecated("Example emitting of deprecation", "2038-01-19");
  });

  // Example: A C++ class has been entirely deprecated, so we reflect that in
  // the bindings by just deprecating its (parameter-initializing) constructor.
  {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    using Class = ExampleCppStruct;
    constexpr auto& cls_doc = doc.ExampleCppStruct;
    py::class_<Class> cls(m, "ExampleCppStruct", cls_doc.doc_deprecated);
    cls  // BR
        .def(DeprecatedParamInit<Class>(cls_doc.doc_deprecated))
        .def_readwrite("i", &Class::i)
        .def_readwrite("j", &Class::j);
#pragma GCC diagnostic pop
  }

  {
    using Class = ExampleCppClass;
    constexpr auto& cls_doc = doc.ExampleCppClass;
    py::class_<Class> cls(m, "ExampleCppClass", cls_doc.doc);
    cls  // BR
        .def(py::init(), cls_doc.ctor.doc_0args)
        .def_readwrite("prop", &Class::prop, cls_doc.prop.doc);

    // Example: A C++ constructor overload has been deprecated, and we must
    // reflect this in the bindings.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def(py_init_deprecated<Class, int>(
                cls_doc.ctor.doc_deprecated_deprecated_1args_x),
        py::arg("x"), cls_doc.ctor.doc_deprecated_deprecated_1args_x);
#pragma GCC diagnostic pop

    // Example: A C++ constructor overload has been deprecated, and was
    // originally bound using a factory-style constructor. We must reflect this
    // in the bindings.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def(py_init_deprecated(cls_doc.ctor.doc_deprecated_deprecated_1args_y,
                [](double arg) { return Class(arg); }),
        py::arg("y"), cls_doc.ctor.doc_deprecated_deprecated_1args_y);
#pragma GCC diagnostic pop

    cls  // BR
        .def("overload", py::overload_cast<>(&Class::overload),
            cls_doc.overload.doc);

    // Example: A C++ function overload has been deprecated, and we must
    // reflect this in the bindings.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def("overload",
        WrapDeprecated(cls_doc.overload.doc_deprecated,
            py::overload_cast<int>(&Class::overload)),
        py::arg("x"), cls_doc.overload.doc_deprecated);
#pragma GCC diagnostic pop

    // Example: A C++ method is entirely deprecated (not just overloads), and
    // we must reflect this in the bindings. Rather than deprecate each
    // overload, we instead deprecate all overloads by deprecating the symbol
    // in the class.
    // The motivation here is that the method access emits a deprecation
    // *immediately*, which means that if a user uses the method as a callback,
    // then this should quickly emit a deprecation.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def("DeprecatedMethod", &Class::DeprecatedMethod,
        cls_doc.DeprecatedMethod.doc_deprecated);
#pragma GCC diagnostic pop
    DeprecateAttribute(
        cls, "DeprecatedMethod", cls_doc.DeprecatedMethod.doc_deprecated);
  }
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
