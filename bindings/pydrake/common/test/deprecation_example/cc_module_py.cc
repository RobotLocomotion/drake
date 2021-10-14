/*
This file provides a set of examples for deprecating methods, overloads,
classes, etc.

Please review this file and the corresponding test,
`deprecation_utility_test.py`.
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

  // Example: Deprecating a constructor bound with ParamInit. Typical when
  // deprecating a C++ struct with no explicit constructor.
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

    // Example: Deprecation of constructor previously bound with `py::init<>`.
    // Can be used to deprecate a class (if all constructor overloads are
    // deprecated) or only deprecate an overloaded constructor.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def(py_init_deprecated<Class, int>(
                cls_doc.ctor.doc_deprecated_deprecated_1args_x),
        py::arg("x"), cls_doc.ctor.doc_deprecated_deprecated_1args_x);
#pragma GCC diagnostic pop

    // Example: A C++ constructor overload has been deprecated, and was
    // originally bound using custom factory constructors. We must reflect this
    // in the bindings.
    // See also:
    // https://pybind11.readthedocs.io/en/stable/advanced/classes.html#custom-constructors
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def(py_init_deprecated(cls_doc.ctor.doc_deprecated_deprecated_1args_y,
                [](double arg) { return Class(arg); }),
        py::arg("y"), cls_doc.ctor.doc_deprecated_deprecated_1args_y);
#pragma GCC diagnostic pop

    cls  // BR
        .def("overload", py::overload_cast<>(&Class::overload),
            cls_doc.overload.doc);

    // Example: Deprecation of an overloaded class method that had previously
    // been bound directly (or possibly as an overload method). The overload
    // may have preceded the deprecation, or it may be introduced by
    // introducing a replacement method to the old.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def("overload",
        WrapDeprecated(cls_doc.overload.doc_deprecated,
            py::overload_cast<int>(&Class::overload)),
        py::arg("x"), cls_doc.overload.doc_deprecated);
#pragma GCC diagnostic pop

    // Example: A C++ function with an argument name that has been renamed.
    // In C++, we simply change the function signature name with no ill effect
    // on the public API. However, in Python, the argument name is part of the
    // function signature, so we should deprecate the old spelling.
    // - Bind the *undeprecated* spelling *first*. This way, a user calling this
    // function without specifying the argument name will not come across the
    // deprecation.
    cls.def("FunctionWithArgumentName", &Class::FunctionWithArgumentName,
        py::arg("new_name"), cls_doc.FunctionWithArgumentName.doc);
    // - Now bind the deprecated spelling.
    constexpr char doc_FunctionWithArgumentName_deprecated[] =
        "FunctionWithArgumentName(old_name) is deprecated, and will be "
        "removed on or around 2038-01-18. Please use "
        "FunctionWithArgumentName(new_name) instead.";
    cls.def("FunctionWithArgumentName",
        WrapDeprecated(doc_FunctionWithArgumentName_deprecated,
            &Class::FunctionWithArgumentName),
        py::arg("old_name"), doc_FunctionWithArgumentName_deprecated);

    // Example: A C++ method with multiple overloads, where we wish to
    // deprecate all of the overloads in the bindings. One option is to
    // deprecate each overload, but instead we instead deprecate all overloads
    // by deprecating the Python symbol in the class using
    // `DeprecateAttribute`.
    // The motivation here is that the method access emits a deprecation
    // *immediately*, which means that if a user uses the method as a callback,
    // then this should quickly emit a deprecation.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def("DeprecatedMethod", py::overload_cast<>(&Class::DeprecatedMethod),
        cls_doc.DeprecatedMethod.doc_deprecated);
    cls.def("DeprecatedMethod",
        py::overload_cast<int>(&Class::DeprecatedMethod),
        cls_doc.DeprecatedMethod.doc_deprecated);
#pragma GCC diagnostic pop
    DeprecateAttribute(
        cls, "DeprecatedMethod", cls_doc.DeprecatedMethod.doc_deprecated);
  }
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
