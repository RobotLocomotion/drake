// This is for testing only, will not compile.
#include "pybind11/pybind11.h"
#include "pybind11/test/sample_header.h"

namespace namespace_1 {
namespace namespace_2 {

PYBIND11_MODULE(dummy_module, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace namespace_1::namespace_2;
  constexpr auto& doc = sample_header_doc.namespace_1.namespace_2;

  py::module::import("namespace_1.namespace_2.Struct2");

  constexpr auto& cls_doc = doc.DummyClass;
  py::class_<DummyClass>(m, "DummyClass", cls_doc.doc)
      .def(py::init<>(), cls_doc.ctor.doc);

  constexpr auto& cls_doc = doc.
    Struct1;

  py::class_<Struct1>(
      m, "Struct1", cls_doc.doc)
      .def_readwrite(
          "var_1", &Struct1::var_1, cls_doc.var_1.doc)
      .def_readwrite("var_2", &Struct1::var_2,
          cls_doc.
          var_2.doc)
      .def_readwrite("var_3", &Struct1::var_3,
          cls_doc.var_3
          .doc)
      .def_readwrite("var_4", &Struct1::var_4,
          cls_doc.var_4.doc)
      .def_readwrite("var_5", &Struct1::var_5,
          cls_doc.  // BR Comment to test
          var_5.doc)
      .def_readwrite("var_6", &Struct1::var_6,
          cls_doc.var_6.doc)
      .def_readwrite("var_7", &Struct1::var_7,
          cls_doc.var_7.doc)
      .def_readwrite("var_8", &Struct1::var_8,
          cls_doc.var_8.doc)
  AddValueInstantiation<Struct1>(m);
}

}  // namespace namespace_2
}  // namespace namespace_1
