#include <array>
#include <utility>

#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/name_value.h"

namespace drake {
namespace pydrake {
namespace {

struct MyData {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(foo));
    a->Visit(DRAKE_NVP(bar));
  }
  double foo{0.0};
  std::vector<double> bar;
};

// This is a manually-created mock up of part of what mkdoc would produce for
// the MyData struct. TODO(jwnimmer-tri) If maintaining this struct by hand
// becomes too brittle, we could instead add BUILD rules to generate it from
// an actual header file.
struct MyDataDocs {
  const char* doc = "MyDataDocs class overview.";
  auto Serialize__fields() const {
    return std::array{
        std::make_pair("foo", "This is the field docstring for foo."),
        std::make_pair("bar", "This is the field docstring for bar.")};
  }
};

struct MyData2 {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(quux));
  }
  double quux{0.0};
};

}  // namespace

PYBIND11_MODULE(serialize_test_util, m) {
  // Bind MyData along with its documentation.
  constexpr MyDataDocs cls_doc;
  py::class_<MyData> cls(m, "MyData", cls_doc.doc);
  cls  // BR
      .def(py::init())
      .def(ParamInit<MyData>());
  DefAttributesUsingSerialize(&cls, cls_doc);
  DefReprUsingSerialize(&cls);

  // Bind MyData2 with no documentation.
  py::class_<MyData2> cls2(m, "MyData2");
  cls2  // BR
      .def(py::init())
      .def(ParamInit<MyData2>());
  DefAttributesUsingSerialize(&cls2);
  DefReprUsingSerialize(&cls2);
}

}  // namespace pydrake
}  // namespace drake
