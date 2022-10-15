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

struct MyData1 {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(quux));
  }
  double quux{0.0};
};

struct MyData2 {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(some_bool));
    a->Visit(DRAKE_NVP(some_int));
    a->Visit(DRAKE_NVP(some_uint64));
    a->Visit(DRAKE_NVP(some_float));
    a->Visit(DRAKE_NVP(some_double));
    a->Visit(DRAKE_NVP(some_string));
    a->Visit(DRAKE_NVP(some_eigen));
    a->Visit(DRAKE_NVP(some_optional));
    a->Visit(DRAKE_NVP(some_vector));
    a->Visit(DRAKE_NVP(some_map));
    a->Visit(DRAKE_NVP(some_variant));
  }
  bool some_bool{};
  int some_int{};
  std::uint64_t some_uint64{};
  float some_float{};
  double some_double{};
  std::string some_string;
  Eigen::MatrixXd some_eigen;
  std::optional<double> some_optional;
  std::vector<double> some_vector;
  std::map<std::string, double> some_map;
  std::variant<double, MyData1> some_variant;
};

// This is a manually-created mock up of part of what mkdoc would produce for
// the MyData2 struct. TODO(jwnimmer-tri) If maintaining this struct by hand
// becomes too brittle, we could instead add BUILD rules to generate it from
// an actual header file.
struct MyData2Docs {
  const char* doc = "MyData2 class overview.";
  auto Serialize__fields() const {
    return std::array{
        std::make_pair("some_bool", "Field docstring for a bool."),
        std::make_pair("some_int", "Field docstring for a int."),
        std::make_pair("some_uint64", "Field docstring for a uint64."),
        std::make_pair("some_float", "Field docstring for a float."),
        std::make_pair("some_double", "Field docstring for a double."),
        std::make_pair("some_string", "Field docstring for a string."),
        std::make_pair("some_eigen", "Field docstring for a eigen."),
        std::make_pair("some_optional", "Field docstring for a optional."),
        std::make_pair("some_vector", "Field docstring for a vector."),
        std::make_pair("some_map", "Field docstring for a map."),
        std::make_pair("some_variant", "Field docstring for a variant.")};
  }
};

}  // namespace

PYBIND11_MODULE(serialize_test_util, m) {
  // Bind MyData1 with no documentation.
  py::class_<MyData1> cls1(m, "MyData1");
  cls1  // BR
      .def(py::init())
      .def(ParamInit<MyData1>());
  DefAttributesUsingSerialize(&cls1);
  DefReprUsingSerialize(&cls1);

  // Bind MyData2 along with its documentation.
  constexpr MyData2Docs cls2_doc;
  py::class_<MyData2> cls2(m, "MyData2", cls2_doc.doc);
  cls2  // BR
      .def(py::init())
      .def(ParamInit<MyData2>());
  DefAttributesUsingSerialize(&cls2, cls2_doc);
  DefReprUsingSerialize(&cls2);
}

}  // namespace pydrake
}  // namespace drake
