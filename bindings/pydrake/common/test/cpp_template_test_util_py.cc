#include "drake/bindings/pydrake/common/cpp_template_pybind.h"

// @file
// Provides bindings to help test the public interfaces in `cpp_template.py`
// and `cpp_template_pybind.h`. The main test code is in
// `cpp_template_test.py`.

#include <string>
#include <vector>

#include "pybind11/eval.h"

#include "drake/common/nice_type_name.h"

using std::string;
using std::vector;

namespace drake {
namespace pydrake {
namespace {

template <typename... Ts>
struct SimpleTemplate {
  vector<string> GetNames() { return {NiceTypeName::Get<Ts>()...}; }
};

template <typename... Ts>
auto BindSimpleTemplate(py::module m) {
  using Class = SimpleTemplate<Ts...>;
  py::class_<Class> py_class(m, TemporaryClassName<Class>().c_str());
  py_class  // BR
      .def(py::init<>())
      .def("GetNames", &Class::GetNames);
  AddTemplateClass(m, "SimpleTemplate", py_class, GetPyParam<Ts...>());
  return py_class;
}

template <typename T>
void CheckValue(const string& expr, const T& expected) {
  EXPECT_EQ(py::eval(expr).cast<T>(), expected);
}

template <typename T>
struct TemplateWithDefault {
  string GetName() { return NiceTypeName::Get<T>(); }
};

template <typename T>
void BindTemplateWithDefault(py::module m) {
  using Class = TemplateWithDefault<T>;
  auto py_class =
      DefineTemplateClassWithDefault<Class>(m, "TemplateWithDefault",
          GetPyParam<T>(), "Documentation", std::nullopt, py::dynamic_attr());
  py_class  // BR
      .def(py::init<>())
      .def("GetName", &Class::GetName);
}

template <typename... Ts>
vector<string> SimpleFunction() {
  return {NiceTypeName::Get<Ts>()...};
}

std::string Callee(int) {
  return "int";
}
std::string Callee(double) {
  return "double";
}

struct SimpleType {
  template <typename... Ts>
  vector<string> SimpleMethod() {
    return {NiceTypeName::Get<Ts>()...};
  }
};

}  // namespace

PYBIND11_MODULE(cpp_template_test_util, m) {
  auto cls_1 = BindSimpleTemplate<int>(m);
  m.attr("DefaultInst") = cls_1;
  auto cls_2 = BindSimpleTemplate<int, double>(m);

  BindTemplateWithDefault<double>(m);
  BindTemplateWithDefault<int>(m);

  m.def("simple_func", [](const SimpleTemplate<int>&) {});

  // Add dummy constructors to check __call__ pseudo-deduction.
  cls_1.def(py::init([](int) { return SimpleTemplate<int>(); }));
  cls_2.def(py::init([](double) { return SimpleTemplate<int, double>(); }));

  AddTemplateFunction(m, "SimpleFunction",  // BR
      &SimpleFunction<int>, GetPyParam<int>());
  AddTemplateFunction(m, "SimpleFunction",  // BR
      &SimpleFunction<int, double>, GetPyParam<int, double>());

  AddTemplateFunction(
      m, "Callee", py::overload_cast<int>(&Callee), GetPyParam<int>());
  AddTemplateFunction(
      m, "Callee", py::overload_cast<double>(&Callee), GetPyParam<double>());

  py::class_<SimpleType> py_class(m, "SimpleType");
  py_class  // BR
      .def(py::init<>());
  AddTemplateMethod(py_class, "SimpleMethod", &SimpleType::SimpleMethod<int>,
      GetPyParam<int>());
  AddTemplateMethod(py_class, "SimpleMethod",
      &SimpleType::SimpleMethod<int, double>, GetPyParam<int, double>());
}

}  // namespace pydrake
}  // namespace drake
