#include "drake/bindings/pydrake/common/cpp_template_pybind.h"

// @file
// Tests the public interfaces in `cpp_template.py` and
// `cpp_template_pybind.h`.

#include <string>
#include <utility>
#include <vector>

#include "pybind11/embed.h"
#include "pybind11/eval.h"
#include "pybind11/pybind11.h"
#include <gtest/gtest.h>

#include "drake/bindings/pydrake/test/test_util_pybind.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/test_utilities/expect_throws_message.h"

using std::string;
using std::vector;

namespace drake {
namespace pydrake {
namespace {

using test::SynchronizeGlobalsForPython3;

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

GTEST_TEST(CppTemplateTest, TemplateClass) {
  py::module m =
      py::module::create_extension_module("__main__", "", new PyModuleDef());

  auto cls_1 = BindSimpleTemplate<int>(m);
  m.attr("DefaultInst") = cls_1;
  auto cls_2 = BindSimpleTemplate<int, double>(m);

  const vector<string> expected_1 = {"int"};
  const vector<string> expected_2 = {"int", "double"};
  SynchronizeGlobalsForPython3(m);

  CheckValue("DefaultInst().GetNames()", expected_1);
  CheckValue("SimpleTemplate[int]().GetNames()", expected_1);
  CheckValue("SimpleTemplate[int, float]().GetNames()", expected_2);

  m.def("simple_func", [](const SimpleTemplate<int>&) {});
  SynchronizeGlobalsForPython3(m);

  // Check error message if a function is called with the incorrect arguments.
  // N.B. We use `[^\0]` because C++ regex does not have an equivalent of
  // Python re's DOTALL flag. `[\s\S]` *should* work, but Apple LLVM 10.0.0
  // does not work with it.
  DRAKE_EXPECT_THROWS_MESSAGE(py::eval("simple_func('incorrect value')"),
      std::runtime_error,
      R"([^\0]*incompatible function arguments[^\0]*\(arg0: __main__\.SimpleTemplate\[int\]\)[^\0]*)");  // NOLINT

  // Add dummy constructors to check __call__ pseudo-deduction.
  cls_1.def(py::init([](int) { return SimpleTemplate<int>(); }));
  cls_2.def(py::init([](double) { return SimpleTemplate<int, double>(); }));
  // int - infer first (cls_1).
  CheckValue("SimpleTemplate(0).GetNames()", expected_1);
  // double - infer second (cls_2).
  CheckValue("SimpleTemplate(0.).GetNames()", expected_2);
}

template <typename... Ts>
vector<string> SimpleFunction() {
  return {NiceTypeName::Get<Ts>()...};
}

GTEST_TEST(CppTemplateTest, TemplateFunction) {
  py::module m =
      py::module::create_extension_module("__main__", "", new PyModuleDef());

  AddTemplateFunction(m, "SimpleFunction",  // BR
      &SimpleFunction<int>, GetPyParam<int>());
  AddTemplateFunction(m, "SimpleFunction",  // BR
      &SimpleFunction<int, double>, GetPyParam<int, double>());

  const vector<string> expected_1 = {"int"};
  const vector<string> expected_2 = {"int", "double"};
  SynchronizeGlobalsForPython3(m);
  CheckValue("SimpleFunction[int]()", expected_1);
  CheckValue("SimpleFunction[int, float]()", expected_2);
}

std::string Callee(int) {
  return "int";
}
std::string Callee(double) {
  return "double";
}

GTEST_TEST(CppTemplateTest, Call) {
  py::module m =
      py::module::create_extension_module("__main__", "", new PyModuleDef());

  AddTemplateFunction(
      m, "Callee", py::overload_cast<int>(&Callee), GetPyParam<int>());
  AddTemplateFunction(
      m, "Callee", py::overload_cast<double>(&Callee), GetPyParam<double>());

  const std::string expected_1 = "int";
  const std::string expected_2 = "double";
  SynchronizeGlobalsForPython3(m);
  CheckValue("Callee(0)", expected_1);
  CheckValue("Callee(0.)", expected_2);
}

struct SimpleType {
  template <typename... Ts>
  vector<string> SimpleMethod() {
    return {NiceTypeName::Get<Ts>()...};
  }
};

GTEST_TEST(CppTemplateTest, TemplateMethod) {
  py::module m =
      py::module::create_extension_module("__main__", "", new PyModuleDef());

  py::class_<SimpleType> py_class(m, "SimpleType");
  py_class  // BR
      .def(py::init<>());
  AddTemplateMethod(py_class, "SimpleMethod", &SimpleType::SimpleMethod<int>,
      GetPyParam<int>());
  AddTemplateMethod(py_class, "SimpleMethod",
      &SimpleType::SimpleMethod<int, double>, GetPyParam<int, double>());

  const vector<string> expected_1 = {"int"};
  const vector<string> expected_2 = {"int", "double"};
  SynchronizeGlobalsForPython3(m);
  CheckValue("SimpleType().SimpleMethod[int]()", expected_1);
  CheckValue("SimpleType().SimpleMethod[int, float]()", expected_2);
}

int main(int argc, char** argv) {
  // Reconstructing `scoped_interpreter` multiple times (e.g. via `SetUp()`)
  // while *also* importing `numpy` wreaks havoc.
  py::scoped_interpreter guard;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace
}  // namespace pydrake
}  // namespace drake

int main(int argc, char** argv) {
  return drake::pydrake::main(argc, argv);
}
