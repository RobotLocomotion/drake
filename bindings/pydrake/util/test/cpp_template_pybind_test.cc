#include "drake/bindings/pydrake/util/cpp_template_pybind.h"

// @file
// Tests the public interfaces in `cpp_template.py` and
// `cpp_template_pybind.h`.

#include <string>
#include <vector>

#include <gtest/gtest.h>
#include "pybind11/embed.h"
#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/common/nice_type_name.h"
#include "drake/common/test_utilities/expect_throws_message.h"

using std::string;
using std::vector;

namespace drake {
namespace pydrake {
namespace {

template <typename ... Ts>
struct SimpleTemplate {
  vector<string> GetNames() {
    return {NiceTypeName::Get<Ts>()...};
  }
};

template <typename ... Ts>
py::object BindSimpleTemplate(py::module m) {
  using Class = SimpleTemplate<Ts...>;
  py::class_<Class> py_class(m, TemporaryClassName<Class>().c_str());
  py_class
      .def(py::init<>())
      .def("GetNames", &Class::GetNames);
  AddTemplateClass(
      m, "SimpleTemplate", py_class, GetPyParam<Ts...>());
  return py_class;
}

template <typename T>
void CheckValue(const string& expr, const T& expected) {
  EXPECT_EQ(py::eval(expr).cast<T>(), expected);
}

GTEST_TEST(CppTemplateTest, TemplateClass) {
  py::module m("__main__");

  m.attr("DefaultInst") = BindSimpleTemplate<int>(m);
  BindSimpleTemplate<int, double>(m);

  const vector<string> expected_1 = {"int"};
  const vector<string> expected_2 = {"int", "double"};
  CheckValue("DefaultInst().GetNames()", expected_1);
  CheckValue("SimpleTemplate[int]().GetNames()", expected_1);
  CheckValue("SimpleTemplate[int, float]().GetNames()", expected_2);

  m.def("simple_func", [](const SimpleTemplate<int>&) {});

  // Check error message if a function is called with the incorrect arguments.
  // N.B. We use `[\s\S]` because C++ regex does not have an equivalent of
  // Python re's DOTALL flag.
  DRAKE_EXPECT_THROWS_MESSAGE(
      py::eval("simple_func('incorrect value')"), std::runtime_error,
      R"([\s\S]*incompatible function arguments[\s\S]*\(arg0: __main__\.SimpleTemplate\[int\]\)[\s\S]*)");  // NOLINT
}

template <typename ... Ts>
vector<string> SimpleFunction() {
  return {NiceTypeName::Get<Ts>()...};
}

GTEST_TEST(CppTemplateTest, TemplateFunction) {
  py::module m("__main__");

  AddTemplateFunction(
      m, "SimpleFunction", &SimpleFunction<int>, GetPyParam<int>());
  AddTemplateFunction(
      m, "SimpleFunction", &SimpleFunction<int, double>,
      GetPyParam<int, double>());

  const vector<string> expected_1 = {"int"};
  const vector<string> expected_2 = {"int", "double"};
  CheckValue("SimpleFunction[int]()", expected_1);
  CheckValue("SimpleFunction[int, float]()", expected_2);
}

struct SimpleType {
  template <typename ... Ts>
  vector<string> SimpleMethod() {
    return {NiceTypeName::Get<Ts>()...};
  }
};

GTEST_TEST(CppTemplateTest, TemplateMethod) {
  py::module m("__main__");

  py::class_<SimpleType> py_class(m, "SimpleType");
  py_class
      .def(py::init<>());
  AddTemplateMethod(
      py_class, "SimpleMethod", &SimpleType::SimpleMethod<int>,
      GetPyParam<int>());
  AddTemplateMethod(
      py_class, "SimpleMethod", &SimpleType::SimpleMethod<int, double>,
      GetPyParam<int, double>());

  const vector<string> expected_1 = {"int"};
  const vector<string> expected_2 = {"int", "double"};
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
