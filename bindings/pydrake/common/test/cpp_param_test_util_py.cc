#include "drake/bindings/pydrake/common/cpp_param_pybind.h"

// @file This file contains direct C++ tests for the public interfaces in
// `cpp_param.py` and `cpp_param_pybind.h`. A python interpreter is required;
// it is provided by having these tests be invoked from cpp_param_test.py.

#include <string>

#include "pybind11/eval.h"

#include "drake/common/drake_assert.h"

using std::string;

namespace drake {
namespace pydrake {
namespace {

// Compare two Python objects directly.
bool PyEquals(py::object lhs, py::object rhs) {
  // TODO(eric.cousineau): Consider using `py::eval` as calling __eq__ may not
  // be robust. Types from `typing` may raise a NotImplemented error when
  // attempting to compare.
  return py::cast<bool>(lhs.attr("__eq__")(rhs));
}

// Ensures that the type `T` maps to the expression in `py_expr_expected`.
template <typename... Ts>
bool CheckPyParam(const string& py_expr_expected, type_pack<Ts...> param = {}) {
  py::object actual = GetPyParam(param);
  py::object expected =
      py::eval(py::str(py_expr_expected.c_str()), py::globals());
  return PyEquals(actual, expected);
}

void CheckPrimitiveTypes() {
  // Tests primitive types that are not exposed directly via `pybind11`, thus
  // needing custom registration.
  // This follows the ordering in `cpp_param_pybind.cc`,
  // `RegisterCommon`.
  DRAKE_THROW_UNLESS(CheckPyParam<bool>("bool,"));
  DRAKE_THROW_UNLESS(CheckPyParam<std::string>("str,"));
  DRAKE_THROW_UNLESS(CheckPyParam<double>("float,"));
  DRAKE_THROW_UNLESS(CheckPyParam<float>("np.float32,"));
  DRAKE_THROW_UNLESS(CheckPyParam<int>("int,"));
  DRAKE_THROW_UNLESS(CheckPyParam<int16_t>("np.int16,"));
  DRAKE_THROW_UNLESS(CheckPyParam<int64_t>("np.int64,"));
  DRAKE_THROW_UNLESS(CheckPyParam<uint16_t>("np.uint16,"));
  DRAKE_THROW_UNLESS(CheckPyParam<uint32_t>("np.uint32,"));
  DRAKE_THROW_UNLESS(CheckPyParam<uint64_t>("np.uint64,"));
  // N.B. CheckPyParam<py::object>(...) should cause a compile-time failure.
  DRAKE_THROW_UNLESS(CheckPyParam<Object>("object,"));
}

// Dummy type.
// - Registered.
struct CustomCppType {};
// - Unregistered.
struct CustomCppTypeUnregistered {};

void CheckCustomTypes() {
  // Tests types that are C++ types registered with `pybind11`.
  DRAKE_THROW_UNLESS(CheckPyParam<CustomCppType>("CustomCppType,"));
  try {
    CheckPyParam<CustomCppTypeUnregistered>("CustomCppTypeUnregistered");
  } catch (const std::runtime_error&) {
    // Test passes.
    return;
  }
  throw std::logic_error(
      fmt::format("{}:{}: request for unregistered type failed to raise an "
                  "exception",
          __FILE__, __LINE__));
}

template <typename T, T Value>
using constant = std::integral_constant<T, Value>;

void CheckLiteralTypes() {
  // Tests that literal types are mapped to literals in Python.
  DRAKE_THROW_UNLESS(CheckPyParam<std::true_type>("True,"));
  DRAKE_THROW_UNLESS((CheckPyParam<constant<int, -1>>("-1,")));
  DRAKE_THROW_UNLESS((CheckPyParam<constant<uint, 1>>("1,")));
}

void CheckPacks() {
  // Tests that type packs are properly interpreted.
  DRAKE_THROW_UNLESS((CheckPyParam<int, bool>("int, bool")));
  DRAKE_THROW_UNLESS(
      (CheckPyParam<bool, constant<bool, false>>("bool, False")));
}

void CheckTyping() {
  DRAKE_THROW_UNLESS(CheckPyParam<std::vector<int>>("List[int],"));
  DRAKE_THROW_UNLESS(
      CheckPyParam<std::vector<std::vector<int>>>("List[List[int]],"));
  DRAKE_THROW_UNLESS(
      CheckPyParam<std::vector<CustomCppType>>("List[CustomCppType],"));
}

}  // namespace

PYBIND11_MODULE(cpp_param_test_util, m) {
  // Define custom class only once here.
  py::class_<CustomCppType>(m, "CustomCppType");

  m.def("execute_tests", [m]() {
    // Import some definitions from the modules where they are defined into the
    // module where the tests will execute.
    py::exec("from pydrake.common.cpp_param_test_util import CustomCppType");
    py::exec("from pydrake.common.cpp_param import List");

    CheckPrimitiveTypes();
    CheckCustomTypes();
    CheckLiteralTypes();
    CheckPacks();
    CheckTyping();
  });
}

}  // namespace pydrake
}  // namespace drake
