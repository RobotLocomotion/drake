#include "drake/bindings/pydrake/common/cpp_param_pybind.h"

// @file
// Tests the public interfaces in `cpp_param.py` and `cpp_param_pybind.h`.

#include <stdexcept>
#include <string>

#include "pybind11/embed.h"
#include "pybind11/eval.h"
#include "pybind11/pybind11.h"
#include <gtest/gtest.h>

#include "drake/bindings/pydrake/test/test_util_pybind.h"

using std::string;

namespace drake {
namespace pydrake {

// Compare two Python objects directly.
bool PyEquals(py::object lhs, py::object rhs) {
  // TODO(eric.cousineau): Consider using `py::eval` as calling __eq__ may not
  // be robust. Types from `typing` may raise a NotImplemented error when
  // attempting to compare.
  return lhs.attr("__eq__")(rhs).cast<bool>();
}

// Ensures that the type `T` maps to the expression in `py_expr_expected`.
template <typename... Ts>
bool CheckPyParam(const string& py_expr_expected, type_pack<Ts...> param = {}) {
  py::object actual = GetPyParam(param);
  py::object expected = py::eval(py_expr_expected.c_str());
  return PyEquals(actual, expected);
}

GTEST_TEST(CppParamTest, PrimitiveTypes) {
  // Tests primitive types that are not expose directly via `pybind11`, thus
  // needing custom registration.
  // This follows the ordering in `cpp_param_pybind.cc`,
  // `RegisterCommon`.
  ASSERT_TRUE(CheckPyParam<bool>("bool,"));
  ASSERT_TRUE(CheckPyParam<std::string>("str,"));
  ASSERT_TRUE(CheckPyParam<double>("float,"));
  ASSERT_TRUE(CheckPyParam<float>("np.float32,"));
  ASSERT_TRUE(CheckPyParam<int>("int,"));
  ASSERT_TRUE(CheckPyParam<uint32_t>("np.uint32,"));
  ASSERT_TRUE(CheckPyParam<int64_t>("np.int64,"));
  // N.B. CheckPyParam<py::object>(...) should cause a compile-time failure.
  ASSERT_TRUE(CheckPyParam<Object>("object,"));
}

// Dummy type.
// - Registered.
struct CustomCppType {};
// - Unregistered.
struct CustomCppTypeUnregistered {};

GTEST_TEST(CppParamTest, CustomTypes) {
  // Tests types that are C++ types registered with `pybind11`.
  ASSERT_TRUE(CheckPyParam<CustomCppType>("CustomCppType,"));
  EXPECT_THROW(
      CheckPyParam<CustomCppTypeUnregistered>("CustomCppTypeUnregistered"),
      std::runtime_error);
}

template <typename T, T Value>
using constant = std::integral_constant<T, Value>;

GTEST_TEST(CppParamTest, LiteralTypes) {
  // Tests that literal types are mapped to literals in Python.
  ASSERT_TRUE(CheckPyParam<std::true_type>("True,"));
  ASSERT_TRUE((CheckPyParam<constant<int, -1>>("-1,")));
  ASSERT_TRUE((CheckPyParam<constant<uint, 1>>("1,")));
}

GTEST_TEST(CppParamTest, Packs) {
  // Tests that type packs are properly interpreted.
  ASSERT_TRUE((CheckPyParam<int, bool>("int, bool")));
  ASSERT_TRUE((CheckPyParam<bool, constant<bool, false>>("bool, False")));
}

GTEST_TEST(CppParamTest, Typing) {
  ASSERT_TRUE(CheckPyParam<std::vector<int>>("List[int],"));
  ASSERT_TRUE(CheckPyParam<std::vector<std::vector<int>>>("List[List[int]],"));
  ASSERT_TRUE(CheckPyParam<std::vector<CustomCppType>>("List[CustomCppType],"));
}

int main(int argc, char** argv) {
  // Reconstructing `scoped_interpreter` multiple times (e.g. via `SetUp()`)
  // while *also* importing `numpy` wreaks havoc.
  py::scoped_interpreter guard;

  // Define common scope, import numpy and List for use in `eval`.
  py::module m =
      py::module::create_extension_module("__main__", "", new PyModuleDef());
  py::globals()["np"] = py::module::import("numpy");
  py::globals()["List"] =
      py::module::import("pydrake.common.cpp_param").attr("List");

  // Define custom class only once here.
  py::class_<CustomCppType>(m, "CustomCppType");

  test::SynchronizeGlobalsForPython3(m);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace pydrake
}  // namespace drake

int main(int argc, char** argv) {
  return drake::pydrake::main(argc, argv);
}
