#include "drake/bindings/pydrake/util/cpp_types_pybind.h"

// @file
// Tests the public interfaces in `cpp_types.py` and `cpp_types_pybind.h`.

#include <string>

#include <gtest/gtest.h>
#include <pybind11/embed.h>
#include <pybind11/eval.h>
#include <pybind11/pybind11.h>

using std::string;

namespace drake {
namespace pydrake {

// Ensures that the type `T` maps to the expression in `py_expr_expected`.
template <typename T>
bool CheckPyType(const string& py_expr_expected, type_pack<T> = {}) {
  py::object actual = GetPyType<T>();
  py::object expected = py::eval(py_expr_expected.c_str());
  return actual.is(expected);
}

GTEST_TEST(CppTypesTest, PrimitiveTypes) {
  // Tests primitive types that are not expose directly via `pybind11`, thus
  // needing custom registration.
  // This follows the ordering in `cpp_types_pybind.cc`,
  // `RegisterCommon`.
  ASSERT_TRUE(CheckPyType<bool>("bool"));
  ASSERT_TRUE(CheckPyType<std::string>("str"));
  ASSERT_TRUE(CheckPyType<double>("float"));
  ASSERT_TRUE(CheckPyType<float>("np.float32"));
  ASSERT_TRUE(CheckPyType<int>("int"));
  ASSERT_TRUE(CheckPyType<uint32_t>("np.uint32"));
  ASSERT_TRUE(CheckPyType<int64_t>("np.int64"));
  ASSERT_TRUE(CheckPyType<py::object>("object"));
}

// Dummy type.
struct CustomCppType {};

GTEST_TEST(CppTypesTest, CustomTypes) {
  // Tests types that are C++ types registered with `pybind11`.
  ASSERT_TRUE(CheckPyType<CustomCppType>("CustomCppType"));
}

template <typename T, T Value>
using constant = std::integral_constant<T, Value>;

GTEST_TEST(CppTypesTest, LiteralTypes) {
  // Tests that literal types are mapped to literals in Python.
  ASSERT_TRUE(CheckPyType<std::true_type>("True"));
  ASSERT_TRUE((CheckPyType<constant<int, -1>>("-1")));
  ASSERT_TRUE((CheckPyType<constant<uint, 1>>("1")));
}

// Compare two Python objects directly.
bool PyEquals(py::object lhs, py::object rhs) {
  return lhs.attr("__eq__")(rhs).cast<bool>();
}

GTEST_TEST(CppTypesTest, Packs) {
  // Tests that type packs are properly interpreted.
  ASSERT_TRUE(PyEquals(GetPyTypes<int, bool>(), py::eval("int, bool")));
}

int main(int argc, char** argv) {
  // Reconstructing `scoped_interpreter` multiple times (e.g. via `SetUp()`)
  // while *also* importing `numpy` wreaks havoc.
  py::scoped_interpreter guard;

  // Define common scope, import numpy for use in `eval`.
  py::module m("__main__");
  py::globals()["np"] = py::module::import("numpy");

  // Define custom class only once here.
  py::class_<CustomCppType>(m, "CustomCppType");

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace pydrake
}  // namespace drake

int main(int argc, char** argv) {
  return drake::pydrake::main(argc, argv);
}
