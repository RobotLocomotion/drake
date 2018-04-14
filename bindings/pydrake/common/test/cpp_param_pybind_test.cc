#include "drake/bindings/pydrake/common/cpp_param_pybind.h"

// @file
// Tests the public interfaces in `cpp_param.py` and `cpp_param_pybind.h`.

#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include "pybind11/embed.h"
#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pybind11_ext/numpy_dtypes_user.h"
#include "drake/bindings/pydrake/test/test_util_pybind.h"

using std::string;

namespace drake {
namespace pydrake {

struct CustomDType {
  int value{};
};

}  // namespace pydrake
}  // namespace drake

PYBIND11_NUMPY_DTYPE_USER(drake::pydrake::CustomDType);

namespace drake {
namespace pydrake {

// Compare two Python objects directly.
bool PyEquals(py::object lhs, py::object rhs) {
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
  ASSERT_TRUE(CheckPyParam<py::object>("object,"));
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
  // Test numpy dtype.
  ASSERT_TRUE(CheckPyParam<CustomDType>("CustomDType,"));
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

int main(int argc, char** argv) {
  // Reconstructing `scoped_interpreter` multiple times (e.g. via `SetUp()`)
  // while *also* importing `numpy` wreaks havoc.
  py::scoped_interpreter guard;

  // Define common scope, import numpy for use in `eval`.
  py::module m("__main__");
  py::globals()["np"] = py::module::import("numpy");

  // Define custom class only once here.
  py::class_<CustomCppType>(m, "CustomCppType");

  test::SynchronizeGlobalsForPython3(m);

  // Define custom dtype.
  py::dtype_user<CustomDType>(m, "CustomDType")
    .def("value", [](const CustomDType* self) {
      return self->value;
    })
    .def("__str__", [](const CustomDType*) {
      return py::str("<CustomDType>");
    })
    .def("__repr__", [](const CustomDType*) {
      return py::str("<CustomDType>");
    });

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace pydrake
}  // namespace drake

int main(int argc, char** argv) {
  return drake::pydrake::main(argc, argv);
}
