/// @file
/// Test binding helper methods in `pydrake_pybind_test`.
/// @note `check_copy` is defind and documented in
/// `_pydrake_pybind_test_extra.py`.
#include "drake/bindings/pydrake/pydrake_pybind.h"

#include <string>

#include "pybind11/embed.h"
#include "pybind11/eval.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include <gtest/gtest.h>

#include "drake/bindings/pydrake/test/test_util_pybind.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace pydrake {
namespace {

GTEST_TEST(PydrakePybindTest, PyReturnValuePolicy) {
  static_assert(
      std::is_same_v<py_rvp, py::return_value_policy>, "Alias is wrong?");
}

// Expects that a given Python expression `expr` evaluates to true, using
// globals and the variables available in `m`.
void PyExpectTrue(py::module m, const char* expr) {
  const bool value =
      py::eval(expr, py::globals(), m.attr("__dict__")).cast<bool>();
  EXPECT_TRUE(value) << expr;
}

class Nonce {};

class ExamplePyKeepAlive {
 public:
  const Nonce* a() const { return &a_; }
  std::vector<const Nonce*> a_list() const { return {&a_}; }

 private:
  Nonce a_{};
};

GTEST_TEST(PydrakePybindTest, PyKeepAlive) {
  py::module m =
      py::module::create_extension_module("test", "", new PyModuleDef());
  {
    using Class = Nonce;
    py::class_<Class>(m, "Nonce");
  }
  {
    using Class = ExamplePyKeepAlive;
    py::class_<Class>(m, "ExamplePyKeepAlive")
        .def(py::init())
        .def("a",
            [](const Class& self) {
              return py_keep_alive(py::cast(self.a()), py::cast(&self));
            })
        .def("a_list", [](const Class& self) {
          return py_keep_alive_iterable<py::list>(
              py::cast(self.a_list()), py::cast(&self));
        });
  }

  PyExpectTrue(m, "isinstance(ExamplePyKeepAlive().a(), Nonce)");
  PyExpectTrue(m,
      "isinstance(ExamplePyKeepAlive().a_list(), list) and "
      "len(ExamplePyKeepAlive().a_list()) == 1 and "
      "isinstance(ExamplePyKeepAlive().a_list()[0], Nonce)");
}

// Class which has a copy constructor, for testing `DefCopyAndDeepCopy`.
struct ExampleDefCopyAndDeepCopy {
  explicit ExampleDefCopyAndDeepCopy(int v) : value(v) {}
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ExampleDefCopyAndDeepCopy);
  int value{};
  bool operator==(const ExampleDefCopyAndDeepCopy& other) const {
    return value == other.value;
  }
};

GTEST_TEST(PydrakePybindTest, DefCopyAndDeepCopy) {
  py::module m =
      py::module::create_extension_module("test", "", new PyModuleDef());
  {
    using Class = ExampleDefCopyAndDeepCopy;
    py::class_<Class> cls(m, "ExampleDefCopyAndDeepCopy");
    cls  // BR
        .def(py::init([](int value) { return Class(value); }))
        .def(py::self == py::self);
    DefCopyAndDeepCopy(&cls);
  }

  PyExpectTrue(m, "check_copy(copy.copy, ExampleDefCopyAndDeepCopy(10))");
  PyExpectTrue(m, "check_copy(copy.deepcopy, ExampleDefCopyAndDeepCopy(20))");
}

// Class which has a `Clone()` method and whose copy constructor is explicitly
// disabled, for testing `DefClone`.
class ExampleDefClone {
 public:
  explicit ExampleDefClone(int value) : value_(value) {}
  ExampleDefClone(ExampleDefClone&&) = delete;
  ExampleDefClone& operator=(ExampleDefClone&) = delete;

  std::unique_ptr<ExampleDefClone> Clone() const {
    return std::unique_ptr<ExampleDefClone>(new ExampleDefClone(*this));
  }

  bool operator==(const ExampleDefClone& other) const {
    return value_ == other.value_;
  }

 private:
  ExampleDefClone(const ExampleDefClone&) = default;
  ExampleDefClone& operator=(const ExampleDefClone&) = default;

  int value_{};
};

GTEST_TEST(PydrakePybindTest, DefClone) {
  py::module m =
      py::module::create_extension_module("test", "", new PyModuleDef());
  {
    using Class = ExampleDefClone;
    py::class_<Class> cls(m, "ExampleDefClone");
    cls  // BR
        .def(py::init<double>())
        .def(py::self == py::self);
    DefClone(&cls);
  }

  PyExpectTrue(m, "check_copy(ExampleDefClone.Clone, ExampleDefClone(5))");
  PyExpectTrue(m, "check_copy(copy.copy, ExampleDefClone(10))");
  PyExpectTrue(m, "check_copy(copy.deepcopy, ExampleDefClone(20))");
}

// Struct which defines attributes which are to be exposed with
// `.def_readwrite`, for testing `ParamInit`.
struct ExampleParamInit {
  int a{0};
  int b{1};
};

GTEST_TEST(PydrakePybindTest, ParamInit) {
  py::module m =
      py::module::create_extension_module("test", "", new PyModuleDef());
  {
    using Class = ExampleParamInit;
    py::class_<Class>(m, "ExampleParamInit")
        .def(ParamInit<Class>())
        .def_readwrite("a", &Class::a)
        .def_readwrite("b", &Class::b)
        // This is purely a sugar method for testing the values.
        .def("compare_values", [](const Class& self, int a, int b) {
          return self.a == a && self.b == b;
        });
  }

  PyExpectTrue(m, "ExampleParamInit().compare_values(0, 1)");
  PyExpectTrue(m, "ExampleParamInit(a=10).compare_values(10, 1)");
  PyExpectTrue(m, "ExampleParamInit(b=20).compare_values(0, 20)");
  PyExpectTrue(m, "ExampleParamInit(a=10, b=20).compare_values(10, 20)");
}

int DoMain(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  // Reconstructing `scoped_interpreter` multiple times (e.g. via `SetUp()`)
  // while *also* importing `numpy` wreaks havoc.
  py::scoped_interpreter guard;
  // Define nominal scope, and use a useful name for `ExecuteExtraPythonCode`
  // below.
  py::module m = py::module::create_extension_module(
      "pydrake.test.pydrake_pybind_test", "", new PyModuleDef());
  // Test coverage and use this method for `check_copy`.
  ExecuteExtraPythonCode(m);
  test::SynchronizeGlobalsForPython3(m);
  return RUN_ALL_TESTS();
}

}  // namespace
}  // namespace pydrake
}  // namespace drake

int main(int argc, char** argv) {
  return drake::pydrake::DoMain(argc, argv);
}
