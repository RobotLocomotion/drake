/// @file
/// Test binding helper methods in `pydrake_pybind_test`.
#include "drake/bindings/pydrake/pydrake_pybind.h"

#include <string>

#include <gtest/gtest.h>
#include "pybind11/embed.h"
#include "pybind11/eval.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/test/test_util_pybind.h"

namespace drake {
namespace pydrake {
namespace {

void PyExpectTrue(py::module m, const char* expr) {
  const bool value =
      py::eval(expr, py::globals(), m.attr("__dict__")).cast<bool>();
  EXPECT_TRUE(value) << expr;
}

// TODO(eric.cousineau): Test coverage of `py_reference`,
// `py_reference_internal`, `py_keep_alive`, etc.

struct TestDefCopyAndDeepCopy {
  TestDefCopyAndDeepCopy(const TestDefCopyAndDeepCopy&) = default;
  int value{};
  bool operator==(const TestDefCopyAndDeepCopy& other) const {
    return value == other.value;
  }
};

GTEST_TEST(PydrakePybindTest, DefCopyAndDeepCopy) {
  py::module m("test");
  {
    using Class = TestDefCopyAndDeepCopy;
    py::class_<Class> cls(m, "TestDefCopyAndDeepCopy");
    cls  // BR
        .def(py::init([](int value) { return Class{value}; }))
        .def(py::self == py::self);
    DefCopyAndDeepCopy(&cls);
  }

  PyExpectTrue(m, "check_copy(copy.copy, TestDefCopyAndDeepCopy(10))");
  PyExpectTrue(m, "check_copy(copy.deepcopy, TestDefCopyAndDeepCopy(20))");
}

class TestDefClone {
 public:
  explicit TestDefClone(int value) : value_(value) {}
  TestDefClone(TestDefClone&&) = delete;

  std::unique_ptr<TestDefClone> Clone() const {
    return std::unique_ptr<TestDefClone>(new TestDefClone(*this));
  }

  bool operator==(const TestDefClone& other) const {
    return value_ == other.value_;
  }

 private:
  TestDefClone(const TestDefClone&) = default;
  int value_{};
};

GTEST_TEST(PydrakePybindTest, DefClone) {
  py::module m("test");
  {
    using Class = TestDefClone;
    py::class_<Class> cls(m, "TestDefClone");
    cls  // BR
        .def(py::init<double>())
        .def(py::self == py::self);
    DefClone(&cls);
  }

  PyExpectTrue(m, "check_copy(TestDefClone.Clone, TestDefClone(5))");
  PyExpectTrue(m, "check_copy(copy.copy, TestDefClone(10))");
  PyExpectTrue(m, "check_copy(copy.deepcopy, TestDefClone(20))");
}

struct TestParamInit {
  int a{0};
  int b{1};
};

GTEST_TEST(PydrakePybindTest, ParamInit) {
  py::module m("test");
  {
    using Class = TestParamInit;
    py::class_<Class>(m, "TestParamInit")
        .def(ParamInit<Class>())
        .def_readwrite("a", &Class::a)
        .def_readwrite("b", &Class::b)
        .def("as_tuple",
            [](const Class& self) { return py::make_tuple(self.a, self.b); });
  }

  PyExpectTrue(m, "TestParamInit().as_tuple() == (0, 1)");
  PyExpectTrue(m, "TestParamInit(a=10).as_tuple() == (10, 1)");
  PyExpectTrue(m, "TestParamInit(b=20).as_tuple() == (0, 20)");
  PyExpectTrue(m, "TestParamInit(a=10, b=20).as_tuple() == (10, 20)");
}

int DoMain(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  // Reconstructing `scoped_interpreter` multiple times (e.g. via `SetUp()`)
  // while *also* importing `numpy` wreaks havoc.
  py::scoped_interpreter guard;
  // Define nominal scope, and use a useful name for `ExecuteExtraPythonCode`
  // below.
  py::module m("pydrake.test.pydrake_pybind_test");
  // Test coverage and use this method.
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
