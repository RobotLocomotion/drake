#include "drake/bindings/pydrake/common/copyable_unique_ptr_pybind.h"

// @file
// Tests the behavior of `copyable_unique_ptr_pybind.h`.

#include "pybind11/embed.h"
#include "pybind11/eval.h"
#include <gtest/gtest.h>

#include "drake/bindings/pydrake/test/test_util_pybind.h"
#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace pydrake {
namespace {

using test::SynchronizeGlobalsForPython3;

template <typename T>
void CheckValue(const std::string& expr, const T& expected) {
  SCOPED_TRACE("Python expression:\n  " + expr);
  EXPECT_EQ(py::eval(expr).cast<T>(), expected);
}

class TestClass {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TestClass)

  TestClass(int value) : value_(value) { }
  int value() const { return value_; }

 private:
  int value_{};
};

class TestContainer {
 public:
  TestContainer(int value)
      : item_(std::make_unique<TestClass>(value)) {}

  copyable_unique_ptr<TestClass>& item() {
    return item_;
  }

 private:
  copyable_unique_ptr<TestClass> item_;
};

GTEST_TEST(TypeSafeIndexTest, CheckCasting) {
  py::module m =
      py::module::create_extension_module("__main__", "", new PyModuleDef());

  py::class_<TestClass>(m, "TestClass")
      .def("value", &TestClass::value);

  py::class_<TestContainer>(m, "TestContainer")
      .def(py::init<int>(), py::arg("value"))
      .def("item", &TestContainer::item, py_rvp::reference_internal);

  SynchronizeGlobalsForPython3(m);

  CheckValue("x = TestContainer(10); x.item().value()", 10);
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
