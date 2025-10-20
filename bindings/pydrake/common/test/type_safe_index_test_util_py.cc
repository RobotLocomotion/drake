#include "drake/bindings/pydrake/common/type_safe_index_pybind.h"

// @file
// Provides both bindings and direct tests for the behavior of
// `type_safe_index_pybind.h`. The main test code is in
// `type_safe_index_test.py`.

#include "pybind11/eval.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace pydrake {
namespace {

void AssertThrowsRuntimeError(std::function<void(void)> func) {
  try {
    func();
  } catch (const std::runtime_error&) {
    // Test passes.
  } catch (...) {
    DRAKE_THROW_UNLESS(false);
  }
}

}  // namespace

PYBIND11_MODULE(type_safe_index_test_util, m) {
  struct Tag {};
  using Index = TypeSafeIndex<Tag>;
  BindTypeSafeIndex<Index>(m, "Index");

  m.def("pass_thru_int", [](int x) {
    DRAKE_ASSERT(x == 10);
    return x;
  });
  m.def("pass_thru_index", [](Index x) {
    DRAKE_ASSERT(x == 10);
    return x;
  });

  struct OtherTag {};
  using OtherIndex = TypeSafeIndex<OtherTag>;
  BindTypeSafeIndex<OtherIndex>(m, "OtherIndex");

  m.def("execute_tests", []() {
    // TypeSafeIndex<> is not implicitly constructible from an int.
    py::object py_int = py::eval("10");
    AssertThrowsRuntimeError([&py_int]() { py_int.cast<Index>(); });
    py::object py_index = py::eval("Index(10)");
    AssertThrowsRuntimeError([&py_index]() { py_index.cast<OtherIndex>(); });
  });
}

}  // namespace pydrake
}  // namespace drake
