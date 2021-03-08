#include "drake/bindings/pydrake/common/type_safe_index_pybind.h"

// @file
// Tests the behavior of `type_safe_index_pybind.h`.

#include <string>
#include <vector>

#include "pybind11/embed.h"
#include "pybind11/eval.h"
#include "pybind11/pybind11.h"
#include <gtest/gtest.h>

#include "drake/bindings/pydrake/test/test_util_pybind.h"

using std::string;
using std::vector;

namespace drake {
namespace pydrake {
namespace {

using test::SynchronizeGlobalsForPython3;

template <typename T>
void CheckValue(const string& expr, const T& expected) {
  SCOPED_TRACE("Python expression:\n  " + expr);
  EXPECT_EQ(py::eval(expr).cast<T>(), expected);
}

GTEST_TEST(TypeSafeIndexTest, CheckCasting) {
  py::module m =
      py::module::create_extension_module("__main__", "", new PyModuleDef());

  struct Tag {};
  using Index = TypeSafeIndex<Tag>;
  BindTypeSafeIndex<Index>(m, "Index");

  m.def("pass_thru_int", [](int x) {
    EXPECT_EQ(x, 10);
    return x;
  });
  SynchronizeGlobalsForPython3(m);
  CheckValue("pass_thru_int(10)", 10);
  CheckValue("pass_thru_int(Index(10))", 10);
  // TypeSafeIndex<> is not implicitly constructible from an int.
  py::object py_int = py::eval("10");
  ASSERT_THROW(py_int.cast<Index>(), std::runtime_error);

  m.def("pass_thru_index", [](Index x) {
    EXPECT_EQ(x, 10);
    return x;
  });

  SynchronizeGlobalsForPython3(m);

  // TypeSafeIndex<> is not implicitly constructible from an int.
  // TODO(eric.cousineau): Consider relaxing this to *only* accept `int`s, and
  // puke if another `TypeSafeIndex<U>` is encountered.
  ASSERT_THROW(py::eval("pass_thru_index(10)"), std::runtime_error);
  CheckValue("pass_thru_index(Index(10))", 10);
  CheckValue("pass_thru_index(Index(10))", Index{10});

  struct OtherTag {};
  using OtherIndex = TypeSafeIndex<OtherTag>;
  BindTypeSafeIndex<OtherIndex>(m, "OtherIndex");

  ASSERT_THROW(py::eval("pass_thru_index(OtherIndex(10))"), std::runtime_error);
  py::object py_index = py::eval("Index(10)");
  ASSERT_THROW(py_index.cast<OtherIndex>(), std::runtime_error);

  // Check basic comparison.
  CheckValue("Index(10) == Index(10)", true);
  CheckValue("Index(10) == 10", true);
  CheckValue("10 == Index(10)", true);
  CheckValue("Index(9) < Index(10)", true);
  CheckValue("Index(11) < Index(10)", false);

  // Store values for hash computation so that their id()s cannot be recycled
  // by the GC; otherwise, if the correct `__hash__` implementation were
  // missing and it used the default implementation (using `id()`), we'd get a
  // false positive.
  py::exec("a = Index(10); b = Index(10); c = Index(9)");
  CheckValue("hash(a) == hash(b)", true);
  CheckValue("hash(a) == hash(c)", false);

  // Check string representation.
  CheckValue("repr(Index(10)) == 'Index(10)'", true);
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
