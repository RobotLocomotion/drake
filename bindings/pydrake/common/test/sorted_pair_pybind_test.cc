#include "drake/bindings/pydrake/common/sorted_pair_pybind.h"

// @file
// Tests the public interfaces in `sorted_pair_pybind.h`.

#include <string>

#include "pybind11/embed.h"
#include "pybind11/eval.h"
#include "pybind11/stl.h"
#include <gtest/gtest.h>

#include "drake/bindings/pydrake/test/test_util_pybind.h"

namespace drake {
namespace pydrake {

SortedPair<std::string> PassThrough(const SortedPair<std::string>& in) {
  return in;
}

using ExampleMap = std::map<SortedPair<std::string>, int>;
ExampleMap PassThroughMap(const ExampleMap& in) {
  return in;
}

// Expects that a given Python expression `expr` evaluates to true, using
// globals and the variables available in `m`.
void PyExpectTrue(py::module m, const char* expr) {
  py::object locals = m.attr("__dict__");
  const bool value = py::eval(expr, py::globals(), locals).cast<bool>();
  EXPECT_TRUE(value) << expr;
}

GTEST_TEST(CppParamTest, PrimitiveTypes) {
  py::module m =
      py::module::create_extension_module("test", "", new PyModuleDef());
  m.def("PassThrough", &PassThrough);
  m.def("PassThroughMap", &PassThroughMap);
  test::SynchronizeGlobalsForPython3(m);

  // Check single value. Pass in an unsorted pair, and confirm that we get a
  // sorted pair out.
  const std::pair<std::string, std::string> example{"a", "b"};
  PyExpectTrue(m, "PassThrough(('b', 'a')) == ('a', 'b')");
  // Check compsite type.
  PyExpectTrue(m, "PassThroughMap({('b', 'a'): 10}) == {('a', 'b'): 10}");
}

int main(int argc, char** argv) {
  // Reconstructing `scoped_interpreter` multiple times (e.g. via `SetUp()`)
  // while *also* importing `numpy` wreaks havoc.
  py::scoped_interpreter guard;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace pydrake
}  // namespace drake

int main(int argc, char** argv) {
  return drake::pydrake::main(argc, argv);
}
