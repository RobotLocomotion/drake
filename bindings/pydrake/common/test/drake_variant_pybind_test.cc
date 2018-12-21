#include "drake/bindings/pydrake/common/drake_variant_pybind.h"

#include <string>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>
#include "pybind11/embed.h"
#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/test/test_util_pybind.h"

using std::string;

namespace drake {
namespace pydrake {
namespace {

using test::SynchronizeGlobalsForPython3;

string VariantToString(const variant<int, double, string>& value) {
  const bool is_int = holds_alternative<int>(value);
  const bool is_double = holds_alternative<double>(value);
  const bool is_string = holds_alternative<string>(value);
  if (is_int) return fmt::format("int({})", get<int>(value));
  if (is_double) return fmt::format("double({})", get<double>(value));
  if (is_string) return fmt::format("string({})", get<string>(value));
  return "FAILED";
}

void ExpectString(const string& expr, const string& expected) {
  EXPECT_EQ(py::eval(expr).cast<string>(), expected) << expr;
}

GTEST_TEST(VariantTest, CheckCasting) {
  py::module m("__main__");

  m.def("VariantToString", &VariantToString, py::arg("value"));
  SynchronizeGlobalsForPython3(m);
  ExpectString("VariantToString(1)", "int(1)");
  ExpectString("VariantToString(0.5)", "double(0.5)");
  ExpectString("VariantToString('foo')", "string(foo)");
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
