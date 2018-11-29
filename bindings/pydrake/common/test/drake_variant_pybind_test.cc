#include "drake/bindings/pydrake/common/drake_variant_pybind.h"

#include <string>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>
#include "pybind11/embed.h"
#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace {

std::string VariantToString(const variant<int, double, std::string>& value) {
  auto int_ptr = get_if<int>(value);
  auto double_ptr = get_if<double>(value);
  auto string_ptr = get_if<std::string>(value);
  return
      int_ptr ? fmt::format("int({})", *int_ptr) :
      double_ptr ? fmt::format("double({})", *double_ptr) :
      string_ptr ? fmt::format("string({})", *string_ptr) :
      "FAILED";
}

void ExpectString(const std::string& expr, const std::string& expected) {
  EXPECT_EQ(py::eval(expr).cast<std::string>(), expected) << expr;
}

GTEST_TEST(VariantTest, CheckCasting) {
  py::module m("__main__");

  m.def("VariantToString", &VariantToString, py::arg("value"));
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
