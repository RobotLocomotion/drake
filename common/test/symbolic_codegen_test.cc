#include <iostream>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
namespace {

using std::string;

// Helper function to combine the function name @p function_name and an
// expression @p e with the proper function header and footer.
string MakeFunctionCode(const string& function_name, int n, const string& e) {
  // Note that fmtlib requires to escape "{'" and "}" using "{{" and "}}".
  return fmt::format(
      R"""(double {}(const double* p) {{
    return {};
}}
int {}_in() {{
    return {};
}}
)""",
      function_name, e, function_name, n);
}

class SymbolicCodeGenTest : public ::testing::Test {
 protected:
  const Variable x_{"x"};
  const Variable y_{"y"};
  const Variable z_{"z"};
};

TEST_F(SymbolicCodeGenTest, Variable) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, x_), MakeFunctionCode("f", 2, "p[0]"));
  EXPECT_EQ(CodeGen("f", {x_, y_, z_}, z_), MakeFunctionCode("f", 3, "p[2]"));
}

TEST_F(SymbolicCodeGenTest, Constant) {
  EXPECT_EQ(CodeGen("f", {}, 3.141592), MakeFunctionCode("f", 0, "3.141592"));
}

TEST_F(SymbolicCodeGenTest, Addition) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, 2.0 + 3.0 * x_ - 7.0 * y_),
            MakeFunctionCode("f", 2, "(2 + (3 * p[0]) + (-7 * p[1]))"));
}

TEST_F(SymbolicCodeGenTest, Multiplication) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, 2.0 * 3.0 * x_ * x_ * -7.0 * y_ * y_ * y_),
            MakeFunctionCode(
                "f", 2, "(-42 * pow(p[0], 2.000000) * pow(p[1], 3.000000))"));
}

TEST_F(SymbolicCodeGenTest, Pow) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, pow(2 + x_, 3 * y_)),
            MakeFunctionCode("f", 2, "pow((2 + p[0]), (3 * p[1]))"));
}

TEST_F(SymbolicCodeGenTest, Division) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, (2 + x_) / (3 * y_)),
            MakeFunctionCode("f", 2, "((2 + p[0]) / (3 * p[1]))"));
}

TEST_F(SymbolicCodeGenTest, Abs) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, abs(2 + x_)),
            MakeFunctionCode("f", 2, "fabs((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Log) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, log(2 + x_)),
            MakeFunctionCode("f", 2, "log((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Exp) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, exp(2 + x_)),
            MakeFunctionCode("f", 2, "exp((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Sqrt) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, sqrt(2 + x_)),
            MakeFunctionCode("f", 2, "sqrt((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Sin) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, sin(2 + x_)),
            MakeFunctionCode("f", 2, "sin((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Cos) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, cos(2 + x_)),
            MakeFunctionCode("f", 2, "cos((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Tan) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, tan(2 + x_)),
            MakeFunctionCode("f", 2, "tan((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Asin) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, asin(2 + x_)),
            MakeFunctionCode("f", 2, "asin((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Acos) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, acos(2 + x_)),
            MakeFunctionCode("f", 2, "acos((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Atan) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, atan(2 + x_)),
            MakeFunctionCode("f", 2, "atan((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Atan2) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, atan2(2 + x_, 3 + y_)),
            MakeFunctionCode("f", 2, "atan2((2 + p[0]), (3 + p[1]))"));
}

TEST_F(SymbolicCodeGenTest, Sinh) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, sinh(2 + x_)),
            MakeFunctionCode("f", 2, "sinh((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Cosh) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, cosh(2 + x_)),
            MakeFunctionCode("f", 2, "cosh((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Tanh) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, tanh(2 + x_)),
            MakeFunctionCode("f", 2, "tanh((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Min) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, min(2 + x_, 3 + y_)),
            MakeFunctionCode("f", 2, "fmin((2 + p[0]), (3 + p[1]))"));
}

TEST_F(SymbolicCodeGenTest, Max) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, max(2 + x_, 3 + y_)),
            MakeFunctionCode("f", 2, "fmax((2 + p[0]), (3 + p[1]))"));
}

TEST_F(SymbolicCodeGenTest, Ceil) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, ceil(2 + x_)),
            MakeFunctionCode("f", 2, "ceil((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Floor) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, floor(2 + x_)),
            MakeFunctionCode("f", 2, "floor((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, IfThenElse) {
  const Expression e{if_then_else(x_ > y_, x_, y_)};
  EXPECT_THROW(CodeGen("f", {x_, y_}, e), std::runtime_error);
}

TEST_F(SymbolicCodeGenTest, UninterpretedFunction) {
  const Expression e{uninterpreted_function("uf", {x_, y_})};
  EXPECT_THROW(CodeGen("f", {x_, y_}, e), std::runtime_error);
}

TEST_F(SymbolicCodeGenTest, ExampleInDocumentation) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, 1 + sin(x_) + cos(y_)),
            MakeFunctionCode("f", 2, "(1 + sin(p[0]) + cos(p[1]))"));
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
