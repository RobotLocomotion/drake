#include <iostream>
#include <sstream>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
namespace {

using std::ostringstream;
using std::string;
using std::vector;

// Helper function to combine the function name @p function_name and an
// expression @p e with the proper function header and footer.
string MakeScalarFunctionCode(const string& function_name, const int n,
                              const string& e) {
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

// Helper function to generate expected codegen outcome for a dense matrix @p
// M. It combines the function name @p function_name, the number of input
// parameters @p in, the number of rows in M, @p rows, the number of columns in
// M, @p cols, and the expected code for the entries in M, @p expressions.
string MakeDenseMatrixFunctionCode(const string& function_name, const int in,
                                   const int rows, const int cols,
                                   const vector<string>& expressions) {
  ostringstream oss;
  // Main function f.
  oss << fmt::format("void {}(const double* p, double* m) {{\n", function_name);
  for (size_t i{0}; i < expressions.size(); ++i) {
    oss << fmt::format("    m[{}] = {};\n", i, expressions[i]);
  }
  oss << "}\n";
  // f_in.
  oss << fmt::format("int {}_in() {{\n    return {};\n}}\n", function_name, in);
  // f_rows.
  oss << fmt::format("int {}_rows() {{\n    return {};\n}}\n", function_name,
                     rows);
  // f_cols.
  oss << fmt::format("int {}_cols() {{\n    return {};\n}}\n", function_name,
                     cols);
  return oss.str();
}

class SymbolicCodeGenTest : public ::testing::Test {
 protected:
  const Variable x_{"x"};
  const Variable y_{"y"};
  const Variable z_{"z"};
  const Variable w_{"w"};
};

TEST_F(SymbolicCodeGenTest, Variable) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, x_), MakeScalarFunctionCode("f", 2, "p[0]"));
  EXPECT_EQ(CodeGen("f", {x_, y_, z_}, z_),
            MakeScalarFunctionCode("f", 3, "p[2]"));
}

TEST_F(SymbolicCodeGenTest, Constant) {
  EXPECT_EQ(CodeGen("f", {}, 3.141592),
            MakeScalarFunctionCode("f", 0, "3.141592"));
}

TEST_F(SymbolicCodeGenTest, Addition) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, 2.0 + 3.0 * x_ - 7.0 * y_),
            MakeScalarFunctionCode("f", 2, "(2 + (3 * p[0]) + (-7 * p[1]))"));
}

TEST_F(SymbolicCodeGenTest, Multiplication) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, 2.0 * 3.0 * x_ * x_ * -7.0 * y_ * y_ * y_),
            MakeScalarFunctionCode(
                "f", 2, "(-42 * pow(p[0], 2.000000) * pow(p[1], 3.000000))"));
}

TEST_F(SymbolicCodeGenTest, Pow) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, pow(2 + x_, 3 * y_)),
            MakeScalarFunctionCode("f", 2, "pow((2 + p[0]), (3 * p[1]))"));
}

TEST_F(SymbolicCodeGenTest, Division) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, (2 + x_) / (3 * y_)),
            MakeScalarFunctionCode("f", 2, "((2 + p[0]) / (3 * p[1]))"));
}

TEST_F(SymbolicCodeGenTest, Abs) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, abs(2 + x_)),
            MakeScalarFunctionCode("f", 2, "fabs((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Log) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, log(2 + x_)),
            MakeScalarFunctionCode("f", 2, "log((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Exp) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, exp(2 + x_)),
            MakeScalarFunctionCode("f", 2, "exp((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Sqrt) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, sqrt(2 + x_)),
            MakeScalarFunctionCode("f", 2, "sqrt((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Sin) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, sin(2 + x_)),
            MakeScalarFunctionCode("f", 2, "sin((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Cos) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, cos(2 + x_)),
            MakeScalarFunctionCode("f", 2, "cos((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Tan) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, tan(2 + x_)),
            MakeScalarFunctionCode("f", 2, "tan((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Asin) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, asin(2 + x_)),
            MakeScalarFunctionCode("f", 2, "asin((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Acos) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, acos(2 + x_)),
            MakeScalarFunctionCode("f", 2, "acos((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Atan) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, atan(2 + x_)),
            MakeScalarFunctionCode("f", 2, "atan((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Atan2) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, atan2(2 + x_, 3 + y_)),
            MakeScalarFunctionCode("f", 2, "atan2((2 + p[0]), (3 + p[1]))"));
}

TEST_F(SymbolicCodeGenTest, Sinh) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, sinh(2 + x_)),
            MakeScalarFunctionCode("f", 2, "sinh((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Cosh) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, cosh(2 + x_)),
            MakeScalarFunctionCode("f", 2, "cosh((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Tanh) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, tanh(2 + x_)),
            MakeScalarFunctionCode("f", 2, "tanh((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Min) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, min(2 + x_, 3 + y_)),
            MakeScalarFunctionCode("f", 2, "fmin((2 + p[0]), (3 + p[1]))"));
}

TEST_F(SymbolicCodeGenTest, Max) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, max(2 + x_, 3 + y_)),
            MakeScalarFunctionCode("f", 2, "fmax((2 + p[0]), (3 + p[1]))"));
}

TEST_F(SymbolicCodeGenTest, Ceil) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, ceil(2 + x_)),
            MakeScalarFunctionCode("f", 2, "ceil((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Floor) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, floor(2 + x_)),
            MakeScalarFunctionCode("f", 2, "floor((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, IfThenElse) {
  const Expression e{if_then_else(x_ > y_, x_, y_)};
  EXPECT_THROW(CodeGen("f", {x_, y_}, e), std::runtime_error);
}

TEST_F(SymbolicCodeGenTest, UninterpretedFunction) {
  const Expression e{uninterpreted_function("uf", {x_, y_})};
  EXPECT_THROW(CodeGen("f", {x_, y_}, e), std::runtime_error);
}

TEST_F(SymbolicCodeGenTest, ScalarExampleInDocumentation) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, 1 + sin(x_) + cos(y_)),
            MakeScalarFunctionCode("f", 2, "(1 + sin(p[0]) + cos(p[1]))"));
}

TEST_F(SymbolicCodeGenTest, DenseMatrixRowMajor) {
  Eigen::Matrix<symbolic::Expression, 3, 2, Eigen::RowMajor> M;
  vector<string> expected;

  M(0, 0) = 3 + 2 * x_ + y_;
  expected.push_back("(3 + (2 * p[0]) + p[1])");

  M(0, 1) = 2 * pow(x_, 2) * pow(y_, 3);
  expected.push_back("(2 * pow(p[0], 2.000000) * pow(p[1], 3.000000))");

  M(1, 0) = 5 + sin(x_) + cos(z_);
  expected.push_back("(5 + sin(p[0]) + cos(p[2]))");

  M(1, 1) = 3 * min(x_, w_);
  expected.push_back("(3 * fmin(p[0], p[3]))");

  M(2, 0) = (x_ + 2) / (y_ - 2);
  expected.push_back("((2 + p[0]) / (-2 + p[1]))");

  M(2, 1) = atan2(w_ + 3, y_ + 4);
  expected.push_back("atan2((3 + p[3]), (4 + p[1]))");

  EXPECT_EQ(CodeGen("f", {x_, y_, z_, w_}, M),
            MakeDenseMatrixFunctionCode("f", 4 /* number of input parameters */,
                                        3 /* number of rows */,
                                        2 /* number of columns */, expected));
}

TEST_F(SymbolicCodeGenTest, DenseMatrixColMajor) {
  Eigen::Matrix<symbolic::Expression, 3, 2, Eigen::ColMajor> M;
  vector<string> expected;

  M(0, 0) = 3 + 2 * x_ + y_;
  expected.push_back("(3 + (2 * p[0]) + p[1])");

  M(1, 0) = 5 + sin(x_) + cos(z_);
  expected.push_back("(5 + sin(p[0]) + cos(p[2]))");

  M(2, 0) = (x_ + 2) / (y_ - 2);
  expected.push_back("((2 + p[0]) / (-2 + p[1]))");

  M(0, 1) = 2 * pow(x_, 2) * pow(y_, 3);
  expected.push_back("(2 * pow(p[0], 2.000000) * pow(p[1], 3.000000))");

  M(1, 1) = 3 * min(x_, w_);
  expected.push_back("(3 * fmin(p[0], p[3]))");

  M(2, 1) = atan2(w_ + 3, y_ + 4);
  expected.push_back("atan2((3 + p[3]), (4 + p[1]))");

  EXPECT_EQ(CodeGen("f", {x_, y_, z_, w_}, M),
            MakeDenseMatrixFunctionCode("f", 4 /* number of input parameters */,
                                        3 /* number of rows */,
                                        2 /* number of columns */, expected));
}

TEST_F(SymbolicCodeGenTest, DenseMatrixExampleInDocumentation) {
  Eigen::Matrix<symbolic::Expression, 2, 2, Eigen::ColMajor> M;
  vector<string> expected;

  M(0, 0) = 1.0;
  expected.push_back("1.000000");

  M(1, 0) = 3 + x_ + y_;
  expected.push_back("(3 + p[0] + p[1])");

  M(0, 1) = 4 * y_;
  expected.push_back("(4 * p[1])");

  M(1, 1) = sin(x_);
  expected.push_back("sin(p[0])");

  EXPECT_EQ(CodeGen("f", {x_, y_}, M),
            MakeDenseMatrixFunctionCode("f", 2 /* number of input parameters */,
                                        2 /* number of rows */,
                                        2 /* number of columns */, expected));
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
