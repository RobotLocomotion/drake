#include "drake/solvers/ibex_converter.h"

#include <sstream>
#include <stdexcept>
#include <string>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <gtest/gtest.h>

#include "drake/common/symbolic.h"

namespace drake {
namespace solvers {
namespace internal {
namespace {

using std::ostringstream;
using std::string;

using symbolic::Expression;
using symbolic::Formula;
using symbolic::Variable;

class IbexConverterTest : public ::testing::Test {
 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};

  const Expression x_{var_x_};
  const Expression y_{var_y_};
  const Expression z_{var_z_};

  IbexConverter converter_{Vector3<Variable>{var_x_, var_y_, var_z_}};
};

::testing::AssertionResult CheckConversionResult(const Expression& e,
                                                 const ibex::ExprNode& n,
                                                 const string& expected) {
  ostringstream oss;
  oss << n;
  const string output{oss.str()};

  if (output == expected) {
    return ::testing::AssertionSuccess();
  } else {
    return ::testing::AssertionFailure() << fmt::format(
               "'{}' is expected for the symbolic expression '{}' but it is "
               "converted to '{}'.",
               expected, e, output);
  }
}

::testing::AssertionResult CheckConversionResult(const Formula& f,
                                                 const ibex::ExprCtr& c,
                                                 const string& expected) {
  ostringstream oss;
  oss << c;
  const string output{oss.str()};

  if (output == expected) {
    return ::testing::AssertionSuccess();
  } else {
    return ::testing::AssertionFailure() << fmt::format(
               "'{}' is expected for the symbolic formula '{}' but it is "
               "converted to '{}'.",
               expected, f, output);
  }
}

TEST_F(IbexConverterTest, Variable) {
  {
    const Expression e{x_};
    const UniquePtrToExprNode n{converter_.Convert(e)};
    EXPECT_TRUE(CheckConversionResult(e, *n, "x"));
  }  // namespace
  {
    const Expression e{Variable{"a"}};
    EXPECT_THROW(converter_.Convert(e), std::exception);
  }
}

TEST_F(IbexConverterTest, Constant) {
  {
    const Expression e{0};
    const UniquePtrToExprNode n{converter_.Convert(e)};
    EXPECT_TRUE(CheckConversionResult(e, *n, "0"));
  }
  {
    const Expression e{-3.14};
    const UniquePtrToExprNode n{converter_.Convert(e)};
    EXPECT_TRUE(CheckConversionResult(e, *n, "-3.14"));
  }
}

TEST_F(IbexConverterTest, Addition) {
  const Expression e{3 + 2 * x_ - 4 * y_};
  const UniquePtrToExprNode n{converter_.Convert(e)};
  EXPECT_TRUE(CheckConversionResult(e, *n, "((3+(2*x))+(-4*y))"));
}

TEST_F(IbexConverterTest, Multiplication) {
  const Expression e{-5 * x_ * pow(y_, 3)};
  const UniquePtrToExprNode n{converter_.Convert(e)};
  EXPECT_TRUE(CheckConversionResult(e, *n, "((-5*x)*y^3)"));
}

TEST_F(IbexConverterTest, Division) {
  const Expression e{x_ / y_};
  const UniquePtrToExprNode n{converter_.Convert(e)};
  EXPECT_TRUE(CheckConversionResult(e, *n, "(x/y)"));
}

TEST_F(IbexConverterTest, Log) {
  const Expression e{log(x_)};
  const UniquePtrToExprNode n{converter_.Convert(e)};
  EXPECT_TRUE(CheckConversionResult(e, *n, "log(x)"));
}

TEST_F(IbexConverterTest, Abs) {
  const Expression e{abs(x_)};
  const UniquePtrToExprNode n{converter_.Convert(e)};
  EXPECT_TRUE(CheckConversionResult(e, *n, "abs(x)"));
}

TEST_F(IbexConverterTest, Exp) {
  const Expression e{exp(x_)};
  const UniquePtrToExprNode n{converter_.Convert(e)};
  EXPECT_TRUE(CheckConversionResult(e, *n, "exp(x)"));
}

TEST_F(IbexConverterTest, Sqrt) {
  const Expression e{sqrt(x_)};
  const UniquePtrToExprNode n{converter_.Convert(e)};
  EXPECT_TRUE(CheckConversionResult(e, *n, "sqrt(x)"));
}

TEST_F(IbexConverterTest, Pow) {
  {
    // x^2 => x^2.
    const Expression e{pow(x_, 2)};
    const UniquePtrToExprNode n{converter_.Convert(e)};
    EXPECT_TRUE(CheckConversionResult(e, *n, "x^2"));
  }
  {
    // x^(2.1) => exp(2.1 * log(x)).
    const Expression e{pow(x_, 2.1)};
    const UniquePtrToExprNode n{converter_.Convert(e)};
    EXPECT_TRUE(CheckConversionResult(e, *n, "exp((2.1*log(x)))"));
  }
  {
    // 3^x => exp(x * log(3)).
    const Expression e{pow(3, x_)};
    const UniquePtrToExprNode n{converter_.Convert(e)};
    EXPECT_TRUE(CheckConversionResult(e, *n, "exp((x*log(3)))"));
  }
  {
    // x^y => exp(y * log(x)).
    const Expression e{pow(x_, y_)};
    const UniquePtrToExprNode n{converter_.Convert(e)};
    EXPECT_TRUE(CheckConversionResult(e, *n, "exp((y*log(x)))"));
  }
}

TEST_F(IbexConverterTest, Sin) {
  const Expression e{sin(x_)};
  const UniquePtrToExprNode n{converter_.Convert(e)};
  EXPECT_TRUE(CheckConversionResult(e, *n, "sin(x)"));
}

TEST_F(IbexConverterTest, Cos) {
  const Expression e{cos(x_)};
  const UniquePtrToExprNode n{converter_.Convert(e)};
  EXPECT_TRUE(CheckConversionResult(e, *n, "cos(x)"));
}

TEST_F(IbexConverterTest, Tan) {
  const Expression e{tan(x_)};
  const UniquePtrToExprNode n{converter_.Convert(e)};
  EXPECT_TRUE(CheckConversionResult(e, *n, "tan(x)"));
}

TEST_F(IbexConverterTest, Asin) {
  const Expression e{asin(x_)};
  const UniquePtrToExprNode n{converter_.Convert(e)};
  EXPECT_TRUE(CheckConversionResult(e, *n, "asin(x)"));
}

TEST_F(IbexConverterTest, Acos) {
  const Expression e{acos(x_)};
  const UniquePtrToExprNode n{converter_.Convert(e)};
  EXPECT_TRUE(CheckConversionResult(e, *n, "acos(x)"));
}

TEST_F(IbexConverterTest, Atan) {
  const Expression e{atan(x_)};
  const UniquePtrToExprNode n{converter_.Convert(e)};
  EXPECT_TRUE(CheckConversionResult(e, *n, "atan(x)"));
}

TEST_F(IbexConverterTest, Atan2) {
  const Expression e{atan2(x_, y_)};
  const UniquePtrToExprNode n{converter_.Convert(e)};
  EXPECT_TRUE(CheckConversionResult(e, *n, "atan2(x,y)"));
}

TEST_F(IbexConverterTest, Sinh) {
  const Expression e{sinh(x_)};
  const UniquePtrToExprNode n{converter_.Convert(e)};
  EXPECT_TRUE(CheckConversionResult(e, *n, "sinh(x)"));
}

TEST_F(IbexConverterTest, Cosh) {
  const Expression e{cosh(x_)};
  const UniquePtrToExprNode n{converter_.Convert(e)};
  EXPECT_TRUE(CheckConversionResult(e, *n, "cosh(x)"));
}

TEST_F(IbexConverterTest, Tanh) {
  const Expression e{tanh(x_)};
  const UniquePtrToExprNode n{converter_.Convert(e)};
  EXPECT_TRUE(CheckConversionResult(e, *n, "tanh(x)"));
}

TEST_F(IbexConverterTest, Min) {
  const Expression e{min(x_, y_)};
  const UniquePtrToExprNode n{converter_.Convert(e)};
  EXPECT_TRUE(CheckConversionResult(e, *n, "min(x,y)"));
}

TEST_F(IbexConverterTest, Max) {
  const Expression e{max(x_, y_)};
  const UniquePtrToExprNode n{converter_.Convert(e)};
  EXPECT_TRUE(CheckConversionResult(e, *n, "max(x,y)"));
}

TEST_F(IbexConverterTest, IfThenElse) {
  const Expression e{if_then_else(x_ > y_, x_, y_)};
  EXPECT_THROW(converter_.Convert(e), std::exception);
}

TEST_F(IbexConverterTest, Ceil) {
  const Expression e{ceil(x_)};
  EXPECT_THROW(converter_.Convert(e), std::exception);
}

TEST_F(IbexConverterTest, Floor) {
  const Expression e{floor(x_)};
  EXPECT_THROW(converter_.Convert(e), std::exception);
}

TEST_F(IbexConverterTest, UninterpretedFunction) {
  const Expression e{symbolic::uninterpreted_function("uf", {var_x_})};
  EXPECT_THROW(converter_.Convert(e), std::exception);
}

TEST_F(IbexConverterTest, True) {
  const Formula f{Formula::True()};
  EXPECT_THROW(converter_.Convert(f), std::exception);
}

TEST_F(IbexConverterTest, False) {
  const Formula f{Formula::False()};
  EXPECT_THROW(converter_.Convert(f), std::exception);
}

TEST_F(IbexConverterTest, BooleanVariable) {
  const Formula f{Variable{"b", Variable::Type::BOOLEAN}};
  EXPECT_THROW(converter_.Convert(f), std::exception);
}

TEST_F(IbexConverterTest, Eq) {
  const Formula f{x_ == y_};
  const UniquePtrToExprCtr c{converter_.Convert(f)};
  EXPECT_TRUE(CheckConversionResult(f, *c, "(x-y)=0"));
}

TEST_F(IbexConverterTest, Neq) {
  const Formula f{x_ != y_};
  EXPECT_THROW(converter_.Convert(f), std::exception);
}

TEST_F(IbexConverterTest, Le) {
  const Formula f{x_ <= y_};
  const UniquePtrToExprCtr c{converter_.Convert(f)};
  EXPECT_TRUE(CheckConversionResult(f, *c, "(x-y)<=0"));
}

TEST_F(IbexConverterTest, Lt) {
  const Formula f{x_ < y_};
  const UniquePtrToExprCtr c{converter_.Convert(f)};
  EXPECT_TRUE(CheckConversionResult(f, *c, "(x-y)<0"));
}

TEST_F(IbexConverterTest, Ge) {
  const Formula f{x_ >= y_};
  const UniquePtrToExprCtr c{converter_.Convert(f)};
  EXPECT_TRUE(CheckConversionResult(f, *c, "(x-y)>=0"));
}

TEST_F(IbexConverterTest, Gt) {
  const Formula f{x_ > y_};
  const UniquePtrToExprCtr c{converter_.Convert(f)};
  EXPECT_TRUE(CheckConversionResult(f, *c, "(x-y)>0"));
}

TEST_F(IbexConverterTest, And) {
  const Formula f{(x_ > 0) && (y_ > 0)};
  EXPECT_THROW(converter_.Convert(f), std::exception);
}

TEST_F(IbexConverterTest, Or) {
  const Formula f{(x_ > 0) || (y_ > 0)};
  EXPECT_THROW(converter_.Convert(f), std::exception);
}

TEST_F(IbexConverterTest, Negation) {
  {
    const Formula f{!(x_ == y_)};
    EXPECT_THROW(converter_.Convert(f), std::exception);
  }
  {
    const Formula f{!(x_ != y_)};
    const UniquePtrToExprCtr c{converter_.Convert(f)};
    EXPECT_TRUE(CheckConversionResult(f, *c, "(x-y)=0"));
  }
  {
    const Formula f{!(x_ <= y_)};
    const UniquePtrToExprCtr c{converter_.Convert(f)};
    EXPECT_TRUE(CheckConversionResult(f, *c, "(x-y)>0"));
  }
  {
    const Formula f{!(x_ < y_)};
    const UniquePtrToExprCtr c{converter_.Convert(f)};
    EXPECT_TRUE(CheckConversionResult(f, *c, "(x-y)>=0"));
  }
  {
    const Formula f{!(x_ >= y_)};
    const UniquePtrToExprCtr c{converter_.Convert(f)};
    EXPECT_TRUE(CheckConversionResult(f, *c, "(x-y)<0"));
  }
  {
    const Formula f{!(x_ > y_)};
    const UniquePtrToExprCtr c{converter_.Convert(f)};
    EXPECT_TRUE(CheckConversionResult(f, *c, "(x-y)<=0"));
  }
}

TEST_F(IbexConverterTest, Forall) {
  const Formula f{forall({var_x_}, x_ > y_)};
  EXPECT_THROW(converter_.Convert(f), std::exception);
}

TEST_F(IbexConverterTest, Isnan) {
  const Formula f{isnan(Expression::Zero())};
  EXPECT_THROW(converter_.Convert(f), std::exception);
}

TEST_F(IbexConverterTest, PSD) {
  Eigen::Matrix<Expression, 2, 2> m;
  // clang-format off
  m << (x_ + y_),  -1.0,
       -1.0,        y_;
  // clang-format on
  const Formula f{positive_semidefinite(m)};
  EXPECT_THROW(converter_.Convert(f), std::exception);
}

}  // namespace
}  // namespace internal
}  // namespace solvers
}  // namespace drake
