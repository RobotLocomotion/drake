#include "drake/common/symbolic/latex.h"

#include <algorithm>
#include <limits>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/symbolic/expression.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace symbolic {
namespace {

GTEST_TEST(SymbolicLatex, BasicTest) {
  Variable x{"x"}, y{"y"};
  Variable b("b", Variable::Type::BOOLEAN);
  const double kInf = std::numeric_limits<double>::infinity();

  // Expressions
  EXPECT_EQ(ToLatex(x), "x");
  EXPECT_EQ(ToLatex(0.0), "0");
  EXPECT_EQ(ToLatex(1.0), "1");
  EXPECT_EQ(ToLatex(1.0, 1), "1");
  EXPECT_EQ(ToLatex(1.01), "1.010");
  EXPECT_EQ(ToLatex(1.01, 1), "1.0");
  EXPECT_EQ(ToLatex(4e9), "4000000000");
  EXPECT_EQ(ToLatex(kInf - kInf), "\\text{NaN}");
  EXPECT_EQ(ToLatex(kInf), "\\infty");
  EXPECT_EQ(ToLatex(-kInf), "-\\infty");
  EXPECT_EQ(ToLatex(M_PI), "\\pi");
  EXPECT_EQ(ToLatex(-M_PI), "-\\pi");
  EXPECT_EQ(ToLatex(14.0 * M_PI), "14\\pi");
  EXPECT_EQ(ToLatex(0 * M_PI), "0");
  EXPECT_EQ(ToLatex(M_PI + 1e-16, 3), "\\pi");
  EXPECT_EQ(ToLatex(M_PI + 1e-13, 3), "3.142");
  EXPECT_EQ(ToLatex(M_PI - 1e-16, 3), "\\pi");
  EXPECT_EQ(ToLatex(M_PI - 1e-13, 3), "3.142");
  EXPECT_EQ(ToLatex(-7.0 * M_E), "-7e");
  EXPECT_EQ(ToLatex(x + y), "(x + y)");
  EXPECT_EQ(ToLatex(x + 2.3), "(2.300 + x)");
  EXPECT_EQ(ToLatex(x - y), "(x - y)");
  EXPECT_EQ(ToLatex(x - 2 * y), "(x - 2y)");
  EXPECT_EQ(ToLatex(2 * x + 3 * x + 4 * y), "(5x + 4y)");
  EXPECT_EQ(ToLatex(2.1 * x + 3.2 * y * y, 1), "(2.1x + 3.2y^{2})");
  EXPECT_EQ(ToLatex(x * pow(y, 2)), "x y^{2}");
  EXPECT_EQ(ToLatex(2 * x * y), "2 x y");
  EXPECT_EQ(ToLatex(pow(x, 3)), "x^{3}");
  EXPECT_EQ(ToLatex(pow(x, 3.1)), "x^{3.100}");
  EXPECT_EQ(ToLatex(x / y), R"""(\frac{x}{y})""");
  EXPECT_EQ(ToLatex(abs(x)), "|x|");
  EXPECT_EQ(ToLatex(log(x)), R"""(\log{x})""");
  EXPECT_EQ(ToLatex(exp(x)), "e^{x}");
  EXPECT_EQ(ToLatex(sqrt(x)), R"""(\sqrt{x})""");
  EXPECT_EQ(ToLatex(sin(x)), R"""(\sin{x})""");
  EXPECT_EQ(ToLatex(cos(x)), R"""(\cos{x})""");
  EXPECT_EQ(ToLatex(tan(x)), R"""(\tan{x})""");
  EXPECT_EQ(ToLatex(asin(x)), R"""(\asin{x})""");
  EXPECT_EQ(ToLatex(acos(x)), R"""(\acos{x})""");
  EXPECT_EQ(ToLatex(atan(x)), R"""(\atan{x})""");
  EXPECT_EQ(ToLatex(atan2(y, x)), R"""(\atan{\frac{y}{x}})""");
  EXPECT_EQ(ToLatex(sinh(x)), R"""(\sinh{x})""");
  EXPECT_EQ(ToLatex(cosh(x)), R"""(\cosh{x})""");
  EXPECT_EQ(ToLatex(tanh(x)), R"""(\tanh{x})""");
  EXPECT_EQ(ToLatex(min(x, y)), R"""(\min\{x, y\})""");
  EXPECT_EQ(ToLatex(max(x, y)), R"""(\max\{x, y\})""");
  EXPECT_EQ(ToLatex(ceil(x)), R"""(\lceil x \rceil)""");
  EXPECT_EQ(ToLatex(floor(x)), R"""(\lfloor x \rfloor)""");
  EXPECT_EQ(ToLatex(if_then_else(x > y, 2 * x, 3)),
            R"""(\begin{cases} 2 x & \text{if } x > y, \\)"""
            R"""( 3 & \text{otherwise}.\end{cases})""");
  DRAKE_EXPECT_THROWS_MESSAGE(
      ToLatex(uninterpreted_function("myfunc", std::vector<Expression>())),
      "ToLatex does not support uninterpreted functions.");

  // Formulas
  EXPECT_EQ(ToLatex(Formula::False()), R"""(\text{false})""");
  EXPECT_EQ(ToLatex(Formula::True()), R"""(\text{true})""");
  EXPECT_EQ(ToLatex(Formula(b)), "b");
  EXPECT_EQ(ToLatex(x == y), R"""(x = y)""");
  EXPECT_EQ(ToLatex(2.5 * x == y, 2), R"""(2.50 x = y)""");
  EXPECT_EQ(ToLatex(2 * x != y), R"""(2 x \neq y)""");
  EXPECT_EQ(ToLatex(2 * x > y), R"""(2 x > y)""");
  EXPECT_EQ(ToLatex(2 * x >= y), R"""(2 x \ge y)""");
  EXPECT_EQ(ToLatex(2 * x < y), R"""(2 x < y)""");
  EXPECT_EQ(ToLatex(2 * x <= y), R"""(2 x \le y)""");
  EXPECT_EQ(ToLatex(x == y && x * y > x), R"""(x = y \land x y > x)""");
  EXPECT_EQ(ToLatex(!(x == y && x * y > x)), R"""(x \neq y \lor x y \le x)""");
  EXPECT_EQ(ToLatex(x == y || x * y < x), R"""(x = y \lor x y < x)""");
  EXPECT_EQ(ToLatex(!(x == y || x * y < x)), R"""(x \neq y \land x y \ge x)""");
  EXPECT_EQ(ToLatex(!(x == y)), R"""(x \neq y)""");
  EXPECT_EQ(ToLatex(forall({x, y}, x > y)), R"""(\forall x, y: (x > y))""");
  EXPECT_EQ(ToLatex(isnan(x)), R"""(\text{isnan}(x))""");
  EXPECT_EQ(ToLatex(!isnan(x)), R"""(\neg \text{isnan}(x))""");

  // Matrix<double>
  Eigen::Matrix<double, 2, 2> M;
  M << 1.2, 3, 4.56, 7;
  EXPECT_EQ(ToLatex(M, 1),
            R"""(\begin{bmatrix} 1.2 & 3 \\ 4.6 & 7 \end{bmatrix})""");

  // Matrix<Expression>
  Eigen::Matrix<Expression, 2, 2> Me;
  Me << x, 2.3 * y, 2.3 * y, x + y;
  EXPECT_EQ(
      ToLatex(Me, 1),
      R"""(\begin{bmatrix} x & 2.3 y \\ 2.3 y & (x + y) \end{bmatrix})""");

  // Formula with a PSD Matrix.
  EXPECT_EQ(ToLatex(positive_semidefinite(Me), 1),
            R"""(\begin{bmatrix} x & 2.3 y \\ 2.3 y & (x + y) \end{bmatrix})"""
            R"""( \succeq 0)""");
}

GTEST_TEST(SymbolicLatex, MatrixSubscripts) {
  const VectorX<Variable> x = MakeVectorVariable(2, "x");
  const Vector2<Variable> y = MakeVectorVariable<2>("y");
  const MatrixX<Variable> A = MakeMatrixVariable(2, 2, "A");

  EXPECT_EQ(ToLatex(x[0]), "x_{0}");
  EXPECT_EQ(ToLatex(x[1]), "x_{1}");
  EXPECT_EQ(ToLatex(y[0]), "y_{0}");
  EXPECT_EQ(ToLatex(y[1]), "y_{1}");
  EXPECT_EQ(ToLatex(A(0, 0)), "A_{0, 0}");
  EXPECT_EQ(ToLatex(A(0, 1)), "A_{0, 1}");
  EXPECT_EQ(ToLatex(A(1, 0)), "A_{1, 0}");
  EXPECT_EQ(ToLatex(A(1, 1)), "A_{1, 1}");
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
