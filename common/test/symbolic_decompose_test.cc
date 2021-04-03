#include "drake/common/symbolic_decompose.h"

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace symbolic {
namespace {

using std::runtime_error;
using std::unordered_map;
using MapVarToIndex = unordered_map<Variable::Id, int>;
constexpr double kEps = std::numeric_limits<double>::epsilon();

class SymbolicDecomposeTest : public ::testing::Test {
 public:
  SymbolicDecomposeTest() = default;
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SymbolicDecomposeTest)

 protected:
  void SetUp() override {
    // clang-format off
  M_  << 1, 0, 3,
        -4, 5, 0,
         7, 0, 9;
  v_ << 3,
       -7,
        2;
  x_ << x0_, x1_, x2_;
    // clang-format on
  }

  const Variable x0_{"x0"};
  const Variable x1_{"x1"};
  const Variable x2_{"x2"};
  VectorX<Variable> x_{3};
  const Variable a_{"a"};
  const Variable b_{"b"};
  const Variable c_{"c"};

  // M and v matrices that we pass to decompose functions as input. We mix
  // static-sized and dynamic-sized matrices intentionally.
  Eigen::Matrix3d M_;
  Eigen::Vector3d v_;

  // Expected values for M and v.
  Eigen::Matrix3d M_expected_static_;
  Eigen::MatrixXd M_expected_dynamic_{3, 3};
  Eigen::Vector3d v_expected_static_;
  Eigen::VectorXd v_expected_dynamic_{3};

  // Extra terms that we add to test exceptional cases.
  Eigen::Matrix<Expression, 3, 1> extra_terms_;
};

TEST_F(SymbolicDecomposeTest, DecomposeLinearExpressionsDynamic) {
  DecomposeLinearExpressions(M_ * x_, x_, &M_expected_dynamic_);
  EXPECT_EQ(M_expected_dynamic_, M_);
}

TEST_F(SymbolicDecomposeTest, DecomposeLinearExpressionsStatic) {
  DecomposeLinearExpressions(M_ * x_, x_, &M_expected_static_);
  EXPECT_EQ(M_expected_static_, M_);
}

// Adds quadratic terms to check if we have an exception.
TEST_F(SymbolicDecomposeTest, DecomposeLinearExpressionsExceptionNonLinear) {
  // clang-format off
  extra_terms_ << x0_ * x0_,
                 x1_ * x1_,
                 x2_ * x2_;
  // clang-format on
  EXPECT_THROW(DecomposeLinearExpressions(M_ * x_ + extra_terms_, x_,
                                          &M_expected_static_),
               runtime_error);
}

// Adds nonlinear terms to check if we have an exception.
TEST_F(SymbolicDecomposeTest,
       DecomposeLinearExpressionsExceptionNonPolynomial) {
  // clang-format off
  extra_terms_ << sin(x0_),
                 cos(x1_),
                 log(x2_);
  // clang-format on
  EXPECT_THROW(DecomposeLinearExpressions(M_ * x_ + extra_terms_, x_,
                                          &M_expected_static_),
               runtime_error);
}

// Adds terms with non-const coefficients to check if we have an exception.
TEST_F(SymbolicDecomposeTest,
       DecomposeLinearExpressionsExceptionNonConstCoefficient) {
  // clang-format off
  extra_terms_ << a_ * x0_,
                  b_ * x1_,
                  c_ * x2_;
  // clang-format on
  EXPECT_THROW(DecomposeLinearExpressions(M_ * x_ + extra_terms_, x_,
                                          &M_expected_static_),
               runtime_error);
}

// Adds constant terms to check if we have an exception.
TEST_F(SymbolicDecomposeTest, DecomposeLinearExpressionsExceptionAffine) {
  // clang-format off
  extra_terms_ << -1,
                  1,
                  M_PI;
  // clang-format on
  EXPECT_THROW(DecomposeLinearExpressions(M_ * x_ + extra_terms_, x_,
                                          &M_expected_static_),
               runtime_error);
}

TEST_F(SymbolicDecomposeTest, DecomposeAffineExpressionsDynamic) {
  DecomposeAffineExpressions(M_ * x_ + v_, x_, &M_expected_dynamic_,
                             &v_expected_dynamic_);
  EXPECT_EQ(M_expected_dynamic_, M_);
  EXPECT_EQ(v_expected_dynamic_, v_);
}

TEST_F(SymbolicDecomposeTest, DecomposeAffineExpressionsStatic) {
  DecomposeAffineExpressions(M_ * x_ + v_, x_, &M_expected_static_,
                             &v_expected_static_);
  EXPECT_EQ(M_expected_static_, M_);
  EXPECT_EQ(v_expected_static_, v_);
}

TEST_F(SymbolicDecomposeTest, DecomposeAffineExpressionsBlock) {
  Eigen::Matrix4d M_expected;
  auto block = M_expected.block(0, 0, 3, 3);
  DecomposeAffineExpressions(M_ * x_ + v_, x_, &block, &v_expected_static_);
  EXPECT_EQ(M_expected.block(0, 0, 3, 3), M_);
  EXPECT_EQ(v_expected_static_, v_);
}

// Adds quadratic terms to check if we have an exception.
TEST_F(SymbolicDecomposeTest, DecomposeAffineExpressionsExceptionNonlinear) {
  // clang-format off
  extra_terms_ << x0_ * x0_,
                 x1_ * x1_,
                 x2_ * x2_;
  // clang-format on
  EXPECT_THROW(
      DecomposeAffineExpressions(M_ * x_ + v_ + extra_terms_, x_,
                                 &M_expected_static_, &v_expected_static_),
      runtime_error);
}

// Adds terms with non-const coefficients to check if we have an exception.
TEST_F(SymbolicDecomposeTest,
       DecomposeAffineExpressionsExceptionNonConstCoefficient) {
  // clang-format off
  extra_terms_ << a_ * x0_,
                  b_ * x1_,
                  c_ * x2_;
  // clang-format on
  EXPECT_THROW(
      DecomposeAffineExpressions(M_ * x_ + v_ + extra_terms_, x_,
                                 &M_expected_static_, &v_expected_static_),
      runtime_error);
}

// Adds nonlinear terms to check if we have an exception.
TEST_F(SymbolicDecomposeTest,
       DecomposeAffineExpressionsExceptionNonPolynomial) {
  // clang-format off
  extra_terms_ << sin(x0_),
                 cos(x1_),
                 log(x2_);
  // clang-format on
  EXPECT_THROW(
      DecomposeAffineExpressions(M_ * x_ + v_ + extra_terms_, x_,
                                 &M_expected_static_, &v_expected_static_),
      runtime_error);
}

// Check expected invariance
void ExpectValidMapVarToIndex(const VectorX<Variable>& vars,
                              const MapVarToIndex& map_var_to_index) {
  EXPECT_EQ(vars.size(), map_var_to_index.size());
  for (int i = 0; i < vars.size(); ++i) {
    const auto& var = vars(i);
    EXPECT_EQ(i, map_var_to_index.at(var.get_id()));
  }
}

GTEST_TEST(SymbolicExtraction, ExtractVariables) {
  const Variable x("x");
  const Variable y("y");
  Expression e = x + y;
  VectorX<Variable> vars_expected(2);
  vars_expected << x, y;

  MapVarToIndex map_var_to_index;
  VectorX<Variable> vars;
  std::tie(vars, map_var_to_index) = ExtractVariablesFromExpression(e);
  EXPECT_EQ(vars_expected, vars);
  ExpectValidMapVarToIndex(vars, map_var_to_index);

  const Variable z("z");
  e += x * (z - y);
  const int vars_size = vars_expected.rows();
  vars_expected.conservativeResize(vars_size + 1, Eigen::NoChange);
  vars_expected(vars_size) = z;

  ExtractAndAppendVariablesFromExpression(e, &vars, &map_var_to_index);
  EXPECT_EQ(vars_expected, vars);
  ExpectValidMapVarToIndex(vars, map_var_to_index);
}

// Make off-diagonal terms symmetric for a given input matrix
template <typename Derived>
void AverageOffDiagonalTerms(const Eigen::MatrixBase<Derived>& Q_asym,
                             Eigen::MatrixBase<Derived>* pQ_sym) {
  DRAKE_ASSERT(Q_asym.rows() == Q_asym.cols());
  Derived& Q_sym = pQ_sym->derived();
  Q_sym = Q_asym;
  int size = Q_sym.rows();
  for (int r = 0; r < size - 1; ++r) {
    for (int c = r + 1; c < size; ++c) {
      Q_sym(r, c) = 0.5 * (Q_asym(r, c) + Q_asym(c, r));
      Q_sym(c, r) = Q_sym(r, c);
    }
  }
}

GTEST_TEST(SymbolicExtraction, DecomposeQuadraticExpression) {
  const int num_variables = 3;
  const Variable x("x");
  const Variable y("y");
  const Variable z("z");
  const Vector3<Variable> vars_expected(x, y, z);

  Eigen::Matrix3d Q_diagonal, Q_symmetric, Q_asymmetric;
  Q_diagonal.setZero().diagonal() = Eigen::Vector3d(1, 2, 3);
  Q_symmetric << 3, 2, 1, 2, 4, 5, 1, 5, 6;
  Q_asymmetric << 3, 2, 1, 4, 6, 5, 7, 8, 9;

  const Eigen::Vector3d b_expected(10, 11, 12);
  const double c_expected = 13;

  for (const Eigen::Matrix3d& Q_in : {Q_diagonal, Q_symmetric, Q_asymmetric}) {
    const Expression e =
        vars_expected.dot(0.5 * Q_in * vars_expected + b_expected) + c_expected;

    const auto pair = ExtractVariablesFromExpression(e);
    const VectorX<Variable>& vars = pair.first;
    const MapVarToIndex& map_var_to_index = pair.second;
    EXPECT_EQ(vars_expected, vars);

    const symbolic::Polynomial poly{e};

    Eigen::MatrixXd Q(num_variables, num_variables);
    Eigen::VectorXd b(num_variables);
    double c;
    DecomposeQuadraticPolynomial(poly, map_var_to_index, &Q, &b, &c);
    Eigen::Matrix3d Q_expected;
    AverageOffDiagonalTerms(Q_in, &Q_expected);
    EXPECT_TRUE(CompareMatrices(Q_expected, Q, kEps));
    EXPECT_TRUE(CompareMatrices(b_expected, b, kEps));
    EXPECT_EQ(c_expected, c);
  }
}

// Return scalar value from an effectively scalar Eigen expression
template <typename Derived>
const typename Derived::Scalar& AsScalar(const Eigen::EigenBase<Derived>& X) {
  DRAKE_ASSERT(X.rows() == 1 && X.cols() == 1);
  return X.derived().coeffRef(0, 0);
}

GTEST_TEST(SymbolicExtraction, DecomposeAffineExpression) {
  const int num_variables = 3;
  const Variable x("x");
  const Variable y("y");
  const Variable z("z");

  const Vector3<Variable> vars_expected(x, y, z);

  // Scalar value case
  {
    Eigen::MatrixXd coeffs_expected(1, num_variables);
    coeffs_expected << 1, 2, 3;
    double c_expected = 4;
    const Expression e = AsScalar(coeffs_expected * vars_expected) + c_expected;

    const auto pair = ExtractVariablesFromExpression(e);
    const VectorX<Variable>& vars = pair.first;
    const MapVarToIndex& map_var_to_index = pair.second;
    EXPECT_EQ(vars_expected, vars);

    Eigen::MatrixXd coeffs(1, num_variables);
    double c;
    DecomposeAffineExpression(e, map_var_to_index, coeffs, &c);

    EXPECT_TRUE(CompareMatrices(coeffs_expected, coeffs, kEps));
    EXPECT_EQ(c_expected, c);
  }
  // Vector value case
  {
    const int num_eq = 3;
    Eigen::MatrixXd coeffs_expected(num_eq, num_variables);
    coeffs_expected << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    Eigen::VectorXd c_expected(num_eq);
    c_expected << 10, 11, 12;

    const VectorX<Expression> ev = coeffs_expected * vars_expected + c_expected;

    Eigen::MatrixXd coeffs(num_eq, num_variables);
    Eigen::VectorXd c(num_eq);
    VectorX<Variable> vars;
    DecomposeAffineExpressions(ev, &coeffs, &c, &vars);

    EXPECT_EQ(vars_expected, vars);
    EXPECT_TRUE(CompareMatrices(coeffs_expected, coeffs, kEps));
    EXPECT_TRUE(CompareMatrices(c_expected, c, kEps));
  }
}

GTEST_TEST(SymbolicExtraction, DecomposeLumpedParameters) {
  const Variable x("x");
  const Variable y("y");
  const Variable a("a");
  const Variable b("b");
  const Variable c("c");

  {
    // e = a + x, params = {a,b,c} should give
    // W = [1], α = [a], w0 = [x]
    const Expression e = a + x;
    const auto [W, alpha, w0] = DecomposeLumpedParameters(
        Vector1<Expression>(e), Vector3<Variable>{a, b, c});
    EXPECT_EQ(W, Vector1<Expression>{1});
    EXPECT_EQ(alpha, Vector1<Expression>{a});
    EXPECT_EQ(w0, Vector1<Expression>{x});
  }

  {
    // e = a*c*x*y, params = {a,b,c} should give
    // W = [x*y], α = [a*c], w0 = [0]
    const Expression e = a * c * x * y;
    const auto [W, alpha, w0] = DecomposeLumpedParameters(
        Vector1<Expression>(e), Vector3<Variable>{a, b, c});
    EXPECT_EQ(W, Vector1<Expression>{x * y});
    EXPECT_EQ(alpha, Vector1<Expression>{a * c});
    EXPECT_EQ(w0, Vector1<Expression>{0});
  }

  {
    // e = [a + x, a*x],  params = {a,b,c} should give
    // W = [1; x], α = [a], w0 = [x; 0]
    const Vector2<Expression> f(a + x, a * x);
    const auto [W, alpha, w0] =
        DecomposeLumpedParameters(f, Vector3<Variable>{a, b, c});
    EXPECT_EQ(W, Vector2<Expression>(1, x));
    EXPECT_EQ(alpha, Vector1<Expression>(a));
    EXPECT_EQ(w0, Vector2<Expression>(x, 0));
  }

  {
    // e = a*x*x, params = {a,b,c} should give
    // W = [x*x], α = [a], w0 = [0]
    const Expression e = a * x * x;
    const auto [W, alpha, w0] = DecomposeLumpedParameters(
        Vector1<Expression>(e), Vector3<Variable>{a, b, c});
    EXPECT_EQ(W, Vector1<Expression>{x * x});
    EXPECT_EQ(alpha, Vector1<Expression>{a});
    EXPECT_EQ(w0, Vector1<Expression>{0});
  }

  {
    // ax + bx + acy + acy² + 2, params = {a,b,c} should give
    //   W = [x, y+y²], α = [a+b, ac], w0 = [2]
    const Expression e = a * x + b * x + a * c * y + a * c * y * y + 2;
    const auto [W, alpha, w0] = DecomposeLumpedParameters(
        Vector1<Expression>(e), Vector3<Variable>{a, b, c});
    EXPECT_EQ(W, RowVector2<Expression>(x, y + y * y));
    EXPECT_EQ(alpha, Vector2<Expression>(a + b, a * c));
    EXPECT_EQ(w0, Vector1<Expression>(2));
  }

  {
    // Test non-polynomials.
    // e = sin(a)*cos(x)+tanh(y), params = {a,b,c} should give
    //   W = [cos(x)], α = [sin(a)], w0 = [tanh(x)]
    const Expression e = sin(a) * cos(x) + tanh(x);
    const auto [W, alpha, w0] = DecomposeLumpedParameters(
        Vector1<Expression>(e), Vector3<Variable>{a, b, c});
    EXPECT_EQ(W, Vector1<Expression>(cos(x)));
    EXPECT_EQ(alpha, Vector1<Expression>(sin(a)));
    EXPECT_EQ(w0, Vector1<Expression>(tanh(x)));
  }

  {
    // Test a case that takes a power of a mixed expression.
    // e = (a*x*sin(b))**2, params = {a,b,c} should give
    // W = [x*x], alpha = [a*a*sin(b)*sin(b)], w0 = [0].
    const Expression e = pow(a * sin(b) * x, 2);
    const auto [W, alpha, w0] = DecomposeLumpedParameters(
        Vector1<Expression>(e), Vector3<Variable>{a, b, c});
    EXPECT_EQ(W, Vector1<Expression>(x * x));
    EXPECT_EQ(alpha, Vector1<Expression>(a * a * sin(b) * sin(b)));
    EXPECT_EQ(w0, Vector1<Expression>(0));
  }

  {
    // Test a case that is not decomposable.
    // e = sin(a*x), params = {a,b,c} should give
    const Expression e = sin(a * x);
    EXPECT_THROW(DecomposeLumpedParameters(Vector1<Expression>(e),
                                           Vector3<Variable>{a, b, c}),
                 std::exception);
  }

  {
    // Test additional symbolic elements.
    const Expression e = x / y + abs(x) + log(x) + exp(x) + sqrt(x) + sin(x) +
                         cos(x) + tan(x) + asin(x) + acos(x) + atan(x) +
                         atan2(x, y) + sinh(x) + cosh(x) + tanh(x) + min(x, y) +
                         max(x, y) + ceil(x) + floor(x);
    // Note: if_then_else(x>0, x, y) doesn't work because Expand is not
    // implemented yet for ifthenelse.
    const auto [W, alpha, w0] = DecomposeLumpedParameters(
        Vector1<Expression>(e), Vector3<Variable>{a, b, c});
    EXPECT_EQ(W.size(), 0);
    EXPECT_EQ(alpha.size(), 0);
    EXPECT_EQ(w0, Vector1<Expression>(e));
  }
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
