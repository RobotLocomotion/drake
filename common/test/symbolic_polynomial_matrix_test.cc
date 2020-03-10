#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace symbolic {
namespace {

using test::ExprEqual;

class SymbolicPolynomialMatrixTest : public ::testing::Test {
 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};

  const Monomial m_x_{var_x_};
  const Monomial m_y_{var_y_};
  const Monomial m_z_{var_z_};

  MatrixX<Polynomial> M_poly_dynamic_{2, 2};
  Eigen::Matrix<Polynomial, 2, 2> M_poly_static_;

  MatrixX<Monomial> M_monomial_dynamic_{2, 2};
  Eigen::Matrix<Monomial, 2, 2> M_monomial_static_;

  MatrixX<double> M_double_dynamic_{2, 2};
  Eigen::Matrix<double, 2, 2> M_double_static_;

  VectorX<Polynomial> v_poly_dynamic_{2};
  Eigen::Matrix<Polynomial, 2, 1> v_poly_static_;

  VectorX<Monomial> v_monomial_dynamic_{2};
  Eigen::Matrix<Monomial, 2, 1> v_monomial_static_;

  VectorX<double> v_double_dynamic_{2};
  Eigen::Matrix<double, 2, 1> v_double_static_;

  void SetUp() override {
    // clang-format off
    M_poly_dynamic_ << Polynomial{},    (m_x_ + m_y_),      // [0        x + y]
                      (2 + m_x_ * m_y_), Polynomial{m_z_};  // [2 + xy   z    ]
    M_poly_static_ << Polynomial{},    (m_x_ + m_y_),       // [0        x + y]
                     (2 + m_x_ * m_y_), Polynomial{m_z_};   // [2 + xy   z    ]

    M_monomial_dynamic_ << Monomial{},   m_x_,                  // [1    x  ]
                          (m_x_ * m_y_), (m_x_ * m_x_ * m_z_);  // [xy   x²z]
    M_monomial_static_ << Monomial{},   m_x_,                   // [1    x  ]
                         (m_x_ * m_y_), (m_x_ * m_x_ * m_z_);   // [xy   x²z]

    M_double_dynamic_ << -2.0,  4.0,  // [-2.0   4.0]
                          5.0, -9.0;  // [ 5.0  -9.0]
    M_double_static_ << -2.0,  4.0,   // [-2.0   4.0]
                         5.0, -9.0;   // [ 5.0  -9.0]

    v_poly_dynamic_ << (m_x_ + m_y_),             // [x+y    ]
                       (3 + m_x_ * m_y_ * m_z_);  // [3 + xyz]
    v_poly_static_ << (m_x_ + m_y_),              // [x+y    ]
                      (3 + m_x_ * m_y_ * m_z_);   // [3 + xyz]

    v_monomial_dynamic_ << (m_x_ * m_y_),         // [xy ]
                           (m_x_ * m_y_ * m_z_);  // [xyz]
    v_monomial_static_ << (m_x_ * m_y_),          // [xy ]
                          (m_x_ * m_y_ * m_z_);   // [xyz]

    v_double_dynamic_ << -1.1,  // [-1.1]
                          5.2;  // [ 5.2]
    v_double_static_ << -1.1,   // [-1.1]
                         5.2;   // [ 5.2]
    // clang-format on
  }
};

// Compares m1 and m2 after expanding both of them.
::testing::AssertionResult CompareMatricesWithExpansion(
    const Eigen::Ref<const MatrixX<Expression>>& m1,
    const Eigen::Ref<const MatrixX<Expression>>& m2) {
  const MatrixX<Expression> m1_expanded{
      m1.unaryExpr([](const Expression& e) { return e.Expand(); })};
  const MatrixX<Expression> m2_expanded{
      m2.unaryExpr([](const Expression& e) { return e.Expand(); })};
  if (m1_expanded == m2_expanded) {
    return ::testing::AssertionSuccess()
           << "m1 and m2 are equal after expansion where m1 = \n"
           << m1 << "\n"
           << "and m2 = " << m2;
  } else {
    return ::testing::AssertionFailure()
           << "m1 and m2 are not equal after expansion where m1 = \n"
           << m1 << "\n"
           << "m2 = " << m2 << "\n"
           << "m1_expanded = \n"
           << m1_expanded << "\n"
           << "m2_expanded = \n"
           << m2_expanded;
  }
}

template <typename Matrix1, typename Matrix2>
::testing::AssertionResult CheckAddition(const Matrix1& M1, const Matrix2& M2) {
  return CompareMatricesWithExpansion(
      (M1 + M2).template cast<Expression>(),
      M1.template cast<Expression>() + M2.template cast<Expression>());
}

template <typename Matrix1, typename Matrix2>
::testing::AssertionResult CheckSubtraction(const Matrix1& M1,
                                            const Matrix2& M2) {
  return CompareMatricesWithExpansion(
      (M1 - M2).template cast<Expression>(),
      M1.template cast<Expression>() - M2.template cast<Expression>());
}

template <typename Matrix1, typename Matrix2>
::testing::AssertionResult CheckMultiplication(const Matrix1& M1,
                                               const Matrix2& M2) {
  return CompareMatricesWithExpansion(
      (M1 * M2).template cast<Expression>(),
      M1.template cast<Expression>() * M2.template cast<Expression>());
}

template <typename Vector1, typename Vector2>
bool CheckDotProduct(const Vector1& v1, const Vector2& v2) {
  const Expression e1{v1.dot(v2).ToExpression()};
  const Expression e2{v1.template cast<Expression>()
                          .dot(v2.template cast<Expression>())
                          .Expand()};
  return e1.EqualTo(e2);
}

TEST_F(SymbolicPolynomialMatrixTest, PolynomialPolynomial) {
  // Checks Dynamic op Dynamic where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_poly_dynamic_, M_poly_dynamic_));
  EXPECT_TRUE(CheckSubtraction(M_poly_dynamic_, M_poly_dynamic_));
  EXPECT_TRUE(CheckMultiplication(M_poly_dynamic_, M_poly_dynamic_));
  EXPECT_TRUE(CheckDotProduct(v_poly_dynamic_, v_poly_dynamic_));

  // Checks Static op Dynamic where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_poly_static_, M_poly_dynamic_));
  EXPECT_TRUE(CheckSubtraction(M_poly_static_, M_poly_dynamic_));
  EXPECT_TRUE(CheckMultiplication(M_poly_static_, M_poly_dynamic_));
  EXPECT_TRUE(CheckDotProduct(v_poly_static_, v_poly_dynamic_));

  // Checks Dynamic op Static where op = {+, -, *}.
  EXPECT_TRUE(CheckAddition(M_poly_dynamic_, M_poly_static_));
  EXPECT_TRUE(CheckSubtraction(M_poly_dynamic_, M_poly_static_));
  EXPECT_TRUE(CheckMultiplication(M_poly_dynamic_, M_poly_static_));
  EXPECT_TRUE(CheckDotProduct(v_poly_dynamic_, v_poly_static_));

  // Checks Static op Static where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_poly_static_, M_poly_static_));
  EXPECT_TRUE(CheckSubtraction(M_poly_static_, M_poly_static_));
  EXPECT_TRUE(CheckMultiplication(M_poly_static_, M_poly_static_));
  EXPECT_TRUE(CheckDotProduct(v_poly_static_, v_poly_static_));
}

TEST_F(SymbolicPolynomialMatrixTest, PolynomialMonomial) {
  // Checks Dynamic op Dynamic where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_poly_dynamic_, M_monomial_dynamic_));
  EXPECT_TRUE(CheckSubtraction(M_poly_dynamic_, M_monomial_dynamic_));
  EXPECT_TRUE(CheckMultiplication(M_poly_dynamic_, M_monomial_dynamic_));
  EXPECT_TRUE(CheckDotProduct(v_poly_dynamic_, v_monomial_dynamic_));

  // Checks Static op Dynamic where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_poly_static_, M_monomial_dynamic_));
  EXPECT_TRUE(CheckSubtraction(M_poly_static_, M_monomial_dynamic_));
  EXPECT_TRUE(CheckMultiplication(M_poly_static_, M_monomial_dynamic_));
  EXPECT_TRUE(CheckDotProduct(v_poly_static_, v_monomial_dynamic_));

  // Checks Dynamic op Static where op = {+, -, *}.
  EXPECT_TRUE(CheckAddition(M_poly_dynamic_, M_monomial_static_));
  EXPECT_TRUE(CheckSubtraction(M_poly_dynamic_, M_monomial_static_));
  EXPECT_TRUE(CheckMultiplication(M_poly_dynamic_, M_monomial_static_));
  EXPECT_TRUE(CheckDotProduct(v_poly_dynamic_, v_monomial_static_));

  // Checks Static op Static where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_poly_static_, M_monomial_static_));
  EXPECT_TRUE(CheckSubtraction(M_poly_static_, M_monomial_static_));
  EXPECT_TRUE(CheckMultiplication(M_poly_static_, M_monomial_static_));
  EXPECT_TRUE(CheckDotProduct(v_poly_static_, v_monomial_static_));
}

TEST_F(SymbolicPolynomialMatrixTest, PolynomialDouble) {
  // Checks Dynamic op Dynamic where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_poly_dynamic_, M_double_dynamic_));
  EXPECT_TRUE(CheckSubtraction(M_poly_dynamic_, M_double_dynamic_));
  EXPECT_TRUE(CheckMultiplication(M_poly_dynamic_, M_double_dynamic_));
  EXPECT_TRUE(CheckDotProduct(v_poly_dynamic_, v_double_dynamic_));

  // Checks Static op Dynamic where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_poly_static_, M_double_dynamic_));
  EXPECT_TRUE(CheckSubtraction(M_poly_static_, M_double_dynamic_));
  EXPECT_TRUE(CheckMultiplication(M_poly_static_, M_double_dynamic_));
  EXPECT_TRUE(CheckDotProduct(v_poly_static_, v_double_dynamic_));

  // Checks Dynamic op Static where op = {+, -, *}.
  EXPECT_TRUE(CheckAddition(M_poly_dynamic_, M_double_static_));
  EXPECT_TRUE(CheckSubtraction(M_poly_dynamic_, M_double_static_));
  EXPECT_TRUE(CheckMultiplication(M_poly_dynamic_, M_double_static_));
  EXPECT_TRUE(CheckDotProduct(v_poly_dynamic_, v_double_static_));

  // Checks Static op Static where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_poly_static_, M_double_static_));
  EXPECT_TRUE(CheckSubtraction(M_poly_static_, M_double_static_));
  EXPECT_TRUE(CheckMultiplication(M_poly_static_, M_double_static_));
  EXPECT_TRUE(CheckDotProduct(v_poly_static_, v_double_static_));
}

TEST_F(SymbolicPolynomialMatrixTest, MonomialPolynomial) {
  // Checks Dynamic op Dynamic where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_monomial_dynamic_, M_poly_dynamic_));
  EXPECT_TRUE(CheckSubtraction(M_monomial_dynamic_, M_poly_dynamic_));
  EXPECT_TRUE(CheckMultiplication(M_monomial_dynamic_, M_poly_dynamic_));
  EXPECT_TRUE(CheckDotProduct(v_monomial_dynamic_, v_poly_dynamic_));

  // Checks Static op Dynamic where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_monomial_static_, M_poly_dynamic_));
  EXPECT_TRUE(CheckSubtraction(M_monomial_static_, M_poly_dynamic_));
  EXPECT_TRUE(CheckMultiplication(M_monomial_static_, M_poly_dynamic_));
  EXPECT_TRUE(CheckDotProduct(v_monomial_static_, v_poly_dynamic_));

  // Checks Dynamic op Static where op = {+, -, *}.
  EXPECT_TRUE(CheckAddition(M_monomial_dynamic_, M_poly_static_));
  EXPECT_TRUE(CheckSubtraction(M_monomial_dynamic_, M_poly_static_));
  EXPECT_TRUE(CheckMultiplication(M_monomial_dynamic_, M_poly_static_));
  EXPECT_TRUE(CheckDotProduct(v_monomial_dynamic_, v_poly_static_));

  // Checks Static op Static where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_monomial_static_, M_poly_static_));
  EXPECT_TRUE(CheckSubtraction(M_monomial_static_, M_poly_static_));
  EXPECT_TRUE(CheckMultiplication(M_monomial_static_, M_poly_static_));
  EXPECT_TRUE(CheckDotProduct(v_monomial_static_, v_poly_static_));
}

TEST_F(SymbolicPolynomialMatrixTest, MonomialMonomial) {
  // Checks Dynamic op Dynamic where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_monomial_dynamic_, M_monomial_dynamic_));
  EXPECT_TRUE(CheckSubtraction(M_monomial_dynamic_, M_monomial_dynamic_));
  EXPECT_TRUE(CheckMultiplication(M_monomial_dynamic_, M_monomial_dynamic_));
  EXPECT_TRUE(CheckDotProduct(v_monomial_dynamic_, v_monomial_dynamic_));

  // Checks Static op Dynamic where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_monomial_static_, M_monomial_dynamic_));
  EXPECT_TRUE(CheckSubtraction(M_monomial_static_, M_monomial_dynamic_));
  EXPECT_TRUE(CheckMultiplication(M_monomial_static_, M_monomial_dynamic_));
  EXPECT_TRUE(CheckDotProduct(v_monomial_static_, v_monomial_dynamic_));

  // Checks Dynamic op Static where op = {+, -, *}.
  EXPECT_TRUE(CheckAddition(M_monomial_dynamic_, M_monomial_static_));
  EXPECT_TRUE(CheckSubtraction(M_monomial_dynamic_, M_monomial_static_));
  EXPECT_TRUE(CheckMultiplication(M_monomial_dynamic_, M_monomial_static_));
  EXPECT_TRUE(CheckDotProduct(v_monomial_dynamic_, v_monomial_static_));

  // Checks Static op Static where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_monomial_static_, M_monomial_static_));
  EXPECT_TRUE(CheckSubtraction(M_monomial_static_, M_monomial_static_));
  EXPECT_TRUE(CheckMultiplication(M_monomial_static_, M_monomial_static_));
  EXPECT_TRUE(CheckDotProduct(v_monomial_static_, v_monomial_static_));
}

TEST_F(SymbolicPolynomialMatrixTest, MonomialDouble) {
  // Checks Dynamic op Dynamic where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_monomial_dynamic_, M_double_dynamic_));
  EXPECT_TRUE(CheckSubtraction(M_monomial_dynamic_, M_double_dynamic_));
  EXPECT_TRUE(CheckMultiplication(M_monomial_dynamic_, M_double_dynamic_));
  EXPECT_TRUE(CheckDotProduct(v_monomial_dynamic_, v_double_dynamic_));

  // Checks Static op Dynamic where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_monomial_static_, M_double_dynamic_));
  EXPECT_TRUE(CheckSubtraction(M_monomial_static_, M_double_dynamic_));
  EXPECT_TRUE(CheckMultiplication(M_monomial_static_, M_double_dynamic_));
  EXPECT_TRUE(CheckDotProduct(v_monomial_static_, v_double_dynamic_));

  // Checks Dynamic op Static where op = {+, -, *}.
  EXPECT_TRUE(CheckAddition(M_monomial_dynamic_, M_double_static_));
  EXPECT_TRUE(CheckSubtraction(M_monomial_dynamic_, M_double_static_));
  EXPECT_TRUE(CheckMultiplication(M_monomial_dynamic_, M_double_static_));
  EXPECT_TRUE(CheckDotProduct(v_monomial_dynamic_, v_double_static_));

  // Checks Static op Static where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_monomial_static_, M_double_static_));
  EXPECT_TRUE(CheckSubtraction(M_monomial_static_, M_double_static_));
  EXPECT_TRUE(CheckMultiplication(M_monomial_static_, M_double_static_));
  EXPECT_TRUE(CheckDotProduct(v_monomial_static_, v_double_static_));
}

TEST_F(SymbolicPolynomialMatrixTest, doublePolynomial) {
  // Checks Dynamic op Dynamic where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_double_dynamic_, M_poly_dynamic_));
  EXPECT_TRUE(CheckSubtraction(M_double_dynamic_, M_poly_dynamic_));
  EXPECT_TRUE(CheckMultiplication(M_double_dynamic_, M_poly_dynamic_));
  EXPECT_TRUE(CheckDotProduct(v_double_dynamic_, v_poly_dynamic_));

  // Checks Static op Dynamic where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_double_static_, M_poly_dynamic_));
  EXPECT_TRUE(CheckSubtraction(M_double_static_, M_poly_dynamic_));
  EXPECT_TRUE(CheckMultiplication(M_double_static_, M_poly_dynamic_));
  EXPECT_TRUE(CheckDotProduct(v_double_static_, v_poly_dynamic_));

  // Checks Dynamic op Static where op = {+, -, *}.
  EXPECT_TRUE(CheckAddition(M_double_dynamic_, M_poly_static_));
  EXPECT_TRUE(CheckSubtraction(M_double_dynamic_, M_poly_static_));
  EXPECT_TRUE(CheckMultiplication(M_double_dynamic_, M_poly_static_));
  EXPECT_TRUE(CheckDotProduct(v_double_dynamic_, v_poly_static_));

  // Checks Static op Static where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_double_static_, M_poly_static_));
  EXPECT_TRUE(CheckSubtraction(M_double_static_, M_poly_static_));
  EXPECT_TRUE(CheckMultiplication(M_double_static_, M_poly_static_));
  EXPECT_TRUE(CheckDotProduct(v_double_static_, v_poly_static_));
}

TEST_F(SymbolicPolynomialMatrixTest, doubleMonomial) {
  // Checks Dynamic op Dynamic where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_double_dynamic_, M_monomial_dynamic_));
  EXPECT_TRUE(CheckSubtraction(M_double_dynamic_, M_monomial_dynamic_));
  EXPECT_TRUE(CheckMultiplication(M_double_dynamic_, M_monomial_dynamic_));
  EXPECT_TRUE(CheckDotProduct(v_double_dynamic_, v_monomial_dynamic_));

  // Checks Static op Dynamic where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_double_static_, M_monomial_dynamic_));
  EXPECT_TRUE(CheckSubtraction(M_double_static_, M_monomial_dynamic_));
  EXPECT_TRUE(CheckMultiplication(M_double_static_, M_monomial_dynamic_));
  EXPECT_TRUE(CheckDotProduct(v_double_static_, v_monomial_dynamic_));

  // Checks Dynamic op Static where op = {+, -, *}.
  EXPECT_TRUE(CheckAddition(M_double_dynamic_, M_monomial_static_));
  EXPECT_TRUE(CheckSubtraction(M_double_dynamic_, M_monomial_static_));
  EXPECT_TRUE(CheckMultiplication(M_double_dynamic_, M_monomial_static_));
  EXPECT_TRUE(CheckDotProduct(v_double_dynamic_, v_monomial_static_));

  // Checks Static op Static where op = {+, -, *, ·}.
  EXPECT_TRUE(CheckAddition(M_double_static_, M_monomial_static_));
  EXPECT_TRUE(CheckSubtraction(M_double_static_, M_monomial_static_));
  EXPECT_TRUE(CheckMultiplication(M_double_static_, M_monomial_static_));
  EXPECT_TRUE(CheckDotProduct(v_double_static_, v_monomial_static_));
}

TEST_F(SymbolicPolynomialMatrixTest, EvaluateMatrix) {
  const Environment env{{{var_x_, 1.0}, {var_y_, 2.0}, {var_z_, 3.0}}};

  EXPECT_EQ(Evaluate(M_poly_dynamic_, env),
            Evaluate(M_poly_dynamic_.cast<Expression>(), env));

  EXPECT_EQ(Evaluate(M_poly_static_, env),
            Evaluate(M_poly_static_.cast<Expression>(), env));

  EXPECT_EQ(Evaluate(v_poly_dynamic_, env),
            Evaluate(v_poly_dynamic_.cast<Expression>(), env));

  EXPECT_EQ(Evaluate(v_poly_static_, env),
            Evaluate(v_poly_static_.cast<Expression>(), env));
}

TEST_F(SymbolicPolynomialMatrixTest, Jacobian) {
  const Vector3<Variable> vars{var_x_, var_y_, var_z_};
  {
    const MatrixX<Polynomial> result{Jacobian(v_poly_dynamic_, vars)};
    for (int i = 0; i < v_poly_dynamic_.size(); ++i) {
      for (int j = 0; j < vars.size(); ++j) {
        const Polynomial p1{v_poly_dynamic_(i).Differentiate(vars(j))};
        const Polynomial& p2{result(i, j)};
        EXPECT_TRUE(p1.EqualTo(p2));
      }
    }
  }
  {
    const MatrixX<Polynomial> result{Jacobian(v_poly_static_, vars)};
    for (int i = 0; i < v_poly_static_.size(); ++i) {
      for (int j = 0; j < vars.size(); ++j) {
        const Polynomial p1{v_poly_static_(i).Differentiate(vars(j))};
        const Polynomial& p2{result(i, j)};
        EXPECT_TRUE(p1.EqualTo(p2));
      }
    }
  }
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
