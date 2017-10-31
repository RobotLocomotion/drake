#include "drake/solvers/create_constraint.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/symbolic_test_util.h"

using drake::symbolic::Expression;

namespace drake {
namespace solvers {
namespace {

void CheckParseLorentzConeConstraint(const Expression& linear_expression,
                                     const Expression& quadratic_expression) {
  Binding<LorentzConeConstraint> binding = internal::ParseLorentzConeConstraint(
      linear_expression, quadratic_expression);
  const Eigen::MatrixXd A = binding.constraint()->A();
  const Eigen::VectorXd b = binding.constraint()->b();
  const VectorX<Expression> z = A * binding.variables() + b;
  EXPECT_TRUE(symbolic::test::PolynomialEqual(
      symbolic::Polynomial(z(0)), symbolic::Polynomial(linear_expression),
      1E-10));
  EXPECT_TRUE(symbolic::test::PolynomialEqual(
      symbolic::Polynomial(z.tail(z.rows() - 1).squaredNorm()),
      symbolic::Polynomial(quadratic_expression), 1E-10));
}

void CheckParseRotatedLorentzConeConstraint(
    const Eigen::Ref<const VectorX<Expression>>& v) {
  Binding<RotatedLorentzConeConstraint> binding =
      internal::ParseRotatedLorentzConeConstraint(v);
  const Eigen::MatrixXd A = binding.constraint()->A();
  const Eigen::VectorXd b = binding.constraint()->b();
  const VectorX<Expression> z = A * binding.variables() + b;
  for (int i = 0; i < z.rows(); ++i) {
    EXPECT_TRUE(symbolic::test::PolynomialEqual(symbolic::Polynomial(z(i)),
                                    symbolic::Polynomial(v(i)), 1E-10));
  }
}

void CheckParseRotatedLorentzConeConstraint(
    const Expression& linear_expression1, const Expression& linear_expression2,
    const Expression& quadratic_expression) {
  Binding<RotatedLorentzConeConstraint> binding =
      internal::ParseRotatedLorentzConeConstraint(
          linear_expression1, linear_expression2, quadratic_expression);
  const Eigen::MatrixXd A = binding.constraint()->A();
  const Eigen::VectorXd b = binding.constraint()->b();
  const VectorX<Expression> z = A * binding.variables() + b;
  const double tol_check{1E-10};
  EXPECT_TRUE(symbolic::test::PolynomialEqual(symbolic::Polynomial(z(0)),
                                  symbolic::Polynomial(linear_expression1),
                                  tol_check));
  EXPECT_TRUE(symbolic::test::PolynomialEqual(symbolic::Polynomial(z(1)),
                                  symbolic::Polynomial(linear_expression2),
                                  tol_check));
  EXPECT_TRUE(symbolic::test::PolynomialEqual(
      symbolic::Polynomial(
          z.tail(z.rows() - 2).cast<Expression>().squaredNorm()),
      symbolic::Polynomial(quadratic_expression), tol_check));
}

class ParseLorentzConeConstraintTest : public ::testing::Test {
 public:
  ParseLorentzConeConstraintTest()
      : x0_{"x0"}, x1_{"x1"}, x2_{"x2"}, x3_{"x3"} {
    x_ << x0_, x1_, x2_, x3_;
  }

 protected:
  symbolic::Variable x0_;
  symbolic::Variable x1_;
  symbolic::Variable x2_;
  symbolic::Variable x3_;
  Vector4<symbolic::Variable> x_;
};

TEST_F(ParseLorentzConeConstraintTest, Test0) {
  // Test x(0) >= sqrt(x(1)² + x(2)² + ... + x(n-1)²)
  CheckParseLorentzConeConstraint(
      x_(0), x_.tail<3>().cast<Expression>().squaredNorm());
}

TEST_F(ParseLorentzConeConstraintTest, Test1) {
  // Test 2 * x(0) >= sqrt(x(1)² + x(2)² + ... + x(n-1)²)
  CheckParseLorentzConeConstraint(
      2 * x_(0), x_.tail<3>().cast<Expression>().squaredNorm());
}

TEST_F(ParseLorentzConeConstraintTest, Test2) {
  // Test 2 * x(0) + 3 * x(1) + 2 >= sqrt(x(1)² + x(2)² + ... + x(n-1)²)
  CheckParseLorentzConeConstraint(
      2 * x_(0) + 3 * x_(1) + 2, x_.tail<3>().cast<Expression>().squaredNorm());
}

TEST_F(ParseLorentzConeConstraintTest, Test3) {
  // Test x(0) >= sqrt(2x(1)² + x(2)² + 3x(3)² + 2)
  CheckParseLorentzConeConstraint(
      x_(0), 2 * x_(1) * x_(1) + x_(2) * x_(2) + 3 * x_(3) * x_(3) + 2);
}

TEST_F(ParseLorentzConeConstraintTest, Test4) {
  // Test x(0) + 2x(2) + 3 >= sqrt((x(1) + x(0) + 2x(3)-1)² + 3(-x(0)+2x(3)+1)²
  // + 3)
  CheckParseLorentzConeConstraint(x_(0) + 2 * x_(2) + 3,
                                  2 * pow(x_(1) + x_(0) + 2 * x_(3) - 1, 2) +
                                      3 * pow(-x_(0) + 2 * x_(3) + 1, 2) + 3);
}

TEST_F(ParseLorentzConeConstraintTest, Test5) {
  // Test 2 >= sqrt((x(1)+2x(2))² + (x(2)-x(0)+2)² + 3)
  CheckParseLorentzConeConstraint(
      2, pow(x_(1) + 2 * x_(2), 2) + pow(x_(2) - x_(0) + 2, 2) + 3);
}

TEST_F(ParseLorentzConeConstraintTest, Test6) {
  // The linear expression is not actually linear.
  EXPECT_THROW(internal::ParseLorentzConeConstraint(
                   2 * x_(0) * x_(1), x_(0) * x_(0) + x_(1) * x_(1)),
               std::runtime_error);
}

TEST_F(ParseLorentzConeConstraintTest, Test7) {
  // The quadratic expression is actually linear.
  EXPECT_THROW(internal::ParseLorentzConeConstraint(2 * x_(0), x_(0) + x_(1)),
               std::runtime_error);
}

TEST_F(ParseLorentzConeConstraintTest, Test8) {
  // The quadratic expression is actually cubic.
  EXPECT_THROW(
      internal::ParseLorentzConeConstraint(2 * x_(0), pow(x_(1), 3) + x_(2)),
      std::runtime_error);
}

TEST_F(ParseLorentzConeConstraintTest, Test9) {
  // The quadratic expression is not positive semidefinite.
  EXPECT_THROW(internal::ParseLorentzConeConstraint(
                   x_(0), pow(x_(1) + 2 * x_(2) + 1, 2) - x_(3)),
               std::runtime_error);
  EXPECT_THROW(internal::ParseLorentzConeConstraint(
                   x_(0), pow(x_(1) + x_(2) + 2, 2) - 1),
               std::runtime_error);
}

class ParseRotatedLorentzConeConstraintTest : public ::testing::Test {
 public:
  ParseRotatedLorentzConeConstraintTest()
      : x0_{"x0"}, x1_{"x1"}, x2_{"x2"}, x3_{"x3"} {
    x_ << x0_, x1_, x2_, x3_;
  }

 protected:
  symbolic::Variable x0_;
  symbolic::Variable x1_;
  symbolic::Variable x2_;
  symbolic::Variable x3_;
  Vector4<symbolic::Variable> x_;
};

TEST_F(ParseRotatedLorentzConeConstraintTest, Test0) {
  // x(0) * x(1) >= x(2)² + x(3)², x(0) >= 0, x(1) >= 0
  CheckParseRotatedLorentzConeConstraint(x_.cast<Expression>());
  CheckParseRotatedLorentzConeConstraint(x_(0), x_(1),
                                         x_(2) * x_(2) + x_(3) * x_(3));
}

TEST_F(ParseRotatedLorentzConeConstraintTest, Test1) {
  // (x(0) + 2) * (2 * x(1) + x(0) + 1) >= x(2)² + x(3)²
  // x(0) + 2 >= 0
  // 2 * x(1) + x(0) + 1 >= 0
  Vector4<Expression> expression;
  expression << x_(0) + 2, 2 * x_(1) + x_(0) + 1, x_(2), x_(3);
  CheckParseRotatedLorentzConeConstraint(expression);
  CheckParseRotatedLorentzConeConstraint(expression(0), expression(1),
                                         x_(2) * x_(2) + x_(3) * x_(3));
}

TEST_F(ParseRotatedLorentzConeConstraintTest, Test2) {
  // (x(0) - x(1) + 2) * (3 * x(2)) >= (x(1) + x(2))² + (x(2) + 2 * x(3) + 2)² +
  // 5
  Eigen::Matrix<Expression, 5, 1> expression;
  expression << x_(0) - x_(1) + 2, 3 * x_(2), x_(1) + x_(2),
      x_(2) + 2 * x_(3) + 2, std::sqrt(5);
  CheckParseRotatedLorentzConeConstraint(expression);
  CheckParseRotatedLorentzConeConstraint(
      expression(0), expression(1),
      pow(x_(1) + x_(2), 2) + pow(x_(2) + 2 * x_(3) + 2, 2) + 5);
}

TEST_F(ParseRotatedLorentzConeConstraintTest, Test3) {
  // Throw a runtime error when the precondition is not satisfied.
  Eigen::Matrix<Expression, 4, 1> expression;
  expression << 2 * x_(0) * x_(1), x_(2), x_(3), 1;
  EXPECT_THROW(internal::ParseRotatedLorentzConeConstraint(expression),
               std::runtime_error);
  EXPECT_THROW(internal::ParseRotatedLorentzConeConstraint(x_(0) * x_(1), x_(1),
                                                           x_(2) * x_(2) + 1),
               std::runtime_error);
  EXPECT_THROW(internal::ParseRotatedLorentzConeConstraint(x_(1), x_(0) * x_(1),
                                                           x_(2) * x_(2) + 1),
               std::runtime_error);
  EXPECT_THROW(internal::ParseRotatedLorentzConeConstraint(x_(0), x_(1),
                                                           x_(2) * x_(2) - 1),
               std::runtime_error);
}
}  // namespace
}  // namespace solvers
}  // namespace drake
