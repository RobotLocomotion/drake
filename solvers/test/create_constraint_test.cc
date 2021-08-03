#include "drake/solvers/create_constraint.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

using drake::symbolic::Expression;

namespace drake {
namespace solvers {
namespace {

void CheckParseQuadraticConstraint(const Expression& e, double lb, double ub) {
  Binding<QuadraticConstraint> binding =
      internal::ParseQuadraticConstraint(e, lb, ub);

  const Expression binding_expression{
      0.5 *
          binding.variables().dot(binding.evaluator()->Q() *
                                  binding.variables()) +
      binding.evaluator()->b().dot(binding.variables())};
  if (!std::isinf(lb)) {
    EXPECT_TRUE(symbolic::test::PolynomialEqual(
        symbolic::Polynomial(e - lb),
        symbolic::Polynomial(binding_expression -
                             binding.evaluator()->lower_bound()(0)),
        1E-10));
  } else {
    EXPECT_TRUE(std::isinf(binding.evaluator()->lower_bound()(0)));
  }
  if (!std::isinf(ub)) {
    EXPECT_TRUE(symbolic::test::PolynomialEqual(
        symbolic::Polynomial(e - ub),
        symbolic::Polynomial(binding_expression -
                             binding.evaluator()->upper_bound()(0)),
        1E-10));
  } else {
    EXPECT_TRUE(std::isinf(binding.evaluator()->upper_bound()(0)));
  }
}

class ParseQuadraticConstraintTest : public ::testing::Test {
 public:
  ParseQuadraticConstraintTest() {
    x_ << x0_, x1_;
  }

 protected:
  symbolic::Variable x0_{"x0"};
  symbolic::Variable x1_{"x1"};
  Vector2<symbolic::Variable> x_;
};

TEST_F(ParseQuadraticConstraintTest, Test0) {
  const double kInf = std::numeric_limits<double>::infinity();
  CheckParseQuadraticConstraint(x0_ * x0_, 1, 1);
  CheckParseQuadraticConstraint(x0_ * x1_, 1, 1);
  CheckParseQuadraticConstraint(x0_ * x0_ + 2 * x0_, 0, 2);
  CheckParseQuadraticConstraint(x0_ * x0_ + 2 * x0_ + 3, 0, 2);

  CheckParseQuadraticConstraint(x0_ * x0_ + 2 * x0_ * x1_ + 4 * x1_ * x1_,
                                -kInf, 1);
  CheckParseQuadraticConstraint(x0_ * x0_ + 2 * x0_ * x1_ + 4 * x1_ * x1_, 1,
                                kInf);
  CheckParseQuadraticConstraint(x0_ * x0_ + 2 * x0_ * x1_ + 4 * x1_ * x1_ + 2,
                                1, kInf);
  CheckParseQuadraticConstraint(x0_ * x0_ + 2 * x0_ * x1_ + 4 * x1_ * x1_ + 2,
                                -kInf, 3);
}

void CheckParseLorentzConeConstraint(
    const Expression& linear_expression, const Expression& quadratic_expression,
    LorentzConeConstraint::EvalType eval_type) {
  Binding<LorentzConeConstraint> binding = internal::ParseLorentzConeConstraint(
      linear_expression, quadratic_expression, 0., eval_type);
  EXPECT_EQ(binding.evaluator()->eval_type(), eval_type);
  const Eigen::MatrixXd A = binding.evaluator()->A();
  const Eigen::VectorXd b = binding.evaluator()->b();
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
  const Eigen::MatrixXd A = binding.evaluator()->A();
  const Eigen::VectorXd b = binding.evaluator()->b();
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
  const Eigen::MatrixXd A = binding.evaluator()->A();
  const Eigen::VectorXd b = binding.evaluator()->b();
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
      x_(0), x_.tail<3>().cast<Expression>().squaredNorm(),
      LorentzConeConstraint::EvalType::kConvexSmooth);
}

TEST_F(ParseLorentzConeConstraintTest, Test1) {
  // Test 2 * x(0) >= sqrt(x(1)² + x(2)² + ... + x(n-1)²)
  CheckParseLorentzConeConstraint(
      2 * x_(0), x_.tail<3>().cast<Expression>().squaredNorm(),
      LorentzConeConstraint::EvalType::kConvexSmooth);
}

TEST_F(ParseLorentzConeConstraintTest, Test2) {
  // Test 2 * x(0) + 3 * x(1) + 2 >= sqrt(x(1)² + x(2)² + ... + x(n-1)²)
  CheckParseLorentzConeConstraint(2 * x_(0) + 3 * x_(1) + 2,
                                  x_.tail<3>().cast<Expression>().squaredNorm(),
                                  LorentzConeConstraint::EvalType::kConvex);
}

TEST_F(ParseLorentzConeConstraintTest, Test3) {
  // Test x(0) >= sqrt(2x(1)² + x(2)² + 3x(3)² + 2)
  CheckParseLorentzConeConstraint(
      x_(0), 2 * x_(1) * x_(1) + x_(2) * x_(2) + 3 * x_(3) * x_(3) + 2,
      LorentzConeConstraint::EvalType::kConvex);
}

TEST_F(ParseLorentzConeConstraintTest, Test4) {
  // Test x(0) + 2x(2) + 3 >= sqrt((x(1) + x(0) + 2x(3)-1)² + 3(-x(0)+2x(3)+1)²
  // + 3)
  CheckParseLorentzConeConstraint(x_(0) + 2 * x_(2) + 3,
                                  2 * pow(x_(1) + x_(0) + 2 * x_(3) - 1, 2) +
                                      3 * pow(-x_(0) + 2 * x_(3) + 1, 2) + 3,
                                  LorentzConeConstraint::EvalType::kConvex);
}

TEST_F(ParseLorentzConeConstraintTest, Test5) {
  // Test 2 >= sqrt((x(1)+2x(2))² + (x(2)-x(0)+2)² + 3)
  CheckParseLorentzConeConstraint(
      2, pow(x_(1) + 2 * x_(2), 2) + pow(x_(2) - x_(0) + 2, 2) + 3,
      LorentzConeConstraint::EvalType::kConvex);
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

class MaybeParseLinearConstraintTest : public ::testing::Test {
 public:
  MaybeParseLinearConstraintTest() {}

 protected:
  const symbolic::Variable x0_{"x0"};
  const symbolic::Variable x1_{"x1"};
  const symbolic::Variable x2_{"x2"};
  const Vector3<symbolic::Variable> x_{x0_, x1_, x2_};
};

TEST_F(MaybeParseLinearConstraintTest, TestBoundingBoxConstraint) {
  // Parse a bounding box constraint
  auto check_bounding_box_constraint = [](
      const Binding<Constraint>& constraint, double lower_expected,
      double upper_expected, const symbolic::Variable& var) {
    const Binding<BoundingBoxConstraint> bounding_box_constraint =
        internal::BindingDynamicCast<BoundingBoxConstraint>(constraint);
    EXPECT_EQ(bounding_box_constraint.variables().size(), 1);
    EXPECT_EQ(bounding_box_constraint.variables()(0), var);
    EXPECT_EQ(bounding_box_constraint.evaluator()->num_constraints(), 1);
    EXPECT_EQ(bounding_box_constraint.evaluator()->lower_bound()(0),
              lower_expected);
    EXPECT_EQ(bounding_box_constraint.evaluator()->upper_bound()(0),
              upper_expected);
  };

  check_bounding_box_constraint(
      *(internal::MaybeParseLinearConstraint(x0_, 1, 2)), 1, 2, x0_);
  check_bounding_box_constraint(
      *(internal::MaybeParseLinearConstraint(-x0_, 1, 2)), -2, -1, x0_);
  check_bounding_box_constraint(
      *(internal::MaybeParseLinearConstraint(2 * x0_, 1, 2)), 0.5, 1, x0_);
  check_bounding_box_constraint(
      *(internal::MaybeParseLinearConstraint(-2 * x0_, 1, 2)), -1, -0.5, x0_);
  check_bounding_box_constraint(
      *(internal::MaybeParseLinearConstraint(x0_ + 1, 1, 2)), 0, 1, x0_);
  check_bounding_box_constraint(
      *(internal::MaybeParseLinearConstraint(x0_ + 1 + 2, 1, 2)), -2, -1, x0_);
  check_bounding_box_constraint(
      *(internal::MaybeParseLinearConstraint(2 * x0_ + 1 + 2, 1, 2)), -1, -0.5,
      x0_);
  check_bounding_box_constraint(
      *(internal::MaybeParseLinearConstraint(-2 * x0_ + 1 + 2, 1, 2)), 0.5, 1,
      x0_);
}

TEST_F(MaybeParseLinearConstraintTest, TestLinearEqualityConstraint) {
  // Parse linear equality constraint.
  auto check_linear_equality_constraint = [](
      const Binding<Constraint>& constraint,
      const Eigen::Ref<const Eigen::RowVectorXd>& a,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& vars, double bound,
      double tol = 1E-14) {
    const Binding<LinearEqualityConstraint> linear_eq_constraint =
        internal::BindingDynamicCast<LinearEqualityConstraint>(constraint);
    if (vars.size() != 0) {
      EXPECT_EQ(linear_eq_constraint.variables(), vars);
    } else {
      EXPECT_EQ(linear_eq_constraint.variables().size(), 0);
    }
    EXPECT_EQ(linear_eq_constraint.evaluator()->num_constraints(), 1);
    EXPECT_TRUE(CompareMatrices(linear_eq_constraint.evaluator()->A(), a, tol));
    EXPECT_NEAR(linear_eq_constraint.evaluator()->lower_bound()(0), bound, tol);
  };

  // without a constant term in the expression.
  check_linear_equality_constraint(
      *internal::MaybeParseLinearConstraint(x_(0) + 2 * x_(1), 1, 1),
      Eigen::RowVector2d(1, 2), Vector2<symbolic::Variable>(x_(0), x_(1)), 1);

  // with a constant term in the expression.
  check_linear_equality_constraint(
      *internal::MaybeParseLinearConstraint(x_(0) + 2 * x_(1) + 2 + 1, 3, 3),
      Eigen::RowVector2d(1, 2), Vector2<symbolic::Variable>(x_(0), x_(1)), 0);

  // Check a constant expression.
  check_linear_equality_constraint(
      *internal::MaybeParseLinearConstraint(2, 3, 3), Eigen::RowVectorXd(0),
      VectorX<symbolic::Variable>(0), 1);
}

TEST_F(MaybeParseLinearConstraintTest, TestLinearConstraint) {
  // Parse linear inequality constraint.
  auto check_linear_constraint = [](
      const Binding<Constraint>& constraint,
      const Eigen::Ref<const Eigen::RowVectorXd>& a,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& vars, double lb,
      double ub, double tol = 1E-14) {
    const Binding<LinearConstraint> linear_constraint =
        internal::BindingDynamicCast<LinearConstraint>(constraint);
    if (vars.size() != 0) {
      EXPECT_EQ(linear_constraint.variables(), vars);
    } else {
      EXPECT_EQ(linear_constraint.variables().size(), 0);
    }
    EXPECT_EQ(linear_constraint.evaluator()->num_constraints(), 1);
    EXPECT_TRUE(CompareMatrices(linear_constraint.evaluator()->A(), a, tol));
    EXPECT_NEAR(linear_constraint.evaluator()->lower_bound()(0), lb, tol);
    EXPECT_NEAR(linear_constraint.evaluator()->upper_bound()(0), ub, tol);
  };

  // without a constant term in the expression.
  check_linear_constraint(
      *internal::MaybeParseLinearConstraint(x_(0) + 2 * x_(1), 1, 2),
      Eigen::RowVector2d(1, 2), Vector2<symbolic::Variable>(x_(0), x_(1)), 1,
      2);

  // with a constant term in the expression.
  check_linear_constraint(
      *internal::MaybeParseLinearConstraint(x_(0) + 2 * x_(1) + 2 + 1, 3, 4),
      Eigen::RowVector2d(1, 2), Vector2<symbolic::Variable>(x_(0), x_(1)), 0,
      1);

  // Check a constant expression.
  check_linear_constraint(*internal::MaybeParseLinearConstraint(2, 3, 4),
                          Eigen::RowVectorXd(0), VectorX<symbolic::Variable>(0),
                          1, 2);
  check_linear_constraint(
      *internal::MaybeParseLinearConstraint(0 * x_(0) + 2, 3, 4),
      Eigen::RowVectorXd(0), VectorX<symbolic::Variable>(0), 1, 2);
}

TEST_F(MaybeParseLinearConstraintTest, NonlinearConstraint) {
  EXPECT_EQ(internal::MaybeParseLinearConstraint(x_(0) * x_(0), 1, 2).get(),
            nullptr);
  EXPECT_EQ(internal::MaybeParseLinearConstraint(sin(x_(0)), 1, 2).get(),
            nullptr);
}
}  // namespace
}  // namespace solvers
}  // namespace drake
