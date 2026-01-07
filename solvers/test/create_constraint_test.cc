#include "drake/solvers/create_constraint.h"

#include <limits>
#include <memory>
#include <set>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/solvers/constraint.h"

using drake::symbolic::Expression;

namespace drake {
namespace solvers {
namespace {

using Eigen::Vector2d;
using symbolic::Variable;

const double kInf = std::numeric_limits<double>::infinity();

void CheckParseQuadraticConstraint(
    const Expression& e, double lb, double ub,
    std::optional<QuadraticConstraint::HessianType> hessian_type,
    QuadraticConstraint::HessianType hessian_type_expected) {
  Binding<QuadraticConstraint> binding =
      internal::ParseQuadraticConstraint(e, lb, ub, hessian_type);

  const Expression binding_expression{
      0.5 * binding.variables().dot(binding.evaluator()->Q() *
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
  EXPECT_EQ(binding.evaluator()->hessian_type(), hessian_type_expected);
}

class ParseQuadraticConstraintTest : public ::testing::Test {
 public:
  ParseQuadraticConstraintTest() { x_ << x0_, x1_; }

 protected:
  symbolic::Variable x0_{"x0"};
  symbolic::Variable x1_{"x1"};
  Vector2<symbolic::Variable> x_;
};

TEST_F(ParseQuadraticConstraintTest, Test0) {
  CheckParseQuadraticConstraint(
      x0_ * x0_, 1, 1, QuadraticConstraint::HessianType::kPositiveSemidefinite,
      QuadraticConstraint::HessianType::kPositiveSemidefinite);
  CheckParseQuadraticConstraint(x0_ * x1_, 1, 1,
                                QuadraticConstraint::HessianType::kIndefinite,
                                QuadraticConstraint::HessianType::kIndefinite);
  CheckParseQuadraticConstraint(
      x0_ * x0_ + 2 * x0_, 0, 2,
      QuadraticConstraint::HessianType::kPositiveSemidefinite,
      QuadraticConstraint::HessianType::kPositiveSemidefinite);
  CheckParseQuadraticConstraint(
      x0_ * x0_ + 2 * x0_ + 3, 0, 2, std::nullopt,
      QuadraticConstraint::HessianType::kPositiveSemidefinite);

  CheckParseQuadraticConstraint(
      x0_ * x0_ + 2 * x0_ * x1_ + 4 * x1_ * x1_, -kInf, 1, std::nullopt,
      QuadraticConstraint::HessianType::kPositiveSemidefinite);
  CheckParseQuadraticConstraint(
      x0_ * x0_ + 2 * x0_ * x1_ + 4 * x1_ * x1_, 1, kInf, std::nullopt,
      QuadraticConstraint::HessianType::kPositiveSemidefinite);
  CheckParseQuadraticConstraint(
      x0_ * x0_ + 2 * x0_ * x1_ + 4 * x1_ * x1_ + 2, 1, kInf, std::nullopt,
      QuadraticConstraint::HessianType::kPositiveSemidefinite);
  CheckParseQuadraticConstraint(
      -x0_ * x0_ + 2 * x0_ * x1_ - 4 * x1_ * x1_ + 2, -kInf, 3, std::nullopt,
      QuadraticConstraint::HessianType::kNegativeSemidefinite);
  CheckParseQuadraticConstraint(
      -x0_ * x0_ + 2 * x1_, -kInf, 3, std::nullopt,
      QuadraticConstraint::HessianType::kNegativeSemidefinite);

  // Special case that the Hessian is a zero-matrix.
  CheckParseQuadraticConstraint(-x0_ + 2 * x1_, -kInf, 3, std::nullopt,
                                QuadraticConstraint::HessianType::kZero);

  // Special case that the Hessian has trace 0.
  CheckParseQuadraticConstraint(-x0_ * x1_ + 2 * x1_, -kInf, 3, std::nullopt,
                                QuadraticConstraint::HessianType::kIndefinite);
}

void CheckParseLorentzConeConstraint(
    const symbolic::Formula& f, LorentzConeConstraint::EvalType eval_type) {
  Binding<LorentzConeConstraint> binding =
      internal::ParseLorentzConeConstraint(f, eval_type);
  EXPECT_EQ(binding.evaluator()->eval_type(), eval_type);
  const Eigen::MatrixXd A = binding.evaluator()->A();
  const Eigen::VectorXd b = binding.evaluator()->b();
  const VectorX<Expression> z = A * binding.variables() + b;
  Expression greater, lesser;
  if (is_greater_than_or_equal_to(f)) {
    greater = get_lhs_expression(f);
    lesser = get_rhs_expression(f);
  } else {
    ASSERT_TRUE(is_less_than_or_equal_to(f));
    greater = get_rhs_expression(f);
    lesser = get_lhs_expression(f);
  }
  EXPECT_TRUE(symbolic::test::PolynomialEqual(
      symbolic::Polynomial(z(0)), symbolic::Polynomial(greater), 1E-10));
  ASSERT_TRUE(is_sqrt(lesser));
  EXPECT_TRUE(symbolic::test::PolynomialEqual(
      symbolic::Polynomial(z.tail(z.rows() - 1).squaredNorm()),
      symbolic::Polynomial(get_argument(lesser)), 1E-10));
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
    EXPECT_TRUE(symbolic::test::PolynomialEqual(
        symbolic::Polynomial(z(i)), symbolic::Polynomial(v(i)), 1E-10));
  }
}

void CheckParseRotatedLorentzConeConstraint(
    const Expression& linear_expression1, const Expression& linear_expression2,
    const Expression& quadratic_expression, const double tol = 0) {
  Binding<RotatedLorentzConeConstraint> binding =
      internal::ParseRotatedLorentzConeConstraint(
          linear_expression1, linear_expression2, quadratic_expression, tol);
  const Eigen::MatrixXd A = binding.evaluator()->A();
  const Eigen::VectorXd b = binding.evaluator()->b();
  const VectorX<Expression> z = A * binding.variables() + b;
  const double tol_check{1E-10};
  EXPECT_TRUE(symbolic::test::PolynomialEqual(
      symbolic::Polynomial(z(0)), symbolic::Polynomial(linear_expression1),
      tol_check));
  EXPECT_TRUE(symbolic::test::PolynomialEqual(
      symbolic::Polynomial(z(1)), symbolic::Polynomial(linear_expression2),
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

TEST_F(ParseLorentzConeConstraintTest, FromFormula) {
  // Test x(0) >= sqrt(x(1)² + x(2)² + ... + x(n-1)²)
  CheckParseLorentzConeConstraint(
      x_(0) >= x_.tail<3>().cast<Expression>().norm(),
      LorentzConeConstraint::EvalType::kConvexSmooth);

  // The order of the arguments should not matter.
  CheckParseLorentzConeConstraint(
      x_.tail<3>().cast<Expression>().norm() <= x_(0),
      LorentzConeConstraint::EvalType::kConvexSmooth);
}

TEST_F(ParseLorentzConeConstraintTest, FromFormulaFail) {
  // Not >= nor <-=.
  DRAKE_EXPECT_THROWS_MESSAGE(
      unused(internal::ParseLorentzConeConstraint(
          x_(0) == x_.tail<3>().cast<Expression>().norm())),
      ".*relational formula.*");

  // The greater side is not affine.
  DRAKE_EXPECT_THROWS_MESSAGE(
      unused(internal::ParseLorentzConeConstraint(
          x_(0) * x_(1) >= x_.tail<2>().cast<Expression>().norm())),
      ".*non-linear.*");

  // The lesser side is not an L2 norm.
  DRAKE_EXPECT_THROWS_MESSAGE(
      unused(internal::ParseLorentzConeConstraint(
          x_(0) >= x_.tail<3>().cast<Expression>().squaredNorm())),
      ".*L2 norm.*");
}

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
  EXPECT_THROW(unused(internal::ParseLorentzConeConstraint(
                   2 * x_(0) * x_(1), x_(0) * x_(0) + x_(1) * x_(1))),
               std::runtime_error);
}

TEST_F(ParseLorentzConeConstraintTest, Test7) {
  // The quadratic expression is actually linear.
  EXPECT_THROW(
      unused(internal::ParseLorentzConeConstraint(2 * x_(0), x_(0) + x_(1))),
      std::runtime_error);
}

TEST_F(ParseLorentzConeConstraintTest, Test8) {
  // The quadratic expression is actually cubic.
  EXPECT_THROW(unused(internal::ParseLorentzConeConstraint(
                   2 * x_(0), pow(x_(1), 3) + x_(2))),
               std::runtime_error);
}

TEST_F(ParseLorentzConeConstraintTest, Test9) {
  // The quadratic expression is not positive semidefinite.
  EXPECT_THROW(unused(internal::ParseLorentzConeConstraint(
                   x_(0), pow(x_(1) + 2 * x_(2) + 1, 2) - x_(3))),
               std::runtime_error);
  EXPECT_THROW(unused(internal::ParseLorentzConeConstraint(
                   x_(0), pow(x_(1) + x_(2) + 2, 2) - 1)),
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
      pow(x_(1) + x_(2), 2) + pow(x_(2) + 2 * x_(3) + 2, 2) + 5,
      4 * std::numeric_limits<double>::epsilon());
}

TEST_F(ParseRotatedLorentzConeConstraintTest, Test3) {
  // Throw a runtime error when the precondition is not satisfied.
  Eigen::Matrix<Expression, 4, 1> expression;
  expression << 2 * x_(0) * x_(1), x_(2), x_(3), 1;
  EXPECT_THROW(unused(internal::ParseRotatedLorentzConeConstraint(expression)),
               std::runtime_error);
  EXPECT_THROW(unused(internal::ParseRotatedLorentzConeConstraint(
                   x_(0) * x_(1), x_(1), x_(2) * x_(2) + 1)),
               std::runtime_error);
  EXPECT_THROW(unused(internal::ParseRotatedLorentzConeConstraint(
                   x_(1), x_(0) * x_(1), x_(2) * x_(2) + 1)),
               std::runtime_error);
  EXPECT_THROW(unused(internal::ParseRotatedLorentzConeConstraint(
                   x_(0), x_(1), x_(2) * x_(2) - 1)),
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
  auto check_bounding_box_constraint =
      [](const Binding<Constraint>& constraint, double lower_expected,
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
  auto check_linear_equality_constraint =
      [](const Binding<Constraint>& constraint,
         const Eigen::Ref<const Eigen::RowVectorXd>& a,
         const Eigen::Ref<const VectorX<symbolic::Variable>>& vars,
         double bound, double tol = 1E-14) {
        const Binding<LinearEqualityConstraint> linear_eq_constraint =
            internal::BindingDynamicCast<LinearEqualityConstraint>(constraint);
        if (vars.size() != 0) {
          EXPECT_EQ(linear_eq_constraint.variables(), vars);
        } else {
          EXPECT_EQ(linear_eq_constraint.variables().size(), 0);
        }
        EXPECT_EQ(linear_eq_constraint.evaluator()->num_constraints(), 1);
        EXPECT_TRUE(CompareMatrices(
            linear_eq_constraint.evaluator()->GetDenseA(), a, tol));
        EXPECT_NEAR(linear_eq_constraint.evaluator()->lower_bound()(0), bound,
                    tol);
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
  auto check_linear_constraint =
      [](const Binding<Constraint>& constraint,
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
        EXPECT_TRUE(CompareMatrices(linear_constraint.evaluator()->GetDenseA(),
                                    a, tol));
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

GTEST_TEST(ParseConstraintTest, FalseFormula) {
  // ParseConstraint with some formula being false.
  symbolic::Variable x("x");
  // ParseConstraint for a vector of symbolic::Formula
  DRAKE_EXPECT_THROWS_MESSAGE(
      internal::ParseConstraint(
          Vector2<symbolic::Formula>(x >= 0, symbolic::Expression(1) >= 2)),
      "ParseConstraint is called with formulas\\(1, 0\\) being always false");

  // ParseConstraint for a single symbolic::Formula
  DRAKE_EXPECT_THROWS_MESSAGE(
      internal::ParseConstraint(symbolic::Expression(1) >= 2),
      "ParseConstraint is called with a formula being always false.");
}

GTEST_TEST(ParseLinearEqualityConstraintTest, FalseFormula) {
  // ParseLinearEqualityConstraint with some formula being false.
  symbolic::Variable x("x");
  // ParseLinearEqualityConstraint for a set of symbolic::Formula
  DRAKE_EXPECT_THROWS_MESSAGE(
      internal::ParseLinearEqualityConstraint(
          std::set<symbolic::Formula>({x == 0, symbolic::Expression(1) == 2})),
      "ParseLinearEqualityConstraint is called with one of formulas being "
      "always false.");

  // ParseLinearEqualityConstraint for a single symbolic::Formula
  DRAKE_EXPECT_THROWS_MESSAGE(
      internal::ParseLinearEqualityConstraint(symbolic::Expression(1) == 2),
      "ParseLinearEqualityConstraint is called with a formula being always "
      "false.");
}

GTEST_TEST(ParseConstraintTest, TrueFormula) {
  // Call ParseConstraint with a formula being always True.
  auto binding1 = internal::ParseConstraint(symbolic::Expression(1) >= 0);
  EXPECT_NE(dynamic_cast<BoundingBoxConstraint*>(binding1.evaluator().get()),
            nullptr);
  EXPECT_EQ(binding1.evaluator()->num_constraints(), 0);
  EXPECT_EQ(binding1.variables().rows(), 0);

  // Call ParseConstraint with a vector of formulas, some of the formulas being
  // always True.
  symbolic::Variable x("x");
  auto binding2 = internal::ParseConstraint(
      Vector2<symbolic::Formula>(x >= 1, symbolic::Expression(1) >= 0));
  EXPECT_EQ(binding2.evaluator()->num_constraints(), 1);
  EXPECT_TRUE(
      binding2.evaluator()->CheckSatisfied((Vector1d() << 2).finished()));
  EXPECT_FALSE(
      binding2.evaluator()->CheckSatisfied((Vector1d() << 0).finished()));

  // Call ParseConstraint with a vector of formulas all being True.
  auto binding3 = internal::ParseConstraint(Vector2<symbolic::Formula>(
      symbolic::Expression(1) >= 0, symbolic::Expression(2) >= 1));
  EXPECT_NE(dynamic_cast<BoundingBoxConstraint*>(binding3.evaluator().get()),
            nullptr);
  EXPECT_EQ(binding3.evaluator()->num_constraints(), 0);
  EXPECT_EQ(binding3.variables().rows(), 0);

  // Call ParseLinearEqualityConstraint with a set of formulas, while some
  // formulas being always true.
  auto binding4 = internal::ParseLinearEqualityConstraint(
      std::set<symbolic::Formula>({2 * x == 1, symbolic::Expression(1) == 1}));
  EXPECT_EQ(binding4.evaluator()->num_constraints(), 1);
  EXPECT_TRUE(
      binding4.evaluator()->CheckSatisfied((Vector1d() << 0.5).finished()));
  EXPECT_FALSE(
      binding4.evaluator()->CheckSatisfied((Vector1d() << 1).finished()));

  // Call ParseLinearEqualityConstraint with a set of formulas all being True.
  auto binding5 =
      internal::ParseLinearEqualityConstraint(std::set<symbolic::Formula>(
          {symbolic::Expression(1) == 1, symbolic::Expression(2) == 2}));
  EXPECT_EQ(binding5.evaluator()->num_constraints(), 0);

  // Call ParseLinearEqualityConstraint with a single formula being always true.
  auto binding6 =
      internal::ParseLinearEqualityConstraint(symbolic::Expression(1) == 1);
  EXPECT_EQ(binding6.evaluator()->num_constraints(), 0);
  EXPECT_EQ(binding6.variables().rows(), 0);
}

// Confirm that ParseConstraint also parses the quadratic constraint.
GTEST_TEST(ParseConstraintTest, Quadratic) {
  symbolic::Variable x0("x0"), x1("x1");
  Binding<Constraint> binding =
      internal::ParseConstraint(-x0 * x0 + 2 * x1, -kInf, 3);
  EXPECT_NE(dynamic_cast<QuadraticConstraint*>(binding.evaluator().get()),
            nullptr);

  // A vector of quadratic constraints is an ExpressionConstraint (not
  // quadratic).
  binding = internal::ParseConstraint(
      Vector2<Expression>(x0 * x0 + 2 * x1, x1 * x1), Vector2d::Constant(-kInf),
      Vector2d::Constant(3));
  EXPECT_NE(dynamic_cast<ExpressionConstraint*>(binding.evaluator().get()),
            nullptr);

  // A scalar non-polynomial constraint is an ExpressionConstraint (not
  // quadratic).
  binding = internal::ParseConstraint(x0 * x0 + 2 * x1 + sin(x1), -kInf, 3);
  EXPECT_NE(dynamic_cast<ExpressionConstraint*>(binding.evaluator().get()),
            nullptr);

  // A polynomial constraint of degree > 2 is an ExpressionConstraint (not
  // quadratic).
  binding = internal::ParseConstraint(x0 * x0 * x1 + 2 * x1, -kInf, 3);
  EXPECT_NE(dynamic_cast<ExpressionConstraint*>(binding.evaluator().get()),
            nullptr);
}

// Confirm that ParseConstraint also parses Lorentz cone constraints.
GTEST_TEST(ParseConstraintTest, LorentzCone) {
  auto x = symbolic::MakeVectorVariable<3>("x");

  Binding<Constraint> binding =
      internal::ParseConstraint(x[0] >= x.tail<2>().cast<Expression>().norm());
  EXPECT_NE(dynamic_cast<LorentzConeConstraint*>(binding.evaluator().get()),
            nullptr);

  binding =
      internal::ParseConstraint(x.tail<2>().cast<Expression>().norm() <= x[0]);
  EXPECT_NE(dynamic_cast<LorentzConeConstraint*>(binding.evaluator().get()),
            nullptr);
}

GTEST_TEST(ParseConstraintTest, FormulaWithInfiniteLowerOrUpperBounds) {
  Variable x0("x0"), x1("x1");
  Vector2<Variable> x(x0, x1);
  Vector2d b(0.12, kInf);
  Binding<Constraint> binding = internal::ParseConstraint(x <= b);
  EXPECT_TRUE(CompareMatrices(binding.evaluator()->upper_bound(), b));
  binding = internal::ParseConstraint(-b <= x);
  // Note: The constraints are sorted via get_operands(f) which returns a
  // std::set. This one gets flipped.
  EXPECT_TRUE(CompareMatrices(binding.evaluator()->lower_bound(),
                              -Vector2d(b[1], b[0])));
  binding = internal::ParseConstraint(b >= x);
  EXPECT_TRUE(CompareMatrices(binding.evaluator()->upper_bound(), b));
  binding = internal::ParseConstraint(x >= -b);
  EXPECT_TRUE(CompareMatrices(binding.evaluator()->lower_bound(), -b));

  DRAKE_EXPECT_THROWS_MESSAGE(internal::ParseConstraint(x <= -b),
                              ".*an upper bound of -inf.*");
  DRAKE_EXPECT_THROWS_MESSAGE(internal::ParseConstraint(b <= x),
                              ".*a lower bound of.*");
  DRAKE_EXPECT_THROWS_MESSAGE(internal::ParseConstraint(-b >= x),
                              ".*an upper bound of -inf.*");
  DRAKE_EXPECT_THROWS_MESSAGE(internal::ParseConstraint(x >= b),
                              ".*a lower bound of.*");
}

std::shared_ptr<RotatedLorentzConeConstraint>
CheckParseQuadraticAsRotatedLorentzConeConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::VectorXd>& b, double c,
    double zero_tol = 0.) {
  const auto dut =
      internal::ParseQuadraticAsRotatedLorentzConeConstraint(Q, b, c, zero_tol);
  // Make sure that dut.A() * x + dub.t() in rotated Lorentz cone is the same
  // expression as 0.5xᵀQx + bᵀx + c<=0.
  const Eigen::MatrixXd A_dense = dut->A_dense();
  EXPECT_TRUE(
      CompareMatrices(A_dense.row(1), Eigen::RowVectorXd::Zero(Q.rows())));
  EXPECT_EQ(dut->b()(1), 1);
  const double tol = 1E-12;
  // Check the Hessian.
  EXPECT_TRUE(
      CompareMatrices(A_dense.bottomRows(A_dense.rows() - 2).transpose() *
                          A_dense.bottomRows(A_dense.rows() - 2),
                      0.25 * (Q + Q.transpose()), tol));
  // Check the linear coefficient.
  EXPECT_TRUE(
      CompareMatrices(2 * A_dense.bottomRows(A_dense.rows() - 2).transpose() *
                              dut->b().tail(dut->b().rows() - 2) -
                          A_dense.row(0).transpose(),
                      b, tol));
  EXPECT_NEAR(dut->b().tail(dut->b().rows() - 2).squaredNorm() - dut->b()(0), c,
              tol);
  return dut;
}

GTEST_TEST(ParseQuadraticAsRotatedLorentzConeConstraint, Test) {
  CheckParseQuadraticAsRotatedLorentzConeConstraint(
      (Vector1d() << 1).finished(), Vector1d::Zero(), 1);
  CheckParseQuadraticAsRotatedLorentzConeConstraint(
      (Vector1d() << 1).finished(), (Vector1d() << 2).finished(), 1);
  // Strictly positive Hessian.
  CheckParseQuadraticAsRotatedLorentzConeConstraint(Eigen::Matrix2d::Identity(),
                                                    Eigen::Vector2d(1, 3), 2);
  // Hessian is positive semidefinite but not positive definite. b is in the
  // range space of F.
  auto dut = CheckParseQuadraticAsRotatedLorentzConeConstraint(
      2 * Eigen::Matrix2d::Ones(), Eigen::Vector2d(2, 2), 0.5);
  EXPECT_EQ(dut->A().rows(), 3);

  // Hessian is positive semidefinite but not positive definite, b is not in the
  // range space of F.
  dut = CheckParseQuadraticAsRotatedLorentzConeConstraint(
      2 * Eigen::Matrix2d::Ones(), Eigen::Vector2d(2, 3), -0.5);
  EXPECT_EQ(dut->A().rows(), 3);
}

GTEST_TEST(ParseQuadraticAsRotatedLorentzConeConstraint, TestException) {
  const Eigen::MatrixXd Q_non_psd = Eigen::Vector2d(1, -2).asDiagonal();
  DRAKE_EXPECT_THROWS_MESSAGE(
      internal::ParseQuadraticAsRotatedLorentzConeConstraint(
          Q_non_psd, Eigen::Vector2d(1, 3), -2),
      ".* is not positive semidefinite.*");

  // Hessian is almost positive semidefinite with one eigenvalue slightly
  // negative.
  Eigen::Matrix2d Q_almost_psd;
  // clang-format off
  Q_almost_psd << 1, 1,
                  1, 1 - 1E-12;
  // clang-format on
  DRAKE_EXPECT_THROWS_MESSAGE(
      internal::ParseQuadraticAsRotatedLorentzConeConstraint(
          Q_almost_psd, Eigen::Vector2d(2, 3), -0.5),
      ".* is not positive semidefinite.*");
  CheckParseQuadraticAsRotatedLorentzConeConstraint(
      Q_almost_psd, Eigen::Vector2d(2, 3), -0.5, 1E-10);

  // Hessian is zero.
  Eigen::MatrixXd Q_zero = Eigen::Matrix2d::Zero();
  DRAKE_EXPECT_THROWS_MESSAGE(
      internal::ParseQuadraticAsRotatedLorentzConeConstraint(
          Q_zero, Eigen::Vector2d(1, 3), -2),
      ".* is numerically zero.* linear.*");
}
}  // namespace
}  // namespace solvers
}  // namespace drake
