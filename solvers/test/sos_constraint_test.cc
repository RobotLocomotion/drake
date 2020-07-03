#include "drake/common/test_utilities/expect_no_throw.h"
/* clang-format off to disable clang-format-includes */
#include "drake/solvers/mathematical_program.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace solvers {
namespace {

using drake::symbolic::Expression;
using drake::symbolic::Monomial;
using drake::symbolic::Variable;
using drake::symbolic::Variables;
using drake::symbolic::internal::DegreeType;

class SosConstraintTest : public ::testing::Test {
 public:
  void CheckNewFreePolynomial(const symbolic::Polynomial& p,
                              const Variables& indeterminates, const int degree,
                              DegreeType degree_type) {
    const drake::VectorX<symbolic::Monomial> monomial_basis{
        symbolic::internal::ComputeMonomialBasis<Eigen::Dynamic>(
            indeterminates, degree, degree_type)};

    switch (degree_type) {
      case DegreeType::kAny: {
        EXPECT_EQ(p.TotalDegree(), degree);
        break;
      }
      case DegreeType::kEven: {
        // p.TotalDegree() should be the largest even number that is no
        // larger than @p degree.
        EXPECT_EQ(p.TotalDegree(), degree % 2 == 0 ? degree : degree - 1);
        break;
      }
      case DegreeType::kOdd: {
        // p.TotalDegree() should be the largest odd number that is no
        // larger than @p degree.
        EXPECT_EQ(p.TotalDegree(), degree % 2 == 1 ? degree : degree - 1);
        break;
      }
    }
    EXPECT_EQ(p.monomial_to_coefficient_map().size(), monomial_basis.size());
    EXPECT_EQ(p.monomial_to_coefficient_map().size(),
              p.decision_variables().size());

    for (const auto& pair : p.monomial_to_coefficient_map()) {
      // Each coefficient is a single decision variable.
      const Expression& coeff_i{pair.second};
      ASSERT_TRUE(is_variable(coeff_i));
      const Variable& decision_variable_i{get_variable(coeff_i)};
      // This decision_variable_i in the polynomial should be a decision
      // variable in the MathematicalProgram.
      DRAKE_EXPECT_NO_THROW(
          prog_.FindDecisionVariableIndex(decision_variable_i));
      switch (degree_type) {
        case DegreeType::kAny: {
          break;
        }
        case DegreeType::kEven: {
          EXPECT_EQ(pair.first.total_degree() % 2, 0);
          break;
        }
        case DegreeType::kOdd: {
          EXPECT_EQ(pair.first.total_degree() % 2, 1);
          break;
        }
      }
    }
  }

  void TestNewFreePolynomial(const Variables& indeterminates, int degree) {
    const symbolic::Polynomial poly1{
        prog_.NewFreePolynomial(indeterminates, degree)};
    CheckNewFreePolynomial(poly1, indeterminates, degree, DegreeType::kAny);
    const symbolic::Polynomial poly2{
        prog_.NewEvenDegreeFreePolynomial(indeterminates, degree)};
    CheckNewFreePolynomial(poly2, indeterminates, degree, DegreeType::kEven);
    const symbolic::Polynomial poly3{
        prog_.NewOddDegreeFreePolynomial(indeterminates, degree)};
    CheckNewFreePolynomial(poly3, indeterminates, degree, DegreeType::kOdd);
  }

  void CheckNewSosPolynomial(const symbolic::Polynomial& p,
                             const MatrixXDecisionVariable& Q,
                             const Variables& indeterminates,
                             const int degree) {
    // p = xᵀ*Q*x.
    // where x = monomial_basis(indeterminates, degree) and
    //       Q is p.s.d.
    const drake::VectorX<symbolic::Monomial> monomial_basis{
        symbolic::MonomialBasis(indeterminates, degree / 2)};
    EXPECT_EQ(p.TotalDegree(), degree);
    // Number of coefficients == number of distinct entries in a symmetric
    // matrix of size n x n where n = monomial_basis.size().
    EXPECT_EQ(p.decision_variables().size(),
              monomial_basis.size() * (monomial_basis.size() + 1) / 2);
    // Q is the coefficient matrix of p.
    VectorXDecisionVariable Q_flat(Q.size());
    for (int i = 0; i < Q.cols(); ++i) {
      Q_flat.segment(i * Q.rows(), Q.rows()) = Q.col(i);
    }
    EXPECT_EQ(p.decision_variables(), Variables(Q_flat));
    result_ = Solve(prog_);
    CheckPositiveDefiniteMatrix(Q, monomial_basis, p.ToExpression());
  }

  // Checks Q has all eigen values as approximately non-negatives.
  // Precondition: result_ = Solve(prog_) has been called.
  void CheckPositiveDefiniteMatrix(
      const MatrixXDecisionVariable& Q,
      const Eigen::Ref<const VectorX<symbolic::Monomial>>& monomial_basis,
      const symbolic::Expression& sos_poly_expected, const double eps = 1e-07) {
    const Eigen::MatrixXd Q_val = result_.GetSolution(Q);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(Q_val);
    EXPECT_TRUE((es.eigenvalues().array() >= -eps).all());
    // Compute mᵀ * Q * m;
    symbolic::Polynomial sos_poly{};
    for (int i = 0; i < Q_val.rows(); ++i) {
      sos_poly.AddProduct(Q_val(i, i), pow(monomial_basis(i), 2));
      for (int j = i + 1; j < Q_val.cols(); ++j) {
        sos_poly.AddProduct(Q_val(i, j) + Q_val(j, i),
                            monomial_basis(i) * monomial_basis(j));
      }
    }
    symbolic::Polynomial diff_poly =
        sos_poly - symbolic::Polynomial(result_.GetSolution(sos_poly_expected));
    diff_poly = diff_poly.RemoveTermsWithSmallCoefficients(eps);
    const symbolic::Polynomial zero_poly{};
    EXPECT_PRED2(symbolic::test::PolyEqual, diff_poly, zero_poly);
  }

 protected:
  void SetUp() override {
    x_ = prog_.NewIndeterminates<3>();
    c_ = prog_.NewContinuousVariables<1>();
  }

  MathematicalProgram prog_;
  VectorIndeterminate<3> x_;
  VectorDecisionVariable<1> c_;
  MathematicalProgramResult result_;
};

TEST_F(SosConstraintTest, NewFreePolynomialUnivariateDegree) {
  const auto& x = x_(0);
  const Variables indeterminates{x};
  TestNewFreePolynomial(indeterminates, 1);
  TestNewFreePolynomial(indeterminates, 2);
  TestNewFreePolynomial(indeterminates, 3);
}

TEST_F(SosConstraintTest, NewFreePolynomialMultivariateDegree) {
  const auto& x0 = x_(0);
  const auto& x1 = x_(1);
  const Variables indeterminates{x0, x1};
  TestNewFreePolynomial(indeterminates, 1);
  TestNewFreePolynomial(indeterminates, 2);
  TestNewFreePolynomial(indeterminates, 3);
}

TEST_F(SosConstraintTest, NewSosPolynomialUnivariate1) {
  const auto& x = x_(0);
  const Variables indeterminates{x};
  const int degree{2};
  const auto p = prog_.NewSosPolynomial(indeterminates, degree);
  const symbolic::Polynomial& poly{p.first};
  const MatrixXDecisionVariable& Q{p.second};
  CheckNewSosPolynomial(poly, Q, indeterminates, degree);
}

TEST_F(SosConstraintTest, NewSosPolynomialUnivariate2) {
  const auto& x = x_(0);
  const Variables indeterminates{x};
  const int degree{4};
  const auto p = prog_.NewSosPolynomial(indeterminates, degree);
  const symbolic::Polynomial& poly{p.first};
  const MatrixXDecisionVariable& Q{p.second};
  CheckNewSosPolynomial(poly, Q, indeterminates, degree);
}

TEST_F(SosConstraintTest, NewSosPolynomialMultivariate1) {
  const auto& x0 = x_(0);
  const auto& x1 = x_(1);
  const Variables indeterminates{x0, x1};
  const int degree{2};
  const auto p = prog_.NewSosPolynomial(indeterminates, degree);
  const symbolic::Polynomial& poly{p.first};
  const MatrixXDecisionVariable& Q{p.second};
  CheckNewSosPolynomial(poly, Q, indeterminates, degree);
}

TEST_F(SosConstraintTest, NewSosPolynomialMultivariate2) {
  const auto& x0 = x_(0);
  const auto& x1 = x_(1);
  const auto& x2 = x_(2);
  const Variables indeterminates{x0, x1, x2};
  const int degree{4};
  const auto p = prog_.NewSosPolynomial(indeterminates, degree);
  const symbolic::Polynomial& poly{p.first};
  const MatrixXDecisionVariable& Q{p.second};
  CheckNewSosPolynomial(poly, Q, indeterminates, degree);
}

TEST_F(SosConstraintTest, NewSosPolynomialViaMonomialBasis) {
  const auto& x0 = x_(0);
  const auto& x1 = x_(1);
  Vector2<Monomial> basis{ x0, x1 };
  const auto p = prog_.NewSosPolynomial(basis);
  const symbolic::Polynomial& poly{p.first};
  const MatrixXDecisionVariable& Q = p.second;
  const symbolic::Polynomial expected_poly{
      Q(0, 0) * x0 * x0 + 2 * Q(0, 1) * x0 * x1 + Q(1, 1) * x1 * x1,
      symbolic::Variables({x0, x1})};
  EXPECT_TRUE(poly.EqualTo(expected_poly));
}

// Shows that f(x) = 2x² + 2x + 1 is SOS.
TEST_F(SosConstraintTest, AddSosConstraintUnivariate1) {
  const auto& x = x_(0);
  const symbolic::Expression e = 2 * pow(x, 2) + 2 * x + 1;
  MatrixXDecisionVariable Q;
  VectorX<symbolic::Monomial> m;
  std::tie(Q, m) = prog_.AddSosConstraint(e);
  result_ = Solve(prog_);
  ASSERT_TRUE(result_.is_success());
  CheckPositiveDefiniteMatrix(Q, m, e);
}

// Finds the global minimum of f(x) = x⁶ − 10x⁵ + 51x⁴ − 166x³ + 342x² − 400x +
// 200, which should be close to zero. The example is taken from page 12 of
// http://www.mit.edu/~parrilo/cdc03_workshop/08_sum_of_squares_2003_12_07_07_screen.pdf.
TEST_F(SosConstraintTest, AddSosConstraintUnivariate2) {
  const auto& x = x_(0);
  const auto& c = c_(0);
  prog_.AddCost(-c);
  const symbolic::Expression e = pow(x, 6) - 10 * pow(x, 5) + 51 * pow(x, 4) -
                                 166 * pow(x, 3) + 342 * pow(x, 2) - 400 * x +
                                 200 - c;
  MatrixXDecisionVariable Q;
  VectorX<symbolic::Monomial> m;
  std::tie(Q, m) = prog_.AddSosConstraint(e);
  result_ = Solve(prog_);
  ASSERT_TRUE(result_.is_success());
  EXPECT_LE(result_.GetSolution(c), 1E-4);
  // Fails with tolerance 1E-6 with solver MOSEK when run under Valgrind.
  CheckPositiveDefiniteMatrix(Q, m, e, 1.03E-6 /* eps */);
}

// Shows that f(x₀, x₁) = 2x₀⁴ + 2x₀³x₁ - x₀²x₁² + 5x₁⁴ is SOS.
TEST_F(SosConstraintTest, AddSosConstraintMultivariate1) {
  const auto& x0 = x_(0);
  const auto& x1 = x_(1);
  const symbolic::Expression e = 2 * pow(x0, 4) + 2 * pow(x0, 3) * x1 -
                                 pow(x0, 2) * pow(x1, 2) + 5 * pow(x1, 4);

  MatrixXDecisionVariable Q;
  VectorX<symbolic::Monomial> m;
  std::tie(Q, m) = prog_.AddSosConstraint(e);
  result_ = Solve(prog_);
  ASSERT_TRUE(result_.is_success());
  CheckPositiveDefiniteMatrix(Q, m, e);
}


TEST_F(SosConstraintTest, AddSosPolynomialViaMonomialBasis) {
  const auto& x = x_(0);
  Vector2<Monomial> basis{ 1, x };
  const symbolic::Expression e = 2 * pow(x, 2) + 2 * x + 1;
  const auto Q = prog_.AddSosConstraint(e, basis);
  result_ = Solve(prog_);
  ASSERT_TRUE(result_.is_success());
  CheckPositiveDefiniteMatrix(Q, basis, e);
}

// Finds the global minimum of the non-convex polynomial f(x₀, x₁) = 4 * x₀² −
// 21 / 10 * x₀⁴ + 1 / 3 * x₀⁶ + x₀ * x₁ − 4 * x₁² + 4 * x₁⁴ through the
// following convex program max c s.t f(x₀, x₁) - c is sum-of-squares.  The
// global minimum value is -1.0316. The example is taken from page 19 of
// https://stanford.edu/class/ee364b/lectures/sos_slides.pdf
TEST_F(SosConstraintTest, AddSosConstraintMultivariate2) {
  const auto& x0 = x_(0);
  const auto& x1 = x_(1);
  const auto& c = c_(0);
  prog_.AddCost(-c);
  const symbolic::Expression e = 4 * pow(x0, 2) - 2.1 * pow(x0, 4) +
                                 1.0 / 3.0 * pow(x0, 6) + x0 * x1 -
                                 4 * x1 * x1 + 4 * pow(x1, 4) - c;
  MatrixXDecisionVariable Q;
  VectorX<symbolic::Monomial> m;
  std::tie(Q, m) = prog_.AddSosConstraint(e);

  result_ = Solve(prog_);
  ASSERT_TRUE(result_.is_success());
  EXPECT_NEAR(result_.GetSolution(c), -1.0316, 1E-4);
  CheckPositiveDefiniteMatrix(Q, m, e);
}

TEST_F(SosConstraintTest, SynthesizeLyapunovFunction) {
  // Find the Lyapunov function V(x) for system:
  //
  //     ẋ₀ = -x₁ + 1.5x₀² - 0.5x₀³
  //     ẋ₁ = 3x₀ - x₁
  //
  // by solving sum-of-squared problems:
  //
  //     V(x) is sum-of-squares
  //    -V̇(x) is sum-of-squares
  VectorX<Variable> x = prog_.NewIndeterminates<2>();
  const auto& x0 = x(0);
  const auto& x1 = x(1);

  // Form the dynamics of the system.
  Vector2<symbolic::Polynomial> dynamics;
  // clang-format off
  dynamics << symbolic::Polynomial{-x1 + 1.5 * x0 * x0 - 0.5 * pow(x0, 3)},
              symbolic::Polynomial{3 * x0 - x1};
  // clang-format on

  // Adds V(x) as a 4th order sum-of-squares polynomial.
  const symbolic::Polynomial V{prog_.NewSosPolynomial({x0, x1}, 4).first};

  // Computes Vdot.
  const symbolic::Polynomial Vdot = V.Jacobian(x).transpose().dot(dynamics);

  // -Vdot is sum-of-squares.
  prog_.AddSosConstraint(-Vdot);
  result_ = Solve(prog_);
  ASSERT_TRUE(result_.is_success());
}
}  // namespace
}  // namespace solvers
}  // namespace drake
