#include "drake/solvers/mathematical_program.h"

#include <gtest/gtest.h>

#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_formula.h"
#include "drake/common/symbolic_monomial_util.h"
#include "drake/common/symbolic_polynomial.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {
namespace solvers {
namespace {

using drake::symbolic::Expression;
using drake::symbolic::Variable;
using drake::symbolic::Variables;

class SosConstraintTest : public ::testing::Test {
 public:
  void CheckNewFreePolynomial(const symbolic::Polynomial& p,
                              const Variables& indeterminates,
                              const int degree) {
    const drake::VectorX<symbolic::Monomial> monomial_basis{
        symbolic::MonomialBasis(indeterminates, degree)};

    EXPECT_EQ(p.TotalDegree(), degree);
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
      EXPECT_NO_THROW(prog_.FindDecisionVariableIndex(decision_variable_i));
    }
  }

  void CheckNewSosPolynomial(
      const symbolic::Polynomial& p,
      const Binding<PositiveSemidefiniteConstraint>& psd_binding,
      const Variables& indeterminates, const int degree) {
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
    EXPECT_EQ(p.decision_variables(), Variables(psd_binding.variables()));
    prog_.Solve();
    CheckPsdBinding(psd_binding);
  }

  // Checks Q has all eigen values as approximately non-negatives.
  // Precondition: prog_.Solve() has been called.
  void CheckPsdBinding(
      const Binding<PositiveSemidefiniteConstraint>& psd_binding,
      const double eps = 1e-07) {
    const VectorXDecisionVariable& variables{psd_binding.variables()};
    const auto values = prog_.GetSolution(variables);
    Eigen::VectorXd eigen_values;
    psd_binding.constraint()->Eval(values, eigen_values);
    EXPECT_TRUE((eigen_values.array() >= -eps).all());
  }

 protected:
  void SetUp() override {
    x_ = prog_.NewIndeterminates<3>();
    c_ = prog_.NewContinuousVariables<1>();
  }

  MathematicalProgram prog_;
  VectorIndeterminate<3> x_;
  VectorDecisionVariable<1> c_;
};

TEST_F(SosConstraintTest, NewFreePolynomialUnivariateDegree1) {
  const auto& x = x_(0);
  const Variables indeterminates{x};
  const int degree{1};
  const symbolic::Polynomial poly{
      prog_.NewFreePolynomial(indeterminates, degree)};
  CheckNewFreePolynomial(poly, indeterminates, degree);
}

TEST_F(SosConstraintTest, NewFreePolynomialUnivariateDegree2) {
  const auto& x = x_(0);
  const Variables indeterminates{x};
  const int degree{2};
  const symbolic::Polynomial poly{
      prog_.NewFreePolynomial(indeterminates, degree)};
  CheckNewFreePolynomial(poly, indeterminates, degree);
}

TEST_F(SosConstraintTest, NewFreePolynomialMultivariateDegree1) {
  const auto& x0 = x_(0);
  const auto& x1 = x_(1);
  const Variables indeterminates{x0, x1};
  const int degree{1};
  const symbolic::Polynomial poly{
      prog_.NewFreePolynomial(indeterminates, degree)};
  CheckNewFreePolynomial(poly, indeterminates, degree);
}

TEST_F(SosConstraintTest, NewFreePolynomialMultivariateDegree2) {
  const auto& x0 = x_(0);
  const auto& x1 = x_(1);
  const Variables indeterminates{x0, x1};
  const int degree{2};
  const symbolic::Polynomial poly{
      prog_.NewFreePolynomial(indeterminates, degree)};
  CheckNewFreePolynomial(poly, indeterminates, degree);
}

TEST_F(SosConstraintTest, NewSosPolynomialUnivariate1) {
  const auto& x = x_(0);
  const Variables indeterminates{x};
  const int degree{2};
  const auto p = prog_.NewSosPolynomial(indeterminates, degree);
  const symbolic::Polynomial& poly{p.first};
  const Binding<PositiveSemidefiniteConstraint>& psd_binding{p.second};
  CheckNewSosPolynomial(poly, psd_binding, indeterminates, degree);
}

TEST_F(SosConstraintTest, NewSosPolynomialUnivariate2) {
  const auto& x = x_(0);
  const Variables indeterminates{x};
  const int degree{4};
  const auto p = prog_.NewSosPolynomial(indeterminates, degree);
  const symbolic::Polynomial& poly{p.first};
  const Binding<PositiveSemidefiniteConstraint>& psd_binding{p.second};
  CheckNewSosPolynomial(poly, psd_binding, indeterminates, degree);
}

TEST_F(SosConstraintTest, NewSosPolynomialMultivariate1) {
  const auto& x0 = x_(0);
  const auto& x1 = x_(1);
  const Variables indeterminates{x0, x1};
  const int degree{2};
  const auto p = prog_.NewSosPolynomial(indeterminates, degree);
  const symbolic::Polynomial& poly{p.first};
  const Binding<PositiveSemidefiniteConstraint>& psd_binding{p.second};
  CheckNewSosPolynomial(poly, psd_binding, indeterminates, degree);
}

TEST_F(SosConstraintTest, NewSosPolynomialMultivariate2) {
  const auto& x0 = x_(0);
  const auto& x1 = x_(1);
  const auto& x2 = x_(2);
  const Variables indeterminates{x0, x1, x2};
  const int degree{4};
  const auto p = prog_.NewSosPolynomial(indeterminates, degree);
  const symbolic::Polynomial& poly{p.first};
  const Binding<PositiveSemidefiniteConstraint>& psd_binding{p.second};
  CheckNewSosPolynomial(poly, psd_binding, indeterminates, degree);
}

// Shows that f(x) = x² + 2x + 1 is SOS.
TEST_F(SosConstraintTest, AddSosConstraintUnivariate1) {
  const auto& x = x_(0);
  const auto binding_pair = prog_.AddSosConstraint(2 * pow(x, 2) + 2 * x + 1);
  const auto result = prog_.Solve();
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
  CheckPsdBinding(binding_pair.first);
}

// Finds the global minimum of f(x) = x⁶ − 10x⁵ + 51x⁴ − 166x³ + 342x² − 400x +
// 200, which should be close to zero. The example is taken from page 12 of
// http://www.mit.edu/~parrilo/cdc03_workshop/08_sum_of_squares_2003_12_07_07_screen.pdf.
TEST_F(SosConstraintTest, AddSosConstraintUnivariate2) {
  const auto& x = x_(0);
  const auto& c = c_(0);
  prog_.AddCost(-c);
  const auto binding_pair = prog_.AddSosConstraint(
      pow(x, 6) - 10 * pow(x, 5) + 51 * pow(x, 4) - 166 * pow(x, 3) +
      342 * pow(x, 2) - 400 * x + 200 - c);
  const auto result = prog_.Solve();
  ASSERT_EQ(result, SolutionResult::kSolutionFound);
  EXPECT_LE(prog_.GetSolution(c), 1E-4);
  CheckPsdBinding(binding_pair.first, 1E-6 /* eps */);
}

// Shows that f(x₀, x₁) = 2x₀⁴ + 2x₀³x₁ - x₀²x₁² + 5x₁⁴ is SOS.
TEST_F(SosConstraintTest, AddSosConstraintMultivariate1) {
  const auto& x0 = x_(0);
  const auto& x1 = x_(1);
  const auto binding_pair =
      prog_.AddSosConstraint(2 * pow(x0, 4) + 2 * pow(x0, 3) * x1 -
                             pow(x0, 2) * pow(x1, 2) + 5 * pow(x1, 4));
  const auto result = prog_.Solve();
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
  CheckPsdBinding(binding_pair.first);
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
  const auto binding_pair = prog_.AddSosConstraint(
      4 * pow(x0, 2) - 2.1 * pow(x0, 4) + 1.0 / 3.0 * pow(x0, 6) + x0 * x1 -
      4 * x1 * x1 + 4 * pow(x1, 4) - c);

  const auto result = prog_.Solve();
  ASSERT_EQ(result, SolutionResult::kSolutionFound);
  EXPECT_NEAR(prog_.GetSolution(c), -1.0316, 1E-4);
  CheckPsdBinding(binding_pair.first);
}
}  // namespace
}  // namespace solvers
}  // namespace drake
