#include "drake/solvers/test/sos_examples.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace solvers {
namespace {
void CheckSymmetricMatrixPSD(const Eigen::Ref<const Eigen::MatrixXd>& mat,
                             double tol) {
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver;
  eigen_solver.compute(mat);
  EXPECT_TRUE((eigen_solver.eigenvalues().array() >=
               Eigen::ArrayXd::Constant(mat.rows(), -tol))
                  .all());
}
}  // namespace

UnivariateQuarticSos::UnivariateQuarticSos() : prog_() {
  auto x = prog_.NewIndeterminates<1>()(0);
  p_ = symbolic::Monomial(x, 4) + 4 * symbolic::Monomial(x, 3) +
       6 * symbolic::Monomial(x, 2) + 4 * symbolic::Monomial(x, 1) + 5;
  std::tie(gram_, monomial_basis_) = prog_.AddSosConstraint(p_);
}

void UnivariateQuarticSos::CheckResult(const MathematicalProgramResult& result,
                                       double tol) const {
  EXPECT_TRUE(result.is_success());
  const Eigen::MatrixXd gram_val = result.GetSolution(gram_);
  EXPECT_TRUE(symbolic::test::PolynomialEqual(
      p_, monomial_basis_.dot(gram_val * monomial_basis_), tol));
  CheckSymmetricMatrixPSD(gram_val, tol);
}

BivariateQuarticSos::BivariateQuarticSos() : prog_() {
  auto x = prog_.NewIndeterminates<1>()(0);
  auto y = prog_.NewIndeterminates<1>()(0);
  p_ = 2 * symbolic::Monomial(x, 4) + 5 * symbolic::Monomial(y, 4) -
       2 * symbolic::Monomial(x, 2) * symbolic::Monomial(y, 2) +
       2 * symbolic::Monomial(x, 3) * symbolic::Monomial(y, 1) +
       2 * symbolic::Monomial(x, 1) + 2;
  std::tie(gram_, monomial_basis_) = prog_.AddSosConstraint(p_);
}

void BivariateQuarticSos::CheckResult(const MathematicalProgramResult& result,
                                      double tol) const {
  EXPECT_TRUE(result.is_success());
  const Eigen::MatrixXd gram_val = result.GetSolution(gram_);
  EXPECT_TRUE(symbolic::test::PolynomialEqual(
      p_, monomial_basis_.dot(gram_val * monomial_basis_), tol));
  CheckSymmetricMatrixPSD(gram_val, tol);
}

SimpleSos1::SimpleSos1() : prog_{} {
  a_ = prog_.NewContinuousVariables<1>("a")(0);
  b_ = prog_.NewContinuousVariables<1>("b")(0);
  x_ = prog_.NewIndeterminates<1>("x")(0);
  prog_.AddLinearCost(-a_ - b_);
  p1_ = symbolic::Monomial(x_, 4) + a_ * symbolic::Monomial(x_, 1) + 2 + b_;
  std::tie(gram1_, monomial_basis1_) = prog_.AddSosConstraint(p1_);
  p2_ = symbolic::Polynomial({{symbolic::Monomial(x_, 2), a_ - b_ + 1},
                              {symbolic::Monomial(x_, 1), 1},
                              {symbolic::Monomial(), 1}});
  std::tie(gram2_, monomial_basis2_) = prog_.AddSosConstraint(p2_);
}

void SimpleSos1::CheckResult(const MathematicalProgramResult& result,
                             double tol) const {
  EXPECT_TRUE(result.is_success());
  const Eigen::MatrixXd gram1_val = result.GetSolution(gram1_);
  const Eigen::MatrixXd gram2_val = result.GetSolution(gram2_);
  EXPECT_TRUE(symbolic::test::PolynomialEqual(
      result.GetSolution(p1_),
      monomial_basis1_.dot(gram1_val * monomial_basis1_), tol));
  EXPECT_TRUE(symbolic::test::PolynomialEqual(
      result.GetSolution(p2_),
      monomial_basis2_.dot(gram2_val * monomial_basis2_), tol));
  CheckSymmetricMatrixPSD(gram1_val, tol);
  CheckSymmetricMatrixPSD(gram2_val, tol);
}

MotzkinPolynomial::MotzkinPolynomial() : prog_{} {
  x_ = prog_.NewIndeterminates<1>()(0);
  y_ = prog_.NewIndeterminates<1>()(0);
  m_ = symbolic::Polynomial({{symbolic::Monomial({{x_, 4}, {y_, 2}}), 1},
                             {symbolic::Monomial({{x_, 2}, {y_, 4}}), 1},
                             {symbolic::Monomial(), 1},
                             {symbolic::Monomial({{x_, 2}, {y_, 2}}), -3}});
  r_ = prog_.NewFreePolynomial({x_, y_}, 2);
  std::tie(gram1_, monomial_basis1_) = prog_.AddSosConstraint(
      r_ - symbolic::Polynomial({{symbolic::Monomial(x_, 2), 1},
                                 {symbolic::Monomial(y_, 2), 1}}));
  std::tie(gram2_, monomial_basis2_) = prog_.AddSosConstraint(m_ * r_);
}

void MotzkinPolynomial::CheckResult(const MathematicalProgramResult& result,
                                    double tol) const {
  EXPECT_TRUE(result.is_success());
  const symbolic::Polynomial m_result =
      symbolic::Polynomial(result.GetSolution(m_.ToExpression()), {x_, y_});
  const symbolic::Polynomial r_result =
      symbolic::Polynomial(result.GetSolution(r_.ToExpression()), {x_, y_});
  const Eigen::MatrixXd gram1_val = result.GetSolution(gram1_);
  const Eigen::MatrixXd gram2_val = result.GetSolution(gram2_);
  EXPECT_TRUE(symbolic::test::PolynomialEqual(
      r_result - symbolic::Polynomial(x_ * x_ + y_ * y_, {x_, y_}),
      monomial_basis1_.dot(gram1_val * monomial_basis1_), tol));
  EXPECT_TRUE(symbolic::test::PolynomialEqual(
      m_result * r_result, monomial_basis2_.dot(gram2_val * monomial_basis2_),
      tol));
  CheckSymmetricMatrixPSD(gram1_val, tol);
  CheckSymmetricMatrixPSD(gram2_val, tol);
}

UnivariateNonnegative1::UnivariateNonnegative1() : prog_{} {
  a_ = prog_.NewContinuousVariables<1>()(0);
  b_ = prog_.NewContinuousVariables<1>()(0);
  c_ = prog_.NewContinuousVariables<1>()(0);
  x_ = prog_.NewIndeterminates<1>()(0);
  p_ = symbolic::Polynomial({{symbolic::Monomial(x_, 4), 1},
                             {symbolic::Monomial(x_, 3), a_},
                             {symbolic::Monomial(x_, 2), b_},
                             {symbolic::Monomial(x_, 1), c_},
                             {symbolic::Monomial(), 1}});
  prog_.AddLinearEqualityConstraint(2 + a_ + b_ + c_ == 1);
  prog_.AddLinearCost(-a_ - b_ - c_);
  std::tie(s_, gram_s_) = prog_.NewSosPolynomial({x_}, 4);
  std::tie(t_, gram_t_) = prog_.NewSosPolynomial({x_}, 2);
  prog_.AddEqualityConstraintBetweenPolynomials(p_, s_ + x_ * t_);
}

void UnivariateNonnegative1::CheckResult(
    const MathematicalProgramResult& result, double tol) const {
  EXPECT_TRUE(result.is_success());
  const symbolic::Polynomial p_result =
      symbolic::Polynomial(result.GetSolution(p_.ToExpression()), {x_});
  const symbolic::Polynomial s_result =
      symbolic::Polynomial(result.GetSolution(s_.ToExpression()), {x_});
  const symbolic::Polynomial t_result =
      symbolic::Polynomial(result.GetSolution(t_.ToExpression()), {x_});
  EXPECT_TRUE(
      symbolic::test::PolynomialEqual(p_result, s_result + x_ * t_result, tol));
  const double a_val = result.GetSolution(a_);
  const double b_val = result.GetSolution(b_);
  const double c_val = result.GetSolution(c_);
  EXPECT_NEAR(result.get_optimal_cost(), -a_val - b_val - c_val, tol);
  CheckSymmetricMatrixPSD(result.GetSolution(gram_s_), tol);
  CheckSymmetricMatrixPSD(result.GetSolution(gram_t_), tol);
}
}  // namespace solvers
}  // namespace drake
