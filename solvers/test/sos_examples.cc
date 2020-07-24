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
}  // namespace solvers
}  // namespace drake
