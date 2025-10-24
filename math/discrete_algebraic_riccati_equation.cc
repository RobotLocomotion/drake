#include "drake/math/discrete_algebraic_riccati_equation.h"

#include <Eigen/Cholesky>
#include <Eigen/LU>

#include "drake/common/drake_assert.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/systems/primitives/linear_system_internal.h"

namespace drake {
namespace math {

Eigen::MatrixXd DiscreteAlgebraicRiccatiEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R) {
  int states = B.rows();
  int inputs = B.cols();

  DRAKE_DEMAND(A.rows() == states && A.cols() == states);
  DRAKE_DEMAND(Q.rows() == states && Q.cols() == states);
  DRAKE_DEMAND(R.rows() == inputs && R.cols() == inputs);

  // Ensure Q is symmetric
  DRAKE_THROW_UNLESS(is_approx_equal_abstol(Q, Q.transpose(), 1e-10));

  // Ensure Q is positive semidefinite
  //
  // If Q is a symmetric matrix with a decomposition LDLᵀ, the number of
  // positive, negative, and zero diagonal entries in D equals the number of
  // positive, negative, and zero eigenvalues respectively in Q (see
  // https://en.wikipedia.org/wiki/Sylvester's_law_of_inertia).
  //
  // Therefore, D having no negative diagonal entries is sufficient to prove Q
  // is positive semidefinite.
  auto Q_ldlt = Q.ldlt();
  DRAKE_THROW_UNLESS(Q_ldlt.info() == Eigen::Success &&
                     (Q_ldlt.vectorD().array() >= 0.0).all());

  // Ensure R is symmetric
  DRAKE_THROW_UNLESS(is_approx_equal_abstol(R, R.transpose(), 1e-10));

  // Ensure R is positive definite (LLT decomposition fails if not)
  Eigen::LLT<Eigen::MatrixXd> R_llt{R};
  DRAKE_THROW_UNLESS(R_llt.info() == Eigen::Success);

  // Ensure (A, B) pair is stabilizable
  DRAKE_THROW_UNLESS(
      drake::systems::internal::IsStabilizable(A, B, false, std::nullopt));

  // Ensure (A, C) pair where Q = CᵀC is detectable
  //
  // Q = CᵀC = PᵀLDLᵀP
  // Cᵀ = PᵀL√(D)
  // C = (PᵀL√(D))ᵀ
  Eigen::MatrixXd C = (Q_ldlt.transpositionsP().transpose() *
                       // NOLINTNEXTLINE(whitespace/braces)
                       Eigen::MatrixXd{Q_ldlt.matrixL()} *
                       Q_ldlt.vectorD().cwiseSqrt().asDiagonal())
                          .transpose();
  DRAKE_THROW_UNLESS(
      drake::systems::internal::IsDetectable(A, C, false, std::nullopt));

  // Implements the SDA algorithm on page 5 of [1].
  //
  // [1] E. K.-W. Chu, H.-Y. Fan, W.-W. Lin & C.-S. Wang "Structure-Preserving
  //     Algorithms for Periodic Discrete-Time Algebraic Riccati Equations",
  //     International Journal of Control, 77:8, 767-788, 2004.
  //     DOI: 10.1080/00207170410001714988

  // A₀ = A
  Eigen::MatrixXd A_k = A;

  // G₀ = BR⁻¹Bᵀ
  //
  // See equation (4) of [1].
  Eigen::MatrixXd G_k = B * R_llt.solve(B.transpose());

  // H₀ = Q
  //
  // See equation (4) of [1].
  Eigen::MatrixXd H_k;
  Eigen::MatrixXd H_k1 = Q;

  do {
    H_k = H_k1;

    // W = I + GₖHₖ
    Eigen::MatrixXd W =
        Eigen::MatrixXd::Identity(H_k.rows(), H_k.cols()) + G_k * H_k;

    auto W_solver = W.lu();

    // Solve WV₁ = Aₖ for V₁
    Eigen::MatrixXd V_1 = W_solver.solve(A_k);

    // Solve V₂Wᵀ = Gₖ for V₂
    //
    // We want to put V₂Wᵀ = Gₖ into Ax = b form so we can solve it more
    // efficiently.
    //
    // V₂Wᵀ = Gₖ
    // (V₂Wᵀ)ᵀ = Gₖᵀ
    // WV₂ᵀ = Gₖᵀ
    //
    // The solution of Ax = b can be found via x = A.solve(b).
    //
    // V₂ᵀ = W.solve(Gₖᵀ)
    // V₂ = W.solve(Gₖᵀ)ᵀ
    Eigen::MatrixXd V_2 = W_solver.solve(G_k.transpose()).transpose();

    // Gₖ₊₁ = Gₖ + AₖV₂Aₖᵀ
    G_k += A_k * V_2 * A_k.transpose();

    // Hₖ₊₁ = Hₖ + V₁ᵀHₖAₖ
    H_k1 = H_k + V_1.transpose() * H_k * A_k;

    // Aₖ₊₁ = AₖV₁
    A_k *= V_1;

    // while |Hₖ₊₁ − Hₖ| > ε |Hₖ₊₁|
  } while ((H_k1 - H_k).norm() > 1e-10 * H_k1.norm());

  return H_k1;
}

Eigen::MatrixXd DiscreteAlgebraicRiccatiEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N) {
  int states = B.rows();
  int inputs = B.cols();
  DRAKE_DEMAND(N.rows() == states && N.cols() == inputs);

  // Ensure R is positive definite (LLT decomposition fails if not)
  auto R_llt = R.llt();
  DRAKE_THROW_UNLESS(R_llt.info() == Eigen::Success);

  // This is a change of variables to make the DARE that includes Q, R, and N
  // cost matrices fit the form of the DARE that includes only Q and R cost
  // matrices.
  Eigen::MatrixXd A_2 = A - B * R_llt.solve(N.transpose());
  Eigen::MatrixXd Q_2 = Q - N * R_llt.solve(N.transpose());
  return DiscreteAlgebraicRiccatiEquation(A_2, B, Q_2, R);
}

}  // namespace math
}  // namespace drake
