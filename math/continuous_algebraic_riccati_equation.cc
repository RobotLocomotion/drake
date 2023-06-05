#include "drake/math/continuous_algebraic_riccati_equation.h"

#include <optional>

#include "drake/common/drake_assert.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/systems/primitives/linear_system_internal.h"

namespace drake {
namespace math {

Eigen::MatrixXd ContinuousAlgebraicRiccatiEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::LLT<Eigen::MatrixXd>& R_cholesky) {
  const Eigen::Index n = B.rows(), m = B.cols();
  DRAKE_DEMAND(A.rows() == n && A.cols() == n);
  DRAKE_DEMAND(Q.rows() == n && Q.cols() == n);
  DRAKE_DEMAND(R_cholesky.matrixL().rows() == m &&
               R_cholesky.matrixL().cols() == m);

  DRAKE_DEMAND(is_approx_equal_abstol(Q, Q.transpose(), 1e-10));

  Eigen::MatrixXd H(2 * n, 2 * n);

  H << A, B * R_cholesky.solve(B.transpose()), Q, -A.transpose();
  // For the closed-loop system ẋ = (A+BK)x (where K is the LQR controller
  // gain), we denote the eigenvalues of A+BK as λ₁, ..., λₙ, then with Q
  // satisfying the algebraic Riccati equation, the eigenvalues of H are λ₁,
  // ..., λₙ and -λ₁, ..., -λₙ (refer to
  // https://stanford.edu/class/ee363/lectures/clqr.pdf for more details). Since
  // the LQR gains should stabilize the closed-loop system, we know that the
  // real parts of λ₁, ..., λₙ should be strictly negative. Hence half of the
  // eigenvalues of H should have strictly negative real parts, and the other
  // half have strictly positive real parts.
  if (H.determinant() == 0) {
    // If H.determinant() is 0, then H must have at least one eigenvalue being
    // 0, contradicting to the requirement on H. This is a sufficient condition
    // for the system being stabilizable and detectable, and checking the
    // determinant is cheap.
    throw std::runtime_error(
        "ContinuousAlgebraicRiccatiEquation fails. The Hamiltonian is not "
        "invertible. Either your (A, B) is not stabilizable, or (Q, A) is not "
        "detectable.");
  }
  // Now we actually check if the system is stabilizable and detectable.
  constexpr bool continuous_time = true;
  constexpr std::optional<double> threshold = std::nullopt;
  if (!systems::internal::IsStabilizable(A, B, continuous_time, threshold)) {
    throw std::runtime_error(
        "ContinuousAlgebraicRiccatiEquation fails. The system is not "
        "stabilizable.");
  }
  if (!systems::internal::IsDetectable(A, /* C = */ Q, continuous_time,
                                       threshold)) {
    throw std::runtime_error(
        "ContinuousAlgebraicRiccatiEquation fails. The system is not "
        "detectable.");
  }

  Eigen::MatrixXd Z = H;
  Eigen::MatrixXd Z_old;

  // these could be options
  const double tolerance = 1e-9;
  const int max_iterations = 100;

  double relative_norm;
  size_t iteration = 0;

  const double p = static_cast<double>(Z.rows());

  do {
    Z_old = Z;
    // R. Byers. Solving the algebraic Riccati equation with the matrix sign
    // function. Linear Algebra Appl., 85:267–279, 1987
    // Added determinant scaling to improve convergence (converges in rough half
    // the iterations with this)
    double ck = std::pow(std::abs(Z.determinant()), -1.0 / p);
    Z *= ck;
    Z = Z - 0.5 * (Z - Z.inverse());
    relative_norm = (Z - Z_old).norm();
    iteration++;
  } while (iteration < max_iterations && relative_norm > tolerance);

  Eigen::MatrixXd W11 = Z.block(0, 0, n, n);
  Eigen::MatrixXd W12 = Z.block(0, n, n, n);
  Eigen::MatrixXd W21 = Z.block(n, 0, n, n);
  Eigen::MatrixXd W22 = Z.block(n, n, n, n);

  Eigen::MatrixXd lhs(2 * n, n);
  Eigen::MatrixXd rhs(2 * n, n);
  Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(n, n);
  lhs << W12, W22 + eye;
  rhs << W11 + eye, W21;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      lhs, Eigen::ComputeThinU | Eigen::ComputeThinV);

  return svd.solve(rhs);
}

Eigen::MatrixXd ContinuousAlgebraicRiccatiEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R) {
  DRAKE_DEMAND(is_approx_equal_abstol(R, R.transpose(), 1e-10));

  Eigen::LLT<Eigen::MatrixXd> R_cholesky(R);
  if (R_cholesky.info() != Eigen::Success)
    throw std::runtime_error("R must be positive definite");
  return ContinuousAlgebraicRiccatiEquation(A, B, Q, R_cholesky);
}

}  // namespace math
}  // namespace drake
