#include "drake/math/continuous_lyapunov_equation.h"

#include <cmath>
#include <complex>
#include <limits>
#include <stdexcept>

#include "drake/common/drake_assert.h"
#include "drake/common/is_approx_equal_abstol.h"

namespace drake {
namespace math {

using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::VectorXcd;

namespace {
const double kTolerance = 5 * std::numeric_limits<double>::epsilon();
bool is_zero(double x, double eps = 1e-10) { return std::fabs(x) < eps; }
// TODO(weiqiao.han): figure out what the tolerance ε ought to be.
}  // namespace

MatrixXd RealContinuousLyapunovEquation(const Eigen::Ref<const MatrixXd>& A,
                                        const Eigen::Ref<const MatrixXd>& Q) {
  if (A.rows() != A.cols() || Q.rows() != Q.cols() || A.rows() != Q.rows()) {
    throw std::runtime_error(
        "RealContinuousLyapunovEquation(): A and Q must be square and of equal "
        "size!");
  }
  DRAKE_DEMAND(is_approx_equal_abstol(Q, Q.transpose(), 1e-10));

  // The cases where A is 1-by-1 or 2-by-2 are caught as we have an efficient
  // methods at hand to solve these without the use of Schur factorization.
  if (static_cast<int>(A.cols()) == 1) {
    if (is_zero(A(0, 0))) {
      throw std::runtime_error(
          "RealContinuousLyapunovEquation(): Solution is not unique!");
    }
    return internal::Solve1By1RealContinuousLyapunovEquation(A, Q);
  }
  if (static_cast<int>(A.cols()) == 2) {
    // If A has two real eigenvalues λ₁ and λ₂, then we check (1) if λ₁ or λ₂ is
    // 0, i.e. if the determinant of A is 0; (2) if λ₁ + λ₂ = 0, i.e., if the
    // trace of A is 0. If A has two complex eigenvalues λ₁ and λ̅₁, then we
    // check if λ₁ + λ̅₁ = 0, i.e., if the trace of A is 0.
    if (is_zero(A(0, 0) + A(1, 1)) ||
        is_zero(A(0, 0) * A(1, 1) - A(1, 0) * A(0, 1))) {
      throw std::runtime_error(
          "RealContinuousLyapunovEquation(): Solution is not unique!");
    }
    // Reducing Q to upper triangular form.
    MatrixXd Q_upper{Q};
    Q_upper(1, 0) = NAN;
    return internal::Solve2By2RealContinuousLyapunovEquation(A, Q_upper);
  }
  VectorXcd eig_val{A.eigenvalues()};
  for (int i = 0; i < eig_val.size(); ++i) {
    for (int j = i; j < eig_val.size(); ++j) {
      std::complex<double> eig_sum = eig_val(i) + std::conj(eig_val(j));
      if (is_zero(eig_sum.real()) && is_zero(eig_sum.imag())) {
        throw std::runtime_error(
            "RealContinuousLyapunovEquation(): Solution is not unique!");
      }
    }
  }
  // Transform into reduced form S'*X̅  + X̅ *S = -Q̅, where A = U*S*U', X̅ =
  // U'*X*U, Q̅ = U'*Q*U, U is the unitary matrix, and S the reduced form. Note
  // that the expression for X̅ in [2] is incorrect.
  Eigen::RealSchur<MatrixXd> schur(A);
  const MatrixXd U{schur.matrixU()};
  const MatrixXd S{schur.matrixT()};
  if (schur.info() != Eigen::Success) {
    throw std::runtime_error(
        "RealContinuousLyapunovEquation(): Schur factorization failed.");
  }
  // Reduce the symmetric Q̅ to its upper triangular form Q̅_ᵤₚₚₑᵣ.
  MatrixXd Q_bar_upper{MatrixXd::Constant(Q.rows(), Q.cols(), NAN)};
  Q_bar_upper.triangularView<Eigen::Upper>() = U.transpose() * Q * U;
  return (U *
          internal::SolveReducedRealContinuousLyapunovEquation(S, Q_bar_upper) *
          U.transpose());
}

namespace internal {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

Vector1d Solve1By1RealContinuousLyapunovEquation(
    const Eigen::Ref<const Vector1d>& A, const Eigen::Ref<const Vector1d>& Q) {
  return Vector1d(-Q(0) / (2 * A(0)));
}

Matrix2d Solve2By2RealContinuousLyapunovEquation(
    const Eigen::Ref<const Matrix2d>& A, const Eigen::Ref<const Matrix2d>& Q) {
  // Rewrite AᵀX + XA + Q = 0 (case where A,Q are 2-by-2) into linear set A_vec
  // *vec(X) = -vec(Q) and solve for X.
  // Only the upper triangular part of Q is used, thus it is treated to be
  // symmetric.
  Matrix3d A_vec;
  A_vec << 2 * A(0, 0), 2 * A(1, 0), 0, A(0, 1), A(0, 0) + A(1, 1), A(1, 0), 0,
      2 * A(0, 1), 2 * A(1, 1);
  const Vector3d Q_vec(-Q(0, 0), -Q(0, 1), -Q(1, 1));

  // See https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html for
  // quick overview of possible solver algorithms.
  // ColPivHouseholderQR is accurate and fast for small systems.
  Vector3d x{A_vec.colPivHouseholderQr().solve(Q_vec)};
  Matrix2d X;
  X << x(0), x(1), x(1), x(2);
  return X;
}

MatrixXd SolveReducedRealContinuousLyapunovEquation(
    const Eigen::Ref<const MatrixXd>& S,
    const Eigen::Ref<const MatrixXd>& Q_bar) {
  const int m{static_cast<int>(S.rows())};
  if (m == 1) return Solve1By1RealContinuousLyapunovEquation(S, Q_bar);
  if (m == 2) return Solve2By2RealContinuousLyapunovEquation(S, Q_bar);
  // Notation & partition adapted from SB03MD:
  //
  //
  //    S = [s₁₁    s']   Q̅ = [q̅₁₁   q̅']   X̅ = [x̅₁₁    x̅']
  //        [0      S₁] ,     [q̅     Q₁] ,     [x̅      X₁]
  //
  // where s₁₁ is a 1-by-1 or 2-by-2 matrix.
  //
  //    s₁₁'x̅₁₁ + x̅₁₁s₁₁ = -q̅₁₁                                      (3.1)
  //
  //    S₁'x̅ + x̅s₁₁      = -q̅₁₁ - sx̅₁₁                               (3.2)
  //
  //    S₁'X₁ + X₁S₁     = -Q₁ - sx̅' - x̅s'                           (3.3)

  int n{1};  // n equals the size of the n-by-n block in the left-upper
             // corner of S.
  // Check if the top block of S is 1-by-1 or 2-by-2.
  if (!(std::abs(S(1, 0)) < kTolerance * S.norm())) {
    n = 2;
  }
  // TODO(FischerGundlach) Reserve memory and pass it to recursive function
  // calls.
  const MatrixXd s_11{S.topLeftCorner(n, n)};
  const MatrixXd s{S.transpose().bottomLeftCorner(m - n, n)};
  const MatrixXd S_1{S.bottomRightCorner(m - n, m - n)};
  const MatrixXd q_bar_11{Q_bar.topLeftCorner(n, n)};
  const MatrixXd q_bar{Q_bar.transpose().bottomLeftCorner(m - n, n)};
  const MatrixXd Q_1{Q_bar.bottomRightCorner(m - n, m - n)};

  MatrixXd x_bar_11(n, n), x_bar(m - n, n);
  if (n == 1) {
    // solving equation 3.1
    x_bar_11 << Solve1By1RealContinuousLyapunovEquation(s_11, q_bar_11);
    // solving equation 3.2
    const MatrixXd lhs{S_1.transpose() +
                       MatrixXd::Identity(S_1.cols(), S_1.rows()) * s_11(0)};
    const VectorXd rhs{-q_bar - s * x_bar_11(0)};
    x_bar << lhs.colPivHouseholderQr().solve(rhs);
  } else {
    x_bar_11 << Solve2By2RealContinuousLyapunovEquation(s_11, q_bar_11);
    // solving equation 3.2
    // The equation reads as S₁'x̅ + x̅s₁₁ = -q̅₁₁ - sx̅₁₁,
    // where S₁ ∈ ℝᵐ⁻²ˣᵐ⁻²; x̅, q̅, s ∈ ℝᵐ⁻²ˣ² and s₁₁, x̅₁₁ ∈ ℝ²ˣ².
    // We solve the linear equation by vectorization and feeding it into
    // colPivHouseHolderQr().
    //
    //  Notation:
    //
    //  The elements in s₁₁ are names as the following:
    //  s₁₁ = [s₁₁(0,0) s₁₁(0,1); s₁₁(1,0) s₁₁(1,1)].
    //  Note that eigen starts counting at 0, and not as above at 1.
    //
    //  The ith column of a matrix is accessed by [i], i.e. the first column
    //  of x̅ is x̅[0].
    //
    //  Define:
    //
    //  S_1_vec = [S₁' 0; 0 S₁'] with 0 \in ∈ℝᵐˣᵐ
    //
    //  S_11_vec = [s₁₁(0,0)*I s₁₁(1,0)*I; s₁₁(0,1)*I s₁₁(1,1)*I] with I ∈ ℝᵐˣᵐ.
    //
    //  Now define lhs = S_1_vec + S_11_vec.
    //
    //  x_vec = [x̅[0]' x̅[1]']' where [i] is the ith column
    //
    //  rhs = - [(q̅+sx̅₁₁)[0]' (q̅+s*x̅₁₁)[1]']'
    //
    //  Now S₁'x̅ + x̅s₁₁ = -q̅₁₁ - sx̅₁₁  can be rewritten into:
    //
    //  lhs*x_vec = rhs.
    //
    //  This expression can be fed into colPivHouseHolderQr() and solved.
    //  Finally, construct x_bar from x_vec.
    //

    MatrixXd S_1_vec(2 * (m - 2), 2 * (m - 2));
    S_1_vec << S_1.transpose(), MatrixXd::Zero(m - 2, m - 2),
        MatrixXd::Zero(m - 2, m - 2), S_1.transpose();

    MatrixXd s_11_vec(2 * (m - 2), 2 * (m - 2));
    s_11_vec << s_11(0, 0) * MatrixXd::Identity(m - 2, m - 2),
        s_11(1, 0) * MatrixXd::Identity(m - 2, m - 2),
        s_11(0, 1) * MatrixXd::Identity(m - 2, m - 2),
        s_11(1, 1) * MatrixXd::Identity(m - 2, m - 2);
    MatrixXd lhs{S_1_vec + s_11_vec};

    VectorXd rhs(2 * (m - 2), 1);
    rhs << (-q_bar - s * x_bar_11).col(0), (-q_bar - s * x_bar_11).col(1);

    VectorXd x_bar_vec{lhs.colPivHouseholderQr().solve(rhs)};
    x_bar.col(0) = x_bar_vec.segment(0, m - 2);
    x_bar.col(1) = x_bar_vec.segment(m - 2, m - 2);
  }

  // Set up equation 3.3
  const Eigen::MatrixXd temp_summand{s * x_bar.transpose()};
  // Since Q₁ is only an upper triangular matrix, with NAN in the lower part,
  // Q_NEW_ᵤₚₚₑᵣ is so as well.
  const Eigen::MatrixXd Q_new_upper{
      (Q_1 + temp_summand + temp_summand.transpose())};
  // The solution is found recursively.
  MatrixXd X_bar(S.cols(), S.rows());
  X_bar << x_bar_11, x_bar.transpose(), x_bar,
      SolveReducedRealContinuousLyapunovEquation(S_1, Q_new_upper);

  return X_bar;
}

}  // namespace internal
}  // namespace math
}  // namespace drake
