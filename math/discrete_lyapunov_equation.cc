#include "drake/math/discrete_lyapunov_equation.h"

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

MatrixXd RealDiscreteLyapunovEquation(const Eigen::Ref<const MatrixXd>& A,
                                      const Eigen::Ref<const MatrixXd>& Q) {
  if (A.rows() != A.cols() || Q.rows() != Q.cols() || A.rows() != Q.rows()) {
    throw std::runtime_error(
        "RealDiscreteLyapunovEquation(): A and Q must be square and of equal "
        "size!");
  }
  DRAKE_DEMAND(is_approx_equal_abstol(Q, Q.transpose(), 1e-10));

  VectorXcd eig_val{A.eigenvalues()};
  for (int i = 0; i < eig_val.size(); ++i) {
    if (is_zero(eig_val(i).imag()) &&
        (is_zero(eig_val(i).real() - 1) || is_zero(eig_val(i).real() + 1))) {
      throw std::runtime_error(
          "RealDiscreteLyapunovEquation(): Solution is not unique!");
    }
    for (int j = i + 1; j < eig_val.size(); ++j) {
      std::complex<double> eig_prod = eig_val(i) * eig_val(j);
      if (is_zero(eig_prod.real() - 1) && is_zero(eig_prod.imag())) {
        throw std::runtime_error(
            "RealDiscreteLyapunovEquation(): Solution is not unique!");
      }
    }
  }

  // The cases where A is 1-by-1 or 2-by-2 are caught as we have an efficient
  // methods at hand to solve these without the use of Schur factorization.
  if (static_cast<int>(A.cols()) == 1)
    return internal::Solve1By1RealDiscreteLyapunovEquation(A, Q);
  if (static_cast<int>(A.cols()) == 2) {
    // Reducing Q to upper triangular form.
    MatrixXd Q_upper{Q};
    Q_upper(1, 0) = NAN;
    return internal::Solve2By2RealDiscreteLyapunovEquation(A, Q_upper);
  }

  // Transform into reduced form S' * X̅ * S - X̅ = -Q̅, where A = U*S*U', X =
  // U*X̅*U', Q = U*Q̅*U', U is the unitary matrix, and S the reduced form.
  Eigen::RealSchur<MatrixXd> schur(A);
  const MatrixXd U{schur.matrixU()};
  MatrixXd S{schur.matrixT()};

  if (schur.info() != Eigen::Success) {
    throw std::runtime_error(
        "RealDiscreteLyapunovEquation(): Schur factorization failed.");
  }
  // Reduce the symmetric Q̅ to its upper triangular form Q̅_ᵤₚₚₑᵣ.
  MatrixXd Q_bar_upper{MatrixXd::Constant(Q.rows(), Q.cols(), NAN)};
  Q_bar_upper.triangularView<Eigen::Upper>() = U.transpose() * Q * U;
  // This transformation is not in adherence to [2]! In keeping with the
  // routine, the transformation should be U.transpose()*X*U, however this is
  // a typo! See [1] for references!
  return (U *
          internal::SolveReducedRealDiscreteLyapunovEquation(S, Q_bar_upper) *
          U.transpose());
}

namespace internal {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

Vector1d Solve1By1RealDiscreteLyapunovEquation(
    const Eigen::Ref<const Vector1d>& A, const Eigen::Ref<const Vector1d>& Q) {
  return Vector1d(-Q(0) / (A(0) * A(0) - 1));
}

Matrix2d Solve2By2RealDiscreteLyapunovEquation(
    const Eigen::Ref<const Matrix2d>& A, const Eigen::Ref<const Matrix2d>& Q) {
  DRAKE_DEMAND(std::isnan(Q(1, 0)));
  // Rewrite AᵀXA - X + Q = 0 (case where A,Q are 2-by-2) into linear set A_vec
  // *vec(X) = -vec(Q) and solve for X.
  Matrix3d A_vec;
  A_vec << A(0, 0) * A(0, 0) - 1, 2 * A(0, 0) * A(1, 0), A(1, 0) * A(1, 0),
      A(0, 0) * A(0, 1), A(0, 0) * A(1, 1) + A(0, 1) * A(1, 0) - 1,
      A(1, 0) * A(1, 1), A(0, 1) * A(0, 1), 2 * A(0, 1) * A(1, 1),
      A(1, 1) * A(1, 1) - 1;
  const Vector3d Q_vec(-Q(0, 0), -Q(0, 1), -Q(1, 1));

  // See https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html for
  // quick overview of possible solver algorithms.
  // ColPivHouseholderQR is accurate and fast for small systems.
  const Vector3d x{A_vec.colPivHouseholderQr().solve(Q_vec)};
  Matrix2d X;
  X << x(0), x(1), x(1), x(2);

  return X;
}

MatrixXd SolveReducedRealDiscreteLyapunovEquation(
    const Eigen::Ref<const MatrixXd>& S,
    const Eigen::Ref<const MatrixXd>& Q_bar) {
  const int m{static_cast<int>(S.rows())};
  if (m == 1) return Solve1By1RealDiscreteLyapunovEquation(S, Q_bar);
  if (m == 2) return Solve2By2RealDiscreteLyapunovEquation(S, Q_bar);
  // Notation & partition adapted from SB03MD:
  //
  //
  //    S = [s₁₁    s']   Q̅ = [q̅₁₁   q̅']   X̅ = [x̅₁₁    x̅']
  //        [0      S₁] ,     [q̅     Q₁] ,     [x̅      X₁]
  //
  // where s₁₁ is a 1-by-1 or 2-by-2 matrix.
  //
  //    s₁₁'x̅₁₁s₁₁ - x̅₁₁ = -q̅₁₁                                      (4.1)
  //
  //    S₁'x̅s₁₁ - x̅      = -q̅₁₁ - sx̅₁₁s₁₁                            (4.2)
  //
  //    S₁'X₁S₁ - X₁     = -Q₁ - sx̅₁₁s' - [s(S₁'x̅)' + (S₁'x̅)s']      (4.3)

  int n{1};  // n equals the size of the n-by-n block in the left-upper
             // corner of S.
  // Check if the top block of S is 1-by-1 or 2-by-2.
  if (std::abs(S(1, 0)) > kTolerance * S.norm()) {
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
    // solving equation 4.1
    x_bar_11 << Solve1By1RealDiscreteLyapunovEquation(s_11, q_bar_11);
    // solving equation 4.2
    const MatrixXd lhs{s_11(0) * S_1.transpose() -
                       MatrixXd::Identity(S_1.cols(), S_1.rows())};
    const VectorXd rhs{-q_bar - s * x_bar_11 * s_11};
    x_bar << lhs.colPivHouseholderQr().solve(rhs);
  } else {
    // solving equation 4.1
    x_bar_11 << Solve2By2RealDiscreteLyapunovEquation(s_11, q_bar_11);
    // solving equation 4.2
    // The equation reads as S₁'x̅s₁₁ - x̅      = -q̅₁₁ - sx̅₁₁s₁,
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
    //  Writing out the rhs of equation 4.2 gives:
    //
    //  S₁'x̅s₁₁ - x̅ =
    //
    //  [ s₁₁(0,0)*S₁'x̅[0]+s₁₁(1,0)*S₁₁'x̅[1],
    //            s₁₁(0,1)*S₁'x̅[0]+s₁₁(1,1)*S₁'x̅[1] ]
    //
    //  This equation can be vectorized by stacking the columns of x̅.
    //
    //  Define:
    //
    //  x_bar_vec = [x̅[0]' x̅[1]']' where [i] is the ith column
    //
    //  lhs = [s₁(0,0)*S₁' s₁₁(1,0)*S₁'; s₁₁(0,1)*S₁' s₁₁(1,1)*S₁']
    //
    //  rhs = -[(q̅+sx̅₁₁s₁₁)[0]' (q̅+sx̅₁₁s₁₁)[1]']'
    //
    //  lhs*x_vec = rhs.
    //
    //  This expression can be fed into colPivHouseHolderQr() and solved.
    //  Finally, construct x_bar from x_vec.
    //

    MatrixXd lhs(2 * (m - 2), 2 * (m - 2));
    lhs << s_11(0, 0) * S_1.transpose(), s_11(1, 0) * S_1.transpose(),
        s_11(0, 1) * S_1.transpose(), s_11(1, 1) * S_1.transpose();

    VectorXd rhs(2 * (m - 2), 1);
    rhs << (-q_bar - s * x_bar_11 * s_11).col(0),
        (-q_bar - s * x_bar_11 * s_11).col(1);

    VectorXd x_bar_vec{lhs.colPivHouseholderQr().solve(rhs)};
    x_bar.col(0) = x_bar_vec.segment(0, m - 2);
    x_bar.col(1) = x_bar_vec.segment(m - 2, m - 2);
  }

  // Set up equation 4.3
  const Eigen::MatrixXd temp_summand{s * (S_1.transpose() * x_bar).transpose()};
  // Since Q₁ is only a upper triangular matrix, with NAN in the lower part,
  // Q_NEW_ᵤₚₚₑᵣ is so as well.
  const Eigen::MatrixXd Q_new_upper{Q_1 + s * x_bar_11 * s.transpose() +
                                    temp_summand + temp_summand.transpose()};
  // The solution is found recursively.
  MatrixXd X_bar(S.cols(), S.rows());
  X_bar << x_bar_11, x_bar.transpose(), x_bar,
      SolveReducedRealDiscreteLyapunovEquation(S_1, Q_new_upper);

  return X_bar;
}

}  // namespace internal
}  // namespace math
}  // namespace drake
