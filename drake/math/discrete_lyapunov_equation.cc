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

const double kTolerance = 5 * std::numeric_limits<double>::epsilon();

MatrixXd RealDiscreteLyapunovEquation(const Eigen::Ref<const MatrixXd>& A,
                                      const Eigen::Ref<const MatrixXd>& Q) {
  if (A.rows() != A.cols() || Q.rows() != Q.cols() || A.rows() != Q.rows()) {
    throw std::runtime_error("A and Q must be square and of equal size!");
  }

  VectorXcd eig_val{A.eigenvalues()};
  for (int i = 0; i < eig_val.size(); ++i) {
    for (int j = i; j < eig_val.size(); ++j) {
      if (((eig_val(i) * eig_val(j)).real() == 1) &&
          ((eig_val(i) * eig_val(j)).imag() == 0)) {
        throw std::runtime_error("Solution is not unique!");
      }
    }
  }

  // The cases where A is 1-by-1 or 2-by-2 are caught as we have an efficient
  // methods at hand to solve these without the use of Schur factorization.
  if (static_cast<int>(A.cols()) == 1) {
    return internal::Solve1By1RealDiscreteLyapunovEquation(A, Q);
  } else if (static_cast<int>(A.cols()) == 2) {
    // Reducing Q to upper triangular form.
    MatrixXd Q_upper{Q};
    Q_upper(1, 0) = NAN;
    return internal::Solve2By2RealDiscreteLyapunovEquation(A, Q_upper);
  } else {
    // Transform into reduced form S'*X̅  + X̅ *S = -Q̅, where A = U*S*U',
    // U is the unitary matrix, and S the reduced form.
    Eigen::RealSchur<MatrixXd> schur(A);
    MatrixXd U{schur.matrixU()};
    MatrixXd S{schur.matrixT()};

    if (schur.info() != Eigen::Success) {
      throw std::runtime_error("Schur factorization failed.");
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
  // Rewrite A'XA - X + Q = 0 (case where A,Q are 2-by-2) into linear set A_vec
  // *vec(X) = -vec(Q) and
  // solve for X.
  Matrix3d A_vec;
  A_vec << A(0, 0) * A(0, 0) - 1, 2 * A(0, 0) * A(1, 0), A(1, 0) * A(1, 0),
      A(0, 0) * A(0, 1), A(0, 0) * A(1, 1) + A(0, 1) * A(1, 0) - 1,
      A(1, 0) * A(1, 1), A(0, 1) * A(0, 1), 2 * A(0, 1) * A(1, 1),
      A(1, 1) * A(1, 1) - 1;
  Vector3d Q_vec;
  Q_vec << -Q(0, 0), -Q(0, 1), -Q(1, 1);

  // See https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html for
  // quick overview of possible solver algorithms.
  // ColPivHouseholderQR is accurate and fast for small systems.
  Vector3d x{A_vec.colPivHouseholderQr().solve(Q_vec)};
  Matrix2d X;
  X << x(0), x(1), x(1), x(2);

  return X;
}

MatrixXd SolveReducedRealDiscreteLyapunovEquation(
    const Eigen::Ref<const MatrixXd>& S,
    const Eigen::Ref<const MatrixXd>& Q_bar) {
  const int m{static_cast<int>(S.rows())};
  if (m == 1) {
    return Solve1By1RealDiscreteLyapunovEquation(S, Q_bar);
  } else if (m == 2) {
    return Solve2By2RealDiscreteLyapunovEquation(S, Q_bar);
  } else {
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
    if (!(S(1, 0) < kTolerance * S.norm() &&
          S(1, 0) > -kTolerance * S.norm())) {
      n = 2;
    }
    // TODO(FischerGundlach) Reserve memory and pass it to recusrive function
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
      // where S₁ \in R^(m-2)x(m-2); x̅, q̅, s \in R^(m-2)x2 and s₁₁,
      // x̅₁₁ \in
      // R^2x2.
      // We solve the linear equation by vectorization and feeding it into
      // colPivHouseHolderQr().
      //
      //  Notation:
      //
      //  The elements in s₁₁ are names as the following:
      //  s₁₁ = [s₁₁(1,1) s₁₁(1,2); s₁₁(2,1) s₁₁(2,2)].
      //  Note that eigen starts counting at 0, and not as above at 1.
      //
      //  The ith column of a matrix is accessed by [i], i.e. the first column
      //  of x̅ is x̅[1].
      //
      //  Writing out the rhs of equation 4.2 gives:
      //
      //  S₁'x̅s₁₁ - x̅ =
      //
      //  [ s₁₁(1,1)*S₁'x̅[1]+s₁₁(2,1)*S₁₁'x̅[2],
      //            s₁₁(1,2)*S₁'x̅[1]+s₁₁(2,2)*S₁'x̅[2] ]
      //
      //  This equation can be vectorized by stacking the columns of x̅.
      //
      //  Define:
      //
      //  x_bar_vec = [x̅[1]' x̅[2]']' where [i] is the ith column
      //
      //  rhs = [s₁(1,1)*S₁' s₁₁(2,1)*S₁'; s₁₁(1,2)*S₁' s₁₁(2,2)*S₁']
      //
      //  lhs = -[(q̅+sx̅₁₁s₁₁)[1]' (q̅+sx̅₁₁s₁₁)[2]']'
      //
      //  rhs*x_vec = lhs.
      //
      //  This expression can be feed into colPivHouseHolderQr() and solved.
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
    const Eigen::MatrixXd temp_summand{s *
                                       (S_1.transpose() * x_bar).transpose()};
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
}

}  // namespace internal
}  // namespace math
}  // namespace drake
