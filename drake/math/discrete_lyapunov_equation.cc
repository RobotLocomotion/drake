#include "drake/math/discrete_lyapunov_equation.h"

#include <complex>
#include <stdexcept>

#include "drake/common/drake_assert.h"
#include "drake/common/is_approx_equal_abstol.h"

namespace drake {
namespace math {

using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::VectorXcd;

MatrixXd RealDiscreteLyapunovEquation(const Eigen::Ref<const MatrixXd>& A,
                                      const Eigen::Ref<const MatrixXd>& Q) {
  if (A.rows() != A.cols() || Q.rows() != Q.cols() || A.rows() != Q.rows()) {
    throw std::runtime_error("A and Q must be square and of equal size!");
  }

  if (!(is_approx_equal_abstol(Q, Q.transpose(), 0))) {
    throw std::runtime_error("Q must be symmetric!");
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

  if (static_cast<int>(A.cols()) == 1) {
    return internal::Solve1By1RealDiscreteLyapunovEquation(A, Q);
  } else if (static_cast<int>(A.cols()) == 2) {
    return internal::Solve2By2RealDiscreteLyapunovEquation(A, Q);
  } else {
    // Transform into S'*X_bar + X_bar*S = -Q_bar, where A = U*S*U'.
    Eigen::RealSchur<MatrixXd> schur(A);
    MatrixXd U{schur.matrixU()};
    MatrixXd S{schur.matrixT()};

    if (schur.info() != Eigen::Success) {
      throw std::runtime_error("Schur factorization failed.");
    }
    MatrixXd Q_bar{U.transpose() * Q * U};
    return (U.transpose() *
            internal::SolveReducedRealDiscreteLyapunovEquation(S, Q_bar) * U);
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
  DRAKE_DEMAND(
      is_approx_equal_abstol(Q.block(1, 0, 1, 1), Q.block(0, 1, 1, 1), 1e-10));
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
  int m{static_cast<int>(S.rows())};
  if (m == 1) {
    return Solve1By1RealDiscreteLyapunovEquation(S, Q_bar);
  } else if (m == 2) {
    return Solve2By2RealDiscreteLyapunovEquation(S, Q_bar);
  } else {
    // Notation & partition adapted from SB03MD:
    //        (s    s')      (q   q')      (x   x')
    //        ( 11    )  _   ( 11   )  _   ( 11   )
    //    S = (       ), Q = (      ), X = (      )
    //        (       )      ( _    )      ( _    )
    //        ( 0   S )      ( q  Q )      ( x  X )
    //               1             1             1
    // where s_11 is a 1-by-1 or 2-by-2 matrix.
    //
    //    s' x  s  - x     = -q                                     (4.1)
    //     11 11 11   11       11
    //
    //      _     _           _    _
    //    S'xs  - x        = -q - sx  s                             (4.2)
    //     1  11                    11 11
    //
    //      _            _        _
    //    S'X S - X        = -Q - sx  s' - [s(S'x)' + (S'x)s']      (4.3)
    //     1 1 1   1           1    11         1        1

    int n{1};  // n equals the size of the n-by-n block in the left-upper
               // corner of S.
    if (!(is_approx_equal_abstol(S.block(1, 0, 1, 1), Vector1d(0), 1e-3))) {
      n = 2;
    }
    MatrixXd s_11{S.topLeftCorner(n, n)};
    MatrixXd s{S.transpose().bottomLeftCorner(m - n, n)};
    MatrixXd S_1{S.bottomRightCorner(m - n, m - n)};
    MatrixXd q_bar_11{Q_bar.topLeftCorner(n, n)};
    MatrixXd q_bar{Q_bar.transpose().bottomLeftCorner(m - n, n)};
    MatrixXd Q_1{Q_bar.bottomRightCorner(m - n, m - n)};

    MatrixXd x_bar_11(n, n), x_bar(m - n, n);
    if (n == 1) {
      // solving equation 4.1
      x_bar_11 << Solve1By1RealDiscreteLyapunovEquation(s_11, q_bar_11);
      // solving equation 4.2
      MatrixXd lhs{s_11(0) * S_1.transpose() -
                   MatrixXd::Identity(S_1.cols(), S_1.rows())};
      VectorXd rhs{-q_bar - s * x_bar_11 * s_11};
      x_bar << lhs.colPivHouseholderQr().solve(rhs);
    } else {
      // solving equation 4.1
      x_bar_11 << Solve2By2RealDiscreteLyapunovEquation(s_11, q_bar_11);
      // solving equation 4.2
      // The equation reads as S_1'*x_bar*s_11 - x_bar =
      // -(q_bar+s*x_bar_11*s_11),
      // where S_1 \in R^(m-2)x(m-2); x_bar, q_bar, s \in R^(m-2)x2 and s_11,
      // x_bar_11 \in
      // R^2x2.
      // We solve the linear equation by vectorization and feeding it into
      // colPivHouseHolderQr().
      //
      //  Notation:
      //
      //  The elements in s_11 are names as the following:
      //  s_11 = [s_11(1,1) s_11(1,2); s_11(2,1) s_11(2,2)].
      //  Note that eigen starts counting at 0, and not as above at 1.
      //
      //  The ith column of a matrix is accessed by [i], i.e. the first column
      //  of x_bar is x_bar[1].
      //
      //  Writing out the rhs of equation 4.2 gives:
      //
      //  S_1'*x_bar*s_11 - x_bar =
      //
      //  [ s_11(1,1)*S_1'*x_bar[1]+s_11(2,1)*S_1'*x_bar[2],
      //            s_11(1,2)*S_1'*x_bar[1]+s_11(2,2)*S_1'*x_bar[2] ]
      //
      //  This equation can be vectorized by stacking the columns of x_bar.
      //
      //  Define:
      //
      //  x_vec = [x_bar[1]' x_bar[2]']' where [i] is the ith column
      //
      //  rhs = [s_11(1,1)*S_1' s_11(2,1)*S_1'; s_11(1,2)*S_1' s_11(2,2)*S_1']
      //
      //  lhs = -[(q_bar+s*x_bar_11*s_11)[1]' (q_bar+s*x_bar_11*s_11)[2]']'
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
    Eigen::MatrixXd Q_new{Q_1 + s * x_bar_11 * s.transpose() +
                          s * (S_1.transpose() * x_bar).transpose() -
                          S_1.transpose() * x_bar * s.transpose()};

    // The solution is found recursively.
    MatrixXd X_bar(S.cols(), S.rows());
    X_bar << x_bar_11, x_bar.transpose(), x_bar,
        SolveReducedRealDiscreteLyapunovEquation(S_1, Q_new);

    return X_bar;
  }
}

}  // namespace internal
}  // namespace math
}  // namespace drake
