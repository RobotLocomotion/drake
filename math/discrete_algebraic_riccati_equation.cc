#include "drake/math/discrete_algebraic_riccati_equation.h"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/is_approx_equal_abstol.h"

namespace drake {
namespace math {
namespace {
/* helper functions */
template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}
void check_stabilizable(const Eigen::Ref<const Eigen::MatrixXd>& A,
                        const Eigen::Ref<const Eigen::MatrixXd>& B) {
  // This function checks if (A,B) is a stabilizable pair.
  // (A,B) is stabilizable if and only if the uncontrollable eigenvalues of
  // A, if any, have absolute values less than one, where an eigenvalue is
  // uncontrollable if Rank[lambda * I - A, B] < n.
  int n = B.rows(), m = B.cols();
  Eigen::EigenSolver<Eigen::MatrixXd> es(A);
  for (int i = 0; i < n; i++) {
    if (es.eigenvalues()[i].real() * es.eigenvalues()[i].real() +
            es.eigenvalues()[i].imag() * es.eigenvalues()[i].imag() <
        1)
      continue;
    Eigen::MatrixXcd E(n, n + m);
    E << es.eigenvalues()[i] * Eigen::MatrixXcd::Identity(n, n) - A, B;
    Eigen::ColPivHouseholderQR<Eigen::MatrixXcd> qr(E);
    DRAKE_THROW_UNLESS(qr.rank() == n);
  }
}
void check_detectable(const Eigen::Ref<const Eigen::MatrixXd>& A,
                      const Eigen::Ref<const Eigen::MatrixXd>& Q) {
  // This function check if (A,C) is a detectable pair, where Q = C' * C.
  // (A,C) is detectable if and only if the unobservable eigenvalues of A,
  // if any, have absolute values less than one, where an eigenvalue is
  // unobservable if Rank[lambda * I - A; C] < n.
  // Also, (A,C) is detectable if and only if (A',C') is stabilizable.
  int n = A.rows();
  Eigen::LDLT<Eigen::MatrixXd> ldlt(Q);
  Eigen::MatrixXd L = ldlt.matrixL();
  Eigen::MatrixXd D = ldlt.vectorD();
  Eigen::MatrixXd D_sqrt = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < n; i++) {
    D_sqrt(i, i) = sqrt(D(i));
  }
  Eigen::MatrixXd C = L * D_sqrt;
  check_stabilizable(A.transpose(), C.transpose());
}

// "Givens rotation" computes an orthogonal 2x2 matrix R such that
// it eliminates the 2nd coordinate of the vector [a,b]', i.e.,
// R * [ a ] = [ a_hat ]
//     [ b ]   [   0   ]
// The implementation is based on
// https://en.wikipedia.org/wiki/Givens_rotation#Stable_calculation
void Givens_rotation(double a, double b, Eigen::Ref<Eigen::Matrix2d> R,
                     double eps = 1e-10) {
  double c, s;
  if (fabs(b) < eps) {
    c = (a < -eps ? -1 : 1);
    s = 0;
  } else if (fabs(a) < eps) {
    c = 0;
    s = -sgn(b);
  } else if (fabs(a) > fabs(b)) {
    double t = b / a;
    double u = sgn(a) * fabs(sqrt(1 + t * t));
    c = 1 / u;
    s = -c * t;
  } else {
    double t = a / b;
    double u = sgn(b) * fabs(sqrt(1 + t * t));
    s = -1 / u;
    c = -s * t;
  }
  R(0, 0) = c, R(0, 1) = -s, R(1, 0) = s, R(1, 1) = c;
}

// The arguments S, T, and Z will be changed.
void swap_block_11(Eigen::Ref<Eigen::MatrixXd> S, Eigen::Ref<Eigen::MatrixXd> T,
                   Eigen::Ref<Eigen::MatrixXd> Z, int p) {
  // Dooren, Case I, p124-125.
  int n2 = S.rows();
  Eigen::Matrix2d A = S.block<2, 2>(p, p);
  Eigen::Matrix2d B = T.block<2, 2>(p, p);
  Eigen::MatrixXd Z1 = Eigen::MatrixXd::Identity(n2, n2);
  Eigen::Matrix2d H = A(1, 1) * B - B(1, 1) * A;
  Givens_rotation(H(0, 1), H(0, 0), Z1.block<2, 2>(p, p));
  S = (S * Z1).eval();
  T = (T * Z1).eval();
  Z = (Z * Z1).eval();
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(n2, n2);
  Givens_rotation(T(p, p), T(p + 1, p), Q.block<2, 2>(p, p));
  S = (Q * S).eval();
  T = (Q * T).eval();
  S(p + 1, p) = 0;
  T(p + 1, p) = 0;
}

// The arguments S, T, and Z will be changed.
void swap_block_21(Eigen::Ref<Eigen::MatrixXd> S, Eigen::Ref<Eigen::MatrixXd> T,
                   Eigen::Ref<Eigen::MatrixXd> Z, int p) {
  // Dooren, Case II, p126-127.
  int n2 = S.rows();
  Eigen::Matrix3d A = S.block<3, 3>(p, p);
  Eigen::Matrix3d B = T.block<3, 3>(p, p);
  // Compute H and eliminate H(1,0) by row operation.
  Eigen::Matrix3d H = A(2, 2) * B - B(2, 2) * A;
  Eigen::Matrix3d R = Eigen::MatrixXd::Identity(3, 3);
  Givens_rotation(H(0, 0), H(1, 0), R.block<2, 2>(0, 0));
  H = (R * H).eval();
  Eigen::MatrixXd Z1 = Eigen::MatrixXd::Identity(n2, n2);
  Eigen::MatrixXd Z2 = Eigen::MatrixXd::Identity(n2, n2);
  Eigen::MatrixXd Q1 = Eigen::MatrixXd::Identity(n2, n2);
  Eigen::MatrixXd Q2 = Eigen::MatrixXd::Identity(n2, n2);
  // Compute Z1, Z2, Q1, Q2.
  Givens_rotation(H(1, 2), H(1, 1), Z1.block<2, 2>(p + 1, p + 1));
  H = (H * Z1.block<3, 3>(p, p)).eval();
  Givens_rotation(H(0, 1), H(0, 0), Z2.block<2, 2>(p, p));
  S = (S * Z1).eval();
  T = (T * Z1).eval();
  Z = (Z * Z1 * Z2).eval();
  Givens_rotation(T(p + 1, p + 1), T(p + 2, p + 1),
                  Q1.block<2, 2>(p + 1, p + 1));
  S = (Q1 * S * Z2).eval();
  T = (Q1 * T * Z2).eval();
  Givens_rotation(T(p, p), T(p + 1, p), Q2.block<2, 2>(p, p));
  S = (Q2 * S).eval();
  T = (Q2 * T).eval();
  S(p + 1, p) = 0;
  S(p + 2, p) = 0;
  T(p + 1, p) = 0;
  T(p + 2, p) = 0;
  T(p + 2, p + 1) = 0;
}

// The arguments S, T, and Z will be changed.
void swap_block_12(Eigen::Ref<Eigen::MatrixXd> S, Eigen::Ref<Eigen::MatrixXd> T,
                   Eigen::Ref<Eigen::MatrixXd> Z, int p) {
  int n2 = S.rows();
  // Swap the role of S and T.
  Eigen::MatrixXd Z1 = Eigen::MatrixXd::Identity(n2, n2);
  Eigen::MatrixXd Z2 = Eigen::MatrixXd::Identity(n2, n2);
  Eigen::MatrixXd Q0 = Eigen::MatrixXd::Identity(n2, n2);
  Eigen::MatrixXd Q1 = Eigen::MatrixXd::Identity(n2, n2);
  Eigen::MatrixXd Q2 = Eigen::MatrixXd::Identity(n2, n2);
  Eigen::MatrixXd Q3 = Eigen::MatrixXd::Identity(n2, n2);
  Givens_rotation(S(p + 1, p + 1), S(p + 2, p + 1),
                  Q0.block<2, 2>(p + 1, p + 1));
  S = (Q0 * S).eval();
  T = (Q0 * T).eval();
  Eigen::Matrix3d A = S.block<3, 3>(p, p);
  Eigen::Matrix3d B = T.block<3, 3>(p, p);
  // Compute H and eliminate H(2,1) by column operation.
  Eigen::Matrix3d H = B(0, 0) * A - A(0, 0) * B;
  Eigen::Matrix3d R = Eigen::MatrixXd::Identity(3, 3);
  Givens_rotation(H(2, 2), H(2, 1), R.block<2, 2>(1, 1));
  H = (H * R).eval();
  // Compute Q1, Q2, Z1, Z2.
  Givens_rotation(H(0, 1), H(1, 1), Q1.block<2, 2>(p, p));
  H = (Q1.block<3, 3>(p, p) * H).eval();
  Givens_rotation(H(1, 2), H(2, 2), Q2.block<2, 2>(p + 1, p + 1));
  S = (Q1 * S).eval();
  T = (Q1 * T).eval();
  Givens_rotation(S(p + 1, p + 1), S(p + 1, p), Z1.block<2, 2>(p, p));
  S = (Q2 * S * Z1).eval();
  T = (Q2 * T * Z1).eval();
  Givens_rotation(S(p + 2, p + 2), S(p + 2, p + 1),
                  Z2.block<2, 2>(p + 1, p + 1));
  S = (S * Z2).eval();
  T = (T * Z2).eval();
  Z = (Z * Z1 * Z2).eval();
  // Swap back the role of S and T.
  Givens_rotation(T(p, p), T(p + 1, p), Q3.block<2, 2>(p, p));
  S = (Q3 * S).eval();
  T = (Q3 * T).eval();
  S(p + 2, p) = 0;
  S(p + 2, p + 1) = 0;
  T(p + 1, p) = 0;
  T(p + 2, p) = 0;
  T(p + 2, p + 1) = 0;
}

// The arguments S, T, and Z will be changed.
void swap_block_22(Eigen::Ref<Eigen::MatrixXd> S, Eigen::Ref<Eigen::MatrixXd> T,
                   Eigen::Ref<Eigen::MatrixXd> Z, int p) {
  // Direct Swapping Algorithm based on
  // "Numerical Methods for General and Structured Eigenvalue Problems" by
  // Daniel Kressner, p108-111.
  // ( http://sma.epfl.ch/~anchpcommon/publications/kressner_eigenvalues.pdf ).
  // Also relevant but not applicable here:
  // "On Swapping Diagonal Blocks in Real Schur Form" by Zhaojun Bai and James
  // W. Demmelt;
  int n2 = S.rows();
  Eigen::MatrixXd A = S.block<4, 4>(p, p);
  Eigen::MatrixXd B = T.block<4, 4>(p, p);
  // Solve
  // A11 * X - Y A22 = A12
  // B11 * X - Y B22 = B12
  // Reduce to solve Cx=D, where x=[x1;...;x4;y1;...;y4].
  Eigen::Matrix<double, 8, 8> C = Eigen::Matrix<double, 8, 8>::Zero();
  Eigen::Matrix<double, 8, 1> D;
  // clang-format off
  C << A(0, 0), 0, A(0, 1), 0, -A(2, 2), -A(3, 2), 0, 0,
       0, A(0, 0), 0, A(0, 1), -A(2, 3), -A(3, 3), 0, 0,
       A(1, 0), 0, A(1, 1), 0, 0, 0, -A(2, 2), -A(3, 2),
       0, A(1, 0), 0, A(1, 1), 0, 0, -A(2, 3), -A(3, 3),
       B(0, 0), 0, B(0, 1), 0, -B(2, 2), -B(3, 2), 0, 0,
       0, B(0, 0), 0, B(0, 1), -B(2, 3), -B(3, 3), 0, 0,
       B(1, 0), 0, B(1, 1), 0, 0, 0, -B(2, 2), -B(3, 2),
       0, B(1, 0), 0, B(1, 1), 0, 0, -B(2, 3), -B(3, 3);
  // clang-format on
  D << A(0, 2), A(0, 3), A(1, 2), A(1, 3), B(0, 2), B(0, 3), B(1, 2), B(1, 3);
  Eigen::MatrixXd x = C.colPivHouseholderQr().solve(D);
  // Q * [ -Y ] = [ R_Y ] ,  Z' * [ -X ] = [ R_X ] .
  //     [ I  ]   [  0  ]         [ I  ] = [  0  ]
  Eigen::Matrix<double, 4, 2> X, Y;
  X << -x(0, 0), -x(1, 0), -x(2, 0), -x(3, 0), Eigen::Matrix2d::Identity();
  Y << -x(4, 0), -x(5, 0), -x(6, 0), -x(7, 0), Eigen::Matrix2d::Identity();
  Eigen::MatrixXd Q1 = Eigen::MatrixXd::Identity(n2, n2);
  Eigen::MatrixXd Z1 = Eigen::MatrixXd::Identity(n2, n2);
  Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 4, 2> > qr1(X);
  Z1.block<4, 4>(p, p) = qr1.householderQ();
  Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 4, 2> > qr2(Y);
  Q1.block<4, 4>(p, p) = qr2.householderQ().adjoint();
  // Apply transform Q1 * (S,T) * Z1.
  S = (Q1 * S * Z1).eval();
  T = (Q1 * T * Z1).eval();
  Z = (Z * Z1).eval();
  // Eliminate the T(p+3,p+2) entry.
  Eigen::MatrixXd Q2 = Eigen::MatrixXd::Identity(n2, n2);
  Givens_rotation(T(p + 2, p + 2), T(p + 3, p + 2),
                  Q2.block<2, 2>(p + 2, p + 2));
  S = (Q2 * S).eval();
  T = (Q2 * T).eval();
  // Eliminate the T(p+1,p) entry.
  Eigen::MatrixXd Q3 = Eigen::MatrixXd::Identity(n2, n2);
  Givens_rotation(T(p, p), T(p + 1, p), Q3.block<2, 2>(p, p));
  S = (Q3 * S).eval();
  T = (Q3 * T).eval();
  S(p + 2, p) = 0;
  S(p + 2, p + 1) = 0;
  S(p + 3, p) = 0;
  S(p + 3, p + 1) = 0;
  T(p + 1, p) = 0;
  T(p + 2, p) = 0;
  T(p + 2, p + 1) = 0;
  T(p + 3, p) = 0;
  T(p + 3, p + 1) = 0;
  T(p + 3, p + 2) = 0;
}

// Functionality of "swap_block" function:
// swap the 1x1 or 2x2 blocks pointed by p and q.
// There are four cases: swapping 1x1 and 1x1 matrices, swapping 2x2 and 1x1
// matrices, swapping 1x1 and 2x2 matrices, and swapping 2x2 and 2x2 matrices.
// Algorithms are described in the papers
// "A generalized eigenvalue approach for solving Riccati equations" by P. Van
// Dooren, 1981 ( http://epubs.siam.org/doi/pdf/10.1137/0902010 ), and
// "Numerical Methods for General and Structured Eigenvalue Problems" by
// Daniel Kressner, 2005.
void swap_block(Eigen::Ref<Eigen::MatrixXd> S, Eigen::Ref<Eigen::MatrixXd> T,
                Eigen::Ref<Eigen::MatrixXd> Z, int p, int q, int q_block_size,
                double eps = 1e-10) {
  int p_tmp = q, p_block_size;
  while (p_tmp-- > p) {
    p_block_size = 1;
    if (p_tmp >= 1 && fabs(S(p_tmp, p_tmp - 1)) > eps) {
      p_block_size = 2;
      p_tmp--;
    }
    switch (p_block_size * 10 + q_block_size) {
      case 11:
        swap_block_11(S, T, Z, p_tmp);
        break;
      case 21:
        swap_block_21(S, T, Z, p_tmp);
        break;
      case 12:
        swap_block_12(S, T, Z, p_tmp);
        break;
      case 22:
        swap_block_22(S, T, Z, p_tmp);
        break;
    }
  }
}

// Functionality of "reorder_eigen" function:
// Reorder the eigenvalues of (S,T) such that the top-left n by n matrix has
// stable eigenvalues by multiplying Q's and Z's on the left and the right,
// respectively.
// Stable eigenvalues are inside the unit disk.
//
// Algorithm:
// Go along the diagonals of (S,T) from the top left to the bottom right.
// Once find a stable eigenvalue, push it to top left.
// In implementation, use two pointers, p and q.
// p points to the current block (1x1 or 2x2) and q points to the block with the
// stable eigenvalue(s).
// Push the block pointed by q to the position pointed by p.
// Finish when n stable eigenvalues are placed at the top-left n by n matrix.
// The algorithm for swapping blocks is described in the papers
// "A generalized eigenvalue approach for solving Riccati equations" by P. Van
// Dooren, 1981, and "Numerical Methods for General and Structured Eigenvalue
// Problems" by Daniel Kressner, 2005.
void reorder_eigen(Eigen::Ref<Eigen::MatrixXd> S, Eigen::Ref<Eigen::MatrixXd> T,
                   Eigen::Ref<Eigen::MatrixXd> Z, double eps = 1e-10) {
  // abs(a) < eps => a = 0
  int n2 = S.rows();
  int n = n2 / 2, p = 0, q = 0;

  // Find the first unstable p block.
  while (p < n) {
    if (fabs(S(p + 1, p)) < eps) {  // p block size = 1
      if (fabs(T(p, p)) > eps && fabs(S(p, p)) <= fabs(T(p, p))) {  // stable
        p++;
        continue;
      }
    } else {  // p block size = 2
      const double det_T =
          T(p, p) * T(p + 1, p + 1) - T(p + 1, p) * T(p, p + 1);
      if (fabs(det_T) > eps) {
        const double det_S =
            S(p, p) * S(p + 1, p + 1) - S(p + 1, p) * S(p, p + 1);
        if (fabs(det_S) <= fabs(det_T)) {  // stable
          p += 2;
          continue;
        }
      }
    }
    break;
  }
  q = p;

  // Make the first n generalized eigenvalues stable.
  while (p < n && q < n2) {
    // Update q.
    int q_block_size = 0;
    while (q < n2) {
      if (q == n2 - 1 || fabs(S(q + 1, q)) < eps) {  // block size = 1
        if (fabs(T(q, q)) > eps && fabs(S(q, q)) <= fabs(T(q, q))) {
          q_block_size = 1;
          break;
        }
        q++;
      } else {  // block size = 2
        const double det_T =
            T(q, q) * T(q + 1, q + 1) - T(q + 1, q) * T(q, q + 1);
        if (fabs(det_T) > eps) {
          const double det_S =
              S(q, q) * S(q + 1, q + 1) - S(q + 1, q) * S(q, q + 1);
          if (fabs(det_S) <= fabs(det_T)) {
            q_block_size = 2;
            break;
          }
        }
        q += 2;
      }
    }
    if (q >= n2)
      throw std::runtime_error("fail to find enough stable eigenvalues");
    // Swap blocks pointed by p and q.
    if (p != q) {
      swap_block(S, T, Z, p, q, q_block_size);
      p += q_block_size;
      q += q_block_size;
    }
  }
  if (p < n && q >= n2)
    throw std::runtime_error("fail to find enough stable eigenvalues");
}
}  // namespace

/**
 * DiscreteAlgebraicRiccatiEquation function
 * computes the unique stabilizing solution X to the discrete-time algebraic
 * Riccati equation:
 *
 * AᵀXA − X − AᵀXB(BᵀXB + R)⁻¹BᵀXA + Q = 0
 *
 * @throws std::exception if Q is not positive semi-definite.
 * @throws std::exception if R is not positive definite.
 *
 * Based on the Schur Vector approach outlined in this paper:
 * "On the Numerical Solution of the Discrete-Time Algebraic Riccati Equation"
 * by Thrasyvoulos Pappas, Alan J. Laub, and Nils R. Sandell, in TAC, 1980,
 * http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=1102434
 *
 * Note: When, for example, n = 100, m = 80, and entries of A, B, Q_half,
 * R_half are sampled from standard normal distributions, where
 * Q = Q_halfᵀ Q_half and similar for R, the absolute error of the solution
 * is 10⁻⁶, while the absolute error of the solution computed by Matlab is
 * 10⁻⁸.
 *
 * TODO(weiqiao.han): I may overwrite the RealQZ function to improve the
 * accuracy, together with more thorough tests.
 */

Eigen::MatrixXd DiscreteAlgebraicRiccatiEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R) {
  int n = B.rows(), m = B.cols();

  DRAKE_DEMAND(m <= n);
  DRAKE_DEMAND(A.rows() == n && A.cols() == n);
  DRAKE_DEMAND(Q.rows() == n && Q.cols() == n);
  DRAKE_DEMAND(R.rows() == m && R.cols() == m);
  DRAKE_DEMAND(is_approx_equal_abstol(Q, Q.transpose(), 1e-10));
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(Q);
  for (int i = 0; i < n; i++) {
    DRAKE_THROW_UNLESS(es.eigenvalues()[i] >= 0);
  }
  DRAKE_DEMAND(is_approx_equal_abstol(R, R.transpose(), 1e-10));
  Eigen::LLT<Eigen::MatrixXd> R_cholesky(R);
  DRAKE_THROW_UNLESS(R_cholesky.info() == Eigen::Success);
  check_stabilizable(A, B);
  check_detectable(A, Q);

  Eigen::MatrixXd M(2 * n, 2 * n), L(2 * n, 2 * n);
  M << A, Eigen::MatrixXd::Zero(n, n), -Q, Eigen::MatrixXd::Identity(n, n);
  L << Eigen::MatrixXd::Identity(n, n), B * R.inverse() * B.transpose(),
      Eigen::MatrixXd::Zero(n, n), A.transpose();

  // QZ decomposition of M and L
  // QMZ = S, QLZ = T
  // where Q and Z are real orthogonal matrixes
  // T is upper-triangular matrix, and S is upper quasi-triangular matrix
  Eigen::RealQZ<Eigen::MatrixXd> qz(2 * n);
  qz.compute(M, L);  // M = Q S Z,  L = Q T Z (Q and Z computed by Eigen package
                     // are adjoints of Q and Z above)
  Eigen::MatrixXd S = qz.matrixS(), T = qz.matrixT(),
                  Z = qz.matrixZ().adjoint();

  // Reorder the generalized eigenvalues of (S,T).
  Eigen::MatrixXd Z2 = Eigen::MatrixXd::Identity(2 * n, 2 * n);
  reorder_eigen(S, T, Z2);
  Z = (Z * Z2).eval();

  // The first n columns of Z is ( U1 ) .
  //                             ( U2 )
  //            -1
  // X = U2 * U1   is a solution of the discrete time Riccati equation.
  Eigen::MatrixXd U1 = Z.block(0, 0, n, n), U2 = Z.block(n, 0, n, n);
  Eigen::MatrixXd X = U2 * U1.inverse();
  X = (X + X.adjoint().eval()) / 2.0;
  return X;
}

Eigen::MatrixXd DiscreteAlgebraicRiccatiEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N) {
    DRAKE_DEMAND(N.rows() == B.rows() && N.cols() == B.cols());

    // This is a change of variables to make the DARE that includes Q, R, and N
    // cost matrices fit the form of the DARE that includes only Q and R cost
    // matrices.
    Eigen::MatrixXd A2 = A - B * R.llt().solve(N.transpose());
    Eigen::MatrixXd Q2 = Q - N * R.llt().solve(N.transpose());
    return DiscreteAlgebraicRiccatiEquation(A2, B, Q2, R);
}

}  // namespace math
}  // namespace drake
