#include "drake/math/matrix_exponential.h"

// This file was copied and modified from the Eigen library.
//
// Copyright (C) 2009, 2010, 2013 Jitse Niesen <jitse@maths.leeds.ac.uk>
// Copyright (C) 2011, 2013 Chen-Pang He <jdh8@ms63.hinet.net>
// Copyright (C) 2025 Drake Authors
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include <Eigen/LU>

#include "drake/common/drake_assert.h"

namespace drake {
namespace internal {
namespace {

using Eigen::MatrixXd;

/* Computes the (3,3)-Padé approximant to the exponential.
After exit, (V+U)(V-U)⁻¹ is the Padé approximant of eᴬ around A = 0. */
void CalcMatrixExpPade3(const MatrixXd& A, MatrixXd* U, MatrixXd* V) {
  const double b[] = {120.0, 60.0, 12.0, 1.0};
  const MatrixXd A2 = A * A;
  const MatrixXd tmp =
      b[3] * A2 + b[1] * MatrixXd::Identity(A.rows(), A.cols());
  U->noalias() = A * tmp;
  *V = b[2] * A2 + b[0] * MatrixXd::Identity(A.rows(), A.cols());
}

/* Computes the (5,5)-Padé approximant to the exponential.
After exit, (V+U)(V-U)⁻¹ is the Padé approximant of eᴬ around A = 0. */
void CalcMatrixExpPade5(const MatrixXd& A, MatrixXd* U, MatrixXd* V) {
  const double b[] = {30240.0, 15120.0, 3360.0, 420.0, 30.0, 1.0};
  const MatrixXd A2 = A * A;
  const MatrixXd A4 = A2 * A2;
  const MatrixXd tmp =
      b[5] * A4 + b[3] * A2 + b[1] * MatrixXd::Identity(A.rows(), A.cols());
  U->noalias() = A * tmp;
  *V = b[4] * A4 + b[2] * A2 + b[0] * MatrixXd::Identity(A.rows(), A.cols());
}

/* Computes the (7,7)-Padé approximant to the exponential.
After exit, (V+U)(V-U)⁻¹ is the Padé approximant of eᴬ around A = 0. */
void CalcMatrixExpPade7(const MatrixXd& A, MatrixXd* U, MatrixXd* V) {
  const double b[] = {17297280.0, 8648640.0, 1995840.0, 277200.0,
                      25200.0,    1512.0,    56.0,      1.0};
  const MatrixXd A2 = A * A;
  const MatrixXd A4 = A2 * A2;
  const MatrixXd A6 = A4 * A2;
  const MatrixXd tmp = b[7] * A6 + b[5] * A4 + b[3] * A2 +
                       b[1] * MatrixXd::Identity(A.rows(), A.cols());
  U->noalias() = A * tmp;
  *V = b[6] * A6 + b[4] * A4 + b[2] * A2 +
       b[0] * MatrixXd::Identity(A.rows(), A.cols());
}

/* Computes the (9,9)-Padé approximant to the exponential.
After exit, (V+U)(V-U)⁻¹ is the Padé approximant of eᴬ around A = 0. */
void CalcMatrixExpPade9(const MatrixXd& A, MatrixXd* U, MatrixXd* V) {
  const double b[] = {17643225600.0, 8821612800.0, 2075673600.0, 302702400.0,
                      30270240.0,    2162160.0,    110880.0,     3960.0,
                      90.0,          1.0};
  const MatrixXd A2 = A * A;
  const MatrixXd A4 = A2 * A2;
  const MatrixXd A6 = A4 * A2;
  const MatrixXd A8 = A6 * A2;
  const MatrixXd tmp = b[9] * A8 + b[7] * A6 + b[5] * A4 + b[3] * A2 +
                       b[1] * MatrixXd::Identity(A.rows(), A.cols());
  U->noalias() = A * tmp;
  *V = b[8] * A8 + b[6] * A6 + b[4] * A4 + b[2] * A2 +
       b[0] * MatrixXd::Identity(A.rows(), A.cols());
}

/* Computes the (13,13)-Padé approximant to the exponential.
After exit, (V+U)(V-U)⁻¹ is the Padé approximant of eᴬ around A = 0. */
void CalcMatrixExpPade13(const MatrixXd& A, MatrixXd* U, MatrixXd* V) {
  const double b[] = {64764752532480000.0,
                      32382376266240000.0,
                      7771770303897600.0,
                      1187353796428800.0,
                      129060195264000.0,
                      10559470521600.0,
                      670442572800.0,
                      33522128640.0,
                      1323241920.0,
                      40840800.0,
                      960960.0,
                      16380.0,
                      182.0,
                      1.0};
  const MatrixXd A2 = A * A;
  const MatrixXd A4 = A2 * A2;
  const MatrixXd A6 = A4 * A2;
  *V = b[13] * A6 + b[11] * A4 + b[9] * A2;  // used for temporary storage
  MatrixXd tmp = A6 * *V;
  tmp += b[7] * A6 + b[5] * A4 + b[3] * A2 +
         b[1] * MatrixXd::Identity(A.rows(), A.cols());
  U->noalias() = A * tmp;
  tmp = b[12] * A6 + b[10] * A4 + b[8] * A2;
  V->noalias() = A6 * tmp;
  *V += b[6] * A6 + b[4] * A4 + b[2] * A2 +
        b[0] * MatrixXd::Identity(A.rows(), A.cols());
}

/* Computes Padé approximant to the exponential. Computes `U`, `V`, and
`squarings` such that (V+U)(V-U)⁻¹ is a Padé approximant of `e^(2⁻ˢ𐞥ᵘᵃʳⁱⁿᵍˢM)`
around M = 0. The degree of the Padé approximant and the value of squarings are
chosen such that the approximation error is no more than the round-off error.
@returns `squarings` */
int ComputePadeApproximant(const MatrixXd& M, MatrixXd* U, MatrixXd* V) {
  const double l1norm = M.cwiseAbs().colwise().sum().maxCoeff();
  int squarings = 0;
  if (l1norm < 1.495585217958292e-002) {
    CalcMatrixExpPade3(M, U, V);
  } else if (l1norm < 2.539398330063230e-001) {
    CalcMatrixExpPade5(M, U, V);
  } else if (l1norm < 9.504178996162932e-001) {
    CalcMatrixExpPade7(M, U, V);
  } else if (l1norm < 2.097847961257068e+000) {
    CalcMatrixExpPade9(M, U, V);
  } else {
    const double maxnorm = 5.371920351148152;
    std::frexp(l1norm / maxnorm, &squarings);
    if (squarings < 0) {
      squarings = 0;
    }
    const MatrixXd A = M.unaryExpr([squarings](double x) {
      return std::ldexp(x, -squarings);
    });
    CalcMatrixExpPade13(A, U, V);
  }
  return squarings;
}

}  // namespace

MatrixXd CalcMatrixExponential(const MatrixXd& M) {
  DRAKE_THROW_UNLESS(M.rows() == M.cols());
  // Pade approximant is (U+V) / (-U+V).
  MatrixXd U, V;
  const int squarings = ComputePadeApproximant(M, &U, &V);
  const MatrixXd numer = U + V;
  const MatrixXd denom = -U + V;
  MatrixXd result = denom.partialPivLu().solve(numer);
  // Undo scaling by repeated squaring.
  for (int i = 0; i < squarings; ++i) {
    result *= result;
  }
  return result;
}

}  // namespace internal
}  // namespace drake
