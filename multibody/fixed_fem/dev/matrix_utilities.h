#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace internal {
/* Some of the following methods involves calculations about 4th order tensor
(call it A) of dimension 3*3*3*3. We follow the following convention to flatten
the 4th order tensor into 9*9 matrices that are organized as following

                  l = 1       l = 2       l = 3
              -------------------------------------
              |           |           |           |
    j = 1     |   Aᵢ₁ₖ₁   |   Aᵢ₁ₖ₂   |   Aᵢ₁ₖ₃   |
              |           |           |           |
              -------------------------------------
              |           |           |           |
    j = 2     |   Aᵢ₂ₖ₁   |   Aᵢ₂ₖ₂   |   Aᵢ₂ₖ₃   |
              |           |           |           |
              -------------------------------------
              |           |           |           |
    j = 3     |   Aᵢ₃ₖ₁   |   Aᵢ₃ₖ₂   |   Aᵢ₃ₖ₃   |
              |           |           |           |
              -------------------------------------
Namely the ik-th entry in the jl-th block corresponds to the value Aᵢⱼₖₗ. */

/* Calculates the unique polar decomposition of a 3-by-3 matrix F = RS where R
 is a rotation matrix and S is a symmetric matrix. */
template <typename T>
void PolarDecompose(const Matrix3<T>& F, EigenPtr<Matrix3<T>> R,
                    EigenPtr<Matrix3<T>> S) {
  const Eigen::JacobiSVD<Matrix3<T>> svd(
      F, Eigen::ComputeFullU | Eigen::ComputeFullV);
  auto U = svd.matrixU();
  const auto& V = svd.matrixV();
  auto sigma = svd.singularValues();
  *R = U * V.transpose();
  /* Ensure that R is a proper rotation. If R is a reflection, flip the sign of
   the last column of U and the last singular value. */
  if (R->determinant() < 0) {
    U.col(2) *= -1.0;
    sigma(2) *= -1.0;
    *R = U * V.transpose();
  }
  *S = V * sigma.asDiagonal() * V.transpose();
}

/* Some notes on derivation on the derivative of the rotation matrix from polar
 decomposition: we start with the result from section 2 of [McAdams, 2011] about
 the differential of the rotation matrix, which states that
               δR = R[ε : ((tr(S)I − S)⁻¹(εᵀ : (RᵀδF)))]. (1)
 For simplicity of notation, we define A = tr(S)I − S and B = RᵀδF
 In index notation, equation (1) then reads
               δRᵢⱼ = Rᵢₘεₘⱼₙ A⁻¹ₙₚ εₚₖₗBₖₗ. (2)
 From there, we make use of the identity
               A⁻¹ₙₚ = 1/(2*det(A)) εₙₛᵣεₚₜᵤAₜₛAᵤᵣ.
 Plugging it into (2) gives
               δRᵢⱼ = 1/(2*det(A)) * RᵢₘεₘⱼₙεₙₛᵣεₚₜᵤεₚₖₗAₜₛAᵤᵣBₖₗ.
 Then, make use of the identity
               εₚₜᵤεₚₖₗ = δₜₖδᵤₗ − δₜₗδᵤₖ,
 we get
  δRᵢⱼ = 1/(2*det(A)) * Rᵢₘ(δₘₛδⱼᵣ − δₘᵣδⱼₛ)(δₜₖδᵤₗ − δₜₗδᵤₖ)AₜₛAᵤᵣBₖₗ.
 Cleaning up deltas, we get:
               δRᵢⱼ = 1/det(A) * Rᵢₘ(AₖₘAₗⱼ−AₖⱼAₗₘ)Bₖₗ.
 Finally, using ∂Bₖₗ/∂Fₐᵦ = Rₐₖδᵦₗ, we get
               δRᵢⱼ/∂Fₐᵦ = 1/det(A) * Rᵢₘ(AₖₘAₗⱼ−AₖⱼAₗₘ)Rₐₖδᵦₗ
                        = 1/det(A) * Rᵢₘ(AₖₘAᵦⱼ− AₖⱼAᵦₘ)Rₐₖ
                        = 1/det(A) * (RARᵀ)ᵢₐAⱼᵦ - (RA)ᵢᵦ(RA)ₐⱼ
 where we used the fact that A is symmetric in the last equality.

 [McAdams, 2011] McAdams, Aleka, et al. "Technical Notes for Efficient
 elasticity for character skinning with contact and collisions." ACM SIGGRAPH
 2011 papers. 2011. 1-12.
 https://disneyanimation.com/publications/efficient-elasticity-for-character-skinning-with-contact-and-collisions.
*/

/* Computes the derivative of the rotation matrix from the polar decomposition
 (see PolarDecompose()) with respect to the original matrix.
 @param[in] R The rotation matrix in the polar decomposition F = RS.
 @param[in] S The symmetric matrix in the polar decomposition F = RS.
 @param[in] scale The scalar multiple of the result.
 @param[out] scaled_dRdF The variable to which scale * dR/dF is added. */
template <typename T>
void AddScaledRotationalDerivative(
    const Matrix3<T>& R, const Matrix3<T>& S, const T& scale,
    EigenPtr<Eigen::Matrix<T, 9, 9>> scaled_dRdF) {
  Matrix3<T> A = -S;
  A.diagonal().array() += S.trace();
  const T J = A.determinant();
  DRAKE_ASSERT(J != 0);
  const T scale_over_J = scale / J;
  const auto RA = R * A;
  const auto sRA = scale_over_J * RA;
  const auto sRART = sRA * R.transpose();
  for (int a = 0; a < 3; a++) {
    for (int b = 0; b < 3; b++) {
      const int column_index = 3 * b + a;
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          const int row_index = 3 * j + i;
          (*scaled_dRdF)(row_index, column_index) +=
              sRART(i, a) * A(j, b) - sRA(i, b) * RA(a, j);
        }
      }
    }
  }
}

/* Calculates the cofactor matrix of the given input 3-by-3 matrix M. */
template <typename T>
void CalcCofactorMatrix(const Matrix3<T>& M, EigenPtr<Matrix3<T>> cofactor) {
  (*cofactor)(0, 0) = M(1, 1) * M(2, 2) - M(1, 2) * M(2, 1);
  (*cofactor)(0, 1) = M(1, 2) * M(2, 0) - M(1, 0) * M(2, 2);
  (*cofactor)(0, 2) = M(1, 0) * M(2, 1) - M(1, 1) * M(2, 0);
  (*cofactor)(1, 0) = M(0, 2) * M(2, 1) - M(0, 1) * M(2, 2);
  (*cofactor)(1, 1) = M(0, 0) * M(2, 2) - M(0, 2) * M(2, 0);
  (*cofactor)(1, 2) = M(0, 1) * M(2, 0) - M(0, 0) * M(2, 1);
  (*cofactor)(2, 0) = M(0, 1) * M(1, 2) - M(0, 2) * M(1, 1);
  (*cofactor)(2, 1) = M(0, 2) * M(1, 0) - M(0, 0) * M(1, 2);
  (*cofactor)(2, 2) = M(0, 0) * M(1, 1) - M(0, 1) * M(1, 0);
}

/* Computes the derivative of the cofactor matrix C of a 3-by-3 matrix M
 with respect to the matrix M itself.
 @param[in] M The input matrix.
 @param[in] scale The scalar multiple of the result.
 @param[out] scaled_dCdF The variable to which scale * dC/dM is added. */
template <typename T>
void AddScaledCofactorMatrixDerivative(
    const Matrix3<T>& M, const T& scale,
    EigenPtr<Eigen::Matrix<T, 9, 9>> scaled_dCdM) {
  /* See the convention for ordering the 9-by-9 derivative at the top of the
   file. */
  const auto& A = scale * M;
  (*scaled_dCdM)(4, 0) += A(2, 2);
  (*scaled_dCdM)(5, 0) += -A(1, 2);
  (*scaled_dCdM)(7, 0) += -A(2, 1);
  (*scaled_dCdM)(8, 0) += A(1, 1);
  (*scaled_dCdM)(3, 1) += -A(2, 2);
  (*scaled_dCdM)(5, 1) += A(0, 2);
  (*scaled_dCdM)(6, 1) += A(2, 1);
  (*scaled_dCdM)(8, 1) += -A(0, 1);
  (*scaled_dCdM)(3, 2) += A(1, 2);
  (*scaled_dCdM)(4, 2) += -A(0, 2);
  (*scaled_dCdM)(6, 2) += -A(1, 1);
  (*scaled_dCdM)(7, 2) += A(0, 1);
  (*scaled_dCdM)(1, 3) += -A(2, 2);
  (*scaled_dCdM)(2, 3) += A(1, 2);
  (*scaled_dCdM)(7, 3) += A(2, 0);
  (*scaled_dCdM)(8, 3) += -A(1, 0);
  (*scaled_dCdM)(0, 4) += A(2, 2);
  (*scaled_dCdM)(2, 4) += -A(0, 2);
  (*scaled_dCdM)(6, 4) += -A(2, 0);
  (*scaled_dCdM)(8, 4) += A(0, 0);
  (*scaled_dCdM)(0, 5) += -A(1, 2);
  (*scaled_dCdM)(1, 5) += A(0, 2);
  (*scaled_dCdM)(6, 5) += A(1, 0);
  (*scaled_dCdM)(7, 5) += -A(0, 0);
  (*scaled_dCdM)(1, 6) += A(2, 1);
  (*scaled_dCdM)(2, 6) += -A(1, 1);
  (*scaled_dCdM)(4, 6) += -A(2, 0);
  (*scaled_dCdM)(5, 6) += A(1, 0);
  (*scaled_dCdM)(0, 7) += -A(2, 1);
  (*scaled_dCdM)(2, 7) += A(0, 1);
  (*scaled_dCdM)(3, 7) += A(2, 0);
  (*scaled_dCdM)(5, 7) += -A(0, 0);
  (*scaled_dCdM)(0, 8) += A(1, 1);
  (*scaled_dCdM)(1, 8) += -A(0, 1);
  (*scaled_dCdM)(3, 8) += -A(1, 0);
  (*scaled_dCdM)(4, 8) += A(0, 0);
}
}  // namespace internal
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
