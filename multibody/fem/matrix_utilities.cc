#include "drake/multibody/fem/matrix_utilities.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

// TODO(xuchenhan-tri): For T = AutoDiffXd, this function is wasting a lot of
//  math computing the derivatives, only to throw them away at the end. Consider
//  only accepting matrices with double as the scalar type.
template <typename T>
double CalcConditionNumberOfInvertibleMatrix(
    const Eigen::Ref<const MatrixX<T>>& A) {
  DRAKE_THROW_UNLESS(A.rows() == A.cols());
  Eigen::JacobiSVD<MatrixX<T>> svd(A);
  /* Eigen::JacobiSVD::singularValues() returns sigma as positive, monotonically
   decreasing values.
   See https://eigen.tuxfamily.org/dox/classEigen_1_1JacobiSVD.html. */
  const VectorX<T>& sigma = svd.singularValues();
  /* Prevents division by zero for singular matrix. */
  DRAKE_DEMAND(sigma.size() > 0);
  const T& sigma_min = sigma(sigma.size() - 1);
  DRAKE_DEMAND(sigma_min > 0);
  const T& sigma_max = sigma(0);
  const T cond = sigma_max / sigma_min;
  return ExtractDoubleOrThrow(cond);
}

template <typename T>
void PolarDecompose(const Matrix3<T>& F, EigenPtr<Matrix3<T>> R,
                    EigenPtr<Matrix3<T>> S) {
  /* According to https://eigen.tuxfamily.org/dox/classEigen_1_1BDCSVD.html,
   for matrix of size < 16, it's preferred to used JacobiSVD. */
  const Eigen::JacobiSVD<Matrix3<T>, Eigen::HouseholderQRPreconditioner> svd(
      F, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Matrix3<T> U = svd.matrixU();
  const Matrix3<T>& V = svd.matrixV();
  Vector3<T> sigma = svd.singularValues();
  (*R).noalias() = U * V.transpose();
  /* Ensure that R is a proper rotation. If R is a reflection, flip the sign of
   the last column of U and the last singular value. */
  if (R->determinant() < 0) {
    U.col(2) *= -1.0;
    sigma(2) *= -1.0;
    (*R).noalias() = U * V.transpose();
  }
  (*S).noalias() = V * sigma.asDiagonal() * V.transpose();
}

template <typename T>
void RotationSvd(const Matrix3<T>& F, EigenPtr<Matrix3<T>> U,
                 EigenPtr<Matrix3<T>> V, EigenPtr<Vector3<T>> sigma) {
  /* According to https://eigen.tuxfamily.org/dox/classEigen_1_1BDCSVD.html,
   for matrix of size < 16, it's preferred to used JacobiSVD. */
  const Eigen::JacobiSVD<Matrix3<T>, Eigen::HouseholderQRPreconditioner> svd(
      F, Eigen::ComputeFullU | Eigen::ComputeFullV);
  *U = svd.matrixU();
  *V = svd.matrixV();
  *sigma = svd.singularValues();
  /* We flip an arbitrary singular vector if needed to ensure U and V are
   rotation matrices. */
  if (U->determinant() < 0.0) {
    U->col(0) *= -1.0;
    (*sigma)(0) *= -1.0;
  }
  if (V->determinant() < 0.0) {
    V->col(0) *= -1.0;
    (*sigma)(0) *= -1.0;
  }
}

template <typename T>
void AddScaledRotationalDerivative(
    const Matrix3<T>& R, const Matrix3<T>& S, const T& scale,
    math::internal::FourthOrderTensor<T>* scaled_dRdF) {
  /* Some notes on derivation on the derivative of the rotation matrix from
   polar decomposition: we start with the result from section 2 of [McAdams,
   2011] about the differential of the rotation matrix, which states that δR =
   R[ε : ((tr(S)I − S)⁻¹(εᵀ : (RᵀδF)))]. (1) For simplicity of notation, we
   define A = tr(S)I − S and B = RᵀδF In index notation, equation (1) then reads
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
  Matrix3<T> A = -S;
  A.diagonal().array() += S.trace();
  const T J = A.determinant();
  DRAKE_DEMAND(J != 0);
  const T scale_over_J = scale / J;
  const Matrix3<T> RA = R * A;
  const Matrix3<T> sRA = scale_over_J * RA;
  const Matrix3<T> sRART = sRA * R.transpose();
  for (int a = 0; a < 3; ++a) {
    for (int b = 0; b < 3; ++b) {
      const int column_index = 3 * b + a;
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          const int row_index = 3 * j + i;
          scaled_dRdF->mutable_data()(row_index, column_index) +=
              sRART(i, a) * A(j, b) - sRA(i, b) * RA(a, j);
        }
      }
    }
  }
}

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

template <typename T>
void AddScaledCofactorMatrixDerivative(
    const Matrix3<T>& M, const T& scale,
    math::internal::FourthOrderTensor<T>* scaled_dCdM) {
  /* See the convention for ordering the 9-by-9 derivatives at the top of the
   header file. */
  const Matrix3<T> A = scale * M;
  scaled_dCdM->mutable_data()(4, 0) += A(2, 2);
  scaled_dCdM->mutable_data()(5, 0) += -A(1, 2);
  scaled_dCdM->mutable_data()(7, 0) += -A(2, 1);
  scaled_dCdM->mutable_data()(8, 0) += A(1, 1);
  scaled_dCdM->mutable_data()(3, 1) += -A(2, 2);
  scaled_dCdM->mutable_data()(5, 1) += A(0, 2);
  scaled_dCdM->mutable_data()(6, 1) += A(2, 1);
  scaled_dCdM->mutable_data()(8, 1) += -A(0, 1);
  scaled_dCdM->mutable_data()(3, 2) += A(1, 2);
  scaled_dCdM->mutable_data()(4, 2) += -A(0, 2);
  scaled_dCdM->mutable_data()(6, 2) += -A(1, 1);
  scaled_dCdM->mutable_data()(7, 2) += A(0, 1);
  scaled_dCdM->mutable_data()(1, 3) += -A(2, 2);
  scaled_dCdM->mutable_data()(2, 3) += A(1, 2);
  scaled_dCdM->mutable_data()(7, 3) += A(2, 0);
  scaled_dCdM->mutable_data()(8, 3) += -A(1, 0);
  scaled_dCdM->mutable_data()(0, 4) += A(2, 2);
  scaled_dCdM->mutable_data()(2, 4) += -A(0, 2);
  scaled_dCdM->mutable_data()(6, 4) += -A(2, 0);
  scaled_dCdM->mutable_data()(8, 4) += A(0, 0);
  scaled_dCdM->mutable_data()(0, 5) += -A(1, 2);
  scaled_dCdM->mutable_data()(1, 5) += A(0, 2);
  scaled_dCdM->mutable_data()(6, 5) += A(1, 0);
  scaled_dCdM->mutable_data()(7, 5) += -A(0, 0);
  scaled_dCdM->mutable_data()(1, 6) += A(2, 1);
  scaled_dCdM->mutable_data()(2, 6) += -A(1, 1);
  scaled_dCdM->mutable_data()(4, 6) += -A(2, 0);
  scaled_dCdM->mutable_data()(5, 6) += A(1, 0);
  scaled_dCdM->mutable_data()(0, 7) += -A(2, 1);
  scaled_dCdM->mutable_data()(2, 7) += A(0, 1);
  scaled_dCdM->mutable_data()(3, 7) += A(2, 0);
  scaled_dCdM->mutable_data()(5, 7) += -A(0, 0);
  scaled_dCdM->mutable_data()(0, 8) += A(1, 1);
  scaled_dCdM->mutable_data()(1, 8) += -A(0, 1);
  scaled_dCdM->mutable_data()(3, 8) += -A(1, 0);
  scaled_dCdM->mutable_data()(4, 8) += A(0, 0);
}

template <typename T>
VectorX<T> PermuteBlockVector(const Eigen::Ref<const VectorX<T>>& v,
                              const std::vector<int>& block_permutation) {
  DRAKE_DEMAND(static_cast<int>(block_permutation.size() * 3) == v.size());
  VectorX<T> permuted_v(v.size());
  for (int i = 0; i < static_cast<int>(block_permutation.size()); ++i) {
    permuted_v.template segment<3>(3 * block_permutation[i]) =
        v.template segment<3>(3 * i);
  }
  return permuted_v;
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&CalcConditionNumberOfInvertibleMatrix<T>,  // BR
     &PolarDecompose<T>,                         // BR
     &RotationSvd<T>,                            // BR
     &AddScaledRotationalDerivative<T>,          // BR
     &CalcCofactorMatrix<T>,                     // BR
     &AddScaledCofactorMatrixDerivative<T>,      // BR
     &PermuteBlockVector<T>));

template void PolarDecompose<float>(const Matrix3<float>& F,
                                    EigenPtr<Matrix3<float>> R,
                                    EigenPtr<Matrix3<float>> S);
template void RotationSvd<float>(const Matrix3<float>& F,
                                 EigenPtr<Matrix3<float>> U,
                                 EigenPtr<Matrix3<float>> V,
                                 EigenPtr<Vector3<float>> sigma);
template void AddScaledRotationalDerivative<float>(
    const Matrix3<float>& R, const Matrix3<float>& S, const float& scale,
    math::internal::FourthOrderTensor<float>* scaled_dRdF);
template void CalcCofactorMatrix(const Matrix3<float>& M,
                                 EigenPtr<Matrix3<float>> cofactor);
template void AddScaledCofactorMatrixDerivative<float>(
    const Matrix3<float>& M, const float& scale,
    math::internal::FourthOrderTensor<float>* scaled_dCdM);

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
