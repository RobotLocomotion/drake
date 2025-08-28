#pragma once

#include <vector>

#include <Eigen/SparseCore>

#include "drake/common/eigen_types.h"
#include "drake/math/fourth_order_tensor.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Calculates the condition number for the given matrix A.
 The condition number is calculated via
     k(A) = σₘₐₓ / σₘᵢₙ
 where σₘₐₓ and σₘᵢₙ are the largest and smallest singular values (in magnitude)
 of A.
 @note This function uses Eigen::JacobiSVD under the hood and may be slow on
 large matrices. See
 https://eigen.tuxfamily.org/dox/group__DenseDecompositionBenchmark.html.
 @pre A is invertible.
 @tparam_nonsymbolic_scalar */
template <typename T>
double CalcConditionNumberOfInvertibleMatrix(
    const Eigen::Ref<const MatrixX<T>>& A);

/* Calculates the polar decomposition of a 3-by-3 matrix F = RS where R is a
 rotation matrix and S is a symmetric matrix. The decomposition is unique when F
 is non-singular.
 @tparam T The scalar type, can be a double, float, or AutoDiffXd. */
template <typename T>
void PolarDecompose(const Matrix3<T>& F, EigenPtr<Matrix3<T>> R,
                    EigenPtr<Matrix3<T>> S);

/* Calculates a Singular Value Decomposition (SVD) of a 3-by-3 matrix
 F=U⋅S⋅Vᵀ where U and V are rotation matrices and S = diag(σ₁, σ₂, σ₃) is a
 diagonal matrix. Such SVD is not unique and we pick an arbitrary valid one.
 See Appendix F of [Kim and Eberle, 2020] for details.
 [Kim and Eberle, 2020] Kim, Theodore, and David Eberle. "Dynamic deformables:
 implementation and production practicalities." Acm siggraph 2020 courses. 2020.
 1-182.
 @tparam T The scalar type, can be a double, float, or AutoDiffXd. */
template <typename T>
void RotationSvd(const Matrix3<T>& F, EigenPtr<Matrix3<T>> U,
                 EigenPtr<Matrix3<T>> V, EigenPtr<Vector3<T>> sigma);

/* Computes the derivative of the rotation matrix from the polar decomposition
 (see PolarDecompose()) with respect to the original matrix.
 @param[in] R               The rotation matrix in the polar decomposition
                            F = RS.
 @param[in] S               The symmetric matrix in the polar decomposition
                            F = RS.
 @param[in] scale           The scalar multiple of the result.
 @param[in,out] scaled_dRdF The variable to which scale * dR/dF is added.
 @pre tr(S)I − S is invertible.
 @pre scaled_dRdF != nullptr.
 @tparam T The scalar type, can be a double, float, or AutoDiffXd. */
template <typename T>
void AddScaledRotationalDerivative(
    const Matrix3<T>& R, const Matrix3<T>& S, const T& scale,
    math::internal::FourthOrderTensor<T>* scaled_dRdF);

/* Calculates the cofactor matrix of the given input 3-by-3 matrix M.
 @pre cofactor != nullptr.
 @tparam T The scalar type, can be a double, float, or AutoDiffXd. */
template <typename T>
void CalcCofactorMatrix(const Matrix3<T>& M, EigenPtr<Matrix3<T>> cofactor);

/* Computes the derivative of the cofactor matrix C of a 3-by-3 matrix M
 with respect to the matrix M itself.
 @param[in] M               The input matrix.
 @param[in] scale           The scalar multiple of the result.
 @param[in,out] scaled_dCdF The variable to which scale * dC/dM is added.
 @pre scaled_dCdM != nullptr.
 @tparam T The scalar type, can be a double, float, or AutoDiffXd. */
template <typename T>
void AddScaledCofactorMatrixDerivative(
    const Matrix3<T>& M, const T& scale,
    math::internal::FourthOrderTensor<T>* scaled_dCdM);

/* Given a size 3N vector with block structure with size 3 block entries Bᵢ
 where i ∈ V = {0, ..., N-1} and a permutation P on V, this function builds the
 permuted vector with size 3 block entries C's such that Cₚ₍ᵢ₎ = Bᵢ.
 For example, suppose the input `v` is given by
       a a a b b b c c c
and the input `block_permutation` is {1, 2, 0}, then the result of the
permutation will be:
       c c c a a a b b b
@param[in] v                  The original block vector to be permuted.
@param[in] block_permutation  block_permutation[i] gives the index of the
                              permuted block whose original index is `i`.
@pre v.size() % 3 == 0.
@pre block_permutation is a permutation of {0, 1, ..., v.size()/3 - 1}.
@tparam_nonsymbolic_scalar */
template <typename T>
VectorX<T> PermuteBlockVector(const Eigen::Ref<const VectorX<T>>& v,
                              const std::vector<int>& block_permutation);

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
