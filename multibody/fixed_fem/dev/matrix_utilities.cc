#include "drake/multibody/fixed_fem/dev/matrix_utilities.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

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
void AddScaledRotationalDerivative(
    const Matrix3<T>& R, const Matrix3<T>& S, const T& scale,
    EigenPtr<Eigen::Matrix<T, 9, 9>> scaled_dRdF) {
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
          (*scaled_dRdF)(row_index, column_index) +=
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
    EigenPtr<Eigen::Matrix<T, 9, 9>> scaled_dCdM) {
  /* See the convention for ordering the 9-by-9 derivatives at the top of the
   header file. */
  const Matrix3<T> A = scale * M;
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
    (&CalcConditionNumberOfInvertibleMatrix<T>, &PolarDecompose<T>,
     &AddScaledRotationalDerivative<T>, &CalcCofactorMatrix<T>,
     &AddScaledCofactorMatrixDerivative<T>, &PermuteBlockVector<T>))

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
