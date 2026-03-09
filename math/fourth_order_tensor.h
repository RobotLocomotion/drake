#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace math {
namespace internal {

/* This class provides functionalities related to 4th-order tensors of
 dimension 3*3*3*3.  @tparam float, double, AutoDiffXd. */
template <typename T>
class FourthOrderTensor {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FourthOrderTensor)
  using MatrixType = Eigen::Matrix<T, 9, 9>;

  /* Constructs a 4th-order tensor represented by the given matrix using the
   convention layed out in the class documentation. */
  explicit FourthOrderTensor(const MatrixType& data);

  /* Constructs a zero 4th-order tensor. */
  FourthOrderTensor();

  /* Performs contraction between this 4th-order tensor A and two vectors u and
   v and outputs 2nd order tensor B. In Einstein notation, the contraction being
   done is Bᵢₖ = uⱼ Aᵢⱼₖₗ vₗ. */
  void ContractWithVectors(const Eigen::Ref<const Vector3<T>>& u,
                           const Eigen::Ref<const Vector3<T>>& v,
                           EigenPtr<Matrix3<T>> B) const;

  /* Sets this 4th-order tensor as the outer product of the matrices (2nd-order
   tensor) M and N. More specifically, in Einstein notion, sets Aᵢⱼₖₗ = MᵢⱼNₖₗ.
   @warn This function assumes the input matrices are _not_ aliasing the data in
   this 4th-order tensor. */
  void SetAsOuterProduct(const Eigen::Ref<const Matrix3<T>>& M,
                         const Eigen::Ref<const Matrix3<T>>& N);

  /* Returns a a scaled symmetric identity 4th-order tensor. In Einstein
   notation, the result is scale * 1/2 * (δᵢₖδⱼₗ + δᵢₗδⱼₖ). */
  static FourthOrderTensor<T> MakeSymmetricIdentity(T scale);

  /* Returns a scaled major identity 4th-order tensor. In Einstein
   notation, the result is scale * δᵢₖδⱼₗ.  */
  static FourthOrderTensor<T> MakeMajorIdentity(T scale);

  FourthOrderTensor<T>& operator+=(const FourthOrderTensor<T>& other);

  /* Returns this 4th-order tensor encoded as a 2D matrix.
   The tensor is represented using a a 9*9 matrix organized as following

                  l = 0       l = 1       l = 2
              -------------------------------------
              |           |           |           |
    j = 0     |   Aᵢ₀ₖ₀   |   Aᵢ₀ₖ₁   |   Aᵢ₀ₖ₂   |
              |           |           |           |
              -------------------------------------
              |           |           |           |
    j = 1     |   Aᵢ₁ₖ₀   |   Aᵢ₁ₖ₁   |   Aᵢ₁ₖ₂   |
              |           |           |           |
              -------------------------------------
              |           |           |           |
    j = 2     |   Aᵢ₂ₖ₀   |   Aᵢ₂ₖ₁   |   Aᵢ₂ₖ₂   |
              |           |           |           |
              -------------------------------------
  Namely the ik-th entry in the jl-th block corresponds to the value Aᵢⱼₖₗ. */
  const MatrixType& data() const { return data_; }

  /* Returns this 4th-order tensor encoded as a mutable 2D matrix.
   @see data() for the layout of the matrix. */
  MatrixType& mutable_data() { return data_; }

  /* Sets this 4th-order tensor to be scale * δᵢₖδⱼₗ. */
  void SetToDiagonal(T scale);

  /* Returns the value of the 4th-order tensor at the given indices.
   @pre 0 <= i, j, k, l < 3. */
  const T& operator()(int i, int j, int k, int l) const {
    DRAKE_ASSERT(0 <= i && i < 3);
    DRAKE_ASSERT(0 <= j && j < 3);
    DRAKE_ASSERT(0 <= k && k < 3);
    DRAKE_ASSERT(0 <= l && l < 3);
    return data_(3 * j + i, 3 * l + k);
  }

  /* Returns a mutable reference to the value of the 4th-order tensor at the
   given indices.
   @pre 0 <= i, j, k, l < 3. */
  T& operator()(int i, int j, int k, int l) {
    DRAKE_ASSERT(0 <= i && i < 3);
    DRAKE_ASSERT(0 <= j && j < 3);
    DRAKE_ASSERT(0 <= k && k < 3);
    DRAKE_ASSERT(0 <= l && l < 3);
    return data_(3 * j + i, 3 * l + k);
  }

  /* Sets the data of this 4th-order tensor to the given matrix, using the
   convention laid out in the function data(). */
  void set_data(const MatrixType& data) { data_ = data; }

 private:
  MatrixType data_{MatrixType::Zero()};
};

template <typename T>
FourthOrderTensor<T> operator+(const FourthOrderTensor<T>& t1,
                               const FourthOrderTensor<T>& t2) {
  return FourthOrderTensor<T>(t1) += t2;
}

}  // namespace internal
}  // namespace math
}  // namespace drake
