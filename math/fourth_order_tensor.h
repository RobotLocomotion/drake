#pragma once

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace math {
namespace internal {

/* This class provides functionalities related to 4th-order tensors of
 dimension 3*3*3*3. The tensor is represented using a a 9*9 matrix that is
 organized as following

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
 Namely the ik-th entry in the jl-th block corresponds to the value Aᵢⱼₖₗ.
 @tparam float, double, AutoDiffXd. */
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

  /* Returns this 4th-order tensor encoded as a matrix according to the class
   documentation. */
  const MatrixType& data() const { return data_; }

  /* Returns this 4th-order tensor encoded as a mutable matrix according to the
   class documentation. */
  MatrixType& mutable_data() { return data_; }

  /* Returns the value of the 4th-order tensor at the given indices.
   @pre 0 <= i, j, k, l < 3. */
  const T& operator()(int i, int j, int k, int l) const {
    DRAKE_ASSERT(0 <= i && i < 3);
    DRAKE_ASSERT(0 <= j && j < 3);
    DRAKE_ASSERT(0 <= k && k < 3);
    DRAKE_ASSERT(0 <= l && l < 3);
    return data_(3 * j + i, 3 * l + k);
  }

  /* Returns the value of the 4th-order tensor at the given indices.
   @pre 0 <= i, j, k, l < 3. */
  T& operator()(int i, int j, int k, int l) {
    DRAKE_ASSERT(0 <= i && i < 3);
    DRAKE_ASSERT(0 <= j && j < 3);
    DRAKE_ASSERT(0 <= k && k < 3);
    DRAKE_ASSERT(0 <= l && l < 3);
    return data_(3 * j + i, 3 * l + k);
  }

  /* Returns the value of the 4th-order tensor at the given indices,
   interpreted as indices into the 9x9 matrix, using the convention layed out in
   the class documentation.
   @pre 0 <= i, j < 9. */
  const T& operator()(int i, int j) const {
    DRAKE_ASSERT(0 <= i && i < 9);
    DRAKE_ASSERT(0 <= j && j < 9);
    return data_(i, j);
  }

  /* Returns the value of the 4th-order tensor at the given indices,
   interpreted as indices into the 9x9 matrix, using the convention layed out in
   the class documentation.
   @pre 0 <= i, j < 9. */
  T& operator()(int i, int j) {
    DRAKE_ASSERT(0 <= i && i < 9);
    DRAKE_ASSERT(0 <= j && j < 9);
    return data_(i, j);
  }

  /* Sets the data of this 4th-order tensor to the given matrix, using the
   convention layed out in the class documentation. */
  void set_data(const MatrixType& data) { data_ = data; }

 private:
  MatrixType data_{MatrixType::Zero()};
};

}  // namespace internal
}  // namespace math
}  // namespace drake
