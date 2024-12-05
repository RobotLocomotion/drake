#include "drake/math/fourth_order_tensor.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace math {
namespace internal {

template <typename T>
FourthOrderTensor<T>::FourthOrderTensor(const MatrixType& data) : data_(data) {}

template <typename T>
FourthOrderTensor<T>::FourthOrderTensor() = default;

template <typename T>
void FourthOrderTensor<T>::ContractWithVectors(
    const Eigen::Ref<const Vector3<T>>& u,
    const Eigen::Ref<const Vector3<T>>& v, EigenPtr<Matrix3<T>> B) const {
  B->setZero();
  for (int l = 0; l < 3; ++l) {
    for (int j = 0; j < 3; ++j) {
      *B += data_.template block<3, 3>(3 * j, 3 * l) * u(j) * v(l);
    }
  }
}

template <typename T>
void FourthOrderTensor<T>::SetAsOuterProduct(
    const Eigen::Ref<const Matrix3<T>>& M,
    const Eigen::Ref<const Matrix3<T>>& N) {
  const auto M_vec = Eigen::Map<const Vector<T, 9>>(M.data());
  const auto N_vec = Eigen::Map<const Vector<T, 9>>(N.data());
  data_.noalias() = M_vec * N_vec.transpose();
}

template <typename T>
FourthOrderTensor<T> FourthOrderTensor<T>::MakeSymmetricIdentity(T scale) {
  const T half_scale = 0.5 * scale;
  FourthOrderTensor<T> result;
  /* The δᵢₖδⱼₗ term. */
  result.SetToDiagonal(half_scale);
  /* Outerloop j(k) and inner loop i(l) to be more memory friendly (the matrix
   data is column major). */
  for (int j = 0; j < 3; ++j) {
    /* The δᵢₗδⱼₖ term. */
    for (int i = 0; i < 3; ++i) {
      const int l = i;
      const int k = j;
      result.data_(3 * j + i, 3 * l + k) += half_scale;
    }
  }
  return result;
}

template <typename T>
FourthOrderTensor<T> FourthOrderTensor<T>::MakeMajorIdentity(T scale) {
  FourthOrderTensor<T> result;
  result.SetToDiagonal(scale);
  return result;
}

template <typename T>
FourthOrderTensor<T>& FourthOrderTensor<T>::operator+=(
    const FourthOrderTensor<T>& other) {
  data_ += other.data_;
  return *this;
}

template <typename T>
void FourthOrderTensor<T>::SetToDiagonal(T scale) {
  /* This generates better instructions than calling.
   data_ = scale*Matrix<T,9,9>::Identity() */
  data_.setZero();
  data_.diagonal().array() = scale;
}

}  // namespace internal
}  // namespace math
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::math::internal::FourthOrderTensor);
template class drake::math::internal::FourthOrderTensor<float>;
