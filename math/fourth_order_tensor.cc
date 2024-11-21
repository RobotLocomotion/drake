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
  const auto M_vec = Eigen::Map<const Vector<T, 9>>(M.data(), 9);
  const auto N_vec = Eigen::Map<const Vector<T, 9>>(N.data(), 9);
  data_.noalias() = M_vec * N_vec.transpose();
}

template <typename T>
FourthOrderTensor<T> FourthOrderTensor<T>::MakeSymmetricIdentity(T scale) {
  T half_scale = 0.5 * scale;
  FourthOrderTensor<T> result;
  result.data_ = half_scale * Eigen::Matrix<T, 9, 9>::Identity();
  for (int k = 0; k < 3; ++k) {
    /* Second term. */
    for (int l = 0; l < 3; ++l) {
      const int i = l;
      const int j = k;
      result.data_(3 * j + i, 3 * l + k) += half_scale;
    }
  }
  return result;
}

template <typename T>
FourthOrderTensor<T> FourthOrderTensor<T>::MakeMajorIdentity(T scale) {
  return FourthOrderTensor<T>(scale * Eigen::Matrix<T, 9, 9>::Identity());
}

template <typename T>
FourthOrderTensor<T>& FourthOrderTensor<T>::operator+=(
    const FourthOrderTensor<T>& other) {
  data_.noalias() += other.data_;
  return *this;
}

}  // namespace internal
}  // namespace math
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::math::internal::FourthOrderTensor);
template class drake::math::internal::FourthOrderTensor<float>;
