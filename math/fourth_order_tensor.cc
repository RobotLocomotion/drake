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

}  // namespace internal
}  // namespace math
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::math::internal::FourthOrderTensor);
template class drake::math::internal::FourthOrderTensor<float>;
