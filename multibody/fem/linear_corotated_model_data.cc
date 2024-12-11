#include "drake/multibody/fem/linear_corotated_model_data.h"

#include "drake/common/autodiff.h"
#include "drake/multibody/fem/matrix_utilities.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
LinearCorotatedModelData<T>::LinearCorotatedModelData() {
  R0_ = Matrix3<T>::Identity();
  strain_ = Matrix3<T>::Zero();
  trace_strain_ = 0.0;
}
template <typename T>
void LinearCorotatedModelData<T>::UpdateFromDeformationGradient() {
  const Matrix3<T>& F = this->deformation_gradient();
  const Matrix3<T>& F0 = this->previous_step_deformation_gradient();
  Matrix3<T> unused_S;
  internal::PolarDecompose<T>(F0, &R0_, &unused_S);
  const Matrix3<T> corotated_F = R0_.transpose() * F;
  strain_ =
      0.5 * (corotated_F + corotated_F.transpose()) - Matrix3<T>::Identity();
  trace_strain_ = strain_.trace();
}

template class LinearCorotatedModelData<float>;
template class LinearCorotatedModelData<double>;
template class LinearCorotatedModelData<AutoDiffXd>;

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
