#include "drake/multibody/fem/linear_constitutive_model_data.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
LinearConstitutiveModelData<T>::LinearConstitutiveModelData() {
  /* Initialize data members to be consistent with the deformation gradient
   which is initialized to the identity matrix. This achieves the same result
   as invoking `UpdateFromDeformationGradient()` but is slightly more
   efficient.*/
  strain_ = Matrix3<T>::Zero();
  trace_strain_ = 0.0;
}

template <typename T>
void LinearConstitutiveModelData<T>::UpdateFromDeformationGradient() {
  const Matrix3<T>& F = this->deformation_gradient();
  strain_ = 0.5 * (F + F.transpose()) - Matrix3<T>::Identity();
  trace_strain_ = strain_.trace();
}

template class LinearConstitutiveModelData<float>;
template class LinearConstitutiveModelData<double>;
template class LinearConstitutiveModelData<AutoDiffXd>;

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
