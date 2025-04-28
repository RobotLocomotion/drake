#include "drake/multibody/fem/neohookean_model_data.h"

#include "drake/common/autodiff.h"
#include "drake/multibody/fem/matrix_utilities.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
NeoHookeanModelData<T>::NeoHookeanModelData() {
  Ic_ = 3.0;
  Jm1_ = 0;
  dJdF_ = Matrix3<T>::Identity();
}

template <typename T>
void NeoHookeanModelData<T>::UpdateFromDeformationGradient() {
  const Matrix3<T>& F = this->deformation_gradient();
  Ic_ = F.squaredNorm();
  Jm1_ = F.determinant() - 1.0;
  internal::CalcCofactorMatrix<T>(F, &dJdF_);
}

template class NeoHookeanModelData<float>;
template class NeoHookeanModelData<double>;
template class NeoHookeanModelData<AutoDiffXd>;

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
