#include "drake/multibody/fem/corotated_model_data.h"

#include "drake/common/autodiff.h"
#include "drake/multibody/fem/matrix_utilities.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
CorotatedModelData<T>::CorotatedModelData() {
  R_ = Matrix3<T>::Identity();
  S_ = Matrix3<T>::Identity();
  Jm1_ = 0;
  JFinvT_ = Matrix3<T>::Identity();
}

template <typename T>
void CorotatedModelData<T>::UpdateFromDeformationGradient() {
  const Matrix3<T>& F = this->deformation_gradient();
  internal::PolarDecompose<T>(F, &R_, &S_);
  Jm1_ = F.determinant() - 1.0;
  internal::CalcCofactorMatrix<T>(F, &JFinvT_);
}

template class CorotatedModelData<float>;
template class CorotatedModelData<double>;
template class CorotatedModelData<AutoDiffXd>;

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
