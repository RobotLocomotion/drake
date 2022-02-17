#include "drake/multibody/fem/corotated_model_data.h"

#include "drake/common/autodiff.h"
#include "drake/multibody/fem/matrix_utilities.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T, int num_locations>
CorotatedModelData<T, num_locations>::CorotatedModelData() {
  std::fill(R_.begin(), R_.end(), Matrix3<T>::Identity());
  std::fill(S_.begin(), S_.end(), Matrix3<T>::Identity());
  std::fill(Jm1_.begin(), Jm1_.end(), 0);
  std::fill(JFinvT_.begin(), JFinvT_.end(), Matrix3<T>::Identity());
}
template <typename T, int num_locations>
void CorotatedModelData<T, num_locations>::UpdateFromDeformationGradient() {
  const std::array<Matrix3<T>, num_locations>& F = this->deformation_gradient();
  for (int i = 0; i < num_locations; ++i) {
    Matrix3<T>& local_R = R_[i];
    Matrix3<T>& local_S = S_[i];
    Matrix3<T>& local_JFinvT = JFinvT_[i];
    internal::PolarDecompose<T>(F[i], &local_R, &local_S);
    Jm1_[i] = F[i].determinant() - 1.0;
    internal::CalcCofactorMatrix<T>(F[i], &local_JFinvT);
  }
}

template class CorotatedModelData<double, 1>;
template class CorotatedModelData<AutoDiffXd, 1>;

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
