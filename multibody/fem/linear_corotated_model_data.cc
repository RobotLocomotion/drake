#include "drake/multibody/fem/linear_corotated_model_data.h"

#include "drake/common/autodiff.h"
#include "drake/multibody/fem/matrix_utilities.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T, int num_locations>
LinearCorotatedModelData<T, num_locations>::LinearCorotatedModelData() {
  std::fill(R0_.begin(), R0_.end(), Matrix3<T>::Identity());
  std::fill(strain_.begin(), strain_.end(), Matrix3<T>::Zero());
  std::fill(trace_strain_.begin(), trace_strain_.end(), 0);
}
template <typename T, int num_locations>
void LinearCorotatedModelData<T,
                              num_locations>::UpdateFromDeformationGradient() {
  const std::array<Matrix3<T>, num_locations>& F = this->deformation_gradient();
  const std::array<Matrix3<T>, num_locations>& F0 =
      this->time_step_deformation_gradient();
  for (int i = 0; i < num_locations; ++i) {
    Matrix3<T>& local_R0 = R0_[i];
    Matrix3<T>& local_strain = strain_[i];
    Matrix3<T> unused_S;
    internal::PolarDecompose<T>(F0[i], &local_R0, &unused_S);
    Matrix3<T> R0tF = local_R0.transpose() * F[i];
    local_strain = 0.5 * (R0tF + R0tF.transpose()) - Matrix3<T>::Identity();
    trace_strain_[i] = local_strain.trace();
  }
}

template class LinearCorotatedModelData<double, 1>;
template class LinearCorotatedModelData<AutoDiffXd, 1>;

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
