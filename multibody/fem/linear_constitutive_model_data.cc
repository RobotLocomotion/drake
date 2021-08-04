#include "drake/multibody/fem/linear_constitutive_model_data.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T, int num_locations>
LinearConstitutiveModelData<T, num_locations>::LinearConstitutiveModelData() {
  /* Intialize data memebers to be consistent with the deformation gradient
   which is initialized to the identity matrix. This achieves the same result
   as invoking `UpdateFromDeformationGradient()` but is slightly more
   efficient.*/
  strain_.fill(Matrix3<T>::Zero());
  trace_strain_.fill(0.0);
}

template <typename T, int num_locations>
void LinearConstitutiveModelData<
    T, num_locations>::UpdateFromDeformationGradient() {
  const std::array<Matrix3<T>, num_locations>& F = this->deformation_gradient();
  for (int i = 0; i < num_locations; ++i) {
    strain_[i] = 0.5 * (F[i] + F[i].transpose()) - Matrix3<T>::Identity();
    trace_strain_[i] = strain_[i].trace();
  }
}

template class LinearConstitutiveModelData<double, 1>;
template class LinearConstitutiveModelData<AutoDiffXd, 1>;

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
