#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/deformation_gradient_data.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Data supporting calculations in LinearCorotatedModel.
 @tparam T The scalar type, can be a double, float, or AutoDiffXd. */
template <typename T>
class LinearCorotatedModelData
    : public DeformationGradientData<LinearCorotatedModelData<T>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LinearCorotatedModelData);

  /* Constructs a LinearCorotatedModelData with no deformation. */
  LinearCorotatedModelData();

  /* Returns the rotation matrices from the polar decomposition of F₀ = R₀*S₀.
   */
  const Matrix3<T>& R0() const { return R0_; }

  /* Returns the strain matrix 1/2 * (R₀ᵀ * F + Fᵀ * R₀) - I. */
  const Matrix3<T>& strain() const { return strain_; }

  /* Returns the trace of strain. */
  const T& trace_strain() const { return trace_strain_; }

 private:
  /* Allow base class friend access to the private CalcFooImpl functions. */
  friend DeformationGradientData<LinearCorotatedModelData<T>>;

  /* Shadows DeformationGradientData::UpdateFromDeformationGradient() as
   required by the CRTP base class. */
  void UpdateFromDeformationGradient();

  Matrix3<T> R0_;
  Matrix3<T> strain_;
  T trace_strain_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
