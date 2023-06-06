#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/deformation_gradient_data.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Data supporting calculations in LinearCorotatedModel.
 @tparam_nonsymbolic_scalar
 @tparam num_locations Number of locations at which the deformation gradient
 dependent quantities are evaluated. */
template <typename T, int num_locations>
class LinearCorotatedModelData
    : public DeformationGradientData<
          LinearCorotatedModelData<T, num_locations>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LinearCorotatedModelData);

  /* Constructs a LinearCorotatedModelData with no deformation. */
  LinearCorotatedModelData();

  /* Returns the rotation matrices from the polar decomposition of F₀ = R₀*S₀.
   */
  const std::array<Matrix3<T>, num_locations>& R0() const { return R0_; }

  /* Returns the strain matrix 1/2 * (R₀ᵀ * F + Fᵀ * R₀) - I. */
  const std::array<Matrix3<T>, num_locations>& strain() const {
    return strain_;
  }

  /* Returns the trace of strain. */
  const std::array<T, num_locations>& trace_strain() const {
    return trace_strain_;
  }

 private:
  /* Allow base class friend access to the private CalcFooImpl functions. */
  friend DeformationGradientData<LinearCorotatedModelData<T, num_locations>>;

  /* Shadows DeformationGradientData::UpdateFromDeformationGradient() as
   required by the CRTP base class. */
  void UpdateFromDeformationGradient();

  std::array<Matrix3<T>, num_locations> R0_;
  std::array<Matrix3<T>, num_locations> strain_;
  std::array<T, num_locations> trace_strain_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
