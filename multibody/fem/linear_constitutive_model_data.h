#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/deformation_gradient_data.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Data supporting calculations in LinearConstitutiveModel.
 See LinearConstitutiveModel for how the data is used. See
 DeformationGradientData for more about constitutive model data.
 @tparam_nonsymbolic_scalar T.
 @tparam num_locations Number of locations at which the deformation gradient
 dependent quantities are evaluated. */
template <typename T, int num_locations>
class LinearConstitutiveModelData
    : public DeformationGradientData<
          LinearConstitutiveModelData<T, num_locations>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LinearConstitutiveModelData);

  /* Constructs a LinearConstitutiveModelData with zero strain. */
  LinearConstitutiveModelData();

  /* Returns the infinitesimal strains evaluated at the quadrature locations
   for the associated element. */
  const std::array<Matrix3<T>, num_locations>& strain() const {
    return strain_;
  }

  /* Returns the traces of the infinitesimal strains evaluated at the
   quadrature locations for the associated element. */
  const std::array<T, num_locations>& trace_strain() const {
    return trace_strain_;
  }

 private:
  using Base =
      DeformationGradientData<LinearConstitutiveModelData<T, num_locations>>;

  friend Base;

  /* Shadows DeformationGradientData::UpdateFromDeformationGradient() as
   required by the CRTP base class. */
  void UpdateFromDeformationGradient();

  /* Infinitesimal strain = 0.5 * (F + Fᵀ) - I. */
  std::array<Matrix3<T>, num_locations> strain_;
  /* Trace of `strain_`. */
  std::array<T, num_locations> trace_strain_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
