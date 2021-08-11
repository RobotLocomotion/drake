#pragma once

#include <array>

#include "drake/multibody/fem/deformation_gradient_data.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Data supporting calculations in LinearConstitutiveModel. Specifically, the
 supporting data are:
 1. the infinitesimal strain, E = 0.5 * (F + Fᵀ) - I, and
 2. the trace of the infinitesimal strain, Tr(E).

 See LinearConstitutiveModel for how the data is used. See
 DeformationGradientData for more about constitutive model data.
 @tparam_nonsymbolic_scalar.
 @tparam num_locations Number of locations at which the deformation gradient
 dependent quantities are evaluated. We currently only provide one instantiation
 of this template with `num_locations = 1`, but more instantiations can easily
 be added when needed. */
template <typename T, int num_locations>
class LinearConstitutiveModelData
    : public DeformationGradientData<
          LinearConstitutiveModelData<T, num_locations>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LinearConstitutiveModelData);

  /* Constructs a LinearConstitutiveModelData with zero strain. */
  LinearConstitutiveModelData();

  /* Returns the infinitesimal strains (E = 0.5 * (F + Fᵀ) - I) evaluated with
   the deformation gradient passed when DeformationGradientData::UpdateData()
   was last invoked. */
  const std::array<Matrix3<T>, num_locations>& strain() const {
    return strain_;
  }

  /* Returns the traces of the infinitesimal strains (Tr(E)) evaluated with the
   deformation gradient passed when DeformationGradientData::UpdateData() was
   last invoked. */
  const std::array<T, num_locations>& trace_strain() const {
    return trace_strain_;
  }

 private:
  friend DeformationGradientData<LinearConstitutiveModelData<T, num_locations>>;

  /* Shadows DeformationGradientData::UpdateFromDeformationGradient() as
   required by the CRTP base class. */
  void UpdateFromDeformationGradient();

  /* The infinitesimal strain. */
  std::array<Matrix3<T>, num_locations> strain_;
  /* Trace of `strain_`. */
  std::array<T, num_locations> trace_strain_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
