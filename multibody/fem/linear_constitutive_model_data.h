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
 @tparam T The scalar type, can be a double, float, or AutoDiffXd. */
template <typename T>
class LinearConstitutiveModelData
    : public DeformationGradientData<LinearConstitutiveModelData<T>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LinearConstitutiveModelData);

  /* Constructs a LinearConstitutiveModelData with zero strain. */
  LinearConstitutiveModelData();

  /* Returns the infinitesimal strains (E = 0.5 * (F + Fᵀ) - I) evaluated with
   the deformation gradient passed when DeformationGradientData::UpdateData()
   was last invoked. */
  const Matrix3<T>& strain() const { return strain_; }

  /* Returns the traces of the infinitesimal strains (Tr(E)) evaluated with the
   deformation gradient passed when DeformationGradientData::UpdateData() was
   last invoked. */
  const T& trace_strain() const { return trace_strain_; }

 private:
  friend DeformationGradientData<LinearConstitutiveModelData<T>>;

  /* Shadows DeformationGradientData::UpdateFromDeformationGradient() as
   required by the CRTP base class. */
  void UpdateFromDeformationGradient();

  /* The infinitesimal strain. */
  Matrix3<T> strain_;
  /* Trace of `strain_`. */
  T trace_strain_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
