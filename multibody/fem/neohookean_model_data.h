#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/deformation_gradient_data.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Data supporting calculations in NeoHookeanModel. The constitutive model is
 described in section 3.4 in [Smith, 2019]. In particular, this class stores
 the polar decomposition of the deformation gradient F, along with J-1 and JF⁻ᵀ
 (J = det(F)) used in evaluating the energy density and its derivatives.
 See DeformationGradientData for more about constitutive model data.
 @tparam T The scalar type, can be a double, float, or AutoDiffXd. */
template <typename T>
class NeoHookeanModelData
    : public DeformationGradientData<NeoHookeanModelData<T>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(NeoHookeanModelData);

  /* Constructs a NeoHookeanModelData with no deformation. */
  NeoHookeanModelData();

  /* Returns the tr(FᵀF) where F is the deformation gradient. */
  const T& Ic() const { return Ic_; }

  /* Returns the J-1 where J is the determinant of the deformation gradient. */
  const T& Jm1() const { return Jm1_; }

  /* Returns the JF⁻ᵀ. */
  const Matrix3<T>& dJdF() const { return dJdF_; }

 private:
  friend DeformationGradientData<NeoHookeanModelData<T>>;

  /* Shadows DeformationGradientData::UpdateFromDeformationGradient() as
   required by the CRTP base class. */
  void UpdateFromDeformationGradient();
  /* tr(FᵀF) */
  T Ic_;
  /* The determinant of F minus 1, or J - 1. */
  T Jm1_;
  /* The derivative of J wrt F, which is the the cofactor matrix of F, or JF⁻ᵀ.
   */
  Matrix3<T> dJdF_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
