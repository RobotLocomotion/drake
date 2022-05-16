#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/deformation_gradient_data.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Data supporting calculations in CorotatedModel. The constitutive model is
 described in section 3.4 in [Stomakhin, 2012]. In particular, this class stores
 the polar decomposition of the deformation gradient F, along with J-1 and JF⁻ᵀ
 (J = det(F)) used in evaluating the energy density and its derivatives.
 See DeformationGradientData for more about constitutive model data.
 @tparam_nonsymbolic_scalar
 @tparam num_locations Number of locations at which the deformation gradient
 dependent quantities are evaluated. */
template <typename T, int num_locations>
class CorotatedModelData
    : public DeformationGradientData<CorotatedModelData<T, num_locations>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CorotatedModelData);

  /* Constructs a CorotatedModelData with no deformation. */
  CorotatedModelData();

  /* Returns the rotation matrices from the polar decomposition of F = R*S. */
  const std::array<Matrix3<T>, num_locations>& R() const { return R_; }

  /* Returns the symmetric matrices from the polar decomposition of F = R*S. */
  const std::array<Matrix3<T>, num_locations>& S() const { return S_; }

  /* Returns the J-1 where J is the determinant of the deformation gradient. */
  const std::array<T, num_locations>& Jm1() const { return Jm1_; }

  /* Returns the JF⁻ᵀ. */
  const std::array<Matrix3<T>, num_locations>& JFinvT() const {
    return JFinvT_;
  }

 private:
  friend DeformationGradientData<CorotatedModelData<T, num_locations>>;

  /* Shadows DeformationGradientData::UpdateFromDeformationGradient() as
   required by the CRTP base class. */
  void UpdateFromDeformationGradient();

  /* Let F = RS be the polar decomposition of the deformation gradient where R
   is a rotation matrix and S is symmetric. */
  std::array<Matrix3<T>, num_locations> R_;
  std::array<Matrix3<T>, num_locations> S_;
  /* The determinant of F minus 1, or J - 1. */
  std::array<T, num_locations> Jm1_;
  /* The cofactor matrix of F, or JF⁻ᵀ. */
  std::array<Matrix3<T>, num_locations> JFinvT_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
