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
 @tparam T The scalar type, can be a double, float, or AutoDiffXd. */
template <typename T>
class CorotatedModelData
    : public DeformationGradientData<CorotatedModelData<T>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CorotatedModelData);

  /* Constructs a CorotatedModelData with no deformation. */
  CorotatedModelData();

  /* Returns the rotation matrices from the polar decomposition of F = R*S. */
  const Matrix3<T>& R() const { return R_; }

  /* Returns the symmetric matrices from the polar decomposition of F = R*S. */
  const Matrix3<T>& S() const { return S_; }

  /* Returns the J-1 where J is the determinant of the deformation gradient. */
  const T& Jm1() const { return Jm1_; }

  /* Returns the JF⁻ᵀ. */
  const Matrix3<T>& JFinvT() const { return JFinvT_; }

 private:
  friend DeformationGradientData<CorotatedModelData<T>>;

  /* Shadows DeformationGradientData::UpdateFromDeformationGradient() as
   required by the CRTP base class. */
  void UpdateFromDeformationGradient();

  /* Let F = RS be the polar decomposition of the deformation gradient where R
   is a rotation matrix and S is symmetric. */
  Matrix3<T> R_;
  Matrix3<T> S_;
  /* The determinant of F minus 1, or J - 1. */
  T Jm1_;
  /* The cofactor matrix of F, or JF⁻ᵀ. */
  Matrix3<T> JFinvT_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
