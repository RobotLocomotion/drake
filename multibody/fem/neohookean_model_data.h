#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/deformation_gradient_data.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Data supporting calculations in NeoHookeanModel. The constitutive model is
 described in section 3.4 in [Smith et al., 2019]. In particular, this class
 stores the SVD of the deformation gradient F, along with J-1, the first
 Cauchy-Green invariant (Ic = tr(FᵀF)). and JF⁻ᵀ (J = det(F)) used in evaluating
 the energy density and its derivatives. See DeformationGradientData for more
 about constitutive model data.
 [Smith et al., 2019] Smith, Breannan, Fernando De Goes, and Theodore Kim.
 "Stable Neo-Hookean flesh simulation." ACM Transactions on Graphics (TOG) 37.2
 (2018): 1-15.
 @tparam T The scalar type, can be a double, float, or AutoDiffXd. */
template <typename T>
class NeoHookeanModelData
    : public DeformationGradientData<NeoHookeanModelData<T>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(NeoHookeanModelData);

  /* Constructs a NeoHookeanModelData with no deformation. */
  NeoHookeanModelData();

  /* Returns the first Cauchy-Green invariant Ic = tr(FᵀF) where F is the
   deformation gradient. */
  const T& Ic() const { return Ic_; }

  /* Returns the J-1 where J is the determinant of the deformation gradient. */
  const T& Jm1() const { return Jm1_; }

  /* Returns the dJdF = JF⁻ᵀ. */
  const Matrix3<T>& dJdF() const { return dJdF_; }

  /* Returns the U matrix from the SVD of F = USVᵀ. */
  const Matrix3<T>& U() const { return U_; }

  /* Returns the V matrix from the SVD of F = USVᵀ. */
  const Matrix3<T>& V() const { return V_; }

  /* Returns the singular values of F from the SVD of F = USVᵀ. */
  const Vector3<T>& sigma() const { return sigma_; }

 private:
  friend DeformationGradientData<NeoHookeanModelData<T>>;

  /* Shadows DeformationGradientData::UpdateFromDeformationGradient() as
   required by the CRTP base class. */
  void UpdateFromDeformationGradient();
  /* tr(FᵀF) */
  T Ic_;
  /* The determinant of F minus 1, or J-1. */
  T Jm1_;
  /* The derivative of J wrt F, which is the the cofactor matrix of F, or JF⁻ᵀ.
   */
  Matrix3<T> dJdF_;
  /* The SVD of F = USVᵀ where U and V are both rotation matrices and
   S = diag(σ₁, σ₂, σ₃) is a diagonal matrix with the singular values of F (that
   may be negative) to force U and V to be rotation matrices. */
  Matrix3<T> U_;
  Matrix3<T> V_;
  Vector3<T> sigma_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
