#pragma once

#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"

namespace drake {
namespace fem {

/** A constitutive model relates the strain to the stress of the material. It
 governs the material response under deformation. For hyperelastic materials,
 the constitutive relation is defined through the potential energy, which
 increases with non-rigid deformation from the initial state.
*/
template <typename T>
class HyperelasticConstitutiveModel {
 public:
  HyperelasticConstitutiveModel(T E, T nu, T alpha, T beta)
      : E_(E), nu_(nu), alpha_(alpha), beta_(beta) {
    VerifyParameterValidity(E, nu, alpha, beta);
    SetLameParameters(E, nu);
  }

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(HyperelasticConstitutiveModel)

  virtual ~HyperelasticConstitutiveModel() {}

  /** Update the states that depend on the deformation gradient F.
    @param[in] F  The deformation gradient evaluated at where the constitutive
    model lives.
  */
  void UpdateDeformationBasedState(const Eigen::Ref<const Matrix3<T>>& F) {
    DoUpdateDeformationBasedState(F);
  }

  /** Updates the states that depend on the positions the control vertices of
    the element that the constitutive model lives on.
    @param[in] q  The positions of the control vertices of the elements where
    the constitutive model lives.
  */
  /* TODO(xuchenhan-tri): Make the argument type more general to support
   elements with more than 4 vertices. This should be straightforward when we
   introduce the FemElement class. */
  void UpdatePositionBasedState(
      const Eigen::Ref<const Eigen::Matrix<T, 3, 4>>& q) {
    DoUpdatePositionBasedState(q);
  }

  T get_E() const { return E_; }
  T get_nu() const { return nu_; }
  T get_alpha() const { return alpha_; }
  T get_beta() const { return beta_; }
  T get_mu() const { return mu_; }
  T get_lambda() const { return lambda_; }

  /** Calculates the energy density. */
  T CalcEnergyDensity() const { return DoCalcEnergyDensity(); }

  /** Calculates the First Piola stress under current states. */
  Matrix3<T> CalcFirstPiola() const { return DoCalcFirstPiola(); }

  /** Calculates the First Piola stress Differential dP(dF) = dP/dF * dF under
   * current states. */
  Matrix3<T> CalcFirstPiolaDifferential(
      const Eigen::Ref<const Matrix3<T>>& dF) const {
    return DoCalcFirstPiolaDifferential(dF);
  }

  /** Calculates the First Piola stress derivative dP/dF under current states.
     The resulting 4th order tensor in index notation is Aᵢⱼₐᵦ = ∂Pᵢⱼ/∂Fₐᵦ. We
     flatten it out into a matrix, and the indices should be laid out as
     following:
                     β = 0       β = 1       β = 2
                 -------------------------------------
                 |           |           |           |
       j = 0     |   Aᵢ₀ₐ₀   |   Aᵢ₀ₐ₁   |   Aᵢ₀ₐ₂   |
                 |           |           |           |
                 -------------------------------------
                 |           |           |           |
       j = 1     |   Aᵢ₁ₐ₀   |   Aᵢ₁ₐ₁   |   Aᵢ₁ₐ₂   |
                 |           |           |           |
                 -------------------------------------
                 |           |           |           |
       j = 2     |   Aᵢ₂ₐ₀   |   Aᵢ₂ₐ₁   |   Aᵢ₂ₐ₂   |
                 |           |           |           |
                 -------------------------------------

                 where each jβ-th block assumes the standard matrix layout:

                     a = 0     a = 1      a = 2
                 ----------------------------------
                 |                                |
       i = 0     |   A₀ⱼ₀ᵦ      A₀ⱼ₁ᵦ      A₀ⱼ₂ᵦ   |
                 |                                |
                 |                                |
       i = 1     |   A₁ⱼ₀ᵦ      A₁ⱼ₁ᵦ      A₁ⱼ₂ᵦ   |
                 |                                |
                 |                                |
       i = 2     |   A₂ⱼ₀ᵦ      A₂ⱼ₁ᵦ      A₂ⱼ₂ᵦ   |
                 |                                |
                 ----------------------------------
   */
  Eigen::Matrix<T, 9, 9> CalcFirstPiolaDerivative() const {
    return DoCalcFirstPiolaDerivative();
  }

 protected:
  /* Update the states that depend on the deformation gradient F.
    @param[in] F  The deformation gradient evaluated at where the constitutive
    model lives.
  */
  virtual void DoUpdateDeformationBasedState(
      const Eigen::Ref<const Matrix3<T>>& F) = 0;

  /* Updates the states that depend on the positions the control vertices of
    the element that the constitutive model lives on.
    @param[in] q  The positions of the control vertices of the elements where
    the constitutive model lives.
  */
  /* TODO(xuchenhan-tri): Make the argument type more general to support
   elements with more than 4 vertices. This should be straightforward when we
   introduce the FemElement class. */
  virtual void DoUpdatePositionBasedState(
      const Eigen::Ref<const Eigen::Matrix<T, 3, 4>>&) {
    throw std::runtime_error("DoUpdatePositionBasedState(): Concrete type " +
                             NiceTypeName::Get(*this) +
                             " must provide an implementation.");
  }

  /* Calculates the energy density. */
  virtual T DoCalcEnergyDensity() const = 0;

  /* Calculates the First Piola stress under current states. */
  virtual Matrix3<T> DoCalcFirstPiola() const = 0;

  /* Calculates the First Piola stress Differential dP(dF) = dP/dF * dF under
    current states. */
  virtual Matrix3<T> DoCalcFirstPiolaDifferential(
      const Eigen::Ref<const Matrix3<T>>& dF) const = 0;

  /* Calculates the First Piola stress derivative dP/dF under current states.
   */
  virtual Eigen::Matrix<T, 9, 9> DoCalcFirstPiolaDerivative() const = 0;

 private:
  /* Set the Lamé parameters from Young's modulus and Poisson ratio. It's
     important to keep the Lamé Parameters in sync with Young's modulus and
     Poisson ratio as most computations use Lame parameters. */
  void VerifyParameterValidity(T E, T nu, T alpha, T beta) {
    if (E < 0.0) {
      throw std::logic_error("Young's modulus must be nonnegative.");
    }
    if (nu >= 0.5 || nu <= -1) {
      throw std::logic_error("Poisson ratio must be in (-1, 0.5).");
    }
    if (alpha < 0.0) {
      throw std::logic_error("Mass damping parameter must be nonnegative.");
    }
    if (beta < 0.0) {
      throw std::logic_error(
          "Stiffness damping parameter must be nonnegative.");
    }
  }

  void SetLameParameters(T E, T nu) {
    mu_ = E / (2.0 * (1.0 + nu));
    lambda_ = E * nu / ((1.0 + nu) * (1.0 - 2.0 * nu));
  }

  // Young's modulus.
  T E_{0};
  // Poisson ratio.
  T nu_{0};
  // Mass damping factor.
  T alpha_{0};
  // Stiffness damping factor.
  T beta_{0};
  // Lamé's second parameter/Shear modulus.
  T mu_{0};
  // Lamé's first parameter.
  T lambda_{0};
};

}  // namespace fem
}  // namespace drake
