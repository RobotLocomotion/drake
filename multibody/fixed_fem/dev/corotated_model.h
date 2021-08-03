#pragma once

#include <array>
#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/multibody/fixed_fem/dev/constitutive_model.h"
#include "drake/multibody/fixed_fem/dev/constitutive_model_utilities.h"
#include "drake/multibody/fixed_fem/dev/corotated_model_data.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Forward declare the model to be referred to in the traits class. */
template <typename T, int num_locations>
class CorotatedModel;

/* Traits for CorotatedModel. */
template <typename T, int num_locations>
struct CorotatedModelTraits {
  using Scalar = T;
  using ModelType = CorotatedModel<T, num_locations>;
  using DeformationGradientDataType = CorotatedModelData<T, num_locations>;
  static constexpr int kNumLocations = num_locations;
};

/* Implements the fixed corotated hyperelastic constitutive model as
 described in [Stomakhin, 2012].
 @tparam_nonsymbolic_scalar T.

 [Stomakhin, 2012] Stomakhin, Alexey, et al. "Energetically consistent
 invertible elasticity." Proceedings of the 11th ACM SIGGRAPH/Eurographics
 conference on Computer Animation. 2012. */
template <typename T, int num_locations>
class CorotatedModel final
    : public ConstitutiveModel<CorotatedModel<T, num_locations>,
                               CorotatedModelTraits<T, num_locations>> {
 public:
  using Traits = CorotatedModelTraits<T, num_locations>;
  using ModelType = typename Traits::ModelType;
  using DeformationGradientDataType =
      typename Traits::DeformationGradientDataType;
  using Base = ConstitutiveModel<ModelType, Traits>;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CorotatedModel)

  /* Constructs a CorotatedModel constitutive model with the
   prescribed Young's modulus and Poisson ratio.
   @param youngs_modulus Young's modulus of the model, with unit N/m²
   @param poisson_ratio Poisson ratio of the model, unitless.
   @pre youngs_modulus must be non-negative.
   @pre poisson_ratio must be strictly greater than -1 and strictly smaller than
   0.5. */
  CorotatedModel(const T& youngs_modulus, const T& poisson_ratio)
      : E_(youngs_modulus), nu_(poisson_ratio) {
    std::tie(lambda_, mu_) = CalcLameParameters(E_, nu_);
  }

  const T& youngs_modulus() const { return E_; }

  const T& poisson_ratio() const { return nu_; }

  const T& shear_modulus() const { return mu_; }

  const T& lame_first_parameter() const { return lambda_; }

 private:
  friend Base;

  /* Implements ConstitutiveModel::CalcElasticEnergyDensity() in the CRTP base
   class. */
  void DoCalcElasticEnergyDensity(
      const CorotatedModelData<T, num_locations>& data,
      std::array<T, num_locations>* Psi) const {
    for (int i = 0; i < num_locations; ++i) {
      const T& Jm1 = data.Jm1()[i];
      const Matrix3<T>& F = data.deformation_gradient()[i];
      const Matrix3<T>& R = data.R()[i];
      (*Psi)[i] = mu_ * (F - R).squaredNorm() + 0.5 * lambda_ * Jm1 * Jm1;
    }
  }

  /* Implements ConstitutiveModel::CalcFirstPiolaStress() in the CRTP base
   class. */
  void DoCalcFirstPiolaStress(const DeformationGradientDataType& data,
                              std::array<Matrix3<T>, num_locations>* P) const {
    for (int i = 0; i < num_locations; ++i) {
      const T& Jm1 = data.Jm1()[i];
      const Matrix3<T>& F = data.deformation_gradient()[i];
      const Matrix3<T>& R = data.R()[i];
      const Matrix3<T>& JFinvT = data.JFinvT()[i];
      (*P)[i].noalias() = 2.0 * mu_ * (F - R) + lambda_ * Jm1 * JFinvT;
    }
  }

  /* Implements ConstitutiveModel::CalcFirstPiolaStressDerivative() in the CRTP
   base class. */
  void DoCalcFirstPiolaStressDerivative(
      const DeformationGradientDataType& data,
      std::array<Eigen::Matrix<T, 9, 9>, num_locations>* dPdF) const {
    for (int i = 0; i < num_locations; ++i) {
      const T& Jm1 = data.Jm1()[i];
      const Matrix3<T>& F = data.deformation_gradient()[i];
      const Matrix3<T>& R = data.R()[i];
      const Matrix3<T>& S = data.S()[i];
      const Matrix3<T>& JFinvT = data.JFinvT()[i];
      const Vector<T, 3 * 3>& flat_JFinvT =
          Eigen::Map<const Vector<T, 3 * 3>>(JFinvT.data(), 3 * 3);
      auto& local_dPdF = (*dPdF)[i];
      /* The contribution from derivatives of Jm1. */
      local_dPdF.noalias() = lambda_ * flat_JFinvT * flat_JFinvT.transpose();
      /* The contribution from derivatives of F. */
      local_dPdF.diagonal().array() += 2.0 * mu_;
      /* The contribution from derivatives of R. */
      internal::AddScaledRotationalDerivative<T>(R, S, -2.0 * mu_, &local_dPdF);
      /* The contribution from derivatives of JFinvT. */
      internal::AddScaledCofactorMatrixDerivative<T>(F, lambda_ * Jm1,
                                                     &local_dPdF);
    }
  }

  T E_;       // Young's modulus, N/m².
  T nu_;      // Poisson ratio.
  T mu_;      // Lamé's second parameter/Shear modulus, N/m².
  T lambda_;  // Lamé's first parameter, N/m².
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
