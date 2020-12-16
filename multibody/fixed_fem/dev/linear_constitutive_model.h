#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/multibody/fixed_fem/dev/constitutive_model.h"
#include "drake/multibody/fixed_fem/dev/linear_constitutive_model_cache_entry.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/* Forward declare the model to be referred to in the traits class. */
template <typename T, int NumLocations>
class LinearConstitutiveModel;

/** Traits for LinearConstitutiveModel. */
template <typename T, int NumLocations>
struct LinearConstitutiveModelTraits {
  using Scalar = T;
  using ModelType = LinearConstitutiveModel<T, NumLocations>;
  using DeformationGradientCacheEntryType =
      LinearConstitutiveModelCacheEntry<T, NumLocations>;
  static constexpr int kNumLocations = NumLocations;
};

/** Implements the infinitesimal-strain linear elasticity constitutive model as
 described in Section 7.4 of [Gonzalez, 2008].
 @tparam_nonsymbolic_scalar T.

[Gonzalez, 2008] Gonzalez, Oscar, and Andrew M. Stuart. A first course in
continuum mechanics. Cambridge University Press, 2008. */
template <typename T, int NumLocations>
class LinearConstitutiveModel final
    : public ConstitutiveModel<LinearConstitutiveModel<T, NumLocations>,
                               LinearConstitutiveModelTraits<T, NumLocations>> {
 public:
  using Traits = LinearConstitutiveModelTraits<T, NumLocations>;
  using ModelType = typename Traits::ModelType;
  using DeformationGradientCacheEntryType =
      typename Traits::DeformationGradientCacheEntryType;
  using Base = ConstitutiveModel<ModelType, Traits>;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LinearConstitutiveModel)

  /** Constructs a %LinearConstitutiveModel constitutive model with the
   prescribed Young's modulus and Poisson ratio.
   @param youngs_modulus Young's modulus of the model, with unit N/m²
   @param poisson_ratio Poisson ratio of the model, unitless.
   @pre youngs_modulus must be non-negative.
   @pre poisson_ratio must be strictly greater than -1 and strictly smaller than
   0.5. */
  LinearConstitutiveModel(const T& youngs_modulus, const T& poisson_ratio)
      : E_(youngs_modulus), nu_(poisson_ratio) {
    VerifyParameterValidity(E_, nu_);
    SetLameParameters(E_, nu_);
    /* Recall that
          Pᵢⱼ = 2μ * εᵢⱼ + λ * εₐₐ * δᵢⱼ,
      So,
          ∂Pᵢⱼ/∂Fₖₗ = 2μ * ∂εᵢⱼ/∂Fₖₗ + λ * ∂εₐₐ/∂Fₖₗ * δᵢⱼ,
      Since
          ∂εᵢⱼ/∂Fₖₗ = 0.5 * δᵢₖ δⱼₗ  + 0.5 * δᵢₗ δₖⱼ.
      Plugging in, we get:
          ∂Pᵢⱼ/∂Fₖₗ = μ * (δᵢₖδⱼₗ + δᵢₗ δⱼₖ) +  λ * δₖₗ * δᵢⱼ.
      Keep in mind that the indices are laid out such that the ik-th entry in
      the jl-th block corresponds to the value dPᵢⱼ/dFₖₗ.  */
    // First term.
    dPdF_ = mu_ * Eigen::Matrix<T, 9, 9>::Identity();
    for (int k = 0; k < 3; ++k) {
      // Second term.
      for (int l = 0; l < 3; ++l) {
        const int i = l;
        const int j = k;
        dPdF_(3 * j + i, 3 * l + k) += mu_;
      }
      // Third term.
      for (int i = 0; i < 3; ++i) {
        const int l = k;
        const int j = i;
        dPdF_(3 * j + i, 3 * l + k) += lambda_;
      }
    }
  }

  ~LinearConstitutiveModel() = default;

  T youngs_modulus() const { return E_; }

  T poisson_ratio() const { return nu_; }

  T shear_modulus() const { return mu_; }

  T lame_first_parameter() const { return lambda_; }

 private:
  friend Base;

  /* Implements the interface ConstitutiveModel::CalcElasticEnergyDensity() in
   the CRTP base class. */
  void DoCalcElasticEnergyDensity(
      const LinearConstitutiveModelCacheEntry<T, NumLocations>& cache_entry,
      std::array<T, NumLocations>* Psi) const {
    for (int i = 0; i < NumLocations; ++i) {
      const auto& strain = cache_entry.strain()[i];
      const auto& trace_strain = cache_entry.trace_strain()[i];
      (*Psi)[i] = mu_ * strain.squaredNorm() +
                  0.5 * lambda_ * trace_strain * trace_strain;
    }
  }

  /* Implements the interface ConstitutiveModel::CalcFirstPiolaStress()
   in the CRTP base class. */
  void DoCalcFirstPiolaStress(
      const DeformationGradientCacheEntryType& cache_entry,
      std::array<Matrix3<T>, NumLocations>* P) const {
    for (int i = 0; i < NumLocations; ++i) {
      const auto& strain = cache_entry.strain()[i];
      const auto& trace_strain = cache_entry.trace_strain()[i];
      (*P)[i] =
          2.0 * mu_ * strain + lambda_ * trace_strain * Matrix3<T>::Identity();
    }
  }

  /* Implements the interface
   ConstitutiveModel::CalcFirstPiolaStressDerivative() in the CRTP base class.
  */
  void DoCalcFirstPiolaStressDerivative(
      const DeformationGradientCacheEntryType& cache_entry,
      std::array<Eigen::Matrix<T, 9, 9>, NumLocations>* dPdF) const {
    unused(cache_entry);
    std::fill(dPdF->begin(), dPdF->end(), dPdF_);
  }

  /* Set the Lamé parameters from Young's modulus and Poisson ratio. It's
   important to keep the Lamé Parameters in sync with Young's modulus and
   Poisson ratio as most computations use Lame parameters. */
  void VerifyParameterValidity(const T& youngs_modulus,
                               const T& poisson_ratio) const {
    if (youngs_modulus < 0.0) {
      throw std::logic_error("Young's modulus must be nonnegative.");
    }
    if (poisson_ratio >= 0.5 || poisson_ratio <= -1) {
      throw std::logic_error("Poisson ratio must be in (-1, 0.5).");
    }
  }

  void SetLameParameters(const T& youngs_modulus, const T& poisson_ratio) {
    mu_ = youngs_modulus / (2.0 * (1.0 + poisson_ratio));
    lambda_ = youngs_modulus * poisson_ratio /
              ((1.0 + poisson_ratio) * (1.0 - 2.0 * poisson_ratio));
  }

  T E_;       // Young's modulus, N/m².
  T nu_;      // Poisson ratio.
  T mu_;      // Lamé's second parameter/Shear modulus, N/m².
  T lambda_;  // Lamé's first parameter, N/m².
  Eigen::Matrix<T, 9, 9>
      dPdF_;  // The First Piola stress derivative is constant and precomputed.
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
