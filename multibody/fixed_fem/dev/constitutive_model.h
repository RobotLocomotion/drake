#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/deformation_gradient_cache_entry.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** A constitutive model relates the strain to the stress of the material and
 governs the material response under deformation. This constitutive relationship
 is defined through a hyperelastic potential energy, which increases with
 non-rigid deformation from the initial state.

 %ConstitutiveModel serves as the interface base class for all hyperelastic
 constitutive models. Since constitutive models are usually evaluated in
 computationally intensive inner loops of the simulation, the overhead caused by
 virtual methods may be significant. Therefore, this class uses a CRTP pattern
 to achieve compile-time polymorphism and avoids the overhead of virtual
 methods and facilitates inlining instead. Derived constitutive models must
 inherit from this base class and implement the interface this class provides.
 The derived constitutive model must also be accompanied by a corresponding
 traits class that declares the compile time quantities and type declarations
 that this base class requires.
 @tparam DerivedConstitutiveModel The concrete constitutive model that inherits
 from %ConstitutiveModel through CRTP.
 @tparam DerivedTraits The traits class associated with the
 DerivedConstitutiveModel. */
template <class DerivedConstitutiveModel, class DerivedTraits>
class ConstitutiveModel {
 public:
  static_assert(
      std::is_same_v<typename DerivedTraits::ModelType,
                     DerivedConstitutiveModel>,
      "The DerivedConstitutiveModel and the DerivedTraits must be compatible.");
  using Traits = DerivedTraits;
  using T = typename Traits::Scalar;

  ~ConstitutiveModel() = default;

  /** The number of locations at which the constitutive relationship is
   evaluated. */
  static constexpr int num_locations() { return DerivedTraits::kNumLocations; }

  /** @name "Calc" Methods
   Methods for calculating the energy density and its derivatives given the
   cache entry required for these calculations. The constitutive model expects
   that the input cache entries are up-to-date. FemElement is
   responsible for updating these cache entries. ConstitutiveModel will
   not and cannot verify the cache entries provided are up-to-date.
   @{ */

  /** Calculates the energy density in reference configuration, in unit J/m³,
   given the model cache entry.
   @pre `Psi != nullptr`. */
  void CalcElasticEnergyDensity(
      const typename DerivedTraits::DeformationGradientCacheEntryType&
          cache_entry,
      std::array<T, num_locations()>* Psi) const {
    DRAKE_ASSERT(Psi != nullptr);
    static_cast<const DerivedConstitutiveModel*>(this)
        ->DoCalcElasticEnergyDensity(cache_entry, Psi);
  }

  /** Calculates the First Piola stress, in unit Pa, given the model cache
   entry.
   @pre `P != nullptr`. */
  void CalcFirstPiolaStress(
      const typename DerivedTraits::DeformationGradientCacheEntryType&
          cache_entry,
      std::array<Matrix3<T>, num_locations()>* P) const {
    DRAKE_ASSERT(P != nullptr);
    static_cast<const DerivedConstitutiveModel*>(this)->DoCalcFirstPiolaStress(
        cache_entry, P);
  }

  /** Calculates the derivative of First Piola stress with respect to the
   deformation gradient, given the model cache entry. The stress derivative
   dPᵢⱼ/dFₖₗ is a 4-th order tensor that is flattened to a 9-by-9 matrix. The
   9-by-9 matrix is organized into 3-by-3 blocks of 3-by-3 submatrices. The
   ik-th entry in the jl-th block corresponds to the value dPᵢⱼ/dFₖₗ. Let A
   denote the fourth order tensor dP/dF, then A is flattened to a 9-by-9 matrix
   in the following way:

                       l = 1       l = 2       l = 3
                   -------------------------------------
                   |           |           |           |
         j = 1     |   Aᵢ₁ₖ₁   |   Aᵢ₁ₖ₂   |   Aᵢ₁ₖ₃   |
                   |           |           |           |
                   -------------------------------------
                   |           |           |           |
         j = 2     |   Aᵢ₂ₖ₁   |   Aᵢ₂ₖ₂   |   Aᵢ₂ₖ₃   |
                   |           |           |           |
                   -------------------------------------
                   |           |           |           |
         j = 3     |   Aᵢ₃ₖ₁   |   Aᵢ₃ₖ₂   |   Aᵢ₃ₖ₃   |
                   |           |           |           |
                   -------------------------------------
  @pre `dPdF != nullptr`. */
  void CalcFirstPiolaStressDerivative(
      const typename DerivedTraits::DeformationGradientCacheEntryType&
          cache_entry,
      std::array<Eigen::Matrix<T, 9, 9>, num_locations()>* dPdF) const {
    DRAKE_ASSERT(dPdF != nullptr);
    static_cast<const DerivedConstitutiveModel*>(this)
        ->DoCalcFirstPiolaStressDerivative(cache_entry, dPdF);
  }
  /** @} */

  /** Creates a DeformationGradientCacheEntry that is compatible
   with this %ConstitutiveModel. */
  typename DerivedTraits::DeformationGradientCacheEntryType
  MakeDeformationGradientCacheEntry(ElementIndex element_index) const {
    return typename DerivedTraits::DeformationGradientCacheEntryType(
        element_index);
  }

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConstitutiveModel);

  /** The base class constructor are made protected to prevent explicit
   construction of a base class object. Concrete instances should be obtained
   through the constructors of the derived constitutive models. */
  ConstitutiveModel() = default;
};

template <class Model>
struct is_constitutive_model {
  static constexpr bool value =
      std::is_base_of<ConstitutiveModel<Model, typename Model::Traits>,
                      Model>::value;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
