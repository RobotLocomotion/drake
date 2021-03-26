#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/deformation_gradient_cache_entry.h"

namespace drake {
namespace multibody {
namespace fem {
/** A constitutive model relates the strain to the stress of the material. It
 governs the material response under deformation. This constitutive relationship
 is defined through the potential energy, which increases with non-rigid
 deformation from the initial state.
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class ConstitutiveModel {
 public:
  /** @name     Does not allow copy, move, or assignment. */
  /** @{ */
  /* Copy constructor is made "protected" to facilitate Clone() and therefore it
   is not publicly available. */
  ConstitutiveModel(ConstitutiveModel&&) = delete;
  ConstitutiveModel& operator=(ConstitutiveModel&&) = delete;
  ConstitutiveModel& operator=(const ConstitutiveModel&) = delete;
  /** @} */

  ConstitutiveModel() = default;

  /** Creates an identical copy of the concrete ConstitutiveModel object.
   */
  std::unique_ptr<ConstitutiveModel<T>> Clone() const { return DoClone(); }

  virtual ~ConstitutiveModel() {}

  /** Creates a DeformationGradientCacheEntry that is compatible with this
   %ConstitutiveModel. See ElasticityElement::ElasticityElement() for more about
   the compatibility requirement. */
  std::unique_ptr<DeformationGradientCacheEntry<T>>
  MakeDeformationGradientCacheEntry(ElementIndex element_index,
                                    int num_quadrature_points) const {
    DRAKE_DEMAND(element_index.is_valid());
    DRAKE_DEMAND(num_quadrature_points > 0);
    return DoMakeDeformationGradientCacheEntry(element_index,
                                               num_quadrature_points);
  }
  // TODO(xuchenhan-tri) Update the list of methods here as more methods are
  // introduced.
  /** @name "Calc" Methods
   "Calc" methods that calculate the energy density and stress given the cache
   entry required for these calculations. The constitutive model expects
   that the input cache entries are up-to-date. FemElement is responsible for
   updating these cache entries. ConstitutiveModel will not and cannot verify
   the cache entries provided are up-to-date.
   @warning Derived classes will static cast `cache_entry` into derived cache
   entry classes that match the derived %ConstitutiveModel. Make sure the
   `cache_entry` that is passed in matches the %ConstitutiveModel. */

  /** Calculates the energy density per unit reference volume, in unit J/m³,
   given the model cache entry. */
  std::vector<T> CalcElasticEnergyDensity(
      const DeformationGradientCacheEntry<T>& cache_entry) const {
    std::vector<T> Psi(cache_entry.num_quadrature_points());
    CalcElasticEnergyDensity(cache_entry, &Psi);
    return Psi;
  }

  /** Alternative signature for CalcElasticEnergyDensity that writes the result
   in the output argument. */
  void CalcElasticEnergyDensity(
      const DeformationGradientCacheEntry<T>& cache_entry,
      std::vector<T>* Psi) const {
    DRAKE_DEMAND(Psi != nullptr);
    DRAKE_DEMAND(static_cast<int>(Psi->size()) ==
                 cache_entry.num_quadrature_points());
    DoCalcElasticEnergyDensity(cache_entry, Psi);
  }

  /** Calculates the First Piola stress, in unit Pa, given the model cache
   entry. */
  std::vector<Matrix3<T>> CalcFirstPiolaStress(
      const DeformationGradientCacheEntry<T>& cache_entry) const {
    std::vector<Matrix3<T>> P(cache_entry.num_quadrature_points());
    CalcFirstPiolaStress(cache_entry, &P);
    return P;
  }

  /** Alternative signature for CalcFirstPiolaStress that writes the result in
   the output argument. */
  void CalcFirstPiolaStress(const DeformationGradientCacheEntry<T>& cache_entry,
                            std::vector<Matrix3<T>>* P) const {
    DRAKE_DEMAND(P != nullptr);
    DRAKE_DEMAND(static_cast<int>(P->size()) ==
                 cache_entry.num_quadrature_points());
    DoCalcFirstPiolaStress(cache_entry, P);
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
                   -------------------------------------                  */
  std::vector<Eigen::Matrix<T, 9, 9>> CalcFirstPiolaStressDerivative(
      const DeformationGradientCacheEntry<T>& cache_entry) const {
    std::vector<Eigen::Matrix<T, 9, 9>> dPdF(
        cache_entry.num_quadrature_points());
    CalcFirstPiolaStressDerivative(cache_entry, &dPdF);
    return dPdF;
  }

  /** Alternative signature for CalcFirstPiolaStressDerivative that writes the
   result in the output argument. */
  void CalcFirstPiolaStressDerivative(
      const DeformationGradientCacheEntry<T>& cache_entry,
      std::vector<Eigen::Matrix<T, 9, 9>>* dPdF) const {
    DRAKE_DEMAND(dPdF != nullptr);
    DRAKE_DEMAND(static_cast<int>(dPdF->size()) ==
                 cache_entry.num_quadrature_points());
    DoCalcFirstPiolaStressDerivative(cache_entry, dPdF);
  }

  // TODO(xuchenhan-tri): The above Calc methods use the First Piola stress /
  // Deformation Gradient stress-strain pair. For many constitutive models, the
  // Second Piola stress / Finite Strain (Green-Lagrangian strain) can also be
  // used and may be more efficient as it exploits symmetry in the stress and
  // the strain. Consider supporting it.

 protected:
  /** Copy constructor for the base ConstitutiveModel class to facilitate
   `DoClone()` in derived classes. */
  ConstitutiveModel(const ConstitutiveModel&) = default;

  /** Creates an identical copy of the concrete ConstitutiveModel object.
   Derived classes must implement this so that it performs the complete
   deep copy of the object, including all base class members. */
  virtual std::unique_ptr<ConstitutiveModel<T>> DoClone() const = 0;

  /** Derived class must create a DeformationGradientCacheEntry that is
   compatible with this ConstitutiveModel. */
  virtual std::unique_ptr<DeformationGradientCacheEntry<T>>
  DoMakeDeformationGradientCacheEntry(ElementIndex element_index,
                                      int num_quadrature_points) const = 0;

  /** Implements the NVI of CalcElasticEnergyDensity().
   Derived class must calculate the energy density, in unit J/m³, given the
   model cache entry. */
  virtual void DoCalcElasticEnergyDensity(
      const DeformationGradientCacheEntry<T>& cache_entry,
      std::vector<T>* Psi) const = 0;

  /** Implements the NVI of CalcFirstPiolaStress().
   Derived class must calculate the First Piola stress, in unit Pa, given the
   model cache entry. */
  virtual void DoCalcFirstPiolaStress(
      const DeformationGradientCacheEntry<T>& cache_entry,
      std::vector<Matrix3<T>>* P) const = 0;

  /** Implements the NVI of CalcFirstPiolaStressDerivative().
   Derived class must calculate the derivative of First Piola stress with
   respect to the deformation gradient, given the model cache entry. */
  virtual void DoCalcFirstPiolaStressDerivative(
      const DeformationGradientCacheEntry<T>& cache_entry,
      std::vector<Eigen::Matrix<T, 9, 9>>* dPdF) const = 0;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
