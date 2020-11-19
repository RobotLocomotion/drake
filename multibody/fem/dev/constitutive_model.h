#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/deformation_gradient_cache.h"

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

  // TODO(xuchenhan-tri) Update the list of methods here as more methods are
  // introduced.
  // TODO(xuchenhan-tri) Clarify that FemElement is responsible for updating
  // the cached quantities once FemElement is introduced.
  /** @name "Calc" Methods
   "Calc" methods that calculate the energy density and stress given the cached
   quantities required for these calculations. The constitutive model expects
   that the input cached quantities are up-to-date.
   @warning Derived classes will static cast `cache` into derived cache classes
   that match the derived %ConstitutiveModel. Make sure the `cache` that is
   passed in matches the %ConstitutiveModel. */

  /** Calculates the energy density per unit reference volume, in unit J/m³,
   given the model cache. */
  std::vector<T> CalcElasticEnergyDensity(
      const DeformationGradientCache<T>& cache) const {
    std::vector<T> Psi(cache.num_quadrature_points());
    CalcElasticEnergyDensity(cache, &Psi);
    return Psi;
  }

  /** Alternative signature for CalcElasticEnergyDensity that writes the result
   in the output argument. */
  void CalcElasticEnergyDensity(const DeformationGradientCache<T>& cache,
                                std::vector<T>* Psi) const {
    DRAKE_DEMAND(Psi != nullptr);
    DRAKE_DEMAND(static_cast<int>(Psi->size()) ==
                 cache.num_quadrature_points());
    DoCalcElasticEnergyDensity(cache, Psi);
  }

  /** Calculates the First Piola stress, in unit Pa, given the model cache. */
  std::vector<Matrix3<T>> CalcFirstPiolaStress(
      const DeformationGradientCache<T>& cache) const {
    std::vector<Matrix3<T>> P(cache.num_quadrature_points());
    CalcFirstPiolaStress(cache, &P);
    return P;
  }

  /** Alternative signature for CalcFirstPiolaStress that writes the result in
   the output argument. */
  void CalcFirstPiolaStress(const DeformationGradientCache<T>& cache,
                            std::vector<Matrix3<T>>* P) const {
    DRAKE_DEMAND(P != nullptr);
    DRAKE_DEMAND(static_cast<int>(P->size()) == cache.num_quadrature_points());
    DoCalcFirstPiolaStress(cache, P);
  }

  /** Creates a DeformationGradientCache that is compatible with this
   %ConstitutiveModel. See ElasticityElement for more about the compatibility
   requirement. */
  std::unique_ptr<DeformationGradientCache<T>> MakeDeformationGradientCache(
      ElementIndex element_index, int num_quadrature_points) const {
    DRAKE_DEMAND(element_index.is_valid());
    DRAKE_DEMAND(num_quadrature_points > 0);
    return DoMakeDeformationGradientCache(element_index, num_quadrature_points);
  }

 protected:
  /** Copy constructor for the base ConstitutiveModel class to facilitate
   `DoClone()` in derived classes. */
  ConstitutiveModel(const ConstitutiveModel&) = default;

  /** Creates an identical copy of the concrete ConstitutiveModel object.
   Derived classes must implement this so that it performs the complete
   deep copy of the object, including all base class members. */
  virtual std::unique_ptr<ConstitutiveModel<T>> DoClone() const = 0;

  /** Derived class must calculate the energy density, in unit J/m³, given the
   model cache. */
  virtual void DoCalcElasticEnergyDensity(
      const DeformationGradientCache<T>& cache, std::vector<T>* Psi) const = 0;

  /** Derived class must calculate the First Piola stress, in unit Pa, given the
   model cache. */
  virtual void DoCalcFirstPiolaStress(const DeformationGradientCache<T>& cache,
                                      std::vector<Matrix3<T>>* P) const = 0;

  /** Derived class must create a DeformationGradientCache that is compatible
   with this ConstitutiveModel. */
  virtual std::unique_ptr<DeformationGradientCache<T>>
  DoMakeDeformationGradientCache(ElementIndex element_index,
                                 int num_quadrature_points) const = 0;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
