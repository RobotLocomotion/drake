#pragma once

#include <array>

#include <fmt/format.h>

#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* A constitutive model relates the strain to the stress of the material and
 governs the material response under deformation. This constitutive relationship
 is defined through a hyperelastic potential energy, which increases with
 non-rigid deformation from the initial state.

 ConstitutiveModel is a CRTP base class that provides the common interface to
 calculate energy density/stress/stress derivatives, and facilitates inlining,
 but is not intended to be a polymorphic class. Derived constitutive models must
 implement the `DoCalc()` methods. The derived constitutive model must also be
 accompanied by a corresponding traits class that declares the compile time
 quantities and type declarations that this base class requires.
 @tparam DerivedConstitutiveModel The concrete constitutive model that inherits
 from ConstitutiveModel through CRTP.
 @tparam DerivedTraits The traits class associated with the
 DerivedConstitutiveModel. It muist provide the type definitions `Scalar`
 (the scalar type the constitutive model is templated on) and `Data` (the
 DeformationGradientData that works in tandem with the derived constitutive
 model). */
template <class DerivedConstitutiveModel, class DerivedTraits>
class ConstitutiveModel {
 public:
  using Traits = DerivedTraits;
  using T = typename Traits::Scalar;

  /* The number of locations at which the constitutive relationship is
   evaluated. */
  static constexpr int num_locations = DerivedTraits::Data::num_locations;

  /* "Calc" Methods
   Methods for calculating the energy density and its derivatives given the
   data required for these calculations. The constitutive model expects
   that the input data are up-to-date, but cannot and will not verify this
   prerequisite. It is the responsibility of the caller to provide
   up-to-date input data. */

  /* Calculates the energy density in reference configuration, in unit J/m³,
   given the model data.
   @pre `Psi != nullptr`. */
  void CalcElasticEnergyDensity(const typename DerivedTraits::Data& data,
                                std::array<T, num_locations>* Psi) const {
    DRAKE_ASSERT(Psi != nullptr);
    derived().DoCalcElasticEnergyDensity(data, Psi);
  }

  /* Calculates the First Piola stress, in unit Pa, given the deformation
   gradient data.
   @pre `P != nullptr`. */
  void CalcFirstPiolaStress(const typename DerivedTraits::Data& data,
                            std::array<Matrix3<T>, num_locations>* P) const {
    DRAKE_ASSERT(P != nullptr);
    derived().DoCalcFirstPiolaStress(data, P);
  }

  /* Calculates the derivative of First Piola stress with respect to the
   deformation gradient, given the model data. The stress derivative
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
      const typename DerivedTraits::Data& data,
      std::array<Eigen::Matrix<T, 9, 9>, num_locations>* dPdF) const {
    DRAKE_ASSERT(dPdF != nullptr);
    derived().DoCalcFirstPiolaStressDerivative(data, dPdF);
  }

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConstitutiveModel);

  /* The base class constructor are made protected to prevent explicit
   construction of a base class object. Concrete instances should be obtained
   through the constructors of the derived constitutive models. */
  ConstitutiveModel() = default;

  /* Derived classes *must* shadow these methods to compute energy
   density/stress/stress derivatives from deformation gradient data. The output
   argument is guaranteed to be non-null. */
  void DoCalcElasticEnergyDensity(const typename DerivedTraits::Data& data,
                                  std::array<T, num_locations>* Psi) const {
    throw std::logic_error(
        fmt::format("The derived class {} must provide a shadow definition of "
                    "DoCalcElasticEnergyDensity() to be correct.",
                    NiceTypeName::Get(derived())));
  }

  void DoCalcFirstPiolaStress(const typename DerivedTraits::Data& data,
                              std::array<Matrix3<T>, num_locations>* P) const {
    throw std::logic_error(
        fmt::format("The derived class {} must provide a shadow definition of "
                    "DoCalcFirstPiolaStress() to be correct.",
                    NiceTypeName::Get(derived())));
  }

  void DoCalcFirstPiolaStressDerivative(
      const typename DerivedTraits::Data& data,
      std::array<Eigen::Matrix<T, 9, 9>, num_locations>* dPdF) const {
    throw std::logic_error(
        fmt::format("The derived class {} must provide a shadow definition of "
                    "DoCalcFirstPiolaStressDerivative() to be correct.",
                    NiceTypeName::Get(derived())));
  }

 private:
  const DerivedConstitutiveModel& derived() const {
    return *static_cast<const DerivedConstitutiveModel*>(this);
  }
};

template <class Model>
struct is_constitutive_model {
  static constexpr bool value =
      std::is_base_of_v<ConstitutiveModel<Model, typename Model::Traits>,
                        Model>;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
