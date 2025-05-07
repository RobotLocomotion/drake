#pragma once

#include <array>

#include <fmt/format.h>

#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"
#include "drake/math/fourth_order_tensor.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* A constitutive model relates the strain to the stress of a material and
 governs the material response under deformation. This constitutive relationship
 is defined through a hyperelastic potential energy, which increases with
 non-rigid deformation from the initial state.

 ConstitutiveModel is a CRTP base class that facilitates inlining and provides
 the common interface to calculate energy density, stress, and stress
 derivatives, but is not intended to be a polymorphic class. Derived
 constitutive models must shadow the `CalcFooImpl()` methods. The derived
 constitutive model must also be accompanied by a corresponding traits class
 that declares the compile time quantities and type declarations that this base
 class requires.
 @tparam DerivedConstitutiveModel The concrete constitutive model that inherits
 from ConstitutiveModel through CRTP.
 @tparam DerivedTraits The traits class associated with the
 DerivedConstitutiveModel. It must provide the type definitions `Scalar`
 (the scalar type of the constitutive model) and `Data` (the derived
 DeformationGradientData that works in tandem with the derived constitutive
 model). */
template <class DerivedConstitutiveModel, class DerivedTraits>
class ConstitutiveModel {
 public:
  using Traits = DerivedTraits;
  using T = typename Traits::Scalar;
  using Data = typename Traits::Data;

  /* Is the constitutive model linear. */
  static constexpr bool is_linear = Traits::is_linear;

  /* "Calc" Methods
   Methods for calculating the energy density and its derivatives given the
   deformation gradient and other deformation gradient dependent data required
   for these calculations.
   Note that these Calc methods take output parameters
   instead of returning the output value with the following considerations in
   mind:
   1. A typical usage of the return value is not used for initialization (See
      ElasticityElement::DoComputeData()) and thus RVO can't be used.
   2. The output value is of type `std::array` for which there is no helpful
      move semantic.

   Calculates the energy density in reference configuration in unit of J/m³,
   given the deformation gradient related quantities contained in `data`.
   @pre `Psi != nullptr`. */
  void CalcElasticEnergyDensity(const Data& data, T* Psi) const {
    DRAKE_ASSERT(Psi != nullptr);
    derived().CalcElasticEnergyDensityImpl(data, Psi);
  }

  /* Calculates the First Piola stress in unit of Pa, given the deformation
   gradient related quantities contained in `data`.
   @pre `P != nullptr`. */
  void CalcFirstPiolaStress(const Data& data, Matrix3<T>* P) const {
    DRAKE_ASSERT(P != nullptr);
    derived().CalcFirstPiolaStressImpl(data, P);
  }

  /* Calculates the derivative of first Piola stress with respect to the
   deformation gradient, given the deformation gradient related quantities
   contained in `data`. The stress derivative dPᵢⱼ/dFₖₗ is a 4-th order tensor
   that is flattened to a 9-by-9 matrix. The 9-by-9 matrix is organized into
   3-by-3 blocks of 3-by-3 submatrices. The ik-th entry in the jl-th block
   corresponds to the value dPᵢⱼ/dFₖₗ. Let A denote the fourth order tensor
   dP/dF, then A is flattened to a 9-by-9 matrix in the following way:

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
      const Data& data, math::internal::FourthOrderTensor<T>* dPdF) const {
    DRAKE_ASSERT(dPdF != nullptr);
    derived().CalcFirstPiolaStressDerivativeImpl(data, dPdF);
  }

  /* Calculates the filtered Hessian of the first Piola stress with respect to
   the deformation gradient, given the deformation gradient related quantities
   contained in `data`. The filtered Hessian is the result of
   `CalcFirstPiolaStressDerivative` with negative eigenvalues replaced with tiny
   positive ones.*/
  void CalcFilteredHessian(
      const Data& data, math::internal::FourthOrderTensor<T>* hessian) const {
    DRAKE_ASSERT(hessian != nullptr);
    derived().CalcFilteredHessianImpl(data, hessian);
  }

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConstitutiveModel);

  /* The base class constructor are made protected to prevent explicit
   construction of a base class object. Concrete instances should be obtained
   through the constructors of the derived constitutive models. */
  ConstitutiveModel() = default;

  /* Derived classes *must* shadow these methods to compute energy
   density, stress, and stress derivatives from the given `data`. The output
   argument is guaranteed to be non-null. */
  void CalcElasticEnergyDensityImpl(const Data& data, T* Psi) const {
    throw std::logic_error(
        fmt::format("The derived class {} must provide a shadow definition of "
                    "CalcElasticEnergyDensityImpl() to be correct.",
                    NiceTypeName::Get(derived())));
  }

  void CalcFirstPiolaStressImpl(const Data& data, Matrix3<T>* P) const {
    throw std::logic_error(
        fmt::format("The derived class {} must provide a shadow definition of "
                    "CalcFirstPiolaStressImpl() to be correct.",
                    NiceTypeName::Get(derived())));
  }

  void CalcFirstPiolaStressDerivativeImpl(
      const Data& data, math::internal::FourthOrderTensor<T>* dPdF) const {
    throw std::logic_error(
        fmt::format("The derived class {} must provide a shadow definition of "
                    "CalcFirstPiolaStressDerivativeImpl() to be correct.",
                    NiceTypeName::Get(derived())));
  }

  /* Derived classes *may* shadow this method to provide a more efficient
   implementation of the filtered Hessian. The default implementation is
   CalcFirstPiolaStressDerivative() followed by an eigenvalue decomposition of
   the 9-by-9 matrix. The negative eigenvalues are replaced with tiny positive
   ones and the filtered Hessian is reconstructed from the eigenvalues and
   eigenvectors. The output argument is guaranteed to be non-null. */
  void CalcFilteredHessianImpl(
      const Data& data, math::internal::FourthOrderTensor<T>* hessian) const {
    CalcFirstPiolaStressDerivative(data, hessian);
    const T kTol = 1e-14;
    using Matrix9 = Eigen::Matrix<T, 9, 9>;
    using Vector9 = Eigen::Matrix<T, 9, 1>;
    const Matrix9& matrix = hessian->data();
    Eigen::SelfAdjointEigenSolver<Matrix9> eigensolver(matrix);
    DRAKE_THROW_UNLESS(eigensolver.info() == Eigen::Success);
    Vector9 eigenvalues = eigensolver.eigenvalues();
    for (int i = 0; i < 9; ++i) {
      if (eigenvalues(i) < kTol) eigenvalues(i) = kTol;
    }
    const Matrix9& eigenvectors = eigensolver.eigenvectors();
    Matrix9& filtered_hessian = hessian->mutable_data();
    filtered_hessian =
        eigenvectors * eigenvalues.asDiagonal() * eigenvectors.transpose();
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
