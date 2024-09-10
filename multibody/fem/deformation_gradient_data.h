#pragma once

#include <array>
#include <utility>

#include <fmt/format.h>

#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <class>
class DeformationGradientData;
/* DeformationGradientData stores data that works in tandem with
 ConstitutiveModel. It is a CRTP base class that provides some common
 functionality and interface, and facilitates inlining, but is not intended to
 be a polymorphic class. For every concrete implementation of a constitutive
 model (`Foo`) there should be a corresponding `FooData` derived from
 DeformationGradientData.

 It stores the deformation gradients calculated at the prescribed location. A
 derived class will further store derived quantities which solely depend on the
 stored deformation gradients (e.g., strain) that facilitate calculations of
 energy density, stress, and stress derivatives.

 As part of a derivation, the child `FooData` class must implement the method:

     void UpdateFromDeformationGradient();

 Note that this is not a virtual method but failure to implement the method
 correctly will lead to an exception about "undefined" methods when
 `UpdateData()` is invoked.
 @tparam_nonsymbolic_scalar T. */
template <template <typename> class DerivedDeformationGradientData, typename T>
class DeformationGradientData<DerivedDeformationGradientData<T>> {
 public:
  using Derived = DerivedDeformationGradientData<T>;

  /* Updates the data with the given deformation gradients. The deformation
   gradient dependent quantities are also updated with the given
   deformation gradients evaluated at the current and previous time steps. */
  void UpdateData(const Matrix3<T>& deformation_gradient,
                  const Matrix3<T>& previous_step_deformation_gradient) {
    deformation_gradient_ = deformation_gradient;
    previous_step_deformation_gradient_ = previous_step_deformation_gradient;
    static_cast<Derived*>(this)->UpdateFromDeformationGradient();
  }

  const Matrix3<T>& deformation_gradient() const {
    return deformation_gradient_;
  }

  /* Returns the deformation gradient evaluated at all quadrature points at the
   previous time step t₀. */
  const Matrix3<T>& previous_step_deformation_gradient() const {
    return previous_step_deformation_gradient_;
  }

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeformationGradientData);

  /* Constructs a DeformationGradientData with identity deformation gradients.
   */
  DeformationGradientData()
      : deformation_gradient_(Matrix3<T>::Identity()),
        previous_step_deformation_gradient_(Matrix3<T>::Identity()) {}

  /* Derived classes *must* shadow this method to compute quantities derived
   from deformation gradients. `deformation_gradient()` will be up to date
   before any call to this method. */
  void UpdateFromDeformationGradient() {
    throw std::logic_error(
        fmt::format("The derived class {} must provide a shadow definition of "
                    "UpdateFromDeformationGradient() to be correct.",
                    NiceTypeName::Get(*static_cast<Derived*>(this))));
  }

 private:
  Matrix3<T> deformation_gradient_;
  Matrix3<T> previous_step_deformation_gradient_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
