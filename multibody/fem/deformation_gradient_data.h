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

 It stores the deformation gradients calculated at the prescribed "locations"
 (the number of which could depend on varying criteria, e.g., quadrature points,
 etc.). A derived class will further store derived quantities which solely
 depend on the stored deformation gradients (e.g., strain) that facilitate
 calculations of energy density, stress, and stress derivatives.

 As part of a derivation, the child `FooData` class must implement the method:

     void UpdateFromDeformationGradient();

 Note that this is not a virtual method but failure to implement the method
 correctly will lead to an exception about "undefined" methods when
 `UpdateData()` is invoked.
 @tparam_nonsymbolic_scalar T.
 @tparam num_locations_at_compile_time Number of locations at which the data are
 evaluated. */
template <template <typename, int> class DerivedDeformationGradientData,
          typename T, int num_locations_at_compile_time>
class DeformationGradientData<
    DerivedDeformationGradientData<T, num_locations_at_compile_time>> {
 public:
  using Derived =
      DerivedDeformationGradientData<T, num_locations_at_compile_time>;

  /* The number of locations at which the data needs to be evaluated. */
  static constexpr int num_locations = num_locations_at_compile_time;

  /* Updates the data with the given deformation gradients. The deformation
   gradient dependent quantities are also updated with the given
   `deformation_gradient`.
   @param deformation_gradient The up-to-date deformation gradients evaluated at
   the prescribed locations. */
  void UpdateData(std::array<Matrix3<T>, num_locations> deformation_gradient) {
    deformation_gradient_ = std::move(deformation_gradient);
    static_cast<Derived*>(this)->UpdateFromDeformationGradient();
  }

  const std::array<Matrix3<T>, num_locations>& deformation_gradient() const {
    return deformation_gradient_;
  }

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeformationGradientData);

  /* Constructs a DeformationGradientData with identity deformation gradients.
   */
  DeformationGradientData() {
    deformation_gradient_.fill(Matrix3<T>::Identity());
  }

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
  std::array<Matrix3<T>, num_locations> deformation_gradient_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
