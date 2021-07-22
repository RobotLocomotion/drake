#pragma once

#include <array>
#include <utility>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <class>
class DeformationGradientData;

/* DeformationGradientData stores per element data that work in tandem with
 ConstitutiveModel. It is a static interface that concrete constitutive model
 data must inherit from to store the set of specific data that facilitate
 computation in the specific model. There should be a one-to-one correspondence
 between the constitutive model `Foo` that inherits from ConstitutiveModel and
 its computation data `FooData` that inherits from DeformationGradientData.
 These computation data depend solely on deformation gradients, and they
 facilitate calculations such as energy density, stress and stress derivative in
 the constitutive model. ConstitutiveModel takes the corresponding data as an
 argument when performing various calculations. Similar to ConstitutiveModel,
 this class also utilizes CRTP to eliminate the need for virtual methods and
 facilitate inlining instead.
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

  ~DeformationGradientData() = default;

  /* Updates the data with the given deformation gradients.
   Derived class need to define a `DoUpdateData()` method with the following
   signature:
       void DoUpdateData(const std::array<Matrix3<T>, num_locations>& F);
   @param F The up-to-date deformation gradients evaluated at the quadrature
   locations for the associated element. */
  void UpdateData(std::array<Matrix3<T>, num_locations> F) {
    deformation_gradient_ = std::move(F);
    static_cast<Derived*>(this)->DoUpdateData(deformation_gradient_);
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

 private:
  std::array<Matrix3<T>, num_locations> deformation_gradient_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
