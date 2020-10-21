#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/constitutive_model_cache.h"

namespace drake {
namespace multibody {
namespace fem {
/** @anchor constitutive_model
 A constitutive model relates the strain to the stress of the material. It
 governs the material response under deformation. This constitutive relationship
 is defined through the potential energy, which increases with non-rigid
 deformation from the initial state.
 @tparam_nonsymbolic_scalar T.
 @tparam SpatialDim The spatial dimension of the domain. */
template <typename T, int SpatialDim>
class ConstitutiveModel {
 public:
  using MatrixD = Eigen::Matrix<T, SpatialDim, SpatialDim>;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConstitutiveModel);

  ConstitutiveModel() = default;

  virtual ~ConstitutiveModel() {}

  /** Calculates the energy density with the given model cache.
   @warning Derived class will static cast `cache` into derived cache class that
   matches the derived ConstitutiveModel. Make sure the `cache` that is passed
   in matches the ConstitutiveModel. */
  virtual std::vector<T> CalcPsi(
      const ConstitutiveModelCache<T, SpatialDim>& cache) const = 0;

  /** Calculates the First Piola stress with the given model cache.
   @warning Derived class will static cast `cache` into derived cache class
   that matches the derived ConstitutiveModel. Make sure the `cache` that is
   passed in matches the ConstitutiveModel. */
  virtual std::vector<MatrixD> CalcP(
      const ConstitutiveModelCache<T, SpatialDim>& cache) const = 0;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
