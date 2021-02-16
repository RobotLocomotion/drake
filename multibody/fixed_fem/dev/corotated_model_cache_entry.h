#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/deformation_gradient_cache_entry.h"
#include "drake/multibody/fixed_fem/dev/fem_indexes.h"
#include "drake/multibody/fixed_fem/dev/matrix_utilities.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** Cache entry for the CorotatedModel constitutive model. See CorotatedModel
 for how the cache entry is used. See DeformationGradientCacheEntry for more
 about cached quantities for constitutive models.
 @tparam_nonsymbolic_scalar T.
 @tparam num_locations Number of locations at which the cached quantities are
 evaluated. */
template <typename T, int num_locations>
class CorotatedModelCacheEntry
    : public DeformationGradientCacheEntry<
          CorotatedModelCacheEntry<T, num_locations>> {
 public:
  using Base =
      DeformationGradientCacheEntry<CorotatedModelCacheEntry<T, num_locations>>;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CorotatedModelCacheEntry);

  ~CorotatedModelCacheEntry() = default;

  /** Constructs a %CorotatedModelCacheEntry with no deformation. */
  CorotatedModelCacheEntry() {
    std::fill(R_.begin(), R_.end(), Matrix3<T>::Identity());
    std::fill(S_.begin(), S_.end(), Matrix3<T>::Identity());
    std::fill(Jm1_.begin(), Jm1_.end(), 0);
    std::fill(JFinvT_.begin(), JFinvT_.end(), Matrix3<T>::Identity());
  }

  const std::array<Matrix3<T>, num_locations>& R() const { return R_; }

  const std::array<Matrix3<T>, num_locations>& S() const { return S_; }

  const std::array<T, num_locations>& Jm1() const { return Jm1_; }

  const std::array<Matrix3<T>, num_locations>& JFinvT() const {
    return JFinvT_;
  }

 private:
  friend Base;

  /* Implements the interface DeformationGradientCacheEntry::UpdateCacheEntry().
   @param F The up-to-date deformation gradients evaluated at the quadrature
   locations for the associated element. */
  void DoUpdateCacheEntry(const std::array<Matrix3<T>, num_locations>& F) {
    for (int i = 0; i < num_locations; ++i) {
      Matrix3<T>& local_R = R_[i];
      Matrix3<T>& local_S = S_[i];
      Matrix3<T>& local_JFinvT = JFinvT_[i];
      internal::PolarDecompose<T>(F[i], &local_R, &local_S);
      Jm1_[i] = F[i].determinant() - 1.0;
      internal::CalcCofactorMatrix<T>(F[i], &local_JFinvT);
    }
  }

  /* Let F = RS be the polar decomposition of the deformation gradient where R
   is a rotation matrix and S is symmetric. */
  std::array<Matrix3<T>, num_locations> R_;
  std::array<Matrix3<T>, num_locations> S_;
  /* The determinant of F minus 1, or J - 1. */
  std::array<T, num_locations> Jm1_;
  /* The cofactor matrix of F, or JF⁻ᵀ. */
  std::array<Matrix3<T>, num_locations> JFinvT_;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
