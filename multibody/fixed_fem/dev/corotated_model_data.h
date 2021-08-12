#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/deformation_gradient_data.h"
#include "drake/multibody/fixed_fem/dev/matrix_utilities.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Data supporting calculations in CorotatedModel.
 See CorotatedModel for how the data is used. See DeformationGradientData for
 more about constitutive model data.
 @tparam_nonsymbolic_scalar T.
 @tparam num_locations Number of locations at which the deformation gradient
 dependent quantities are evaluated. */
template <typename T, int num_locations>
class CorotatedModelData
    : public DeformationGradientData<
          CorotatedModelData<T, num_locations>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CorotatedModelData);

  /* Constructs a CorotatedModelData with no deformation. */
  CorotatedModelData() {
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
  friend DeformationGradientData<CorotatedModelData<T, num_locations>>;

  /* Shadows DeformationGradientData::UpdateFromDeformationGradient() as
   required by the CRTP base class. */
  void UpdateFromDeformationGradient() {
    const std::array<Matrix3<T>, num_locations>& F =
        this->deformation_gradient();
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

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
