#include "drake/multibody/mpm/particle_data.h"

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/fem/deformable_body_config.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;

template <typename T>
class ParticleDataTest : public ::testing::Test {
 protected:
  ParticleData<T> dut_;
};

using ScalarTypes = ::testing::Types<float, double, AutoDiffXd>;
TYPED_TEST_SUITE(ParticleDataTest, ScalarTypes);

TYPED_TEST(ParticleDataTest, Constructor) {
  using T = TypeParam;
  EXPECT_EQ(this->dut_.num_particles(), 0);
  EXPECT_TRUE(this->dut_.m().empty());
  EXPECT_TRUE(this->dut_.x().empty());
  EXPECT_TRUE(this->dut_.v().empty());
  EXPECT_TRUE(this->dut_.F().empty());
  EXPECT_TRUE(this->dut_.F0().empty());
  EXPECT_TRUE(this->dut_.C().empty());
  EXPECT_TRUE(this->dut_.volume().empty());
  EXPECT_TRUE(this->dut_.constitutive_models().empty());
  EXPECT_TRUE(this->dut_.in_constraint().empty());
  EXPECT_TRUE(this->dut_.tau_volume().empty());
  EXPECT_TRUE((std::is_same_v<typename ParticleData<T>::Scalar, T>));
}

TYPED_TEST(ParticleDataTest, AddParticles) {
  using T = TypeParam;
  const Vector3d x0(1.0, 2.0, 3.0);
  const Vector3d x1(0.0, 2.1, 3.2);
  const std::vector<Vector3d> positions = {x0, x1};
  const double total_volume = 1.0;

  fem::DeformableBodyConfig<double> config;
  config.set_mass_density(2.0);

  this->dut_.AddParticles(positions, total_volume, config);

  EXPECT_EQ(this->dut_.num_particles(), 2);
  EXPECT_EQ(this->dut_.m().size(), 2);
  EXPECT_EQ(this->dut_.x().size(), 2);
  EXPECT_EQ(this->dut_.volume().size(), 2);
  EXPECT_EQ(this->dut_.v().size(), 2);
  EXPECT_EQ(this->dut_.C().size(), 2);
  EXPECT_EQ(this->dut_.F().size(), 2);
  EXPECT_EQ(this->dut_.F0().size(), 2);
  EXPECT_EQ(this->dut_.tau_volume().size(), 2);
  EXPECT_EQ(this->dut_.in_constraint().size(), 2);
  EXPECT_EQ(this->dut_.constitutive_models().size(), 2);

  for (int i = 0; i < 2; ++i) {
    EXPECT_EQ(this->dut_.m()[i], 1.0);
    EXPECT_EQ(this->dut_.x()[i], positions[i].cast<T>());
    EXPECT_EQ(this->dut_.volume()[i], 0.5);
    EXPECT_EQ(this->dut_.v()[i], Vector3<T>::Zero());
    EXPECT_EQ(this->dut_.C()[i], Matrix3<T>::Zero());
    EXPECT_EQ(this->dut_.F()[i], Matrix3<T>::Identity());
    EXPECT_EQ(this->dut_.F0()[i], Matrix3<T>::Identity());
    EXPECT_EQ(this->dut_.tau_volume()[i], Matrix3<T>::Zero());
    EXPECT_EQ(this->dut_.in_constraint()[i], false);
    EXPECT_TRUE(std::holds_alternative<fem::internal::LinearCorotatedModel<T>>(
        this->dut_.constitutive_models()[i]));
  }

  /* Test mutable accessors. */
  this->dut_.mutable_x()[0] = Vector3<T>::Ones();
  this->dut_.mutable_v()[0] = 2.0 * Vector3<T>::Ones();
  this->dut_.mutable_F()[0] = 3.0 * Matrix3<T>::Ones();
  this->dut_.mutable_F0()[0] = 4.0 * Matrix3<T>::Ones();
  this->dut_.mutable_C()[0] = 5.0 * Matrix3<T>::Ones();
  this->dut_.mutable_tau_volume()[0] = 6.0 * Matrix3<T>::Ones();
  this->dut_.mutable_in_constraint()[0] = true;
  EXPECT_EQ(this->dut_.x()[0], Vector3<T>::Ones());
  EXPECT_EQ(this->dut_.v()[0], 2.0 * Vector3<T>::Ones());
  EXPECT_EQ(this->dut_.F()[0], 3.0 * Matrix3<T>::Ones());
  EXPECT_EQ(this->dut_.F0()[0], 4.0 * Matrix3<T>::Ones());
  EXPECT_EQ(this->dut_.C()[0], 5.0 * Matrix3<T>::Ones());
  EXPECT_EQ(this->dut_.tau_volume()[0], 6.0 * Matrix3<T>::Ones());
  EXPECT_EQ(this->dut_.in_constraint()[0], true);
}

TYPED_TEST(ParticleDataTest, ComputeTotalEnergy) {
  using T = TypeParam;
  const std::vector<Matrix3<T>> F = {Matrix3<T>::Identity(),
                                     Matrix3<T>::Identity()};
  const std::vector<Matrix3<T>> F0 = {Matrix3<T>::Identity(),
                                      Matrix3<T>::Identity()};

  const Vector3d x0(1.0, 2.0, 3.0);
  const Vector3d x1(0.0, 2.1, 3.2);
  const std::vector<Vector3d> positions = {x0, x1};
  const double total_volume = 1.0;

  fem::DeformableBodyConfig<double> config;
  config.set_mass_density(1.2);
  this->dut_.AddParticles(positions, total_volume, config);

  const T zero_energy = this->dut_.ComputeTotalEnergy(F, F0);
  EXPECT_EQ(zero_energy, 0);

  const std::vector<Matrix3<T>> nontrivial_F = {2.0 * Matrix3<T>::Identity(),
                                                0.5 * Matrix3<T>::Identity()};
  const T nonzero_energy = this->dut_.ComputeTotalEnergy(nontrivial_F, F0);
  EXPECT_GT(nonzero_energy, 0);
}

TYPED_TEST(ParticleDataTest, ComputeKirchhoffStress) {
  using T = TypeParam;
  const std::vector<Matrix3<T>> F = {Matrix3<T>::Identity(),
                                     Matrix3<T>::Identity()};
  const std::vector<Matrix3<T>> F0 = {Matrix3<T>::Identity(),
                                      Matrix3<T>::Identity()};
  const Vector3d x0(1.0, 2.0, 3.0);
  const Vector3d x1(0.0, 2.1, 3.2);
  const std::vector<Vector3d> positions = {x0, x1};
  const double total_volume = 1.0;
  const fem::DeformableBodyConfig<double> config;
  this->dut_.AddParticles(positions, total_volume, config);

  std::vector<Matrix3<T>> tau_volume(2);
  this->dut_.ComputeKirchhoffStress(F, F0, &tau_volume);

  for (const auto& tau : tau_volume) {
    EXPECT_TRUE(tau.isZero());
  }

  const std::vector<Matrix3<T>> nontrivial_F = {2.0 * Matrix3<T>::Identity(),
                                                0.5 * Matrix3<T>::Identity()};
  this->dut_.ComputeKirchhoffStress(nontrivial_F, F0, &tau_volume);
  for (const auto& tau : tau_volume) {
    EXPECT_GT(tau.norm(), 1.0);
  }
}

TYPED_TEST(ParticleDataTest, ComputePK1StressDerivatives) {
  using T = TypeParam;
  const std::vector<Matrix3<T>> F = {Matrix3<T>::Identity(),
                                     Matrix3<T>::Identity()};
  const std::vector<Matrix3<T>> F0 = {Matrix3<T>::Identity(),
                                      Matrix3<T>::Identity()};
  const Vector3d x0(1.0, 2.0, 3.0);
  const Vector3d x1(0.0, 2.1, 3.2);
  const std::vector<Vector3d> positions = {x0, x1};
  const double total_volume = 1.0;

  fem::DeformableBodyConfig<double> config;
  config.set_mass_density(1.2);
  this->dut_.AddParticles(positions, total_volume, config);

  std::vector<math::internal::FourthOrderTensor<T>> dPdF_volume(2);
  this->dut_.ComputePK1StressDerivatives(F, F0, &dPdF_volume);
  /* Comfirm that the stress derivative is symmetric positive-definite. Here, we
   perform double contraction T : dPdF_volume : T where T is a test matrix and
   confirm the result is positive. */
  Eigen::Vector<T, 9> test_matrix;
  for (int i = 0; i < 9; ++i) {
    test_matrix.setZero();
    test_matrix(i) = 1.0;
    for (int j = 0; j < 2; ++j) {
      const auto& dPdF_matrix = dPdF_volume[j].data();
      EXPECT_GT(test_matrix.dot(dPdF_matrix * test_matrix), 0.0);
    }
  }
}

/* Creates an array of arbitrary autodiff deformation gradients. */
Matrix3<AutoDiffXd> MakeDeformationGradientsWithDerivatives() {
  /* Create an arbitrary AutoDiffXd deformation. */
  Matrix3d F;
  // clang-format off
  F << 0.18, 0.63, 0.54,
       0.13, 0.92, 0.17,
       0.03, 0.86, 0.85;
  // clang-format on
  const Matrix3d deformation_gradients{F};
  Matrix3<AutoDiffXd> deformation_gradients_autodiff;
  const Eigen::Matrix<double, 9, Eigen::Dynamic> derivatives(
      Eigen::Matrix<double, 9, 9>::Identity());
  const auto F_autodiff_flat =
      math::InitializeAutoDiff(Eigen::Map<const Eigen::Matrix<double, 9, 1>>(
                                   deformation_gradients.data(), 9),
                               derivatives);
  deformation_gradients_autodiff =
      Eigen::Map<const Matrix3<AutoDiffXd>>(F_autodiff_flat.data(), 3, 3);
  return deformation_gradients_autodiff;
}

template <typename T>
Matrix3<T> MakeArbitraryMatrix3() {
  Matrix3<T> matrix;
  // clang-format off
  matrix << 0.91, 0.13, 0.54,
            0.11, 1.12, 0.17,
            0.01, 0.06, 0.89;
  // clang-format on
  return matrix;
}

/* Tests that ComputeKirchhoffStress and ComputeTotalEnergy are consistent with
 each other. The Kirchhoff stress should be P * Fᵀ where P is the first
 Piola-Kirchhoff stress (PK1), and PK1 should be the dertivative of the energy
 with respect to the deformation gradient.  */
GTEST_TEST(ParticleDataDerivativeTest, FirstDerivative) {
  const std::vector<Matrix3<AutoDiffXd>> F_ad = {
      MakeDeformationGradientsWithDerivatives()};
  const std::vector<Matrix3<AutoDiffXd>> F0_ad = {
      MakeArbitraryMatrix3<AutoDiffXd>()};
  const Vector3d x0(1.0, 2.0, 3.0);
  const std::vector<Vector3d> positions = {x0};
  const double total_volume = 1.3;
  fem::DeformableBodyConfig<double> config;
  config.set_mass_density(1.2);

  ParticleData<AutoDiffXd> particle_data_ad;
  particle_data_ad.AddParticles(positions, total_volume, config);
  const AutoDiffXd energy = particle_data_ad.ComputeTotalEnergy(F_ad, F0_ad);

  const std::vector<Matrix3d> F_double = {math::DiscardGradient(F_ad[0])};
  const std::vector<Matrix3d> F0_double = {MakeArbitraryMatrix3<double>()};
  ParticleData<double> particle_data_double;
  particle_data_double.AddParticles(positions, total_volume, config);
  /* We scale by an arbitrary factor of 1.23 so that particle_F is not the same
   as F0 to make the test more general. */
  const std::vector<Matrix3d> particle_F = {1.23 *
                                            MakeArbitraryMatrix3<double>()};
  particle_data_double.mutable_F() = particle_F;

  std::vector<Matrix3d> tau_volume_double(1);
  particle_data_double.ComputeKirchhoffStress(F_double, F0_double,
                                              &tau_volume_double);
  const Matrix3d tau_volume = tau_volume_double[0];
  const Matrix3d P =
      Eigen::Map<const Matrix3d>(energy.derivatives().data(), 3, 3);
  const Matrix3d tau_volume_expected = P * particle_F[0].transpose();
  EXPECT_TRUE(CompareMatrices(tau_volume, tau_volume_expected, 1e-14,
                              MatrixCompareType::relative));
}

/* Tests that ComputeKirchhoffStress and ComputePK1StressDerivatives are
 consistent with each other. The Kirchhoff stress should be the same as the
 Piola-Kirchhoff stress (PK1) when the particle F is identity. So
 ComputePK1StressDerivatives should produce the same result as derivative of
 ComputeKirchhoffStress from autodiff. */
GTEST_TEST(ParticleDataDerivativeTest, SecondDerivative) {
  const std::vector<Matrix3<AutoDiffXd>> F_ad = {
      MakeDeformationGradientsWithDerivatives()};
  const std::vector<Matrix3<AutoDiffXd>> F0_ad = {
      MakeArbitraryMatrix3<AutoDiffXd>()};
  const Vector3d x0(1.0, 2.0, 3.0);
  const std::vector<Vector3d> positions = {x0};
  const double total_volume = 1.3;
  fem::DeformableBodyConfig<double> config;
  config.set_mass_density(1.2);

  ParticleData<AutoDiffXd> particle_data_ad;
  particle_data_ad.AddParticles(positions, total_volume, config);
  particle_data_ad.mutable_F() = {Matrix3<AutoDiffXd>::Identity()};
  std::vector<Matrix3<AutoDiffXd>> tau_volume_ad(1);
  /* Now tau_volume_ad should store volume * P. */
  particle_data_ad.ComputeKirchhoffStress(F_ad, F0_ad, &tau_volume_ad);

  const std::vector<Matrix3d> F_double = {math::DiscardGradient(F_ad[0])};
  const std::vector<Matrix3d> F0_double = {MakeArbitraryMatrix3<double>()};
  ParticleData<double> particle_data_double;
  particle_data_double.AddParticles(positions, total_volume, config);
  std::vector<math::internal::FourthOrderTensor<double>> dPdF_volume_double(1);
  particle_data_double.ComputePK1StressDerivatives(F_double, F0_double,
                                                   &dPdF_volume_double);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Matrix3d dPijdF;
      for (int k = 0; k < 3; ++k) {
        for (int l = 0; l < 3; ++l) {
          dPijdF(k, l) = dPdF_volume_double[0](i, j, k, l);
        }
      }
      const Matrix3d expected_dPijdF = Eigen::Map<const Matrix3d>(
          tau_volume_ad[0](i, j).derivatives().data(), 3, 3);
      EXPECT_TRUE(CompareMatrices(expected_dPijdF, dPijdF, 1e-12,
                                  MatrixCompareType::relative));
    }
  }
}

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
