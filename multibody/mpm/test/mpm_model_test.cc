#include "drake/multibody/mpm/mpm_model.h"

#include <limits>
#include <set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/mpm/mock_sparse_grid.h"
#include "drake/multibody/mpm/solver_state.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

using Eigen::Vector3d;
using Eigen::Vector3i;
using Eigen::VectorXd;

template <typename Grid>
class MpmModelTest : public ::testing::Test {};

using MyTypes = ::testing::Types<SparseGrid<double>, SparseGrid<float>>;
TYPED_TEST_SUITE(MpmModelTest, MyTypes);

/* Tests the constructors of MpmModel and SolverState. */
TYPED_TEST(MpmModelTest, Constructor) {
  using Grid = TypeParam;
  using T = typename Grid::Scalar;

  const double dx = 0.01;
  ParticleData<T> particle_data;
  /* Add a particle so that all grid nodes touched by this particle are in a
   single grid block. */
  const Vector3d x0(dx, dx, dx);
  particle_data.AddParticles({x0}, 1.0,
                             multibody::fem::DeformableBodyConfig<double>());
  const T dt = 0.02;

  MpmModel<T> model(dt, dx, std::move(particle_data));
  SolverState<T> state(model);

  EXPECT_EQ(model.dt(), dt);
  EXPECT_EQ(model.dx(), dx);
  EXPECT_EQ(model.num_particles(), 1);
  EXPECT_EQ(model.num_dofs(), 27 * 3);
  EXPECT_EQ(model.grid().dx(), dx);
  EXPECT_TRUE(CompareMatrices(state.dv(), VectorX<T>::Zero(27 * 3)));
  const contact_solvers::internal::PartialPermutation& vertex_permutation =
      model.index_permutation().vertex();
  EXPECT_EQ(vertex_permutation.domain_size(), 27);
  EXPECT_EQ(vertex_permutation.permuted_domain_size(), 0);
  EXPECT_EQ(state.F().size(), 1);
  EXPECT_EQ(state.tau_volume().size(), 1);
}

TYPED_TEST(MpmModelTest, UpdateModelAndReset) {
  using Grid = TypeParam;
  using T = typename Grid::Scalar;

  const double dx = 0.01;
  ParticleData<T> particle_data;
  /* Add a particle so that all grid nodes touched by this particle are in a
   single grid block. */
  const Vector3d x0(dx, dx, dx);
  /* Set the config such that the scale of the result is close to 1.0. */
  multibody::fem::DeformableBodyConfig<double> config;
  config.set_youngs_modulus(1.0);
  config.set_poissons_ratio(0.1);
  config.set_mass_density(1.0);
  particle_data.AddParticles({x0}, 1.0, config);
  const T dt = 0.01;

  MpmModel<T> model(dt, dx, std::move(particle_data));
  SolverState<T> state(model);

  /* Give the grid a uniform velocity field of (1, 0, 0). */
  VectorX<T> ddv = VectorX<T>::Zero(model.num_dofs());
  for (int i = 0; i < model.num_dofs(); ++i) {
    if (i % 3 == 0) {
      ddv[i] = 1.0;
    }
  }
  state.UpdateState(ddv, model);
  EXPECT_EQ(state.dv(), ddv);
  /* Write the change in the state back to the model. Now the particle should be
   moved to (0.02, 0.01, 0.01), with no deformation. */
  model.Update(state);
  constexpr T kTol = 4.0 * std::numeric_limits<T>::epsilon();
  EXPECT_TRUE(CompareMatrices(model.particle_data().x()[0],
                              Vector3<T>(0.02, 0.01, 0.01), kTol));
  EXPECT_TRUE(CompareMatrices(model.particle_data().F()[0],
                              Matrix3<T>::Identity(), kTol));
  /* Reset the state, now the grid should reflect that the particle is at the
   new location. */
  state.Reset(model);
  struct Vector3iLess {
    bool operator()(const Vector3i& a, const Vector3i& b) const {
      return std::lexicographical_compare(a.data(), a.data() + 3, b.data(),
                                          b.data() + 3);
    }
  };
  std::set<Vector3i, Vector3iLess> expected_active_nodes;
  for (int a = -1; a <= 1; ++a) {
    for (int b = -1; b <= 1; ++b) {
      for (int c = -1; c <= 1; ++c) {
        expected_active_nodes.insert(Vector3i(2 + a, 1 + b, 1 + c));
      }
    }
  }
  const std::vector<std::pair<Vector3i, GridData<T>>> grid_data =
      model.grid().GetGridData();
  EXPECT_EQ(grid_data.size(), expected_active_nodes.size());
  for (const auto& [node, node_data] : grid_data) {
    EXPECT_TRUE(expected_active_nodes.contains(node));
    EXPECT_TRUE(CompareMatrices(node_data.v, Vector3<T>(1.0, 0.0, 0.0), kTol));
  }
}

TYPED_TEST(MpmModelTest, CalcCost) {
  using Grid = TypeParam;
  using T = typename Grid::Scalar;

  const double dx = 0.01;
  ParticleData<T> particle_data;
  const Vector3d x0(dx, dx, dx);

  multibody::fem::DeformableBodyConfig<double> config;
  config.set_youngs_modulus(1.0);
  config.set_poissons_ratio(0.1);
  config.set_mass_density(1.0);

  particle_data.AddParticles({x0}, 1.0, config);
  const T dt = 0.02;
  constexpr T kTol = 4.0 * std::numeric_limits<T>::epsilon();

  MpmModel<T> model(dt, dx, particle_data);
  SolverState<T> state(model);
  T energy = model.CalcCost(state);
  EXPECT_EQ(energy, 0.0);

  /* A single particle activates 27 grid ndoes. */
  constexpr int num_dofs = 27 * 3;
  ASSERT_EQ(model.num_dofs(), num_dofs);
  /* Arbitrary velocity field. */
  VectorX<T> ddv = VectorX<T>::LinSpaced(num_dofs, 0.0, 1.0);
  state.UpdateState(ddv, model);
  energy = model.CalcCost(state);

  auto scratch = particle_data.deformation_gradient_data();
  /* This is tested in the ParticleData class. */
  const T elastic_energy =
      particle_data.ComputeTotalEnergy(state.F(), &scratch);

  double kinetic_energy = 0.0;
  const std::vector<std::pair<Vector3i, GridData<T>>> grid_data =
      model.grid().GetGridData();
  for (const auto& [node, node_data] : grid_data) {
    const int a = node[0];
    const int b = node[1];
    const int c = node[2];
    /* We make use of the fact that SpGrid follows lexicographical order
     within a block.*/
    const int node_index = a * 9 + b * 3 + c;
    kinetic_energy +=
        0.5 * node_data.m *
        state.dv().template segment<3>(node_index * 3).squaredNorm();
  }
  EXPECT_NEAR(energy, kinetic_energy + elastic_energy, kTol);
}

TYPED_TEST(MpmModelTest, CalcResidual) {
  using Grid = TypeParam;
  using T = typename Grid::Scalar;

  const double dx = 0.01;
  ParticleData<T> particle_data;
  const Vector3d x0(dx, dx, dx);

  /* Set the config such that the scale of the result is close to 1.0. */
  multibody::fem::DeformableBodyConfig<double> config;
  config.set_youngs_modulus(1.0);
  config.set_poissons_ratio(0.1);
  const double kRho = 1.2;
  config.set_mass_density(kRho);
  const double kVol = 2.3;
  particle_data.AddParticles({x0}, kVol, config);
  const T dt = 0.02;
  constexpr T kTol = 4.0 * std::numeric_limits<T>::epsilon();
  /* The mass of the only particle. */
  ASSERT_EQ(particle_data.m().size(), 1);
  const double particle_mass = particle_data.m()[0];

  MpmModel<T> model(dt, dx, particle_data);
  SolverState<T> state(model);
  const int num_dofs = model.num_dofs();

  /* We set residual be an arbitrary value with an arbitrary size to test that
   the function does not crash. */
  VectorX<T> residual = VectorX<T>::LinSpaced(42, 0, 1);
  model.CalcResidual(state, &residual);
  EXPECT_TRUE(CompareMatrices(residual, VectorX<T>::Zero(num_dofs)));

  /* Give the grid a constant velocity field so that it doesn't induce any
   deformation on the particle. Consequently, the only residual comes from the
   M * dv term. */
  VectorX<T> ddv = VectorX<T>::Ones(num_dofs);
  state.UpdateState(ddv, model);
  EXPECT_TRUE(CompareMatrices(state.dv(), ddv));
  model.CalcResidual(state, &residual);
  /* Get the Bspline weights between the only particle and the pad it
   supports. */
  const BsplineWeights<T> weights(particle_data.x()[0], static_cast<T>(dx));
  for (int a = 0; a < 3; ++a) {
    for (int b = 0; b < 3; ++b) {
      for (int c = 0; c < 3; ++c) {
        const T weight = weights.weight(a, b, c);
        /* We make use of the fact that SpGrid follows lexicographical order
         within a block.*/
        const int node_index = a * 9 + b * 3 + c;
        EXPECT_TRUE(CompareMatrices(
            residual.template segment<3>(node_index * 3),
            weight * particle_mass * ddv.template segment<3>(node_index * 3),
            kTol));
      }
    }
  }

  /* Reset dv to zero. */
  state.UpdateState(-ddv, model);
  EXPECT_TRUE(CompareMatrices(state.dv(), VectorX<T>::Zero(num_dofs), kTol));
  model.CalcResidual(state, &residual);
  EXPECT_TRUE(CompareMatrices(residual, VectorX<T>::Zero(num_dofs), kTol));

  /* Add a non-constant velocity field and confirm that the residual is no
   longer equal to M * dv, except for the center node; The center node is
   right on top of the particle and because of the xᵢ − xₚ term in computing
   the residual, the contribution to the residual from the particle
   deformation at this node is zero. */
  ddv = VectorX<T>::LinSpaced(num_dofs, 0, 1);
  state.UpdateState(ddv, model);
  model.CalcResidual(state, &residual);
  for (int a = 0; a < 3; ++a) {
    for (int b = 0; b < 3; ++b) {
      for (int c = 0; c < 3; ++c) {
        const double weight = weights.weight(a, b, c);
        /* We make use of the fact that SpGrid follows lexicographical order
         within a block.*/
        const int node_index = a * 9 + b * 3 + c;
        if (a == 1 && b == 1 && c == 1) {
          EXPECT_TRUE(CompareMatrices(
              residual.template segment<3>(node_index * 3),
              weight * particle_mass * ddv.template segment<3>(node_index * 3),
              kTol));
        } else {
          /* Even if we allow a lot of slop, these things aren't even nearly the
           same. */
          EXPECT_FALSE(CompareMatrices(
              residual.template segment<3>(node_index * 3),
              weight * particle_mass * ddv.template segment<3>(node_index * 3),
              1e-4));
        }
      }
    }
  }
}

GTEST_TEST(MpmModelTest, ResidualIsDerivativeOfEnergy) {
  const double dx = 0.01;
  const Vector3d x0(dx, dx, dx);
  const Vector3d x1(1.1 * dx, 1.2 * dx, 1.3 * dx);
  /* Set the config such that the scale of the result is close to 1.0. */
  multibody::fem::DeformableBodyConfig<double> config;
  config.set_youngs_modulus(1.0);
  config.set_poissons_ratio(0.1);
  config.set_mass_density(1.0);

  ParticleData<AutoDiffXd> particle_data;
  particle_data.AddParticles({x0, x1}, 1.0, config);

  const double dt = 0.02;
  MpmModel<AutoDiffXd, MockSparseGrid<AutoDiffXd>> model(dt, dx, particle_data);
  SolverState<AutoDiffXd, MockSparseGrid<AutoDiffXd>> state(model);

  const int num_dofs = model.num_dofs();
  ASSERT_EQ(num_dofs, 27 * 3);

  VectorX<double> ddv = VectorX<double>::LinSpaced(num_dofs, 0.0, 1.0);
  VectorX<AutoDiffXd> ddv_ad;
  ddv_ad.resize(num_dofs);
  math::InitializeAutoDiff(ddv, &ddv_ad);
  state.UpdateState(ddv_ad, model);

  const AutoDiffXd energy = model.CalcCost(state);
  VectorX<AutoDiffXd> residual;
  model.CalcResidual(state, &residual);
  const double kTol = 4.0 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(energy.derivatives(),
                              math::DiscardGradient(residual), kTol));
  EXPECT_FALSE(
      CompareMatrices(energy.derivatives(), VectorXd::Zero(num_dofs), 0.1));
}

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
