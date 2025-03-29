#include "drake/multibody/mpm/transfer.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector3i;

template <typename Grid>
void ConvertMomentumToVelocity(Grid* grid) {
  using T = typename Grid::Scalar;
  grid->IterateGrid([](GridData<T>* grid_data) {
    if (grid_data->m > 0) {
      grid_data->v /= grid_data->m;
    }
  });
}

/* Adds a particle at position x0 to the given `particles`. All other data are
 arbitrarily set. However, if `nonzero_F_and_C_and_stress` is false, F, C, and
 tau_volume are set to zero. */
void AddParticle(ParticleData<double>* particles, Vector3d x0,
                 bool nonzero_F_and_C_and_stress = true) {
  particles->AddParticles({x0}, 1.0, fem::DeformableBodyConfig<double>());
  particles->mutable_v().back() = Vector3d(0.1, 0.2, 0.3);
  if (nonzero_F_and_C_and_stress) {
    const Matrix3d F =
        (Matrix3d() << 1.0, 0.1, 0.2, 0.3, 1.0, 0.4, 0.5, 0.6, 1.0).finished();
    particles->mutable_F().back() = F;
    const Matrix3d C =
        (Matrix3d() << 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8).finished();
    particles->mutable_C().back() = C;
    particles->mutable_tau_volume().back() = 1.23 * (C + C.transpose());
  }
}

/* Confirms that the computed grid mass and velocities matches with the expected
 values. */
void CheckGridData(
    std::vector<std::pair<Vector3<int>, GridData<double>>> computed_data,
    std::vector<std::pair<Vector3<int>, GridData<double>>> expected_data) {
  ASSERT_EQ(expected_data.size(), computed_data.size());
  /* Sort both expected data and the computed data based on the grid node
   coordinates. */
  auto compare = [](const std::pair<Vector3<int>, GridData<double>>& a,
                    const std::pair<Vector3<int>, GridData<double>>& b) {
    const auto key_a = a.first;
    const auto key_b = b.first;
    if (key_a[0] != key_b[0]) {
      return key_a[0] < key_b[0];
    }
    if (key_a[1] != key_b[1]) {
      return key_a[1] < key_b[1];
    }
    return key_a[2] < key_b[2];
  };
  std::sort(expected_data.begin(), expected_data.end(), compare);
  std::sort(computed_data.begin(), computed_data.end(), compare);

  const double kTol = 16.0 * std::numeric_limits<double>::epsilon();
  for (int i = 0; i < ssize(expected_data); ++i) {
    const auto& expected = expected_data[i];
    const auto& computed = computed_data[i];
    EXPECT_EQ(expected.first, computed.first);
    EXPECT_TRUE(CompareMatrices(expected.second.v, computed.second.v, kTol));
    EXPECT_DOUBLE_EQ(expected.second.m, computed.second.m);
  }
}

/* Confirms that the mass and momentum for both the grid and the particles match
 with expected values. */
void CheckMomentumConservation(const MassAndMomentum<double>& grid,
                               const MassAndMomentum<double>& particle,
                               const MassAndMomentum<double>& expected) {
  const double kTol = 1e-12;
  EXPECT_DOUBLE_EQ(particle.mass, expected.mass);
  EXPECT_DOUBLE_EQ(grid.mass, expected.mass);
  EXPECT_TRUE(CompareMatrices(particle.linear_momentum,
                              expected.linear_momentum, kTol));
  EXPECT_TRUE(
      CompareMatrices(grid.linear_momentum, expected.linear_momentum, kTol));
  EXPECT_TRUE(CompareMatrices(particle.angular_momentum,
                              expected.angular_momentum, kTol));
  EXPECT_TRUE(
      CompareMatrices(grid.angular_momentum, expected.angular_momentum, kTol));
}

/* Verify that the result of grid to particle transfer matches with analytical
 results with a single particle. */
GTEST_TEST(TransferTest, GridToParticle) {
  SparseGrid<double> grid(0.1);
  ParticleData<double> particle_data;
  const Vector3d x0 = Vector3d(0.11, 0.11, 0.09);
  AddParticle(&particle_data, x0);
  const Matrix3d& F0 = particle_data.F()[0];

  const double dt = 0.0123;
  Transfer transfer(dt, &grid, &particle_data);
  const Vector3d vel = Vector3d(1.2, 2.3, 3.4);
  auto constant_velocity_field = [&](const Vector3i&) {
    return GridData<double>{.v = vel, .m = 1.0};
  };
  grid.SetGridData(constant_velocity_field);
  transfer.GridToParticle();

  EXPECT_TRUE(CompareMatrices(particle_data.v()[0], vel, 1e-14));
  EXPECT_TRUE(CompareMatrices(particle_data.x()[0], x0 + vel * dt, 1e-14));
  EXPECT_TRUE(CompareMatrices(particle_data.C()[0], Matrix3d::Zero(), 1e-12));
  EXPECT_TRUE(CompareMatrices(particle_data.F()[0], F0, 1e-12));
}

GTEST_TEST(TransferTest, ParticleToGrid) {
  SparseGrid<double> grid(0.1);
  ParticleData<double> particle_data;
  const Vector3d x0 = Vector3d(0.01, 0.02, 0.03);
  AddParticle(&particle_data, x0, /* nonzero_F_and_C_and_stress */ false);
  const double m0 = particle_data.m()[0];
  const Vector3d v0 = particle_data.v()[0];

  const double dt = 0.0123;
  Transfer transfer(dt, &grid, &particle_data);
  transfer.ParticleToGrid();
  /* Convert momentum to velocity on the grid data after P2G. */
  ConvertMomentumToVelocity(&grid);

  std::vector<std::pair<Vector3<int>, GridData<double>>> expected_data;
  const BsplineWeights<double> bspline(x0, grid.dx());
  for (int i = -1; i <= 1; ++i) {
    for (int j = -1; j <= 1; ++j) {
      for (int k = -1; k <= 1; ++k) {
        const double expected_mass = m0 * bspline.weight(i + 1, j + 1, k + 1);
        const Vector3<double> expected_velocity = v0;
        expected_data.emplace_back(std::make_pair(
            Vector3<int>(i, j, k),
            GridData<double>{.v = expected_velocity, .m = expected_mass}));
      }
    }
  }
  std::vector<std::pair<Vector3<int>, GridData<double>>> computed_data =
      grid.GetGridData();

  CheckGridData(computed_data, expected_data);

  const MassAndMomentum<double> expected_mass_and_momentum{
      .mass = m0,
      .linear_momentum = m0 * v0,
      .angular_momentum = m0 * x0.cross(v0)};
  CheckMomentumConservation(
      grid.ComputeTotalMassAndMomentum(),
      ComputeTotalMassAndMomentum(particle_data, grid.dx()),
      expected_mass_and_momentum);
}

/* Verifies that both P2G and G2P conserves mass and momentum with more than one
 particle and more than one active block in the sparse grid. */
GTEST_TEST(TransferTest, MomentumConservation) {
  SparseGrid<double> grid(0.2);
  ParticleData<double> particle_data;
  /* Sample 3 particles with 2 in the same cell and the other in a separate
   page. */
  const Vector3d x0 = Vector3d(0.001, 0.002, 0.003);
  const Vector3d x1 = Vector3d(-0.001, 0.002, 0.003);
  const Vector3d x2 = Vector3d(1.011, 0.002, 1.013);
  AddParticle(&particle_data, x0);
  AddParticle(&particle_data, x1);
  AddParticle(&particle_data, x2);

  const MassAndMomentum<double> expected =
      ComputeTotalMassAndMomentum(particle_data, grid.dx());

  const double dt = 0.01;
  Transfer transfer(dt, &grid, &particle_data);
  transfer.ParticleToGrid();
  ConvertMomentumToVelocity(&grid);
  CheckMomentumConservation(
      grid.ComputeTotalMassAndMomentum(),
      ComputeTotalMassAndMomentum(particle_data, grid.dx()), expected);

  transfer.GridToParticle();
  CheckMomentumConservation(
      grid.ComputeTotalMassAndMomentum(),
      ComputeTotalMassAndMomentum(particle_data, grid.dx()), expected);
}

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
