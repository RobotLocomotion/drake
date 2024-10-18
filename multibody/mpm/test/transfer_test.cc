#include "../transfer.h"

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

/* Confirms that the computed particle data matches with the expected
 values. */
void CheckParticleData(const ParticleData<double>& expected,
                       const ParticleData<double>& computed) {
  ASSERT_EQ(expected.m.size(), computed.m.size());
  ASSERT_EQ(expected.x.size(), computed.x.size());
  ASSERT_EQ(expected.v.size(), computed.v.size());
  ASSERT_EQ(expected.F.size(), computed.F.size());
  ASSERT_EQ(expected.C.size(), computed.C.size());
  ASSERT_EQ(expected.tau_v0.size(), computed.tau_v0.size());
  const double kTol = 1e-13;
  for (int i = 0; i < ssize(expected.m); ++i) {
    EXPECT_DOUBLE_EQ(expected.m[i], computed.m[i]);
    EXPECT_TRUE(CompareMatrices(expected.x[i], computed.x[i], kTol));
    EXPECT_TRUE(CompareMatrices(expected.v[i], computed.v[i], kTol));
    EXPECT_TRUE(CompareMatrices(expected.F[i], computed.F[i], kTol));
    EXPECT_TRUE(CompareMatrices(expected.C[i], computed.C[i], kTol));
    EXPECT_TRUE(CompareMatrices(expected.tau_v0[i], computed.tau_v0[i], kTol));
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

  const double kTol = 4.0 * std::numeric_limits<double>::epsilon();
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

/* Verify that G2P matches with analytical results with a single particle. */
GTEST_TEST(TransferTest, GridToParticle) {
  SparseGrid<double> grid(0.01);
  ParticleData<double> particles;
  const Vector3d x0 = Vector3d(0.001, 0.001, 0.001);

  particles.x.push_back(x0);
  Matrix3d F0 =
      (Matrix3d() << 1.0, 0.1, 0.2, 0.3, 1.0, 0.4, 0.5, 0.6, 1.0).finished();
  particles.F.push_back(F0);

  /* All other particle data are either unused or overwritten in G2P. */
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const Vector3d nan_vector = Vector3d(nan, nan, nan);
  const Matrix3d nan_matrix = Matrix3d::Constant(nan);
  particles.m.push_back(nan);
  particles.v.push_back(nan_vector);
  particles.C.push_back(nan_matrix);
  particles.tau_v0.push_back(nan_matrix);

  grid.Allocate(particles.x);

  const double dt = 0.0123;
  Transfer<double> transfer(dt, &grid, &particles);
  const Vector3d vel = Vector3d(1.2, 2.3, 3.4);
  auto constant_velocity_field = [&](const Vector3i& coordinate) {
    return GridData<double>{.v = vel, .m = 1.0};
  };
  grid.SetGridData(constant_velocity_field);
  transfer.SerialGridToParticle();

  EXPECT_TRUE(CompareMatrices(particles.v[0], vel, 1e-14));
  EXPECT_TRUE(CompareMatrices(particles.x[0], x0 + vel * dt, 1e-14));
  EXPECT_TRUE(CompareMatrices(particles.C[0], Matrix3d::Zero(), 1e-13));
  EXPECT_TRUE(CompareMatrices(particles.F[0], F0, 1e-13));
}

GTEST_TEST(TransferTest, ParticleToGrid) {
  SparseGrid<double> grid(0.01);
  ParticleData<double> particles;

  const double m0 = 0.42;
  const Vector3d x0 = Vector3d(0.001, 0.001, 0.001);
  const Vector3d v0 = Vector3d(0.1, 0.1, 0.1);
  BsplineWeights<double> bspline(x0, grid.dx());

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const Matrix3d nan_matrix = Matrix3d::Constant(nan);
  particles.m.push_back(m0);
  particles.x.push_back(x0);
  particles.v.push_back(v0);
  particles.F.push_back(nan_matrix);
  particles.C.push_back(Matrix3d::Zero());
  particles.tau_v0.push_back(Matrix3d::Zero());

  grid.Allocate(particles.x);
  const double dt = 0.0123;
  Transfer<double> transfer(dt, &grid, &particles);
  transfer.SerialParticleToGrid();
  grid.ExplicitVelocityUpdate(/* dv = */ Vector3d::Zero());

  std::vector<std::pair<Vector3<int>, GridData<double>>> expected_data;
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
  CheckMomentumConservation(grid.ComputeTotalMassAndMomentum(),
                            ComputeTotalMassAndMomentum(particles, grid.dx()),
                            expected_mass_and_momentum);
}

/* Verifies that both P2G and G2P conserves mass and momentum with more than one
 particle and more than one active block in the sparse grid. */
GTEST_TEST(TransferTest, MomentumConservation) {
  SparseGrid<double> grid(0.01);
  ParticleData<double> particles;
  /* Sample 3 particles with 2 in the same cell and the other in a separate
   page. */
  const Vector3d x0 = Vector3d(0.001, 0.002, 0.003);
  const Vector3d x1 = Vector3d(-0.001, 0.002, 0.003);
  const Vector3d x2 = Vector3d(1.001, 0.002, 1.003);
  particles.x.push_back(x0);
  particles.x.push_back(x1);
  particles.x.push_back(x2);

  Matrix3d nan_matrix =
      Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
  for (int p = 0; p < 3; ++p) {
    /* Set m, v, C, P to arbitrary values. */
    particles.m.push_back(0.042 * p);
    particles.v.push_back(Vector3d(0.1 * p, 0.2 * p, 0.3 * p));
    Matrix3d C;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        C(i, j) = i * j * p;
      }
    }
    particles.C.push_back(C);
    Matrix3d P;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        P(i, j) = i + j + p;
      }
    }
    particles.tau_v0.push_back(P);
    /* F is unused in the transfer. */
    particles.F.push_back(nan_matrix);
  }
  const MassAndMomentum<double> expected =
      ComputeTotalMassAndMomentum(particles, grid.dx());

  const double dt = 0.0123;
  Transfer<double> transfer(dt, &grid, &particles);
  transfer.SerialParticleToGrid();
  grid.ExplicitVelocityUpdate(/* dv = */ Vector3d::Zero());
  CheckMomentumConservation(grid.ComputeTotalMassAndMomentum(),
                            ComputeTotalMassAndMomentum(particles, grid.dx()),
                            expected);

  transfer.SerialGridToParticle();
  CheckMomentumConservation(grid.ComputeTotalMassAndMomentum(),
                            ComputeTotalMassAndMomentum(particles, grid.dx()),
                            expected);
}

/* Verify the variants of all 4 P2G and 4 G2P give the same results. */
GTEST_TEST(TransferTest, Parity) {
  const double dx = 0.01;
  ParticleData<double> particles;
  const int num_nodes_per_dim = 3;
  const int particles_per_cell = 2;
  for (int i = 0; i < num_nodes_per_dim; ++i) {
    for (int j = 0; j < num_nodes_per_dim; ++j) {
      for (int k = 0; k < num_nodes_per_dim; ++k) {
        const Vector3d base_node(dx * i, dx * j, dx * k);
        for (int p = 0; p < particles_per_cell; ++p) {
          particles.m.push_back(0.01);
          const Vector3d x =
              base_node + static_cast<double>(p) * dx /
                              (static_cast<double>(particles_per_cell) + 1.0) *
                              Vector3d::Ones();
          particles.x.push_back(x);
          particles.v.push_back(Vector3d(0.01 * i, 0.02 * j, 0.03 * k));
          particles.F.push_back(0.01 * Matrix3d::Identity());
          particles.C.push_back(0.02 * Matrix3d::Identity());
          particles.tau_v0.push_back(0.03 * Matrix3d::Identity());
        }
      }
    }
  }

  ParticleData particles_simd = particles;
  ParticleData particles_parallel = particles;
  ParticleData particles_parallel_simd = particles;

  SparseGrid<double> grid(dx);
  SparseGrid<double> grid_simd(dx);
  SparseGrid<double> grid_parallel(dx);
  SparseGrid<double> grid_parallel_simd(dx);

  grid.Allocate(particles.x);
  grid_simd.Allocate(particles.x);
  grid_parallel.Allocate(particles.x);
  grid_parallel_simd.Allocate(particles.x);

  const double dt = 0.00123;
  Transfer<double> transfer(dt, &grid, &particles);
  Transfer<double> transfer_simd(dt, &grid_simd, &particles_simd);
  Transfer<double> transfer_parallel(dt, &grid_parallel, &particles_parallel);
  Transfer<double> transfer_parallel_simd(dt, &grid_parallel_simd,
                                          &particles_parallel_simd);

  transfer.SerialParticleToGrid();
  transfer_simd.SerialSimdParticleToGrid();
  transfer_parallel.ParallelParticleToGrid(Parallelism(2));
  transfer_parallel_simd.ParallelSimdParticleToGrid(Parallelism(2));

  CheckGridData(grid.GetGridData(), grid_simd.GetGridData());
  CheckGridData(grid.GetGridData(), grid_parallel.GetGridData());
  CheckGridData(grid.GetGridData(), grid_parallel_simd.GetGridData());

  grid.ExplicitVelocityUpdate(/* dv = */ Vector3d::Zero());
  grid_simd.ExplicitVelocityUpdate(/* dv = */ Vector3d::Zero());
  grid_parallel.ExplicitVelocityUpdate(/* dv = */ Vector3d::Zero());
  grid_parallel_simd.ExplicitVelocityUpdate(/* dv = */ Vector3d::Zero());

  transfer.SerialGridToParticle();
  transfer_simd.SerialSimdGridToParticle();
  transfer_parallel.ParallelGridToParticle(Parallelism(2));
  transfer_parallel_simd.ParallelSimdGridToParticle(Parallelism(2));

  CheckParticleData(particles, particles_simd);
  CheckParticleData(particles, particles_parallel);
  CheckParticleData(particles, particles_parallel_simd);
}

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
