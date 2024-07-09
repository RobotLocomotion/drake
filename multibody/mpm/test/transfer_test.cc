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

GTEST_TEST(TransferTest, GridToParticle) {
  SparseGrid<double> grid(0.01);
  ParticleData<double> particles;
  const Vector3d x0 = Vector3d(0.001, 0.001, 0.001);
  particles.x.push_back(x0);
  particles.v.push_back(Vector3d(0.1, 0.1, 0.1));
  particles.F.push_back(Matrix3d::Identity());
  particles.C.push_back(Matrix3d::Zero());
  particles.P.push_back(Matrix3d::Zero());

  // TODO(xuchenhan-tri): Move this to a Bspline test.
  BSplineWeights<double> bspline(x0, grid.dx());
  double total_weight = 0.0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        total_weight += bspline.weight(i, j, k);
      }
    }
  }
  EXPECT_DOUBLE_EQ(total_weight, 1.0);
  particles.bspline.push_back(bspline);

  grid.Allocate(particles.x);

  /* Move this to a grid test. */
  EXPECT_EQ(grid.dx(), 0.01);

  const std::vector<int> expected_sentinel_particles = {0, 1};
  EXPECT_EQ(grid.sentinel_particles(), expected_sentinel_particles);

  const std::vector<ParticleIndex> particle_indices = grid.particle_indices();
  ASSERT_EQ(particle_indices.size(), 1);
  EXPECT_EQ(particle_indices[0].base_node_offset,
            grid.CoordinateToOffset(0, 0, 0));
  EXPECT_EQ(particle_indices[0].index, 0);

  const double dt = 0.0123;
  Transfer<double> transfer(dt, &grid, &particles);
  const Vector3d vel = Vector3d(1.2, 2.3, 3.4);
  auto constant_velocity_field = [&](const Vector3d& coordinate) {
    return GridData<double>{.v = vel, .m = 1.0};
  };
  grid.SetGridState(constant_velocity_field);
  transfer.GridToParticles();
  EXPECT_TRUE(CompareMatrices(particles.v[0], vel,
                              4.0 * std::numeric_limits<double>::epsilon()));
  EXPECT_TRUE(CompareMatrices(particles.x[0], x0 + vel * dt,
                              4.0 * std::numeric_limits<double>::epsilon()));
  EXPECT_TRUE(CompareMatrices(particles.F[0], Matrix3d::Identity(), 1e-13));
  EXPECT_TRUE(CompareMatrices(particles.C[0], Matrix3d::Zero(), 1e-13));
}

GTEST_TEST(TransferTest, ParticleToGrid) {
  SparseGrid<double> grid(0.01);
  ParticleData<double> particles;

  const double m0 = 0.42;
  const Vector3d x0 = Vector3d(0.001, 0.001, 0.001);
  const Vector3d v0 = Vector3d(0.1, 0.1, 0.1);
  BSplineWeights<double> bspline(x0, grid.dx());

  particles.m.push_back(m0);
  particles.x.push_back(x0);
  particles.v.push_back(v0);
  particles.F.push_back(Matrix3d::Identity());
  particles.C.push_back(Matrix3d::Zero());
  particles.P.push_back(Matrix3d::Zero());
  particles.bspline.push_back(bspline);

  grid.Allocate(particles.x);
  const double dt = 0.0123;
  Transfer<double> transfer(dt, &grid, &particles);
  transfer.ParticleToGrid();
  std::vector<std::pair<Vector3<int>, GridData<double>>> expected_data;
  for (int i = -1; i <= 1; ++i) {
    for (int j = -1; j <= 1; ++j) {
      for (int k = -1; k <= 1; ++k) {
        const double expected_mass = m0 * bspline.weight(i + 1, j + 1, k + 1);
        const Vector3<double> expected_velocity = expected_mass * v0;
        expected_data.emplace_back(std::make_pair(
            Vector3<int>(i, j, k),
            GridData<double>{.v = expected_velocity, .m = expected_mass}));
      }
    }
  }
  std::vector<std::pair<Vector3<int>, GridData<double>>> computed_data =
      grid.GetGridData();
  EXPECT_EQ(expected_data.size(), computed_data.size());
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

  for (int i = 0; i < ssize(expected_data); ++i) {
    const auto& expected = expected_data[i];
    const auto& computed = computed_data[i];
    EXPECT_EQ(expected.first, computed.first);
    // EXPECT_TRUE(CompareMatrices(expected.second.v, computed.second.v,
    //                             4.0 *
    //                             std::numeric_limits<double>::epsilon()));
    EXPECT_DOUBLE_EQ(expected.second.m, computed.second.m);
  }
}

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
