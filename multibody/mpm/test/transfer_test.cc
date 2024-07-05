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
  particles.C.push_back(Matrix3d::Identity());
  particles.P.push_back(Matrix3d::Zero());

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

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
