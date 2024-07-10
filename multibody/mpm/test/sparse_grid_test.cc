#include "../sparse_grid.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

using Eigen::Vector3d;

GTEST_TEST(SparseGridTest, Allocate) {
  SparseGrid<double> grid(0.01);
  std::vector<Vector3d> positions;
  const Vector3d x = Vector3d(0.001, 0.001, 0.001);
  positions.push_back(x);
  grid.Allocate(positions);

  EXPECT_EQ(grid.dx(), 0.01);

  const std::vector<int> expected_sentinel_particles = {0, 1};
  EXPECT_EQ(grid.sentinel_particles(), expected_sentinel_particles);

  const std::vector<ParticleIndex> particle_indices = grid.particle_indices();
  ASSERT_EQ(particle_indices.size(), 1);
  EXPECT_EQ(particle_indices[0].base_node_offset,
            grid.CoordinateToOffset(0, 0, 0));
  EXPECT_EQ(particle_indices[0].index, 0);

  // TODO(xuchenhan)-tri: Check grid data is all zeroed out.
}
}
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
