#include "../sparse_grid.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

using Eigen::Vector3d;
using Eigen::Vector3i;

GTEST_TEST(SparseGridTest, Allocate) {
  const double dx = 0.01;
  SparseGrid<double> grid(dx);
  const Vector3d q_WP = Vector3d(1.001, 0.001, 0.001);
  std::vector<Vector3d> q_WPs = {q_WP};

  grid.Allocate(q_WPs);

  EXPECT_EQ(grid.dx(), 0.01);

  const std::vector<int> expected_sentinel_particles = {0, 1};
  EXPECT_EQ(grid.sentinel_particles(), expected_sentinel_particles);

  const std::vector<int>& data_indices = grid.data_indices();
  ASSERT_EQ(data_indices.size(), 1);
  EXPECT_EQ(data_indices[0], 0);

  const std::vector<uint64_t>& base_node_offsets = grid.base_node_offsets();
  ASSERT_EQ(base_node_offsets.size(), 1);
  EXPECT_EQ(base_node_offsets[0], grid.CoordinateToOffset(100, 0, 0));

  /* Verify grid data is all zeroed out. */
  const std::vector<std::pair<Vector3i, GridData<double>>> grid_data =
      grid.GetGridData();
  for (const auto& [node, data] : grid_data) {
    EXPECT_EQ(data.m, 0.0);
    EXPECT_EQ(data.v, Vector3d::Zero());
  }

  EXPECT_EQ(grid.num_blocks(), 1);
}

GTEST_TEST(SparseGridTest, BaseNodeOffsets) {
  const double dx = 0.01;
  SparseGrid<double> grid(dx);
  /* Particle 0, 1 has the same base node (0, 0, 0).
     Particle 2 has a different base node (1, 0, 0), but is still in the same
     block. Particle 3 with base node (4, 0, 0) is in a different block from
     particles 0, 1, and 2. */
  const Vector3d q_WP0 = Vector3d(0.001, 0.001, 0.001);
  const Vector3d q_WP1 = Vector3d(-0.004, 0.001, 0.001);
  const Vector3d q_WP2 = Vector3d(0.012, 0.004, 0.001);
  const Vector3d q_WP3 = Vector3d(0.04, 0.0, 0.0);
  std::vector<Vector3d> q_WPs = {q_WP0, q_WP1, q_WP2, q_WP3};

  grid.Allocate(q_WPs);

  const std::vector<uint64_t>& base_node_offsets = grid.base_node_offsets();
  ASSERT_EQ(base_node_offsets.size(), 4);
  EXPECT_EQ(base_node_offsets[0], grid.CoordinateToOffset(0, 0, 0));
  EXPECT_EQ(base_node_offsets[1], grid.CoordinateToOffset(0, 0, 0));
  EXPECT_EQ(base_node_offsets[2], grid.CoordinateToOffset(1, 0, 0));
  EXPECT_EQ(base_node_offsets[3], grid.CoordinateToOffset(4, 0, 0));

  EXPECT_EQ(grid.num_blocks(), 2);
}

GTEST_TEST(SparseGridTest, SentinelParticles) {
  const double dx = 0.01;
  SparseGrid<double> grid(dx);
  /* Particle 0, 1 has the same base node (0, 0, 0).
     Particle 2 has a different base node (1, 0, 0), but is still in the same
     block. Particle 3 is in a different block from particles 0, 1, and 2. */
  const Vector3d q_WP0 = Vector3d(0.001, 0.001, 0.001);
  const Vector3d q_WP1 = Vector3d(-0.004, 0.001, 0.001);
  const Vector3d q_WP2 = Vector3d(0.012, 0.004, 0.001);
  const Vector3d q_WP3 = Vector3d(0.04, 0.0, 0.0);
  std::vector<Vector3d> q_WPs = {q_WP0, q_WP1, q_WP2, q_WP3};

  grid.Allocate(q_WPs);

  EXPECT_EQ(grid.dx(), 0.01);

  /* Sentinel particles are particles 0 and 3, marking boundary of new blocks.
   The last entry is the number of particles. */
  const std::vector<int> expected_sentinel_particles = {0, 3, 4};
  EXPECT_EQ(grid.sentinel_particles(), expected_sentinel_particles);

  /* Particles are sorted first based on their base node offsets:

    base_node(0) == base_node(1) < base_node(2) < base_node(3),

    and then on original indices: 0 < 1.*/
  const std::vector<int> expected_data_indices = {0, 1, 2, 3};
  EXPECT_EQ(grid.data_indices(), expected_data_indices);
}

GTEST_TEST(SparseGridTest, DataIndices) {
  const double dx = 0.01;
  SparseGrid<double> grid(dx);
  /* Here we swap the positions of particle 0 and 3 from the previous test. */
  const Vector3d q_WP0 = Vector3d(0.04, 0.0, 0.0);
  const Vector3d q_WP1 = Vector3d(-0.004, 0.001, 0.001);
  const Vector3d q_WP2 = Vector3d(0.012, 0.004, 0.001);
  const Vector3d q_WP3 = Vector3d(0.001, 0.001, 0.001);
  std::vector<Vector3d> q_WPs = {q_WP0, q_WP1, q_WP2, q_WP3};

  grid.Allocate(q_WPs);

  EXPECT_EQ(grid.dx(), 0.01);

  /* The sentinel particles remain the same. */
  const std::vector<int> expected_sentinel_particles = {0, 3, 4};
  EXPECT_EQ(grid.sentinel_particles(), expected_sentinel_particles);

  /* But the data indices are different.
    base_node(1) == base_node(3) < base_node(2) < base_node(0). */
  const std::vector<int> expected_data_indices = {1, 3, 2, 0};
  EXPECT_EQ(grid.data_indices(), expected_data_indices);
}

GTEST_TEST(SparseGridTest, GetPadNodes) {
  const double dx = 0.01;
  SparseGrid<double> grid(dx);
  const Vector3d q_WP = Vector3d(0.001, 0.001, 0.001);
  /* Base node is (0,0,0), so we should get the 27 immediate neighbors of the
   (0, 0, 0) as the pad nodes. */
  const Pad<Vector3d> pad_nodes = grid.GetPadNodes(q_WP);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        const Vector3d node =
            Vector3d((i - 1) * dx, (j - 1) * dx, (k - 1) * dx);
        EXPECT_EQ(pad_nodes[i][j][k], node);
      }
    }
  }
}

GTEST_TEST(SparseGridTest, PadData) {
  const double dx = 0.01;
  SparseGrid<double> grid(dx);
  /* Base node is (2, 3, 0). */
  const Vector3d q_WP = Vector3d(0.021, 0.031, -0.001);
  std::vector<Vector3d> q_WPs = {q_WP};

  grid.Allocate(q_WPs);

  const std::vector<uint64_t>& base_node_offsets = grid.base_node_offsets();
  ASSERT_EQ(base_node_offsets.size(), 1);
  const uint64_t base_node_offset = base_node_offsets[0];

  Pad<GridData<double>> arbitrary_data;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        GridData<double> foo;
        foo.m = i + j + k;
        foo.v = Vector3d(i, j, k);
        arbitrary_data[i][j][k] = foo;
      }
    }
  }

  grid.SetPadData(base_node_offset, arbitrary_data);
  const Pad<GridData<double>> pad_data = grid.GetPadData(base_node_offset);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        EXPECT_EQ(pad_data, arbitrary_data);
      }
    }
  }

  /* Now get the pad centered at (1, 2, -1). It should overlap with the pad
   centered at (2, 3, 0). The non-overlapping portion should be zeroed out
   (during Allocate()).  */
  const Pad<GridData<double>> pad_data2 =
      grid.GetPadData(grid.CoordinateToOffset(1, 2, -1));
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        if (i == 0 || j == 0 || k == 0) {
          EXPECT_EQ(pad_data2[i][j][k].m, 0.0);
          EXPECT_EQ(pad_data2[i][j][k].v, Vector3d::Zero());
        } else {
          EXPECT_EQ(pad_data2[i][j][k], arbitrary_data[i - 1][j - 1][k - 1]);
        }
      }
    }
  }
}

GTEST_TEST(SparseGridTest, CoordinateToOffset) {
  const double dx = 0.01;
  SparseGrid<double> grid(dx);
  /* Base node is (2, 3, 0). */
  const Vector3d q_WP = Vector3d(0.021, 0.031, -0.001);
  std::vector<Vector3d> q_WPs = {q_WP};

  grid.Allocate(q_WPs);

  const uint64_t base_node_offset = grid.base_node_offsets()[0];
  EXPECT_EQ(grid.CoordinateToOffset(2, 3, 0), base_node_offset);
  EXPECT_EQ(grid.OffsetToCoordinate(base_node_offset), Vector3i(2, 3, 0));
}

GTEST_TEST(SparseGridTest, ExplicitVelocityUpdate) {
  const double dx = 0.01;
  SparseGrid<double> grid(dx);
  const Vector3d q_WP = Vector3d(0.001, 0.001, 0.001);
  std::vector<Vector3d> q_WPs = {q_WP};

  grid.Allocate(q_WPs);

  auto set_arbitrary_grid_data = [](const Vector3i& node) {
    GridData<double> result;
    result.m = node[0] + node[1] + node[2];
    result.v = Vector3d(node[0], node[1], node[2]);
    return result;
  };

  grid.SetGridData(set_arbitrary_grid_data);

  /* Verify the grid data is set as expected. */
  const std::vector<std::pair<Vector3i, GridData<double>>> grid_data =
      grid.GetGridData();
  for (const auto& [node, data] : grid_data) {
    EXPECT_EQ(data.m, node[0] + node[1] + node[2]);
    EXPECT_EQ(data.v, Vector3d(node[0], node[1], node[2]));
  }

  /* Convert momentum to velocity and explicitly update the velocity field. */
  const Vector3d dv(1, 2, 3);
  grid.ExplicitVelocityUpdate(dv);

  const std::vector<std::pair<Vector3i, GridData<double>>> grid_data2 =
      grid.GetGridData();
  for (const auto& [node, data] : grid_data2) {
    EXPECT_EQ(data.m, node[0] + node[1] + node[2]);
    EXPECT_EQ(data.v, Vector3d(node[0], node[1], node[2]) / data.m + dv);
  }
}

GTEST_TEST(SparseGridTest, ComputeTotalMassAndMomentum) {
  const double dx = 0.01;
  SparseGrid<double> grid(dx);
  const Vector3d q_WP = Vector3d(0.001, 0.001, 0.001);
  std::vector<Vector3d> q_WPs = {q_WP};

  grid.Allocate(q_WPs);

  const double mass = 1.2;
  const Vector3d velocity = Vector3d(1, 2, 3);
  /* World frame position of the node with non-zero mass. */
  const Vector3d q_WN = Vector3d(dx, dx, dx);
  /* Set grid data so that the grid node (1, 1, 1) has velocity (1, 2, 3) and
   all other grid nodes have zero velocity. */
  auto set_grid_data = [mass, velocity](const Vector3i& node) {
    GridData<double> result;
    if (node[0] == 1 && node[1] == 1 && node[2] == 1) {
      result.m = mass;
      result.v = velocity;
    } else {
      result.set_zero();
    }
    return result;
  };

  grid.SetGridData(set_grid_data);

  const MassAndMomentum<double> computed = grid.ComputeTotalMassAndMomentum();
  EXPECT_EQ(computed.mass, mass);
  EXPECT_TRUE(CompareMatrices(computed.linear_momentum, mass * velocity,
                              4.0 * std::numeric_limits<double>::epsilon()));
  EXPECT_TRUE(CompareMatrices(computed.angular_momentum,
                              mass * q_WN.cross(velocity),
                              4.0 * std::numeric_limits<double>::epsilon()));
}

/* We place 8 particles in the grid to activate one block for each color.
 For doubles, each block is of size 4x4x8 grid nodes. We place particles at
 (5*i*dx, 5*j*dx, 10*k*dx) for i, j, k = 0 or 1 to activate the 8 blocks
 starting at the block containing the origin. */
GTEST_TEST(SparseGridTest, ColoredBlocks) {
  const double dx = 0.01;
  SparseGrid<double> grid(dx);
  std::vector<Vector3d> q_WPs;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      for (int k = 0; k < 2; ++k) {
        q_WPs.emplace_back(Vector3d(5 * i * dx, 5 * j * dx, 10 * k * dx));
      }
    }
  }

  grid.Allocate(q_WPs);

  const auto& sentinel_particles = grid.sentinel_particles();
  ASSERT_EQ(sentinel_particles.size(), 9);

  const std::array<std::vector<int>, 8> expected_blocks = {
      {{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}}};

  EXPECT_EQ(grid.colored_blocks(), expected_blocks);
}

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
