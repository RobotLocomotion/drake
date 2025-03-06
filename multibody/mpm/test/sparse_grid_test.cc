#include "drake/multibody/mpm/sparse_grid.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

/* Our default choice of 10 is too large for Valgrind. */
const int kLog2MaxGridSize = 5;

template <typename T>
class SparseGridTest : public ::testing::Test {};

using MyTypes = ::testing::Types<double, float>;
TYPED_TEST_SUITE(SparseGridTest, MyTypes);

TYPED_TEST(SparseGridTest, Allocate) {
  using T = TypeParam;

  const double dx = 0.1;

  SparseGrid<T, kLog2MaxGridSize> grid(dx);
  EXPECT_EQ(grid.dx(), 0.1);

  const Vector3<T> q_WP(1.001, 0.001, 0.001);
  const std::vector<Vector3<T>> q_WPs = {q_WP};
  grid.Allocate(q_WPs);

  /* Verify grid data is all zeroed out. */
  const auto grid_data = grid.GetGridData();
  for (const auto& [node, data] : grid_data) {
    EXPECT_EQ(data.m, static_cast<T>(0.0));
    EXPECT_EQ(data.v, Vector3<T>::Zero());
  }

  int num_active_nodes = 0;
  auto count_active_nodes = [&num_active_nodes](uint64_t, const GridData<T>&) {
    ++num_active_nodes;
  };
  grid.spgrid().IterateConstGridWithOffset(count_active_nodes);

  const int block_size = std::is_same_v<T, double> ? 64 : 128;
  /* We allocate the block B that contains the particle and the one ring of
   blocks around B, totaling 27 blocks. */
  EXPECT_EQ(num_active_nodes, block_size * 27);
}

TYPED_TEST(SparseGridTest, Clone) {
  using T = TypeParam;

  /* Set up a grid with grid nodes in [0, 2] x [0, 2] x [0, 2] all active. */
  const double dx = 0.5;
  SparseGrid<T, kLog2MaxGridSize> grid(dx);
  std::vector<Vector3<T>> q_WPs;
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      for (int k = 0; k < 5; ++k) {
        q_WPs.emplace_back(Vector3<T>(static_cast<T>(i) * dx,
                                      static_cast<T>(j) * dx,
                                      static_cast<T>(k) * dx));
      }
    }
  }
  grid.Allocate(q_WPs);

  /* Set an arbitrary grid data. */
  auto set_data = [](const Eigen::Vector3i& coordinate) {
    GridData<T> result;
    result.m = coordinate[0] + coordinate[1] + coordinate[2];
    result.v = Vector3<T>(coordinate[0], coordinate[1], coordinate[2]);
    return result;
  };
  grid.SetGridData(set_data);

  /* Clone the grid. */
  auto cloned_grid = grid.Clone();

  /* Verify that the cloned grid has the same grid data. */
  const auto grid_data = grid.GetGridData();
  const auto cloned_grid_data = cloned_grid->GetGridData();
  ASSERT_EQ(grid_data.size(), cloned_grid_data.size());
  for (size_t i = 0; i < grid_data.size(); ++i) {
    const auto& [node, data] = grid_data[i];
    const auto& [cloned_node, cloned_data] = cloned_grid_data[i];
    EXPECT_EQ(node, cloned_node);
    EXPECT_EQ(data, cloned_data);
  }
}

TYPED_TEST(SparseGridTest, GetPadNodes) {
  using T = TypeParam;

  const double dx = 0.01;
  SparseGrid<T, kLog2MaxGridSize> grid(dx);
  const Vector3<T> q_WP(0.001, 0.001, 0.001);
  /* Base node is (0, 0, 0), so we should get the 27 neighbors of (0,0,0). */
  const auto pad_nodes = grid.GetPadNodes(q_WP);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        const Vector3<T> node(static_cast<T>(i - 1) * dx,
                              static_cast<T>(j - 1) * dx,
                              static_cast<T>(k - 1) * dx);
        EXPECT_EQ(pad_nodes[i][j][k], node);
      }
    }
  }
}

TYPED_TEST(SparseGridTest, PadData) {
  using T = TypeParam;

  const double dx = 0.01;
  SparseGrid<T, kLog2MaxGridSize> grid(dx);
  /* Base node is (2, 3, 0). */
  const Vector3<T> q_WP(0.021, 0.031, -0.001);
  const Vector3<int> base_node = ComputeBaseNode<T>(q_WP / static_cast<T>(dx));
  const uint64_t base_node_offset = grid.spgrid().CoordinateToOffset(base_node);
  std::vector<Vector3<T>> q_WPs = {q_WP};
  grid.Allocate(q_WPs);

  Pad<GridData<T>> arbitrary_data;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        GridData<T> foo;
        foo.m = i + j + k;
        foo.v = Vector3<T>(i, j, k);
        arbitrary_data[i][j][k] = foo;
      }
    }
  }

  grid.SetPadData(base_node_offset, arbitrary_data);
  const auto pad_data = grid.GetPadData(base_node_offset);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        EXPECT_EQ(pad_data[i][j][k], arbitrary_data[i][j][k]);
      }
    }
  }

  /* Now get the pad centered at (1, 2, -1). It should overlap with the pad
   centered at (2, 3, 0). The non-overlapping portion should be zeroed out
   (during Allocate()). */
  const auto pad_data2 =
      grid.GetPadData(grid.spgrid().CoordinateToOffset(1, 2, -1));
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        if (i == 0 || j == 0 || k == 0) {
          EXPECT_EQ(pad_data2[i][j][k].m, static_cast<T>(0.0));
          EXPECT_EQ(pad_data2[i][j][k].v, Vector3<T>::Zero());
        } else {
          EXPECT_EQ(pad_data2[i][j][k], arbitrary_data[i - 1][j - 1][k - 1]);
        }
      }
    }
  }
}

TYPED_TEST(SparseGridTest, ComputeTotalMassAndMomentum) {
  using T = TypeParam;

  const double dx = 0.01;
  SparseGrid<T, kLog2MaxGridSize> grid(dx);
  const Vector3<T> q_WP(0.001, 0.001, 0.001);
  std::vector<Vector3<T>> q_WPs = {q_WP};
  grid.Allocate(q_WPs);

  const T mass = 1.2;
  const Vector3<T> velocity(1, 2, 3);

  /* World-frame position of the node with non-zero mass. */
  const Vector3<T> q_WN(dx, dx, dx);

  /* Set grid data so that node (1,1,1) has velocity (1,2,3) and all others
   are zero. */
  auto set_grid_data = [mass, velocity](const Eigen::Vector3i& node) {
    GridData<T> result;
    if (node[0] == 1 && node[1] == 1 && node[2] == 1) {
      result.m = mass;
      result.v = velocity;
    } else {
      result.reset();
    }
    return result;
  };

  grid.SetGridData(set_grid_data);

  const MassAndMomentum<T> computed = grid.ComputeTotalMassAndMomentum();
  EXPECT_EQ(computed.mass, mass);
  EXPECT_TRUE(CompareMatrices(computed.linear_momentum, mass * velocity,
                              4.0 * std::numeric_limits<T>::epsilon()));
  EXPECT_TRUE(CompareMatrices(computed.angular_momentum,
                              mass * q_WN.cross(velocity),
                              4.0 * std::numeric_limits<T>::epsilon()));
}

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
