#include "drake/multibody/mpm/sparse_grid.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/mpm/mock_sparse_grid.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

template <typename Grid>
class SparseGridTest : public ::testing::Test {};

using MyTypes =
    ::testing::Types<SparseGrid<double>, SparseGrid<float>,
                     MockSparseGrid<double>, MockSparseGrid<AutoDiffXd>>;
TYPED_TEST_SUITE(SparseGridTest, MyTypes);

TYPED_TEST(SparseGridTest, Allocate) {
  using Grid = TypeParam;
  using T = typename Grid::Scalar;
  using U = typename Grid::NodeScalarType;

  const double dx = 0.1;

  Grid grid(dx);
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
  auto count_active_nodes = [&num_active_nodes](uint64_t, const GridData<U>&) {
    ++num_active_nodes;
  };
  grid.spgrid().IterateConstGridWithOffset(count_active_nodes);

  const int block_size = std::is_same_v<U, double> ? 64 : 128;
  /* We allocate the block B that contains the particle and the one ring of
   blocks around B, totaling 27 blocks. */
  EXPECT_EQ(num_active_nodes, block_size * 27);
}

TYPED_TEST(SparseGridTest, Clone) {
  using Grid = TypeParam;
  using T = typename Grid::Scalar;

  /* Set up a grid with grid nodes in [0, 2] x [0, 2] x [0, 2] all active. */
  const double dx = 0.5;
  Grid grid(dx);
  std::vector<Vector3<T>> q_WPs;
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      for (int k = 0; k < 5; ++k) {
        q_WPs.emplace_back(Vector3<T>(i * dx, j * dx, k * dx));
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
  using Grid = TypeParam;
  using T = typename Grid::Scalar;
  using NodeType = typename Grid::NodeType;

  const double dx = 0.01;
  Grid grid(dx);
  const Vector3<T> q_WP(0.001, 0.001, 0.001);
  /* Base node is (0, 0, 0), so we should get the 27 neighbors of (0,0,0). */
  const auto pad_nodes = grid.GetPadNodes(q_WP);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        const NodeType node((i - 1) * dx, (j - 1) * dx, (k - 1) * dx);
        EXPECT_EQ(pad_nodes[i][j][k], node);
      }
    }
  }
}

TYPED_TEST(SparseGridTest, PadData) {
  using Grid = TypeParam;
  using T = typename Grid::Scalar;

  const double dx = 0.01;
  Grid grid(dx);
  /* Base node is (2, 3, 0). */
  const Vector3<T> q_WP(0.021, 0.031, -0.001);
  const Vector3<double> q_WP_double(0.021, 0.031, -0.001);
  const Vector3<int> base_node =
      ComputeBaseNode<double>(q_WP_double / static_cast<double>(dx));
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
  using Grid = TypeParam;
  using T = typename Grid::Scalar;

  const double dx = 0.01;
  Grid grid(dx);
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

TYPED_TEST(SparseGridTest, ApplyGridToParticleKernel) {
  using Grid = TypeParam;
  using T = typename Grid::Scalar;
  using PadNodeType = typename Grid::PadNodeType;
  using PadDataType = typename Grid::PadDataType;

  ParticleData<T> particle_data;
  const Vector3<double> x0(0.01, 0.0, 0.0);
  const Vector3<double> x1(1.01, 0.0, 0.0);
  const std::vector<Vector3<double>> positions = {x0, x1};
  const double total_volume = 1.0;
  fem::DeformableBodyConfig<double> config;
  particle_data.AddParticles(positions, total_volume, config);
  const Vector3<T> test_velocity = Vector3<T>(1.0, 2.0, 3.0);
  for (Vector3<T>& v : particle_data.mutable_v()) {
    v = test_velocity;
  }
  /* The mass is divided equally among the two particles. */
  const T expected_particle_mass = config.mass_density() * total_volume / 2.0;

  const double dx = 0.01;
  Grid grid(dx);
  grid.Allocate(particle_data.x());
  /* Arbitrary g2p kernel that accumulates particle mass and velocity to the
   grid. */
  auto p2g_kernel = [&](int data_index, const PadNodeType&,
                        const ParticleData<T>& particles,
                        PadDataType* pad_data) {
    const Vector3<T>& vp = particles.v()[data_index];
    const T& mp = particles.m()[data_index];
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 3; ++k) {
          (*pad_data)[i][j][k].v += vp;
          (*pad_data)[i][j][k].m += mp;
        }
      }
    }
  };

  grid.ApplyParticleToGridKernel(particle_data, p2g_kernel);
  const std::vector<std::pair<Vector3<int>, GridData<T>>> grid_data =
      grid.GetGridData();
  for (const auto& pair : grid_data) {
    /* If the grid node is in the support of the particle, then the velocity
     should be the same as the particle velocity. Otherwise, it should be zero.
    */
    if (pair.second.m > 0.0) {
      EXPECT_TRUE(CompareMatrices(pair.second.v, test_velocity));
      EXPECT_EQ(pair.second.m, expected_particle_mass);
    } else {
      EXPECT_EQ(pair.second.v, Vector3<T>::Zero());
      EXPECT_EQ(pair.second.m, 0.0);
    }
  }

  /* Arbitrary g2p kernel that sets the particle velocity to be double the
   grid velocity average. */
  auto g2p_kernel = [](int data_index, const PadNodeType&,
                       const PadDataType& pad_data,
                       ParticleData<T>* particles) {
    Vector3<T>& vp = particles->mutable_v()[data_index];
    vp.setZero();
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 3; ++k) {
          vp += 2.0 * pad_data[i][j][k].v;
        }
      }
    }
    vp /= 27.0;
  };
  grid.ApplyGridToParticleKernel(&particle_data, g2p_kernel);
}

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
