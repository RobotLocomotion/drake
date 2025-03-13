#include "drake/multibody/mpm/particle_sorter.h"

#include <numeric>

#include <gtest/gtest.h>

#include "drake/multibody/mpm/bspline_weights.h"
#include "drake/multibody/mpm/grid_data.h"
#include "drake/multibody/mpm/spgrid.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

using Eigen::Vector3i;

GTEST_TEST(ConvertToRangeVectorTest, ConvertToRangeVector) {
  std::vector<int> data = {1, 3, 6, 9};
  RangeVector ranges;
  ConvertToRangeVector(data, &ranges);
  ASSERT_EQ(ranges.size(), 3);
  EXPECT_EQ(ranges[0].start(), 1);
  EXPECT_EQ(ranges[0].end(), 3);
  EXPECT_EQ(ranges[1].start(), 3);
  EXPECT_EQ(ranges[1].end(), 6);
  EXPECT_EQ(ranges[2].start(), 6);
  EXPECT_EQ(ranges[2].end(), 9);
}

/* Sample n^3 points in the box [0, 1] x [0, 1] x [0, 1]*/
std::vector<Vector3<double>> SamplePoints(int n) {
  std::vector<Vector3<double>> points;
  for (int i = 0; i < n; ++i) {
    /* Multiply by a large prime number and mod by n to avoid the sampled points
     to be ordered in any obvious way. */
    int ii = (17 * i) % n;
    for (int j = 0; j < n; ++j) {
      int jj = (23 * j) % n;
      for (int k = 0; k < n; ++k) {
        int kk = (29 * k) % n;
        points.push_back(
            Vector3<double>(ii / (n - 1.0), jj / (n - 1.0), kk / (n - 1.0)));
      }
    }
  }
  return points;
}

/* Returns true if the one-rings of the two given grid nodes overlap. */
bool OverlapOneRing(const Vector3i& a, const Vector3i& b) {
  return ((a - b).cwiseAbs().maxCoeff() <= 2);
}

GTEST_TEST(ParticleSorterTest, Sort) {
  using Grid = SpGrid<GridData<double>>;
  const Grid spgrid;
  const double dx = 0.2;
  const std::vector<Vector3<double>> particle_positions = SamplePoints(7);

  ParticleSorter sorter;
  sorter.Sort(spgrid, dx, particle_positions);
  const std::vector<int>& data_indices = sorter.data_indices();
  const std::vector<uint64_t>& base_node_offsets = sorter.base_node_offsets();
  for (int i = 1; i < ssize(data_indices); ++i) {
    /* Confirm that after the sort, the base nodes are in ascending order. */
    EXPECT_LE(base_node_offsets[i - 1], base_node_offsets[i]);
    /* When base nodes are tied, confirm that indices are used to break tie. */
    if (base_node_offsets[i - 1] == base_node_offsets[i]) {
      EXPECT_LT(data_indices[i - 1], data_indices[i]);
    }
  }

  /* Confirm that the sorted base nodes are indeed a permutation of the original
   base nodes. */
  for (int i = 0; i < ssize(particle_positions); ++i) {
    const Vector3<double>& particle_position =
        particle_positions[data_indices[i]];
    const Vector3i expected_base_node =
        ComputeBaseNode<double>(particle_position / dx);
    const uint64_t expected_base_node_offset =
        spgrid.CoordinateToOffset(expected_base_node);
    EXPECT_EQ(base_node_offsets[i], expected_base_node_offset);
  }

  /* Confirm that the data indices are indeed a permutation of the original
   indices. */
  std::vector<int> data_indices_copy = data_indices;
  std::sort(data_indices_copy.begin(), data_indices_copy.end());
  std::vector<int> expected_sorted_indices(ssize(particle_positions));
  std::iota(expected_sorted_indices.begin(), expected_sorted_indices.end(), 0);
  EXPECT_EQ(data_indices_copy, expected_sorted_indices);

  /* Confirm that the colored ranges are indeed write-hazard-free. */
  const std::array<RangeVector, 8>& colored_ranges = sorter.colored_ranges();
  for (int c = 0; c < 8; ++c) {
    const RangeVector& ranges = colored_ranges[c];
    for (int i = 0; i < ssize(ranges); ++i) {
      for (int j = i + 1; j < ssize(ranges); ++j) {
        const Range& range_i = ranges[i];
        const Range& range_j = ranges[j];
        for (int p = range_i.start(); p < range_i.end(); ++p) {
          for (int q = range_j.start(); q < range_j.end(); ++q) {
            const Vector3i base_node_p =
                spgrid.OffsetToCoordinate(base_node_offsets[p]);
            const Vector3i base_node_q =
                spgrid.OffsetToCoordinate(base_node_offsets[q]);
            /* As long as one-rings of the base nodes of the particles don't
             overlap, there's no write-hazard. */
            EXPECT_FALSE(OverlapOneRing(base_node_p, base_node_q));
          }
        }
      }
    }
  }

  /* Confirm that GetActiveBlockOffsets() is correct. To do that, we allocate
   another SpGrid with the returned offsets and then check that all base nodes
   to the particles, as well as their one rings, are allocated. (That's the
   required allocation for the transfer.) */
  const std::vector<uint64_t> active_block_offsets =
      sorter.GetActiveBlockOffsets();
  Grid another_grid;
  another_grid.Allocate(active_block_offsets);
  std::set<uint64_t> allocated_offsets;
  auto callback = [&](uint64_t offset, const GridData<double>&) {
    allocated_offsets.insert(offset);
  };

  another_grid.IterateConstGridWithOffset(callback);
  for (uint64_t offset : base_node_offsets) {
    const Vector3i coord = another_grid.OffsetToCoordinate(offset);
    /* Get the offsets of nodes in the one-ring of the base node. */
    for (int i = -1; i <= 1; ++i) {
      for (int j = -1; j <= 1; ++j) {
        for (int k = -1; k <= 1; ++k) {
          Vector3i neighbor = coord + Vector3i(i, j, k);
          uint64_t neighbor_offset = another_grid.CoordinateToOffset(neighbor);
          EXPECT_TRUE(allocated_offsets.contains(neighbor_offset));
        }
      }
    }
  }
}

}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
