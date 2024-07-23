#pragma once

// TODO(xuchenhan-tri): SPGrid should be built with Haswell on.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <vector>

#include "SPGrid_Allocator.h"
#include "SPGrid_Array.h"
#include "SPGrid_Mask.h"
#include "SPGrid_Page_Map.h"

#include "drake/common/eigen_types.h"
#include "drake/multibody/mpm/math.h"
#include "particles.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

/* With T = float, this is 4*4 = 16 byte.
 With T = double, this is 8*4 = 32 byte. */
template <typename T>
struct GridData {
  void set_zero() {
    v.setZero();
    m = 0.0;
  }

  bool operator==(const GridData<T>& other) const {
    return v == other.v && m == other.m;
  }

  Vector3<T> v{Vector3<T>::Zero()};
  T m{0.0};
};

/* Only grid nodes in the neighborhood of a particle are active.
 ParticleIndices is a helper struct to determine which nodes are active. */
struct ParticleIndex {
  // TODO(xuchenhan-tri): define the concept of "base node" in the
  // documentation somewhere.
  /* The linear offset of the base node of the particle. */
  uint64_t base_node_offset{};
  /* Index into particle data. */
  int index{};
};

template <typename T>
using NeighborArray = std::array<std::array<std::array<T, 3>, 3>, 3>;

/* SparseGrid is a 3D grid that is sparsely populated with GridData implemented
 with a Sparse Paged Grid (SPGrid) data structure. Memory is allocated and used
 in chunks of pages (usually 4KB in size). For 16 byte GridData, each page is
 equivalent to a 4 x 8 x 8 block in physical space. For 32 byte GridData, each
 page is 4 x 8 x 4 block. Memory for each block is continuous. Only the 27
 neighbor grid nodes of a particle has meaningful grid data, and we call these
 grid nodes "active". The 3D coordinate for each grid node is mapped to a unique
 1D index (64 bit integer address) by SPGrid where the data along with the
 coordinate information is stored, and we call that 1D index the offset of the
 node following nomenclature from SPGrid. */
template <typename T>
class SparseGrid {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SparseGrid);

  /* Constructs a SparseGrid with grid spacing `dx` in meters. */
  explicit SparseGrid(T dx);

  /* Allocates memory for the grid pages affected by particles and initialize
   all grid data to zero. */
  void Allocate(ParticleData<T>* particles);

  /* All but last entry store indices of particles marking the boundary of a new
   block. The last entry stores the number of particles. */
  const std::vector<int>& sentinel_particles() const {
    return sentinel_particles_;
  }

  const std::vector<ParticleIndex>& particle_indices() const {
    return particles_;
  }

  const T& dx() const { return dx_; }

  /* The number of blocks that contain particles. */
  int num_blocks() const { return blocks_->Get_Blocks().second; }

  /* Given the position x in the world frame, returns the grid node of the 3x3x3
   neighbors of the base node associated with x. */
  NeighborArray<Vector3<T>> GetNeighborNodes(const Vector3<T>& x) const;

  /* Given the offset of a grid node, returns the grid data in its 3x3x3
   neighbors. */
  NeighborArray<GridData<T>> GetNeighborData(uint64_t base_node_offset) const;

  /* Given the offset of a grid node, writes the grid data in its 3x3x3
   neighbors to the sparse grid. */
  void SetNeighborData(uint64_t base_node_offset,
                       const NeighborArray<GridData<T>>& data);
  
  void ExplicitVelocityUpdate(const T& dt, const Vector3<T>& gravity) {
    const uint64_t page_size = 1 << kLog2Page;
    const uint64_t data_size = sizeof(GridData<T>);
    auto [block_offsets, num_blocks] = padded_blocks_->Get_Blocks();
    Array grid_data = allocator_->Get_Array();
    for (int b = 0; b < static_cast<int>(num_blocks); ++b) {
      const uint64_t block_offset = block_offsets[b];
      for (uint64_t i = 0; i < page_size; i += data_size) {
        const uint64_t node_offset = block_offset + i;
        GridData<T>& node_data = grid_data(node_offset);
        if (node_data.m > 0.0) {
          node_data.v /= node_data.m;
          node_data.v += gravity * dt;
        }
      }
    }
  }

  /* Returns the offset (1D index) of a grid node given its 3D grid coordinates
   in world space. */
  uint64_t CoordinateToOffset(int x, int y, int z) const {
    const uint64_t world_space_offset = Mask::Linear_Offset(x, y, z);
    return Mask::Packed_Add(world_space_offset, origin_offset_);
  }

  /* Returns the 3D grid coordinates in world space given the offset (1D index)
   of a grid node.
   @note Testing only. */
  Vector3<int> OffsetToCoordinate(uint64_t offset) const;

  /* Sets the grid state with the given callback function.
   @note Testing only. */
  void SetGridState(
      const std::function<GridData<T>(const Vector3<T>&)>& callback);

  /* Gets grid data from all active grid nodes (i.e. nodes with non-zero mass).
   @note Testing only. */
  std::vector<std::pair<Vector3<int>, GridData<T>>> GetGridData() const {
    const uint64_t page_size = 1 << kLog2Page;
    const uint64_t data_size = sizeof(GridData<T>);
    std::vector<std::pair<Vector3<int>, GridData<T>>> result;
    auto [block_offsets, num_blocks] = padded_blocks_->Get_Blocks();
    ConstArray grid_data = allocator_->Get_Const_Array();
    for (int b = 0; b < static_cast<int>(num_blocks); ++b) {
      const uint64_t block_offset = block_offsets[b];
      for (uint64_t i = 0; i < page_size; i += data_size) {
        const uint64_t node_offset = block_offset + i;
        const GridData<T>& node_data = grid_data(node_offset);
        if (node_data.m > 0.0) {
          const Vector3<int> node_coordinate = OffsetToCoordinate(node_offset);
          result.emplace_back(std::make_pair(node_coordinate, node_data));
        }
      }
    }
    return result;
  }

  /* Computes the mass, linear momentum, and angular momentum (around world
   origin) on the grid.
   @note Testing only. */
  MassAndMomentum<T> ComputeTotalMassAndMomentum() const {
    MassAndMomentum<T> result;
    const std::vector<std::pair<Vector3<int>, GridData<T>>> grid_data =
        GetGridData();
    for (int i = 0; i < ssize(grid_data); ++i) {
      const T& mi = grid_data[i].second.m;
      const Vector3<T>& vi = grid_data[i].second.v;
      const Vector3<T>& xi = grid_data[i].first.template cast<T>() * dx_;
      result.mass += grid_data[i].second.m;
      result.linear_momentum += mi * vi;
      result.angular_momentum += mi * xi.cross(vi);
    }
    return result;
  }

  const std::array<std::vector<int>, 8>& colored_pages() const {
    return colored_pages_;
  }

 private:
  static constexpr int kLog2Page = 12;  // 4KB page size.
  static constexpr int kDim = 3;        // 3D grid.
  /* The maximum grid size along a single dimension. That is even
   though the grid is sparsely populated, the maximum grid size
   that can ever be allocated is kMaxGridSize^3. With 1cm grid dx, that
   corresponds to more than 40 meters in each dimension, which should be
   enough for most manipulation simulations. */
  static constexpr int kLog2MaxGridSize = 10;
  static constexpr int kMaxGridSize = 1 << kLog2MaxGridSize;
  static constexpr int kIndexBits = 64 - 3 * kLog2MaxGridSize;

  static int get_color(uint64_t page) {
    int color = (page & 0x7);
    DRAKE_DEMAND(color >= 0 && color < 8);
    return color;
  }

  using Allocator = SPGrid::SPGrid_Allocator<GridData<T>, kDim, kLog2Page>;
  /* PageMap keeps track of which blocks in the SPGrid are actually allocated.
   */
  using PageMap = SPGrid::SPGrid_Page_Map<kLog2Page>;
  /* Mask helps convert from offset (1D index) to 3D index and vice versa. */
  using Mask = typename Allocator::Array_mask<GridData<T>>;
  /* Array type for GridData. */
  using Array = typename Allocator::Array_type<GridData<T>>;
  using ConstArray = typename Allocator::Array_type<const GridData<T>>;
  static constexpr int kDataBits = Mask::data_bits;

  /* Helper for `Allocate()` that sorts particles into bins based on their
   positions. In that process, builds `partilces_` and `sentinel_particles_`.
  */
  void SortParticleIndices(ParticleData<T>* particles);
  void Sort(std::vector<uint64_t>* sorter,
            std::vector<ParticleIndex>* particles);

  T dx_{};  // Grid spacing (in meters).
  std::unique_ptr<Allocator> allocator_;
  std::unique_ptr<PageMap> blocks_;  // Helper for computing `padded_blocks_`.
  std::unique_ptr<PageMap>
      padded_blocks_;  // Blocks that are actually allocated.
  /* 3D coordinates in SPGrid starts at (i, j, k) = (0, 0, 0) with i, j, k
   always non-negative. We want the grid to center around (0, 0, 0) in world
   space so we shift the origin by
   (kMaxGridSize/2, kMaxGridSize/2, kMaxGridSize/2)*/
  const uint64_t origin_offset_{Mask::Linear_Offset(
      kMaxGridSize / 2, kMaxGridSize / 2, kMaxGridSize / 2)};

  std::vector<ParticleIndex> particles_;
  std::vector<uint64_t> particle_sorters_;
  std::vector<int> sentinel_particles_;
  /* Stores the difference in linear offset from a given grid node to the grid
   node exactly one block away. For example, let `a` be
   `block_offset_strides_[0][1][2]` and `b` be the linear offset of a grid
   node `n`. Then, `a + b` gives the linear offset of the grid node `m` in the
   block
   (-1, 0, 1) relative to the block containing `n`. Both `m` and `n` reside at
   the same relative position within their respective blocks. */
  std::array<std::array<std::array<uint64_t, 3>, 3>, 3> block_offset_strides_;
  /* Similar to block_offset_strides_, but instead of providing strides for
   nodes a "block" away, provides the strides for nodes a "cell" away (i.e.
   immediate grid neighbors). */
  std::array<std::array<std::array<uint64_t, 3>, 3>, 3>
      neighbor_offset_strides_;

  std::array<std::vector<int>, 1 << kDim> colored_pages_;
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
