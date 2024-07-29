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
#include "drake/common/parallelism.h"
#include "drake/multibody/mpm/math.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

/* GridData stores data at a single a grid node of SparseGrid.

 The Vector3<T> entry contains the velocity of the node (sometimes used
 temporarily to store the momentum of the node), and the scalar entry is mass of
 the node.

 The size of GridData is required to be a power of 2 to work with SPGrid.
 With T = float, GridData is 4 * 4 = 16 byte.
 With T = double, GridData is 8 * 4 = 32 byte.

 @tparam T double or float. */
template <typename T>
struct GridData {
  void set_zero() {
    v.setZero();
    m = 0.0;
  }

  bool operator==(const GridData<T>& other) const = default;

  Vector3<T> v{Vector3<T>::Zero()};
  T m{0.0};
};

/* A Pad is a 3x3x3 subgrid around a particle.

 We use quadratic B-spline kernel to compute the interaction weight between
 particles and a grid node, and the support of a single particle is 3x3x3 grid
 nodes. The Pad is a 3x3x3 grid that stores the grid data of the 3x3x3 neighbors
 of a grid node that is affected by a particle (or a group of particles that
 share the same support). */
template <typename T>
using Pad = std::array<std::array<std::array<T, 3>, 3>, 3>;

/* SparseGrid is a 3D grid that is sparsely populated with GridData implemented
 with a Sparse Paged Grid (SPGrid) data structure. Memory is allocated and used
 in chunks of pages (4KB in size).

 We define a few concepts related to the grid here:

 - A "block" is a subgrid with continuous memory of the size of a page.
 For 16 byte GridData (float), a block consists of 4 x 8 x 8 grid nodes. For 32
 byte GridData (double), a block consists of 4 x 4 x 8 grid nodes.

 - The "base node" of a particle is the center node if the 3x3x3 subgrid that
 affected by the particle.

 - A grid node is "active" if it has non-zero mass. That happens if and only if
 the grid node is in the support of a particle.

 - A block is only allocated if it contains active grid nodes.

 The 3D coordinate for each grid node is mapped to a unique 1D index (64 bit
 integer address) by SPGrid where the data along with the coordinate information
 is stored, and we call that 1D index the "offset" of the node following
 nomenclature from SPGrid.

 This class is used in close conjunction with ParticleData to transfer data
 between particles and the grid. A typical workflow is as follows:

 ```
   SparseGrid grid(dx);

   // `particles` is a ParticleData carrying the particle physical attributes.
   ...

   // Before interacting with the grid, always initialize the grid with the
   // particle positions.
   grid.Allocate(particles.x);

   // grid data is now ready to be used.
   grid.Foo();
   ...

   // If the particle data is modified, the grid must be updated before the next
   // interaction.
   particles.x[0] += Vector3d(0.1, 0.1, 0.1);
   grid.Allocate(particles.x);
   grid.Foo();
   ...
 ```

 @tparam float or double. */
template <typename T>
class SparseGrid {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SparseGrid);

  static constexpr int kDim = 3;
  /* The each block is assigned a unique color to facilitate lock-free particle
   to grid transfer. */
  static constexpr int kNumColors = 1 << kDim;

  /* Constructs a SparseGrid with grid spacing `dx` in meters. */
  explicit SparseGrid(T dx, Parallelism parallelism = false);

  /* Allocates memory for the grid pages affected by particles and initialize
   all grid data to zero.

   As a side effect, this function also orders the particles based on the
   "offset" of their base nodes. In the process, it builds `sentinel_particles`
   and `particle_indices` (see corresponding accessors below). */
  void Allocate(const std::vector<Vector3<T>>& q_WPs);

  /* All but last entry store indices of particles marking the boundary of a new
   block. The last entry stores the number of particles. This always has size
   num_blocks() + 1. */
  const std::vector<int>& sentinel_particles() const {
    return sentinel_particles_;
  }

  /* Index into ParticleData: particle_data[particle_indices()[p]] gives the
   particle data for particle p. */
  const std::vector<int>& data_indices() const { return data_indices_; }

  /* Returns the base node offset of each particle. */
  const std::vector<uint64_t>& base_node_offsets() const {
    return base_node_offsets_;
  }

  /* Grid spacing in meters. */
  const T& dx() const { return dx_; }

  /* The number of blocks that contain grid nodes which serve as base nodes for
   at least one particle. */
  int num_blocks() const { return blocks_->Get_Blocks().second; }

  /* Given the position of a particle in the world frame, returns the world
   frame positions of the grid node in its support. */
  Pad<Vector3<T>> GetPadNodes(const Vector3<T>& q_WP) const;

  /* Given the offset of a grid node, returns the grid data in the pad with the
   given node at the center.
   @pre All nodes in the requested pad are active. */
  Pad<GridData<T>> GetPadData(uint64_t center_node_offset) const;

  /* Given the offset of a grid node, writes the grid data in the pad with the
   given node at the center.
   @pre All nodes in the requested pad are active. */
  void SetPadData(uint64_t center_node_offset,
                  const Pad<GridData<T>>& pad_data);

  /* For each active grid node, divide by the grid node mass to convert the
   momentum to velocity and then increment the velocity by dv.
   @pre the grid data stores momentum of the node, not velocity. */
  void ExplicitVelocityUpdate(const Vector3<T>& dv);

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

  /* Sets the grid state with the given callback function that maps the world
   space coordinate of the grid node to the data at that node.
   @pre The callback function only assigns grid data to active grid nodes.
   @note Testing only. */
  void SetGridData(
      const std::function<GridData<T>(const Vector3<int>&)>& callback);

  /* Returns grid data from all active grid nodes as a pair of world space
   coordinate and grid data of the node.
   @note Testing only. */
  std::vector<std::pair<Vector3<int>, GridData<T>>> GetGridData() const;

  /* Computes the mass, linear momentum, and angular momentum (about world
   origin) on the grid.
   @note This function assumes that the grid data stores the mass and velocity
   (instead of mass and momemtum) of grid nodes.
   @note Testing only. */
  MassAndMomentum<T> ComputeTotalMassAndMomentum() const;

  /* We color that blocks so that writing to different blocks with the same
  color is guaranteed to be free of write hazards. This function returns the
  block indices for each color. */
  const std::array<std::vector<int>, kNumColors>& colored_blocks() const {
    return colored_blocks_;
  }

 private:
  static constexpr int kLog2Page = 12;  // 4KB page size.
  /* The maximum grid size along a single dimension. That is even
   though the grid is sparsely populated, the maximum grid size
   that can ever be allocated is kMaxGridSize^3. With 1cm grid dx, that
   corresponds to more than 10 meters in each dimension, which should be
   enough for most manipulation simulations. */
  static constexpr int kLog2MaxGridSize = 10;
  static constexpr int kMaxGridSize = 1 << kLog2MaxGridSize;

  /* Returns the color of the block given the page bits of the node offset.
   According to [Setaluri et al. 2014], the blocks are arranged in a 3D space
   using Z-order curve. Blocks with 8 blocks apart are guaranteed to be
   non-adjacent. */
  static int get_color(uint64_t page) {
    int color = (page & (kNumColors - 1));
    DRAKE_ASSERT(color >= 0 && color < 8);
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

  /* Helper for `Allocate()` that sorts particles based on their positions. In
   that process, builds `data_indices_` and `sentinel_particles_`. */
  void SortParticleIndices(const std::vector<Vector3<T>>& q_WPs);

  /* Grid spacing (in meters). */
  T dx_{};
  /* SPGrid allocator. */
  std::unique_ptr<Allocator> allocator_;
  /* Blocks containing grid nodes that are base nodes for at least one particle.
   */
  std::unique_ptr<PageMap> blocks_;
  /* Blocks containing all active grid nodes. These are the blocks that are
   actually allocated. */
  std::unique_ptr<PageMap> padded_blocks_;

  // TODO(xuchenhan-tri): Allow moving the maximumly allowed grid around the
  // center of the objects so that the grid can be accommodated to the objects
  // that are translating.
  /* 3D coordinates in SPGrid starts at (i, j, k) = (0, 0, 0) with i, j, k
   always non-negative. We want the grid to center around (0, 0, 0) in world
   space so we shift the origin by
   (kMaxGridSize/2, kMaxGridSize/2, kMaxGridSize/2)*/
  const uint64_t origin_offset_{Mask::Linear_Offset(
      kMaxGridSize / 2, kMaxGridSize / 2, kMaxGridSize / 2)};

  /* See accessors. */
  std::vector<int> sentinel_particles_;
  std::vector<int> data_indices_;
  std::vector<uint64_t> base_node_offsets_;

  /* Helper data to sort the particles according to their base nodes. */
  std::vector<uint64_t> particle_sorters_;

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
  std::array<std::array<std::array<uint64_t, 3>, 3>, 3> cell_offset_strides_;

  std::array<std::vector<int>, kNumColors> colored_blocks_;
  Parallelism parallelism_;
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
