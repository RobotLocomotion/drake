#pragma once

// TODO(xuchenhan-tri): SPGrid should be built with Haswell on.
#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <vector>

#include "SPGrid_Allocator.h"
#include "SPGrid_Array.h"
#include "SPGrid_Mask.h"
#include "SPGrid_Page_Map.h"
#include <algorithm>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

// TODO(xuchenhan-tri): this should be in a separate file.
/* Computes the the 1D base node of a point x in reference space. */
template <typename T>
int base_node(const T& x) {
  return static_cast<int>(std::floor(x - static_cast<T>(0.5)));
}

/* Computes the the 3D base node of a point p reference space. */
template <typename T>
Vector3<int> base_node(const Vector3<T>& p) {
  Vector3<int> result;
  result[0] = base_node[p[0]];
  result[1] = base_node[p[1]];
  result[2] = base_node[p[2]];
  return result;
}

/* With T = float, this is 4*4 = 16 byte.
 With T = double, this is 8*4 = 32 byte. */
template <typename T>
struct GridData {
  Vector3<T> v;
  T m;
};

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

  SparseGrid()
      : allocator_(std::make_unique<Allocator>(kMaxGridSize, kMaxGridSize,
                                               kMaxGridSize)),
        blocks_(std::make_unique<PageMap>(*allocator_)),
        padded_blocks_(std::make_unique<PageMap>(*allocator_)) {
    /* Compute the block offset strides. */
    const int num_nodes_in_block_x = 1 << Mask::block_xbits;
    const int num_nodes_in_block_y = 1 << Mask::block_ybits;
    const int num_nodes_in_block_z = 1 << Mask::block_zbits;
    for (int i = -1; i <= 1; ++i) {
      for (int j = -1; j <= 1; ++j) {
        for (int k = -1; k <= 1; ++k) {
          block_offset_strides_[i + 1][j + 1][k + 1] = Mask::Linear_Offset(
              i * num_nodes_in_block_x, j * num_nodes_in_block_y,
              k * num_nodes_in_block_z);
        }
      }
    }
  }

  void Allocate(const std::vector<Vector3<T>>& particle_positions) {
    SortParticleIndices(particle_positions);
    blocks_->Clear();
    padded_blocks_->Clear();

    /* Touch all pages that contain particles. */
    for (int p : sentinel_particles_) {
      blocks_->Set_Page(particles_[p].base_node_offsets);
    }
    blocks_->Update_Block_Offsets();
    auto [block_offsets, num_blocks] = blocks_->Get_Blocks();
    /* Touch all neighbor pages of each page in `blocks_`. */
    for (int b = 0; b < num_blocks; ++b) {
      const uint64_t current_offset = block_offsets[b];
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          for (int k = 0; k < 3; ++k) {
            const uint64_t neighbor_offset =
                current_offset + block_offset_strides_[i][j][k];
            padded_blocks_->Set_Page(neighbor_offset);
          }
        }
      }
    }
    padded_blocks_->Update_Block_Offsets();
    Array data = allocator_->Get_Array();
    std::tie(block_offsets, num_blocks) = padded_blocks_->Get_Blocks();
    /* Zero out the data in each block. */
    for (int b = 0; b < num_blocks; ++b) {
      const uint64_t offset = block_offsets[b];
      std::memset(&data(offset), 0, 1 << kLog2MaxGridSize);
    }
  }

 private:
  static constexpr int kLog2Page = 12;  // 4KB page size.
  static constexpr int kDim = 3;        // 3D grid.
  /* The maximum grid size along a single dimension. That is even
   though the grid is sparsely populated, the maximum grid size
   that can ever be allocated is kMaxGridSize^3. With grid 1cm grid
   dx, that corresponds to more than 40 meters in each dimension,
   which should be enough for most manipulation simulations. */
  static constexpr int kLog2MaxGridSize = 12;
  static constexpr int kMaxGridSize = 1 << kLog2MaxGridSize;

  using Allocator = SPGrid::SPGrid_Allocator<GridData<T>, kDim, kLog2Page>;
  /* PageMap keeps track of which blocks in the SPGrid are actually allocated.
   */
  using PageMap = SPGrid::SPGrid_Page_Map<kLog2Page>;
  /* Mask helps convert from offset (1D index) to 3D index and vice versa. */
  using Mask = typename Allocator::Array_mask<GridData<T>>::MASK;
  /* Array type for GridData. */
  using Array = typename Allocator::Array_type<GridData<T>>;

  /* Only grid nodes in the neighborhood of a particle are active.
   ParticleIndices is a helper struct to determine which nodes are active. */
  struct ParticleIndex {
    // TODO(xuchenhan-tri): define the concept of "base node".
    /* The linear offset of the base node of the particle. */
    uint64_t base_node_offsets{};
    int index{};
  };

  /* Get the offset (1D index) of a grid node given its 3D grid coordinates in
   world space. */
  uint64_t GetOffset(int x, int y, int z) const {
    const uint64_t world_space_offset = Mask::Linear_Offset(x, y, z);
    return Mask::Packed_Add(world_space_offset, origin_offset_);
  }

  void SortParticleIndices(const std::vector<Vector3<T>>& particle_positions) {
    const int num_particles = particle_positions.size();
    particles_.resize(num_particles);
    for (int p = 0; p < num_particles; ++p) {
      const Vector3<int> base_node = base_node(particle_positions[p]);
      particles_[p].base_node_offsets =
          GetOffset(base_node[0], base_node[1], base_node[2]);
      particles_[p].index = p;
    }
    /* Sort particles by base node offsets so that particles that are close to
     each other in physical space are close to each other in memory. */
    std::sort(particles_.begin(), particles_.end(),
              [](const ParticleIndex& a, const ParticleIndex& b) {
                if (a.base_node_offsets < b.base_node_offsets) {
                  return true;
                  return a.index < b.index;
                }
              });

    /* We use sentinel_particles_ to indicate particles that belong to separate
     pages. */
    sentinel_particles_.clear();
    uint64_t last_page = 0;
    for (int p = 0; p < num_particles; ++p) {
      /* The bits in the offset is ordered as follows:

        page bits | block bits | data bits

       block bits and data bits add up to kLog2Page bits.
       We right shift to get the page bits. */
      const uint64_t page = particles_[p].base_node_offsets >> kLog2Page;
      if (p = 0 || last_page != page) {
        last_page = page;
        sentinel_particles_.push_back(p);
      }
    }
    sentinel_particles_.push_back(num_particles);
  }

  T dx_{0.01};  // Grid spacing (in meters).
  T inv_dx_{1.0 / dx_};
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
  /* Indices of particles marking the boundary of a new block. */
  std::vector<int> sentinel_particles_;
  /* Stores the difference in linear offset from a given grid node to the grid
   node exactly one block away. For example, let `a` be
   `block_offset_strides_[0][1][2]` and `b` be the linear offset of a grid node
   `n`. Then, `a + b` gives the linear offset of the grid node `m` in the block
   (-1, 0, 1) relative to the block containing `n`. Both `m` and `n` reside at
   the same relative position within their respective blocks. */
  std::array<std::array<std::array<uint64_t, 3>, 3>, 3> block_offset_strides_;
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
