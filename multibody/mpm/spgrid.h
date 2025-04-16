#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include <SPGrid_Allocator.h>
#include <SPGrid_Array.h>
#include <SPGrid_Mask.h>
#include <SPGrid_Page_Map.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/mpm/spgrid_flags.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

/* A Pad is a 3x3x3 subgrid. */
template <typename T>
using Pad = std::array<std::array<std::array<T, 3>, 3>, 3>;

/* SpGrid is a wrapper class around the SPGrid library designed for managing
 sparse grid data in 3D space.

 The SPGrid library provides a memory-efficient and high-performance way of
 storing and accessing sparsely populated unit grid data in 3D. It maps 3D array
 data to a linear memory span. Lexicographical order is used to traverse the
 interior of fixed-sized blocks (4kB), while the blocks themselves are laid out
 along a space-filling Morton curve.

 Please refer to [Setaluri et al. 2014] for more details on the SPGrid library.

 This class (SpGrid) simplifies the use of SPGrid by offering convenient methods
 to allocate, access, and manage sparse grid data.

 The concept of "offset" in SPGrid is a 1D index that represents a 3D coordinate
 in the grid. The offset is calculated using the Morton curve, which interleaves
 the bits of the 3D coordinates to form a single 1D index. The offset is used to
 access the data stored in the grid. The conversion between 3D coordinates and
 offsets is provided as a utility, but the conversion has a non-trivial cost
 when used in a tight loop. Therefore, stream the data in the order of the
 offsets whenever possible. Streaming data in the order of offsets is also more
 memory-friendly.

 [Setaluri, 2014] Setaluri, Rajsekhar, et al. "SPGrid: A sparse paged grid
 structure applied to adaptive smoke simulation." ACM Transactions on Graphics
 (TOG) 33.6 (2014): 1-12.

 @note The implementation of this class has many triple-nested loops. We write
 them out explicitly (instead of using a sugar like IterateGrid()) because they
 are called in performance-critical inner loops and can potentially be inlined.

 @tparam GridData The type of data stored in the grid. It must satisfy the
 following requirements:
  - It must have a default constructor.
  - It must have a member function `reset()`.
  - Its size must be less than or equal to 4KB. */
// TODO(xuchenhan-tri): Enumerate all possible GridData so that we can move
// the implementation to the .cc file.
template <typename GridData>
class SpGrid {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpGrid);

  static constexpr int kDim = 3;
  static constexpr int kLog2Page = 12;  // 4KB page size.
  static_assert(std::is_default_constructible<GridData>::value,
                "GridData must be default constructible.");
  static_assert(sizeof(GridData) <= (1 << 12),
                "GridData must be less than or equal to 4KB.");
  using Allocator = SPGrid::SPGrid_Allocator<GridData, kDim, kLog2Page>;
  /* PageMap keeps track of which blocks in the SPGrid are allocated. */
  using PageMap = SPGrid::SPGrid_Page_Map<kLog2Page>;
  /* Mask helps convert from offset (1D index) to 3D index and vice versa. */
  using Mask = typename Allocator::template Array_mask<GridData>;
  /* Array type for GridData. */
  using Array = typename Allocator::template Array_type<GridData>;
  using ConstArray = typename Allocator::template Array_type<const GridData>;
  using Offset = uint64_t;

  /* Creates an empty SpGrid without allocating any memory. Call `Allocate()` to
   allocate memory for grid data. A typical use pattern is constructing an
   SpGrid object once, and repeatedly call `Allocate()` to allocate (as needed)
   and reset the grid data. */
  SpGrid()
      : allocator_(kMaxGridSize, kMaxGridSize, kMaxGridSize),
        helper_blocks_(allocator_),
        blocks_(allocator_) {
    /* Compute the cell offset strides. */
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 3; ++k) {
          cell_offset_strides_[i][j][k] =
              Mask::Linear_Offset(i - 1, j - 1, k - 1);
        }
      }
    }
    /* Compute the block offset strides. */
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 3; ++k) {
          block_offset_strides_[i][j][k] = Mask::Linear_Offset(
              (i - 1) * kNumNodesInBlockX, (j - 1) * kNumNodesInBlockY,
              (k - 1) * kNumNodesInBlockZ);
        }
      }
    }
  }

  /* Makes `this` an exact copy of the `other` SpGrid. */
  void SetFrom(const SpGrid<GridData>& other) {
    /* Copy over the page maps. */
    helper_blocks_.Clear();
    auto [block_offsets, num_blocks] = other.helper_blocks_.Get_Blocks();
    for (int b = 0; b < static_cast<int>(num_blocks); ++b) {
      helper_blocks_.Set_Page(block_offsets[b]);
    }
    helper_blocks_.Update_Block_Offsets();

    blocks_.Clear();
    std::tie(block_offsets, num_blocks) = other.blocks_.Get_Blocks();
    for (int b = 0; b < static_cast<int>(num_blocks); ++b) {
      blocks_.Set_Page(block_offsets[b]);
    }
    blocks_.Update_Block_Offsets();
    /* Copy over the data. */
    IterateGridWithOffset([&](uint64_t offset, GridData* node_data) {
      *node_data = other.get_data(offset);
    });

    block_offset_strides_ = other.block_offset_strides_;
    cell_offset_strides_ = other.cell_offset_strides_;
  }

  /* Clears this SpGrid and ensures that:

   1. All blocks containing the specified offsets have their GridData allocated
      and default-initialized.
   2. Every block in the one-ring surrounding those blocks is also allocated and
      default-initialized.

   @note Actual allocation only happens the first time a block is used in the
   lifetime of an SpGrid. Subsequent calls to `Allocate()` touching the same
   block only resets the grid data to default values. */
  void Allocate(const std::vector<Offset>& offsets) {
    helper_blocks_.Clear();
    for (const Offset& offset : offsets) {
      helper_blocks_.Set_Page(offset);
    }
    helper_blocks_.Update_Block_Offsets();

    auto [block_offsets, num_blocks] = helper_blocks_.Get_Blocks();
    /* Touch all neighboring blocks of each block in `helper_blocks_` to
     ensure all grid nodes in the one ring of the given offsets have memory
     allocated. */
    for (int b = 0; b < static_cast<int>(num_blocks); ++b) {
      const uint64_t current_offset = block_offsets[b];
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          for (int k = 0; k < 3; ++k) {
            const uint64_t neighbor_block_offset = Mask::Packed_Add(
                current_offset, block_offset_strides_[i][j][k]);
            blocks_.Set_Page(neighbor_block_offset);
          }
        }
      }
    }
    blocks_.Update_Block_Offsets();
    IterateGrid([](GridData* node_data) {
      node_data->reset();
    });
  }

  /* Returns the offset (1D index) of a grid node given its 3D grid
   coordinates in world space. */
  Offset CoordinateToOffset(int x, int y, int z) const {
    const uint64_t world_space_offset = Mask::Linear_Offset(x, y, z);
    return Mask::Packed_Add(world_space_offset, origin_offset_);
  }
  Offset CoordinateToOffset(const Vector3<int>& coord) const {
    return CoordinateToOffset(coord[0], coord[1], coord[2]);
  }

  /* Returns the 3D grid coordinates in world space given the offset (1D
   index) of a grid node.
   @note This function is not particularly efficient. Do not use it in
   computationally intensive inner loops. Instead, prefer accessing grid data
   directly using the offsets. */
  Vector3<int> OffsetToCoordinate(uint64_t offset) const {
    const std::array<int, 3> reference_space_coordinate =
        Mask::LinearToCoord(offset);
    const std::array<int, 3> reference_space_origin =
        Mask::LinearToCoord(origin_offset_);
    return Vector3<int>(
        reference_space_coordinate[0] - reference_space_origin[0],
        reference_space_coordinate[1] - reference_space_origin[1],
        reference_space_coordinate[2] - reference_space_origin[2]);
  }

  /* Returns the number of blocks that contain the offsets passed in the last
   call to `Allocate()`. This is the quantity that's often useful in MPM
   transfers, and it is equal to the number of blocks in `helper_blocks_`. */
  int num_blocks() const { return helper_blocks_.Get_Blocks().second; }

  /* Given the offset of a grid node, returns the grid data in the pad with
   the given node at the center.
   @pre All nodes in the requested pad are active. */
  Pad<GridData> GetPadData(uint64_t center_node_offset) const {
    Pad<GridData> result;
    ConstArray data = allocator_.Get_Const_Array();
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 3; ++k) {
          const uint64_t offset = Mask::Packed_Add(
              center_node_offset, cell_offset_strides_[i][j][k]);
          result[i][j][k] = data(offset);
        }
      }
    }
    return result;
  }

  /* Given the offset of a grid node, writes the grid data in the pad with the
   given node at the center.
   @pre All nodes in the requested pad are active. */
  void SetPadData(uint64_t center_node_offset, const Pad<GridData>& pad_data) {
    Array grid_data = allocator_.Get_Array();
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 3; ++k) {
          const uint64_t offset = Mask::Packed_Add(
              center_node_offset, cell_offset_strides_[i][j][k]);
          grid_data(offset) = pad_data[i][j][k];
        }
      }
    }
  }

  // TODO(xuchenhan-tri): Add a parallel version of IterateGrid.
  /* Iterates over all grid nodes in the grid and applies the given function
   `func` to each grid node. */
  void IterateGrid(const std::function<void(GridData*)>& func) {
    const uint64_t data_size = 1 << kDataBits;
    Array grid_data = allocator_.Get_Array();
    auto [block_offsets, num_blocks] = blocks_.Get_Blocks();
    for (int b = 0; b < static_cast<int>(num_blocks); ++b) {
      const uint64_t block_offset = block_offsets[b];
      uint64_t node_offset = block_offset;
      /* The coordinate of the origin of this block. */
      for (int i = 0; i < kNumNodesInBlockX; ++i) {
        for (int j = 0; j < kNumNodesInBlockY; ++j) {
          for (int k = 0; k < kNumNodesInBlockZ; ++k) {
            GridData& node_data = grid_data(node_offset);
            func(&node_data);
            node_offset += data_size;
          }
        }
      }
    }
  }

  /* Iterates over all grid nodes in the grid and applies the given function
   `func` to each grid node. */
  void IterateGrid(const std::function<void(const GridData&)>& func) const {
    const uint64_t data_size = 1 << kDataBits;
    ConstArray grid_data = allocator_.Get_Array();
    auto [block_offsets, num_blocks] = blocks_.Get_Blocks();
    for (int b = 0; b < static_cast<int>(num_blocks); ++b) {
      const uint64_t block_offset = block_offsets[b];
      uint64_t node_offset = block_offset;
      /* The coordinate of the origin of this block. */
      for (int i = 0; i < kNumNodesInBlockX; ++i) {
        for (int j = 0; j < kNumNodesInBlockY; ++j) {
          for (int k = 0; k < kNumNodesInBlockZ; ++k) {
            const GridData& node_data = grid_data(node_offset);
            func(node_data);
            node_offset += data_size;
          }
        }
      }
    }
  }

  /* Iterates over all grid nodes in the grid and applies the given function
   `func` to each grid node's offset and data. */
  void IterateGridWithOffset(
      const std::function<void(Offset, GridData*)>& func) {
    const uint64_t data_size = 1 << kDataBits;
    Array grid_data = allocator_.Get_Array();
    auto [block_offsets, num_blocks] = blocks_.Get_Blocks();
    for (int b = 0; b < static_cast<int>(num_blocks); ++b) {
      const uint64_t block_offset = block_offsets[b];
      uint64_t node_offset = block_offset;
      /* The coordinate of the origin of this block. */
      for (int i = 0; i < kNumNodesInBlockX; ++i) {
        for (int j = 0; j < kNumNodesInBlockY; ++j) {
          for (int k = 0; k < kNumNodesInBlockZ; ++k) {
            GridData& node_data = grid_data(node_offset);
            func(node_offset, &node_data);
            node_offset += data_size;
          }
        }
      }
    }
  }

  /* Iterates over all grid nodes in the grid and applies the given function
   `func` to each grid node's offset and data. */
  void IterateGridWithOffset(
      const std::function<void(Offset, const GridData&)>& func) const {
    const uint64_t data_size = 1 << kDataBits;
    ConstArray grid_data = allocator_.Get_Array();
    auto [block_offsets, num_blocks] = blocks_.Get_Blocks();
    for (int b = 0; b < static_cast<int>(num_blocks); ++b) {
      const uint64_t block_offset = block_offsets[b];
      uint64_t node_offset = block_offset;
      /* The coordinate of the origin of this block. */
      for (int i = 0; i < kNumNodesInBlockX; ++i) {
        for (int j = 0; j < kNumNodesInBlockY; ++j) {
          for (int k = 0; k < kNumNodesInBlockZ; ++k) {
            const GridData& node_data = grid_data(node_offset);
            func(node_offset, node_data);
            node_offset += data_size;
          }
        }
      }
    }
  }

  /* Returns the flags associated with `this` SpGrid. */
  SpGridFlags flags() const {
    return SpGridFlags{.log2_page = kLog2Page,
                       .log2_max_grid_size = kLog2MaxGridSize,
                       .data_bits = kDataBits,
                       .num_nodes_in_block_x = kNumNodesInBlockX,
                       .num_nodes_in_block_y = kNumNodesInBlockY,
                       .num_nodes_in_block_z = kNumNodesInBlockZ};
  }

  /* Returns the color of the block given the page bits of the node offset.
   Blocks with different colors are guaranteed to be non-adjacent. */
  static int get_color(uint64_t page) {
    /* According to [Setaluri et al. 2014], the blocks are arranged in 3D space
     along a Z-order curve, ensuring that any two blocks whose Z-order indices
     are eight apart are guaranteed to be non-adjacent. */
    constexpr int kNumColors = 8;
    int color = (page & (kNumColors - 1));
    return color;
  }

 private:
  /* Returns reference to the data at the given grid offset.
   @pre Given `offset` is valid. */
  const GridData& get_data(uint64_t offset) const {
    return allocator_.Get_Const_Array()(offset);
  }

  /* Returns mutable reference to the data at the given grid offset.
   @pre Given `offset` is valid. */
  GridData& get_mutable_data(uint64_t offset) {
    return allocator_.Get_Array()(offset);
  }

/* SPGrid imposes a limit on the maximum number of grid points per dimension,
 denoted as N. While N can be quite large (as discussed in Section 3.1 of
 [Setaluri, 2014]), increasing it excessively may lead to performance
 degradation due to a known limitation in the SPGrid library. To balance
 grid coverage and performance, we default N to 1024 (logN = 10). With this
 number of grid nodes and a typical grid spacing of 1 cm, the grid can
 span 10.24 meters per dimension. This coverage is sufficient for the
 majority of stationary manipulation simulations. We use a smaller max size in
 memcheck tests because the underlying SPGrid library reserves a span of virtual
 memory space (without actually allocating them) according to the max size, and
 a large reservation upsets Valgrind. */
#ifndef DRAKE_MPM_TESTING_LOG2_MAX_GRID_SIZE
  static constexpr int kLog2MaxGridSize = 10;
#else
  static constexpr int kLog2MaxGridSize = DRAKE_MPM_TESTING_LOG2_MAX_GRID_SIZE;
#endif
  static constexpr int kMaxGridSize = 1 << kLog2MaxGridSize;

  static constexpr int kDataBits = Mask::data_bits;
  static constexpr int kNumNodesInBlockX = 1 << Mask::block_xbits;
  static constexpr int kNumNodesInBlockY = 1 << Mask::block_ybits;
  static constexpr int kNumNodesInBlockZ = 1 << Mask::block_zbits;

  // TODO(xuchenhan-tri): Allow moving the maximumly allowed grid around the
  // center of the objects so that the grid can be accommodated to the objects
  // that are translating.
  /* 3D coordinates in SPGrid start at (i, j, k) = (0, 0, 0) with i, j, k
   always non-negative. We want the grid to center around (0, 0, 0) in world
   space so we shift the origin by
   (kMaxGridSize/2, kMaxGridSize/2, kMaxGridSize/2)*/
  const uint64_t origin_offset_{Mask::Linear_Offset(
      kMaxGridSize / 2, kMaxGridSize / 2, kMaxGridSize / 2)};

  /* SPGrid allocator. */
  Allocator allocator_;
  /* PageMap that helps allocate memory for grid data in `Allocate()`. We
   persist it in the class so that we don't need to construct and destruct it
   any time `Allocate()` is called. */
  PageMap helper_blocks_;
  /* All active/allocated blocks. */
  PageMap blocks_;

  /* Stores the difference in linear offset from a given grid node to the grid
   node exactly one block away. For example, let
   `a = block_offset_strides_[i][j][k]`, and `b` be the linear offset of a grid
   node `n`. Suppose `c = a + b`, then we have
   LinearToCoord(c) = LinearToCoord(b) + ((i - 1) * kNumNodesInBlockX,
                                          (j - 1) * kNumNodesInBlockY,
                                          (k - 1) * kNumNodesInBlockZ). */
  Pad<uint64_t> block_offset_strides_;
  /* Similar to block_offset_strides_, but instead of providing strides for
   nodes a "block" away, provides the strides for nodes a "cell" away (i.e.
   immediate grid neighbors). For example, let
   `a = cell_offset_strides_[i][j][k]`, and `b` be the linear offset of a grid
   node `n`. Suppose `c = a + b`, then we have
   LinearToCoord(c) = LinearToCoord(b) + (i - 1, j - 1, k - 1). */
  Pad<uint64_t> cell_offset_strides_;
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
