#pragma once

#include <cstdint>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include <SPGrid_Allocator.h>
#include <SPGrid_Array.h>
#include <SPGrid_Mask.h>
#include <SPGrid_Page_Map.h>

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

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

 @tparam GridData The type of data stored in the grid. It must satisfy the
 following requirements:
  - It must have a default constructor.
  - It must have a member function `set_zero()` that sets the data to zero.
  - Its size must be less than or equal to 4KB. */
template <typename GridData>
class SpGrid {
 public:
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
        blocks_(allocator_) {}

  /* Clears the contents of this SpGrid and ensures that each block containing
   the given offsets is allocated and zero-initialized.
   @note Actual allocation only happens the first time a block containing an
   offset is used in the lifetime of an SpGrid. Subsequent calls to `Allocate()`
   touching the same block only zero out the grid data. */
  void Allocate(const std::vector<Offset>& offsets) {
    blocks_.Clear();
    for (const Offset& offset : offsets) {
      blocks_.Set_Page(offset);
    }
    blocks_.Update_Block_Offsets();

    auto [block_offsets, num_blocks] = blocks_.Get_Blocks();
    const uint64_t page_size = 1 << kLog2Page;
    const uint64_t data_size = 1 << kDataBits;
    /* Note that Array is a wrapper around pointer to data memory and can be
     cheaply copied. */
    Array data = allocator_.Get_Array();
    /* Zero out the data in each block. */
    for (int b = 0; b < static_cast<int>(num_blocks); ++b) {
      const uint64_t offset = block_offsets[b];
      for (uint64_t i = 0; i < page_size; i += data_size) {
        data(offset + i).set_zero();
      }
    }
  }

  /* Returns the offset (1D index) of a grid node given its 3D grid
   coordinates in world space. */
  Offset CoordinateToOffset(int x, int y, int z) const {
    const uint64_t world_space_offset = Mask::Linear_Offset(x, y, z);
    return Mask::Packed_Add(world_space_offset, origin_offset_);
  }

  /* Returns the number of allocated blocks. */
  int num_blocks() const { return blocks_.Get_Blocks().second; }

 private:
  /* SPGrid imposes a limit on the maximum number of grid points per dimension,
   denoted as N. While N can be quite large (as discussed in Section 3.1 of
   [Setaluri, 2014]), increasing it excessively may lead to performance
   degradation due to a known limitation in the SPGrid library. To balance
   grid coverage and performance, we set N to 1024 (logN = 10). With this
   number of grid nodes and a typical grid spacing of 1 cm, the grid can
   span 10.24 meters per dimension. This coverage is sufficient for the
   majority of stationary manipulation simulations. */
  static constexpr int kLog2MaxGridSize = 10;
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
  /* All active/allocated blocks in the block. */
  PageMap blocks_;
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
