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

// TODO(xuchenhan-tri): Add documentation on SpGrid.
template <typename GridData>
class SpGrid {
 public:
  static constexpr int kDim = 3;
  static constexpr int kLog2Page = 12;  // 4KB page size.
  using Scalar = typename GridData::Scalar;
  using Allocator = SPGrid::SPGrid_Allocator<GridData, kDim, kLog2Page>;
  /* PageMap keeps track of which blocks in the SPGrid are allocated. */
  using PageMap = SPGrid::SPGrid_Page_Map<kLog2Page>;
  /* Mask helps convert from offset (1D index) to 3D index and vice versa. */
  using Mask = typename Allocator::Array_mask<GridData>;
  /* Array type for GridData. */
  using Array = typename Allocator::Array_type<GridData>;
  using ConstArray = typename Allocator::Array_type<const GridData>;
  using Offset = uint64_t;

  explicit SpGrid(Scalar dx)
      : dx_(dx),
        allocator_(kMaxGridSize, kMaxGridSize, kMaxGridSize),
        blocks_(allocator_) {}

  Scalar dx() const { return dx_; }

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
  /* The maximum grid size along a single dimension. That is even
   though the grid is sparsely populated, the maximum grid size
   that can ever be allocated is kMaxGridSize^3. With 1cm grid dx, that
   corresponds to more than 10 meters in each dimension, which should be
   enough for most manipulation simulations. */
  static constexpr int kLog2MaxGridSize = 10;
  static constexpr int kMaxGridSize = 1 << kLog2MaxGridSize;

  static constexpr int kDataBits = Mask::data_bits;
  static constexpr int kNumNodesInBlockX = 1 << Mask::block_xbits;
  static constexpr int kNumNodesInBlockY = 1 << Mask::block_ybits;
  static constexpr int kNumNodesInBlockZ = 1 << Mask::block_zbits;

  // TODO(xuchenhan-tri): Allow moving the maximumly allowed grid around the
  // center of the objects so that the grid can be accommodated to the objects
  // that are translating.
  /* 3D coordinates in SPGrid starts at (i, j, k) = (0, 0, 0) with i, j, k
   always non-negative. We want the grid to center around (0, 0, 0) in world
   space so we shift the origin by
   (kMaxGridSize/2, kMaxGridSize/2, kMaxGridSize/2)*/

  const uint64_t origin_offset_{Mask::Linear_Offset(
      kMaxGridSize / 2, kMaxGridSize / 2, kMaxGridSize / 2)};

  Scalar dx_{};

  /* SPGrid allocator. */
  Allocator allocator_;
  /* Blocks containing grid nodes that are base nodes for at least one particle.
   */
  PageMap blocks_;
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
