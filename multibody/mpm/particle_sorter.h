#pragma once

#include <algorithm>
#include <array>
#include <bitset>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/ssize.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/mpm/bspline_weights.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

/* A range of integers [start, end). */
struct Range {
  int start;
  int end;
};

using RangeVector = std::vector<Range>;

/* Given a sorted vector of integers key points, converts it to a vector of
 consecutive ranges. For example, given the input {1, 3, 6, 9}, the output
 should be {{1, 3}, {3, 6}, {6, 9}}.
 @pre data is sorted in ascending order (no repeats) and has at least two
 elements.
 @pre ranges != nullptr */
void ConvertToRangeVector(const std::vector<int>& data, RangeVector* ranges);

/* ParticleSorter sorts MPM particle data based on their positions within the
 grid.

 Given a set of MPM particles and the background grid, recall that we define the
 base node of a particle to be the grid node that is closest to the particle
 (see multibody::mpm::internal::ComputeBaseNode).

 ParticleSorter provides a `Sort()` function that sorts the particles first
 based on their base node offsets (recall that the offset of a node is the 1D
 index of the node; see multibody::mpm::internal::SpGrid for the full
 definition) in the grid; if they are the same, it then sorts the particles by
 their original indices. This sorting is useful for efficient data access in the
 MPM transfer kernels.

 After `Sort()` is called, the ParticleSorter provides information about the
 particles and its background grid based on the result of the sort. */
class ParticleSorter {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ParticleSorter);
  ParticleSorter() = default;

  // TODO(xuchenhan-tri): The implementation of this function can be moved to
  // the .cc file when SpGrid is no longer an open template.
  /* Sorts the particles based on the particle positions so that the ordering is
   more memory friendly with respect to the given SpGrid.
   @param[in] spgrid             The SPGrid data structure of the MPM grid.
   @param[in] dx                 The MPM grid spacing [meters].
   @param[in] particle_positions All particle positions, measured and expressed
                                 in the world frame. */
  template <typename T, typename SpGrid>
  void Sort(const SpGrid& spgrid, double dx,
            const std::vector<Vector3<T>>& particle_positions) {
    const auto& flags = spgrid.flags();
    const int log2_max_grid_size = flags.log2_max_grid_size;
    const int data_bits = flags.data_bits;
    const int log2_page = flags.log2_page;

    const int num_particles = particle_positions.size();
    data_indices_.resize(num_particles);
    base_node_offsets_.resize(num_particles);
    particle_sorters_.resize(num_particles);

    // TODO(xuchenhan-tri): Switch to using radix sort to exploit the fact that
    // our keys are 64 bit unsigned integers. It's proven to be much faster than
    // std::sort in practice.

    /* We sort particles first based on their base node offsets in the spgrid;
     if they are the same, then we sort the particles by their original indices.
     To achieve this, we pack the base node offsets and the original indices of
     the particles into a single 64 bit unsigned integer as the key for sorting,
     so that the bit representation of the key is:

         bits_for_base_node_offset | bits_for_particle_index.

     However, that's easier said than done, because the base node offset is
     already taking 64 bits! Fortunately, we notice that all base node offsets
     in the spgrid of the particle have the following structure

         page_bits | block_bits | data_bits

     with all the data bits being zero. In addition, a few bits (starting from
     the left) in the page bits are also zero (we explain why below). Therefore,
     we can truncate the zero bits in the base_node_offset and squeeze in the
     original particle index into the lower bits of a single 64 bit unsigned
     integer.

     Q: Why are the left most bits in page_bits zero?

     A: Since we cap the total number of allowed grid nodes. There are at most
     2^(3*log2_max_grid_size) number of grid nodes in the grid, which requires
     3*log2_max_grid_size bits to represent. SPGrid uses page_bits and
     block_bits to represent the grid nodes (and data_bits are used, well, to
     store data). The page_bits and block_bits have (64 - data_bits) in total,
     and only 3*log2_max_grid_size bits are used, so the left most
     (64 - data_bits - 3*log2_max_grid_size) bits are zero.

     Q: So how many zeros can we truncate in the bits for base_node_offsets to
     make room for the particle indices? And is it enough room?

     A: There are (64 - data_bits - 3*log2_max_grid_size) zeros on the left and
     data_bits zeros on the right, so they add up to (64 - 3*log2_max_grid_size)
     zeros in total. with a typical log2_max_grid_size == 10, we have
     (64 - 3*10) = 34 bits to work with, which is more than enough to store the
     32 bit particle indices.

     Therefore, the overall strategy is to left shift the base node offset by
     (64 - data_bits - 3*log2_max_grid_size), which leaves the lower
     (64 - 3*log2_max_grid_size) bits to be zero. We then use the right most 32
     bits to store the original index of the particles. */
    const int index_bits = 64 - 3 * log2_max_grid_size;
    // TODO(xuchenhan-tri): The following check (i.e. that we don't have too
    // many particles) should be enforced when we register the particles to the
    // MPM model.
    /* Make sure that we have enough bits to store the index of the particles.
     */
    DRAKE_DEMAND(uint64_t(1) << index_bits >
                 static_cast<uint64_t>(num_particles));
    const int zero_page_bits = 64 - data_bits - 3 * log2_max_grid_size;

    for (int p = 0; p < num_particles; ++p) {
      /* We allow T == AutoDiffXd only for testing purpose. The positions in
       those tests are guaranteed to have zero gradients. Here we cast
       particle_x to double regardless of what type T is to simplify the code
       path for computing base node immediately below. */
      const auto& particle_x = [&]() -> const Vector3<double>& {
        if constexpr (std::is_same_v<T, double>) {
          return particle_positions[p];
        } else if constexpr (std::is_same_v<T, float>) {
          return particle_positions[p].template cast<double>();
        } else {
          return math::DiscardZeroGradient(particle_positions[p]);
        }
      }();
      const Vector3<int> base_node = ComputeBaseNode<double>(particle_x / dx);
      base_node_offsets_[p] =
          spgrid.CoordinateToOffset(base_node[0], base_node[1], base_node[2]);
      data_indices_[p] = p;
      /* Confirm the data bits of the base node offset are all zero. */
      DRAKE_ASSERT((base_node_offsets_[p] & ((uint64_t(1) << data_bits) - 1)) ==
                   0);
      /* Confirm the left most bits in the page bits are unused. */
      DRAKE_ASSERT((base_node_offsets_[p] &
                    ~((uint64_t(1) << (64 - zero_page_bits)) - 1)) == 0);
      particle_sorters_[p] =
          (base_node_offsets_[p] << zero_page_bits) + data_indices_[p];
    }
    std::sort(particle_sorters_.begin(), particle_sorters_.end());

    /* Peel off the data indices and the base node offsets from
     particle_sorters_. Meanwhile, reorder the data indices and the base node
     offsets based on the sorting results. */
    const uint64_t index_mask = ((uint64_t(1) << index_bits) - 1);
    for (int p = 0; p < ssize(particle_sorters_); ++p) {
      data_indices_[p] = particle_sorters_[p] & index_mask;
      base_node_offsets_[p] = (particle_sorters_[p] >> index_bits) << data_bits;
    }

    /* Record the sentinel particles and the coloring of the blocks. */
    sentinel_particles_.clear();
    for (int b = 0; b < 8; ++b) {
      colored_ranges_[b].clear();
    }
    uint64_t previous_page{};
    int range_index = 0;
    for (int p = 0; p < num_particles; ++p) {
      /* The bits in the offset is ordered as follows:

        page bits | block bits | data bits

       block bits and data bits add up to log2_page bits.
       We right shift to get the page bits. */
      const uint64_t page = base_node_offsets_[p] >> log2_page;
      if (p == 0 || previous_page != page) {
        previous_page = page;
        sentinel_particles_.push_back(p);
        const int color = spgrid.get_color(page);
        ranges_indices_[color].push_back(range_index++);
      }
    }
    sentinel_particles_.push_back(num_particles);

    ConvertToRangeVector(sentinel_particles_, &ranges_);
    for (int c = 0; c < 8; ++c) {
      colored_ranges_[c].clear();
      for (int r : ranges_indices_[c]) {
        colored_ranges_[c].push_back(ranges_[r]);
      }
    }
  }

  /* Returns a vector of representative grid node offsets for all active SpGrid
   blocks.

   A grid block is considered "active" if it contains at least one particle. The
   returned vector has exactly one offset per active block, allowing the caller
   to identify or access the blocks that contain particles. */
  std::vector<uint64_t> GetActiveBlockOffsets() const;

  /* The order in which the particle data is preferably accessed.

    typical usage:

    const std::vector<int>& data_indices = particle_sorter.data_indices();
    const int num_particles = data_indices.ssize();
    // Loop over some particle data `foo`.
    for (int p = 0; p < num_particles; ++p) {
      const auto& particle_data = foo_data[data_indices[p]];
      // do something with particle_data.
    }
  */
  const std::vector<int>& data_indices() const { return data_indices_; }

  /* Returns the offsets of the base nodes for the particles in sorted order
   (i.e. the same order as in `data_indices()`). */
  const std::vector<uint64_t>& base_node_offsets() const {
    return base_node_offsets_;
  }

  /* Returns an array of eight RangeVector objects, each representing a distinct
   color partition of the particle data. After sorting, the disjoint subsets in
   each partition can be processed safely in parallel using MPM transfer
   kernels, without write hazards between concurrent threads.

   Specifically, for each i from 0 to 7, colored_ranges()[i] is a collection of
   non-overlapping sorted particle data indices ranges such that when two
   different threads simultaneously perform particle to grid transfers for
   particles in different ranges, they will not write to the same memory
   locations in the grid data.

   Example usage:
   ```
   for (int color = 0; color < 8; ++color) {
     // It's safe to parallelize over the ranges within a single color.
     #if defined(_OPENMP)
     #pragma omp parallel for
     #endif
     for (const Range& range : particle_sorter.colored_ranges()[color]) {
       for (int i = range.start; i < range.end; ++i) {
         // do something with particle data at index
         // `particle_sorter.data_indices()[i]`.
       }
     }
   }
   ```

   Refer to Section 3.2 of the technical document of [Hu et al. 2018]
   (https://yzhu.io/publication/mpmmls2018siggraph/supp.pdf) for more
   details about how the colors are arranged and why there are 8 colors.

   [Hu et al. 2018] Hu, Yuanming, et al. "A moving least squares material point
   method with displacement discontinuity and two-way rigid body coupling." ACM
   Transactions on Graphics (TOG) 37.4 (2018): 1-14. */
  const std::array<RangeVector, 8>& colored_ranges() const {
    return colored_ranges_;
  }

 private:
  /* All but last entry store (sorted) indices of particles marking the boundary
   of a new block (aka the sentinel particles). Recall that a `block` is a
   subset of all grid nodes that occupy a consecutive blob of memory in SpGrid
   (see class documentation of SpGrid for more details) The last entry stores
   the number of particles. */
  std::vector<int> sentinel_particles_;
  std::vector<int> data_indices_;            // see data_indices().
  std::vector<uint64_t> base_node_offsets_;  // see base_node_offsets().
  /* The data being sorted -- it encodes the base node offset and the original
   index of the particle. */
  std::vector<uint64_t> particle_sorters_;
  std::array<RangeVector, 8> colored_ranges_;  // see_colored_ranges().
  /* The ranges of the particle data indices, each range corresponds to
   particles inside a single SpGrid page. */
  RangeVector ranges_;
  /* Helper data structure for building `colored_ranges_`. */
  std::array<std::vector<int>, 8> ranges_indices_;
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
