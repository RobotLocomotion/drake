#pragma once

#include <algorithm>
#include <array>
#include <bitset>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/mpm/bspline_weights.h"
#include "drake/multibody/mpm/spgrid_flags.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

/* A range of integers [start, end). */
class Range {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Range);

  Range(int start, int end) : start_(start), end_(end) {
    DRAKE_ASSERT(start_ < end_);
  }

  int start() const { return start_; }
  int end() const { return end_; }

 private:
  int start_{};
  int end_{};
};

using RangeVector = std::vector<Range>;

/* Given a sorted vector of integer key points, converts it to a vector of
 consecutive ranges. For example, given the input {1, 3, 6, 9}, the output
 should be {{1, 3}, {3, 6}, {6, 9}}.
 @pre data is sorted in ascending order (no repeats) and has at least two
 elements.
 @pre ranges != nullptr */
void ConvertToRangeVector(const std::vector<int>& data, RangeVector* ranges);

/* Helper alias: Given a grid pointer type, deduce the type of the pad nodes. */
template <typename Grid>
using PadNodeType = typename Grid::PadNodeType;

/* Helper alias: Given a grid pointer type, deduce the type of the pad data. */
template <typename Grid>
using PadDataType = typename Grid::PadDataType;

/* Callback type used by ParticleSorter::Iterate for grid to particle
 operations. */
template <typename Grid, typename ParticleData>
using G2PKernelType = std::function<void(
    int, const PadNodeType<Grid>&, const PadDataType<Grid>&, ParticleData*)>;

/* Callback type used by ParticleSorter::Iterate for particle to grid
 operations. */
template <typename Grid, typename ParticleData>
using P2GKernelType = std::function<void(
    int, const PadNodeType<Grid>&, const ParticleData&, PadDataType<Grid>*)>;

/* Callback type used by ParticleSorter::Iterate for operations that don't
modify either the particles or the grid. */
template <typename Grid, typename ParticleData>
using TraverseKernelType =
    std::function<void(int, const PadNodeType<Grid>&, const PadDataType<Grid>&,
                       const ParticleData&)>;

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
    const int num_particles = particle_positions.size();
    /* Resize data structures and clear old data. */
    Initialize(num_particles);

    /* We sort particles first based on their base node offsets in the spgrid;
     if they are the same, then we sort the particles by their original indices.
     (In other words, we sort the particles using lexigraphical order of the
     base node offsets and the original indices pair. However, we choose the
     following strategy to allow for non-comparative sort in the future.) To
     achieve this, we pack the base node offsets and the original indices of the
     particles into a single 64 bit unsigned integer as the key for sorting, so
     that the bit representation of the key is:

         bits_for_base_node_offset | bits_for_particle_index.

     However, that's easier said than done, because the base node offset is
     already taking 64 bits! Fortunately, there are some dead bits that we can
     truncate to make some room. We now check how many bits we can truncate
     based on information from the spgrid flags and how many particles we have.
    */
    const auto& flags = spgrid.flags();
    ComputeBitInformation(flags, num_particles);

    /* Build 64-bit keys that encode (page_offset << shift) + particle_index. */
    BuildKeys(spgrid, dx, particle_positions);

    // TODO(xuchenhan-tri): Switch to using radix sort to exploit the fact that
    // our keys are 64 bit unsigned integers. It's proven to be much faster than
    // std::sort in practice.
    std::sort(particle_sorters_.begin(), particle_sorters_.end());

    /* Extract sorted data indices and base node offsets from the sorted keys.
     */
    UnpackResults();

    /* Build the results for `colored_ranges()`. */
    BuildColoredRanges(spgrid, num_particles);
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
       for (int i = range.start(); i < range.end(); ++i) {
         // do something with particle data at index
         // `particle_sorter.data_indices()[i]`.
       }
     }
   }
   ```

   Refer to Section 3.2 of the technical document of
   [Hu et al. 2018](https://yzhu.io/publication/mpmmls2018siggraph/supp.pdf) for
   more details about how the colors are arranged and why there are 8 colors.

   [Hu et al. 2018] Hu, Yuanming, et al. "A moving least squares material point
   method with displacement discontinuity and two-way rigid body coupling." ACM
   Transactions on Graphics (TOG) 37.4 (2018): 1-14. */
  const std::array<RangeVector, 8>& colored_ranges() const {
    return colored_ranges_;
  }

  /* Iterates over all particles and their associated supported grid nodes,
   applying a user-defined function.

   This function traverses particles in blocks. For each particle, it retrieves
   the associated pad nodes and pad data from the grid, then invokes the
   provided function. If the passed-in grid pointer is mutable, the modified pad
   data is automatically written back to the grid after processing all particles
   associated with the pad. If a const grid pointer is provided, no grid update
   is performed (but particle update might be performed by the kernel).

   @tparam Grid          MPM Grid type (e.g., SparseGrid<double> or
                         const SparseGrid<float>).
   @tparam ParticleData  ParticleData type (e.g., ParticleData<double>, or
                         const ParticleData<float>).
   @tparam Func          Type of the function to be applied to each particle.
                         Must be either G2PKernelType or P2GKernelType.
   @param grid_ptr           Pointer to the grid object.
   @param particle_data_ptr  Pointer to the particle data object.
   @param func               A function to be applied to each particle.
   @pre  Exactly one of the grid/particle pointer is const and the other is
   mutable. When the grid pointer is mutable, the Func signature is
   P2GKernelType; when the particle pointer is mutable, the Func signature is
   G2PKernelType. */
  template <typename Grid, typename ParticleData, typename Func>
  void Iterate(Grid* grid_ptr, ParticleData* particle_data_ptr,
               const Func& func) const {
    const int num_blocks = grid_ptr->num_blocks();
    DRAKE_DEMAND(ssize(sentinel_particles_) == num_blocks + 1);

    /* Deduce types for pad nodes and pad data. */
    using PadNodeType = typename Grid::PadNodeType;
    using PadDataType = typename Grid::PadDataType;

    constexpr bool const_grid = std::is_const_v<Grid>;
    constexpr bool const_particle = std::is_const_v<ParticleData>;

    constexpr bool is_g2p = const_grid && !const_particle;
    constexpr bool is_p2g = const_particle && !const_grid;
    constexpr bool is_traverse = const_grid && const_particle;
    static_assert(is_g2p || is_p2g || is_traverse);
    if constexpr (is_g2p) {
      static_assert(std::is_same_v<Func, G2PKernelType<Grid, ParticleData>>,
                    "The provided function does not match the expected "
                    "signature for grid-to-particle operations.");
    } else if constexpr (is_p2g) {
      static_assert(std::is_same_v<Func, P2GKernelType<Grid, ParticleData>>,
                    "The provided function does not match the expected "
                    "signature for particle-to-grid operations.");
    } else {
      static_assert(
          std::is_same_v<Func, TraverseKernelType<Grid, ParticleData>>,
          "The provided function does not match the expected "
          "signature for traverse operations.");
    }

    /* Temporary variables for pad nodes and pad data. */
    PadNodeType grid_nodes{};
    PadDataType grid_data{};

    /* Flag indicating when to fetch new pad data. */
    bool need_new_pad = true;

    /* Iterate over each block. */
    for (int b = 0; b < num_blocks; ++b) {
      const int particle_start = sentinel_particles_[b];
      const int particle_end = sentinel_particles_[b + 1];

      /* Process each particle within the current block. */
      for (int p = particle_start; p < particle_end; ++p) {
        const int data_index = data_indices_[p];

        /* Fetch new pad data and nodes when we meet particles belonging to a
         new pad. */
        if (need_new_pad) {
          grid_data = grid_ptr->GetPadData(base_node_offsets_[p]);
          grid_nodes =
              grid_ptr->GetPadNodes(particle_data_ptr->x()[data_index]);
        }

        /* Apply the provided function to the current particle. */
        if constexpr (is_g2p) {
          func(data_index, grid_nodes, grid_data, particle_data_ptr);
        } else if constexpr (is_p2g) {
          func(data_index, grid_nodes, *particle_data_ptr, &grid_data);
        } else {
          static_assert(is_traverse);
          func(data_index, grid_nodes, grid_data, *particle_data_ptr);
        }

        /* Determine if the next particle requires new pad data. */
        need_new_pad = (p + 1 == particle_end) ||
                       (base_node_offsets_[p] != base_node_offsets_[p + 1]);

        /* Write to the pad if this is a P2G operation. */
        if constexpr (is_p2g) {
          if (need_new_pad) {
            grid_ptr->SetPadData(base_node_offsets_[p], grid_data);
          }
        }
      }
    }
  }

 private:
  /* Helper for Sort(). Resizes all containers and clear old data. */
  void Initialize(int num_particles);

  /* Helper for Sort(). Performs bit manipulation and squeezes base node offsets
   and particle indices into a single 64 bit unsigned int. */
  void ComputeBitInformation(const SpGridFlags& flags, int num_particles);

  /* Helper for `Sort()` that builds 64-bit sort keys for each particle. */
  template <typename T, typename SpGrid>
  void BuildKeys(const SpGrid& spgrid, double dx,
                 const std::vector<Vector3<T>>& particle_positions) {
    const int num_particles = particle_positions.size();
    for (int p = 0; p < num_particles; ++p) {
      /* We allow T == AutoDiffXd only for testing purpose. The positions in
       those tests are guaranteed to have zero gradients. Here we cast
       particle_x to double regardless of what type T is to simplify the code
       path for computing base node immediately below.
       The computation is simple enough that converting from float to double
       isn't going to trigger any noticeable overhead. (The main advantage for
       using floats is its compact memory footprint anyway.) */
      const auto& particle_x = [&]() -> const Vector3<double> {
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
      /* Confirm the data bits of the base node offset are all zero. */
      DRAKE_ASSERT(
          (base_node_offsets_[p] & ((uint64_t{1} << data_bits_) - 1)) == 0);
      /* Confirm the left most bits in the page bits are unused. */
      DRAKE_ASSERT((base_node_offsets_[p] &
                    ~((uint64_t{1} << (64 - zero_page_bits_)) - 1)) == 0);
      particle_sorters_[p] = (base_node_offsets_[p] << zero_page_bits_) + p;
    }
  }

  /* Helper for `Sort()` that unpacks the key into base node offsets and data
   indices. */
  void UnpackResults();

  /* Helper for `Sort()` that builds `colored_ranges_`. */
  template <typename SpGrid>
  void BuildColoredRanges(const SpGrid& spgrid, int num_particles) {
    /* Keep track of where each new "page" begins. */
    uint64_t previous_page = 0;
    int range_index = 0;

    for (int p = 0; p < num_particles; ++p) {
      /* The bits in the offset are ordered as follows:

        page bits | block bits | data bits

       block bits and data bits add up to log2_page bits.
       We right shift to get the page bits. If the page bits of two offsets are
       identical, it means that they belong to the same page.

       We color the pages so that pages that are adjacent in 3D space all have
       different colors, as described in [Hu et al. 2018]. */
      const uint64_t page = base_node_offsets_[p] >> log2_page_;

      if (p == 0 || page != previous_page) {
        previous_page = page;
        sentinel_particles_.push_back(p);
        const int color = spgrid.get_color(page);
        ranges_indices_[color].push_back(range_index++);
      }
    }
    /* Final sentinel at the end (always the number of particles). */
    sentinel_particles_.push_back(num_particles);

    /* Convert sentinel indices -> (start, end) ranges. */
    ConvertToRangeVector(sentinel_particles_, &ranges_);

    /* For each color, pick out the relevant [start, end) pairs. */
    for (int c = 0; c < 8; ++c) {
      colored_ranges_[c].clear();
      for (int r : ranges_indices_[c]) {
        colored_ranges_[c].push_back(ranges_[r]);
      }
    }
  }

  /* All but last entry store (sorted) indices of particles marking the
   boundary of a new block (aka the sentinel particles). Recall that a `block`
   is a subset of all grid nodes that occupy a consecutive blob of memory in
   SpGrid (see class documentation of SpGrid for more details) The last entry
   stores the number of particles. */
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

  /* Helper data for `Sort()`. */
  int log2_max_grid_size_{};
  int data_bits_{};
  int log2_page_{};
  int zero_page_bits_{};
  int index_bits_{};
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
