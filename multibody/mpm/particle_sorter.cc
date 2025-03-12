#include "drake/multibody/mpm/particle_sorter.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

void ConvertToRangeVector(const std::vector<int>& data, RangeVector* ranges) {
  ranges->clear();
  ranges->reserve(data.size() - 1);
  for (int i = 1; i < ssize(data); ++i) {
    ranges->push_back({data[i - 1], data[i]});
  }
}

std::vector<uint64_t> ParticleSorter::GetActiveBlockOffsets() const {
  std::vector<uint64_t> offsets(ssize(sentinel_particles_) - 1);
  for (int i = 0; i < ssize(sentinel_particles_) - 1; ++i) {
    offsets[i] = base_node_offsets_[sentinel_particles_[i]];
  }
  return offsets;
}

void ParticleSorter::Initialize(int num_particles) {
  particle_sorters_.resize(num_particles);
  base_node_offsets_.resize(num_particles);
  data_indices_.resize(num_particles);
  sentinel_particles_.clear();
  for (int c = 0; c < 8; ++c) {
    ranges_indices_[c].clear();
    colored_ranges_[c].clear();
  }
}

void ParticleSorter::ComputeBitInformation(const SpGridFlags& flags,
                                           int num_particles) {
  /* Recall that all base node offsets in SPGrid have the following bit
   representation:

       page_bits | block_bits | data_bits

   All the data bits are zero, guaranteed by SPGrid. In addition, a
   few bits (starting from the left) in the page bits are also zero (we
   explain why below). Therefore, we can truncate the zero bits in the
   base_node_offset and squeeze in the original particle index into the lower
   bits of a single 64 bit unsigned integer.

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
  log2_max_grid_size_ = flags.log2_max_grid_size;
  data_bits_ = flags.data_bits;
  log2_page_ = flags.log2_page;
  zero_page_bits_ = 64 - data_bits_ - 3 * log2_max_grid_size_;
  index_bits_ = 64 - 3 * log2_max_grid_size_;  // = zero_page_bits_ + data_bits_
  // TODO(xuchenhan-tri): This should be checked when we register particles to
  // the MPM model.
  /* Confirm that index_bits_ is enough to store num_particles. */
  const uint64_t capacity = (uint64_t{1} << index_bits_);
  DRAKE_DEMAND(capacity > static_cast<uint64_t>(num_particles));
}

void ParticleSorter::UnpackResults() {
  const uint64_t index_mask = ((uint64_t{1} << index_bits_) - 1);

  for (int p = 0; p < static_cast<int>(particle_sorters_.size()); ++p) {
    /* Lower bits store the particle's original index. */
    data_indices_[p] = (particle_sorters_[p] & index_mask);

    /* Upper bits store the base_node_offset (shifted); shift back & re-apply
     data_bits_. */
    base_node_offsets_[p] = (particle_sorters_[p] >> index_bits_) << data_bits_;
  }
}

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
