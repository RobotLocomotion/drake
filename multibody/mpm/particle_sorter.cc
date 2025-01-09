#include "drake/multibody/mpm/particle_sorter.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

void ConvertToRanges(const std::vector<int>& data, Ranges* ranges) {
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

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
