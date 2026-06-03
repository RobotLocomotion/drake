#include "drake/multibody/contact_solvers/icf/icf_partition.h"

#include <algorithm>
#include <iterator>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

void IslandItemMap::Build(int num_islands,
                          std::span<const int> island_of_item) {
  DRAKE_DEMAND(num_islands >= 0);
  const int num_input = static_cast<int>(island_of_item.size());
  num_islands_ = num_islands;

  // Count the items per island. offsets_[i + 1] temporarily holds the count for
  // island i, then we turn it into a prefix sum below. Items with a negative
  // island id are skipped.
  if (std::ssize(offsets_) < num_islands + 1) offsets_.resize(num_islands + 1);
  std::fill(offsets_.begin(), offsets_.begin() + num_islands + 1, 0);
  int num_placed = 0;
  for (int k = 0; k < num_input; ++k) {
    const int island = island_of_item[k];
    if (island < 0) continue;
    DRAKE_ASSERT(island < num_islands);
    ++offsets_[island + 1];
    ++num_placed;
  }
  num_items_ = num_placed;
  for (int i = 0; i < num_islands; ++i) {
    offsets_[i + 1] += offsets_[i];
  }

  // Scatter each item into its island's slot, advancing a per-island cursor.
  // Scanning k in increasing order keeps each island's items sorted.
  if (std::ssize(cursor_) < num_islands) cursor_.resize(num_islands);
  std::copy(offsets_.begin(), offsets_.begin() + num_islands, cursor_.begin());
  if (std::ssize(items_) < num_placed) items_.resize(num_placed);
  for (int k = 0; k < num_input; ++k) {
    const int island = island_of_item[k];
    if (island < 0) continue;
    items_[cursor_[island]++] = k;
  }
}

int IcfPartition::Find(int x) {
  // Find the root.
  int root = x;
  while (parent_[root] != root) root = parent_[root];
  // Compress the path so future queries are O(1).
  while (parent_[x] != root) {
    const int next = parent_[x];
    parent_[x] = root;
    x = next;
  }
  return root;
}

void IcfPartition::Compute(
    const contact_solvers::internal::BlockSparsityPattern& pattern) {
  const std::vector<std::vector<int>>& neighbors = pattern.neighbors();
  const int nc = std::ssize(neighbors);
  num_cliques_ = nc;

  // Initialize the union-find forest with every clique as its own root.
  if (std::ssize(parent_) < nc) parent_.resize(nc);
  for (int c = 0; c < nc; ++c) parent_[c] = c;

  // Union cliques across every off-diagonal edge. neighbors[j] lists the rows
  // i ≥ j of the nonzero blocks in column j (with neighbors[j][0] == j), so
  // iterating all (j, i) with i != j visits every edge exactly once.
  for (int j = 0; j < nc; ++j) {
    for (int i : neighbors[j]) {
      if (i != j) {
        const int ri = Find(i);
        const int rj = Find(j);
        if (ri != rj) parent_[ri] = rj;
      }
    }
  }

  // Assign island ids. Numbering cliques in increasing order makes island i
  // the component whose smallest clique index is the i-th smallest overall,
  // giving a deterministic, scheduling-independent labeling.
  if (std::ssize(root_to_island_) < nc) root_to_island_.resize(nc);
  std::fill(root_to_island_.begin(), root_to_island_.begin() + nc, -1);
  if (std::ssize(clique_to_island_) < nc) clique_to_island_.resize(nc);
  num_islands_ = 0;
  for (int c = 0; c < nc; ++c) {
    const int r = Find(c);
    if (root_to_island_[r] < 0) root_to_island_[r] = num_islands_++;
    clique_to_island_[c] = root_to_island_[r];
  }

  island_cliques_.Build(num_islands_,
                        std::span<const int>(clique_to_island_.data(), nc));
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
