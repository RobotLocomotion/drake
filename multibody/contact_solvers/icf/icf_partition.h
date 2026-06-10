#pragma once

#include <span>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/* A compressed-storage (CSR-like) map from an island index to the list of item
indices assigned to that island. It is used to group items (cliques or
constraints) by the island (connected component) they belong to, so that
per-island computations can iterate only over the items they own.

Storage is grow-only: repeated calls to Build() reuse existing capacity and
only allocate when the number of items or islands exceeds any previous
high-water mark. This matches ICF's "allocate once, reallocate minimally"
philosophy and keeps per-step recomputation allocation-free in steady state. */
class IslandItemMap {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IslandItemMap);
  IslandItemMap() = default;

  /* Groups items by island via a counting sort. On return, items(i) lists
  every item index k (in increasing order) for which island_of_item[k] == i.
  Items with a negative island id are omitted from all groups; this is used to
  drop items that belong to no island (e.g., anchored bodies).
  @param num_islands     The total number of islands (≥ 0).
  @param island_of_item  island_of_item[k] is the island of item k, or a
                         negative value to omit item k.
  @pre Every non-negative entry of island_of_item is in [0, num_islands). */
  void Build(int num_islands, std::span<const int> island_of_item);

  int num_islands() const { return num_islands_; }
  int num_items() const { return num_items_; }

  /* Returns the items assigned to the given island, in increasing order. */
  std::span<const int> items(int island) const {
    DRAKE_ASSERT(0 <= island && island < num_islands_);
    return std::span<const int>(items_.data() + offsets_[island],
                                offsets_[island + 1] - offsets_[island]);
  }

 private:
  int num_islands_{0};
  int num_items_{0};
  std::vector<int> offsets_;  // size = num_islands_ + 1; prefix sums.
  std::vector<int> items_;    // size = num_items_; item indices by island.
  std::vector<int> cursor_;   // size = num_islands_; scratch write cursors.
};

/* Partitions the cliques of an ICF problem into islands: the connected
components of the clique-connectivity graph. Two cliques are in the same island
iff they are coupled (directly or transitively) by a constraint, i.e., iff they
are connected in the block sparsity pattern of the Hessian.

Because the Hessian is block-diagonal across islands, the cost is additive over
islands, and the gradient is block-disjoint, each island is an independent
convex subproblem that can be solved on its own. This class identifies those
subproblems.

Islands are numbered by the smallest clique index they contain, so the
numbering is deterministic across runs (and independent of thread scheduling).
All storage is grow-only across calls to Compute(). */
class IcfPartition {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IcfPartition);
  IcfPartition() = default;

  /* Computes the islands of the clique graph encoded by `pattern`. The diagonal
  blocks of `pattern` are the cliques (graph nodes); an off-diagonal nonzero
  block (i, j) is an edge between cliques i and j. */
  void Compute(const contact_solvers::internal::BlockSparsityPattern& pattern);

  int num_cliques() const { return num_cliques_; }
  int num_islands() const { return num_islands_; }

  /* Returns the island index of the given clique. */
  int clique_to_island(int clique) const {
    DRAKE_ASSERT(0 <= clique && clique < num_cliques_);
    return clique_to_island_[clique];
  }

  /* Returns the per-clique island assignment, indexed by clique. Useful for
  mapping a constraint's representative clique to its island. */
  std::span<const int> clique_to_island() const {
    return std::span<const int>(clique_to_island_.data(), num_cliques_);
  }

  /* Returns the cliques belonging to the given island, in increasing order. */
  std::span<const int> island_cliques(int island) const {
    return island_cliques_.items(island);
  }

  /* Returns the local index of the given clique within its island, i.e., its
  position (0-based) in island_cliques(clique_to_island(clique)). This is the
  block index the clique maps to in the island's local sub-Hessian. Because
  island_cliques() is sorted ascending, local indices preserve the global clique
  ordering within an island. */
  int clique_local_index(int clique) const {
    DRAKE_ASSERT(0 <= clique && clique < num_cliques_);
    return clique_local_index_[clique];
  }

  /* Returns the per-clique local-index map, indexed by clique. */
  std::span<const int> clique_local_index() const {
    return std::span<const int>(clique_local_index_.data(), num_cliques_);
  }

 private:
  /* Union-find "find" with path compression, operating on parent_. */
  int Find(int x);

  int num_cliques_{0};
  int num_islands_{0};
  std::vector<int> parent_;          // union-find forest, size = num_cliques_.
  std::vector<int> root_to_island_;  // scratch root→island, size = nc.
  std::vector<int> clique_to_island_;    // size = num_cliques_.
  std::vector<int> clique_local_index_;  // size = num_cliques_.
  IslandItemMap island_cliques_;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
