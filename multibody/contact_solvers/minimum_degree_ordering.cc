#include "drake/multibody/contact_solvers/minimum_degree_ordering.h"

#include <algorithm>
#include <functional>
#include <queue>
#include <set>
#include <tuple>
#include <utility>

#include "drake/multibody/contact_solvers/sap/partial_permutation.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

void InplaceSortedUnion(const std::vector<int>& b, std::vector<int>* a) {
  auto& vec = *a;
  const int n = ssize(vec);
  const int m = ssize(b);
  vec.resize(n + m);

  int a_index = n - 1;
  int b_index = m - 1;
  int out_index = n + m - 1;

  /* Pull the larger tail into a[out_index]. */
  while (a_index >= 0 && b_index >= 0) {
    const int va = vec[a_index];
    const int vb = b[b_index];
    if (va > vb) {
      vec[out_index] = va;
      --a_index;
    } else if (va < vb) {
      vec[out_index] = vb;
      --b_index;
    } else {
      vec[out_index] = va;
      --a_index;
      --b_index;
    }
    --out_index;
  }
  /* Copy any remaining b. */
  while (b_index >= 0) {
    vec[out_index] = b[b_index];
    --b_index;
    --out_index;
  }
  /* Copy any remaining a. */
  while (a_index >= 0) {
    vec[out_index] = vec[a_index];
    --a_index;
    --out_index;
  }
  /* Now a[out_index+1..end) is already in ascending order, so we just drop the
   unused prefix [0..out_index]. */
  vec.erase(vec.begin(), vec.begin() + (out_index + 1));
}

void InplaceSortedDifference(const std::vector<int>& b, std::vector<int>* a) {
  int a_index = 0, b_index = 0, out_index = 0;
  auto& vec = *a;
  const int n = ssize(vec);
  const int m = ssize(b);
  while (a_index < n && b_index < m) {
    if (vec[a_index] < b[b_index]) {
      vec[out_index++] = vec[a_index++];
    } else if (vec[a_index] == b[b_index]) {
      ++a_index;
      ++b_index;
    } else {
      ++b_index;
    }
  }
  while (a_index < n) {
    vec[out_index++] = vec[a_index++];
  }
  vec.resize(out_index);
}

void RemoveValueFromSortedVector(int value, std::vector<int>* sorted_vector) {
  auto it =
      std::lower_bound(sorted_vector->begin(), sorted_vector->end(), value);
  /* Check if the value was found and remove it. */
  while (it != sorted_vector->end() && *it == value) {
    it = sorted_vector->erase(it);
  }
}

void InsertValueInSortedVector(int value, std::vector<int>* sorted_vector) {
  auto it =
      std::lower_bound(sorted_vector->begin(), sorted_vector->end(), value);
  /* Check if the value doesn't already exist and insert it. */
  if (it == sorted_vector->end() || *it != value) {
    sorted_vector->insert(it, value);
  }
}

void UpdateWeights(const std::vector<int>& Lp, std::vector<Node>* nodes) {
  DRAKE_DEMAND(nodes != nullptr);
  for (Node& n : *nodes) {
    n.weight = -1;
  }
  for (int i : Lp) {
    for (int e : nodes->at(i).E) {
      Node& node_e = (*nodes)[e];
      if (node_e.weight < 0) {
        node_e.weight = 0;
        for (int l : node_e.L) {
          node_e.weight += nodes->at(l).size;
        }
      }
      node_e.weight -= nodes->at(i).size;
    }
  }
}

void Node::UpdateExternalDegree(const std::vector<Node>& nodes,
                                std::vector<uint8_t>* seen) {
  DRAKE_ASSERT(seen != nullptr);
  DRAKE_ASSERT(ssize(*seen) == ssize(nodes));

  degree = 0;
  /* Add in |Aᵢ \ i| term. */
  for (int a : A) {
    degree += nodes[a].size;
  }

  /* Add in |L \ i| term where L = ∪ₑLₑ and e ∈ Eᵢ. */
  (*seen)[index] = 1;
  for (int e : E) {
    for (int l : nodes[e].L) {
      if (!(*seen)[l]) {
        (*seen)[l] = 1;
        degree += nodes[l].size;
      }
    }
  }
  seen->assign(seen->size(), 0);
}

void Node::ApproximateExternalDegree(int p, int Lp_size,
                                     const std::vector<Node>& nodes) {
  degree = 0;

  /* Add in |Aᵢ \ i| term. */
  for (int a : A) {
    degree += nodes[a].size;
  }

  /* Add in |Lp \ i| term. */
  degree += Lp_size;

  for (int e : E) {
    if (e != p) {
      degree += nodes[e].weight >= 0 ? nodes[e].weight : nodes[e].L.size();
    }
  }
}

std::vector<int> ComputeMinimumDegreeOrdering(
    const BlockSparsityPattern& block_sparsity_pattern, bool use_amd) {
  return ComputeMinimumDegreeOrdering(block_sparsity_pattern, {}, use_amd);
}

std::vector<int> ComputeMinimumDegreeOrdering(
    const BlockSparsityPattern& block_sparsity_pattern,
    const std::unordered_set<int>& priority_elements, bool use_amd) {
  /* Initialize for Minimum Degree: Populate index, size, and A, the list of
   adjacent variables. E, L are empty initially. */
  const std::vector<int>& block_sizes = block_sparsity_pattern.block_sizes();
  const int num_nodes = block_sizes.size();
  std::vector<Node> nodes(num_nodes);
  for (int n = 0; n < num_nodes; ++n) {
    nodes[n].index = n;
    nodes[n].size = block_sizes[n];
  }
  for (int n = 0; n < num_nodes; ++n) {
    Node& node = nodes[n];
    const std::vector<int>& neighbors = block_sparsity_pattern.neighbors()[n];
    for (int neighbor : neighbors) {
      if (n != neighbor) {
        /* We modify both `node` and `neighbor` here because for each ij-pair
         only one of (i, j) and (j, i) is recorded in `sparsity pattern` but
         we want j be part of Ai and i to be part of Aj. */
        node.A.push_back(nodes[neighbor].index);
        node.degree += nodes[neighbor].size;
        nodes[neighbor].A.push_back(node.index);
        nodes[neighbor].degree += node.size;
      }
    }
  }
  /* Maintain the invariance that A, E, L vectors in a node are all sorted. */
  for (auto& node : nodes) {
    std::vector<int>& A = node.A;
    /* Only A is non-empty at this point. */
    std::sort(A.begin(), A.end());
  }
  std::vector<uint8_t> seen(num_nodes, 0);

  std::vector<int> priority(num_nodes);
  for (int i = 0; i < num_nodes; ++i)
    priority[i] = priority_elements.count(i) ? 0 : 1;

  /* A simplified version of Node used in a minimum priority queue of that only
   contains the node's index, degree, and priority. */
  using SimplifiedNode =
      std::tuple<int /*priority*/, int /*degree*/, int /*index*/>;
  /* The priority queue is a min-heap. */
  std::priority_queue<SimplifiedNode, std::vector<SimplifiedNode>,
                      std::greater<SimplifiedNode>>
      pq;

  for (int i = 0; i < num_nodes; ++i) {
    pq.emplace(priority[i], nodes[i].degree, i);
  }

  /* The resulting elimination ordering. */
  std::vector<int> result;
  result.reserve(num_nodes);
  /* Elimination tracking */
  std::vector<uint8_t> eliminated(num_nodes, 0);
  /* Scratch buffers for union operations */
  std::vector<int> union_scratch;
  union_scratch.reserve(num_nodes);
  std::vector<int> union_result;
  union_result.reserve(num_nodes);

  while (ssize(result) < num_nodes) {
    int p;  // index of the next node to eliminate
    /* Pop until we get a fresh entry. Note that this in the worst case can be
     O(n^2), but in practice, with up to thousands of dofs, this out performs
     std::set. */
    while (true) {
      const auto [p_priority, p_degree, p_index] = pq.top();
      pq.pop();
      /* Skip nodes that are already eliminated. */
      if (eliminated[p_index]) continue;
      if (p_degree == nodes[p_index].degree) {
        /* We have a fresh entry. Mark it as eliminated. */
        p = p_index;
        eliminated[p] = 1;
        break;
      }
    }
    result.push_back(p);
    Node& node_p = nodes[p];

    /* Absorb into Lp all variables adjacent to element e where e is adjacent to
     p. Make sure to exclude p itself since p is turning from a variable into an
     element. */
    union_scratch = node_p.A;
    for (int e : node_p.E) {
      union_result.clear();
      std::set_union(union_scratch.begin(), union_scratch.end(),
                     nodes[e].L.begin(), nodes[e].L.end(),
                     std::back_inserter(union_result));
      union_scratch.swap(union_result);
    }
    RemoveValueFromSortedVector(p, &union_scratch);
    node_p.L.swap(union_scratch);
    if (use_amd) UpdateWeights(node_p.L, &nodes);

    /* Update each neighbor i in L */
    for (int i : node_p.L) {
      Node& node_i = nodes[i];
      /* Pruning. */
      InplaceSortedDifference(node_p.L, &node_i.A);
      /* Convert p from a variable to an element in node i. Note that Lp was
       already updated prior to the loop, and therefore we can now safely
       remove the absorbed elements in node i */
      InplaceSortedDifference(node_p.E, &node_i.E);
      RemoveValueFromSortedVector(p, &node_i.A);
      InsertValueInSortedVector(p, &node_i.E);
      if (use_amd) {
        /* Compute |Lp \ i|, or Lp_size. */
        const std::vector<int>& Lp = node_p.L;
        int Lp_size = 0;
        for (int n : Lp) {
          if (n != i) {
            Lp_size += nodes[n].size;
          }
        }
        node_i.ApproximateExternalDegree(p, Lp_size, nodes);
      } else {
        node_i.UpdateExternalDegree(nodes, &seen);
      }
      /* Push updated entry i */
      pq.emplace(priority[i], node_i.degree, i);
    }
    /* Convert node p from a supervariable to an element. */
    node_p.A.clear();
    node_p.E.clear();
  }
  return result;
}

/* The complexity of this symbolic factorization is O(|L|) where L is the
 factorized matrix. */
BlockSparsityPattern SymbolicCholeskyFactor(
    const BlockSparsityPattern& block_sparsity) {
  const std::vector<int>& block_sizes = block_sparsity.block_sizes();
  /* sparsity pattern of the original symmetric matrix. */
  const std::vector<std::vector<int>>& A_sparsity = block_sparsity.neighbors();
  const int N = block_sizes.size();

  /* children[p] stores the children of node p in the elimination tree, in
   increasing order. See [Vandenberghe, 2015] Section 4.3 for the definition
   of an elimination tree.

   [Vandenberghe, 2015] Vandenberghe, Lieven, and Martin S. Andersen. "Chordal
   graphs and semidefinite optimization." Foundations and Trends® in
   Optimization 1.4 (2015): 241-433. */
  std::vector<std::vector<int>> children(N);
  /* L_sparsity[j] denotes the non-zero row blocks of column j. */
  std::vector<std::vector<int>> L_sparsity(N);
  for (int j = 0; j < N; ++j) {
    /* The sparsity pattern for the j-th node in L is the union of the
     neighbors of the j-th node in A and all neighbors of children of the j-th
     node that are larger than j. We first take the union of all neighbors of
     j and children of j and then delete all neighbors that are smaller than
     j. This is the same as described in equation 4.3 in [Davis 2006].
      Lⱼ = Aⱼ ∪ {j} ∪ ( ∪ Lₛ \ {s})
     where the union over s is over all children of j. */
    L_sparsity[j] = A_sparsity[j];
    for (int c : children[j]) {
      InplaceSortedUnion(L_sparsity[c], &L_sparsity[j]);
    }
    /* Instead of union over Lₛ \ {s}, we union over Lₛ and remove all entries
     smaller than j here in a single pass. */
    L_sparsity[j] = std::vector<int>(
        std::lower_bound(L_sparsity[j].begin(), L_sparsity[j].end(), j),
        L_sparsity[j].end());

    /* Record the parent of j if j isn't already the root. */
    if (L_sparsity[j].size() > 1) {
      /* The 0-th entry is j itself and the 1st entry is j-th parent. See
       [Vandenberghe, 2015] Section 4.3. */
      const int parent_of_j = L_sparsity[j][1];
      children[parent_of_j].emplace_back(j);
    }
  }
  return BlockSparsityPattern(block_sizes, L_sparsity);
}

std::vector<int> CalcAndConcatenateMdOrderingWithinGroup(
    const BlockSparsityPattern& global_pattern,
    const std::unordered_set<int>& v1) {
  /* Sizes of v, v1, and v2. */
  const int n = global_pattern.block_sizes().size();
  const int n1 = v1.size();
  const int n2 = n - n1;

  /* Mapping from v to v1 and v2 and the inverse mappings. */
  PartialPermutation v1_permutation(n);
  PartialPermutation v2_permutation(n);
  std::vector<int> global_to_local(n);

  auto in_v1 = [&](int b) {
    return v1.contains(b);
  };

  for (int i = 0; i < n; ++i) {
    if (in_v1(i)) {
      v1_permutation.push(i);
      global_to_local[i] = v1_permutation.permuted_index(i);
    } else {
      v2_permutation.push(i);
      global_to_local[i] = v2_permutation.permuted_index(i);
    }
  }

  /* The number of scalar variables in each block (as needed for the block
   sparsity pattern for G1 and G2). */
  std::vector<int> v1_block_sizes(n1);
  std::vector<int> v2_block_sizes(n2);
  const std::vector<int>& global_block_sizes = global_pattern.block_sizes();
  v1_permutation.Apply(global_block_sizes, &v1_block_sizes);
  v2_permutation.Apply(global_block_sizes, &v2_block_sizes);

  /* Build the induced graphs G1 and G2 from the global graph G. */
  const std::vector<std::vector<int>>& G = global_pattern.neighbors();
  std::vector<std::vector<int>> G1(n1);
  std::vector<std::vector<int>> G2(n2);
  for (int a = 0; a < n; ++a) {
    for (int b : G[a]) {
      if (in_v1(a) != in_v1(b)) {
        /* One of a and b is in v1 and the other is in v2, so the edge ab is not
         in either of the induced graph. */
        continue;
      }
      const int j = std::min(global_to_local[a], global_to_local[b]);
      const int i = std::max(global_to_local[a], global_to_local[b]);
      if (in_v1(b)) {
        G1[j].emplace_back(i);
      } else {
        G2[j].emplace_back(i);
      }
    }
  }

  const std::vector<int> v1_ordering = ComputeMinimumDegreeOrdering(
      BlockSparsityPattern(std::move(v1_block_sizes), std::move(G1)));
  const std::vector<int> v2_ordering = ComputeMinimumDegreeOrdering(
      BlockSparsityPattern(std::move(v2_block_sizes), std::move(G2)));

  std::vector<int> result;
  result.reserve(n);
  /* The v1 vertices come first. */
  for (int v : v1_ordering) {
    result.emplace_back(v1_permutation.domain_index(v));
  }
  /* The v2 vertices follow. */
  for (int v : v2_ordering) {
    result.emplace_back(v2_permutation.domain_index(v));
  }
  return result;
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
