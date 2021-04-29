#pragma once

#include <vector>

#include "drake/multibody/tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {
namespace internal {

// Computes a permutation to go from velocities in the order specified by
// `tree_topology` (currently BFS order) to DFS order, so that all dofs for a
// tree are contiguous.
//
// This new ordering depicts the mental model of a "forest of trees" topology.
// Each tree in this forest has a base body which is a direct child of the world
// body. Trees are arbitrarily assigned a, though contiguously increasing,
// positive index.
// In this picture, the world body is the "ground" of this forest and is not
// assigned to any particular tree. Therefore the "ground" is identified with an
// invalid, negative, tree index. Any other bodies or entire kinematic trees
// that are anchored to the world are also assigned a negative index. That is,
// only tress with non-zero dofs are assigned a positive tree index.
//
// The permutation is a bijection, i.e. there is a one-to-cone correspondence
// between the original dofs ordering in `tree_toplogy` and the new BFS
// ordering.
//
// For each velocity dof with index v in tree_topology, the new permutation will
// assign dof vt within tree with index t. A tree will have nt dofs in total,
// and therefore dofs vt for t will be in the range [0, nt-1].
//
// NOTE: We use a "reverse" DFS order (that is, deeper dofs are first)
// since for complex tree structures with a free floating base (consider the
// Allegro hand for instance) the mass matrix will have an arrow sparsity
// pattern (pointing to the bottom right). With a traditional DFS ordering we
// also get an arrow pattern, but pointing to the upper left. The distinction
// here is when performing Cholesky on the resulting mass matrix. The "reverse"
// DFS is "optimal", in that it'll produce a minimum number of zeros in the
// factorization. While more often than not, the traditional DFS ordering will
// lead to a fully dense factorization, unless a permutation is applied (extra
// work).
//
// @param[in] tree_topology The topology of our multibody model. There is not
// requiremnt on the specific ordering, though currently it uses a BFS ordering.
// @param[out] velocity_permutation
//   The permutation is built such that dof vt of tree t maps to dof
//   v = velocity_permutation[t][vt] in the original tree_topology.
//   Valid vt indexes are in the range [0, nt], with nt =
//   velocity_permutation[t].size(), the number of dofs for tree with index t.
//   The total number of trees is given by velocity_permutation.size().
//   The mapping is a bijection, and therefore each and every dof v in
//   tree_topology is mapped by a dof vt in its tree t.
//   Summarizing.
//     v = velocity_permutation[t][vt]
//   where:
//     t: tree index.
//     vt: local velocity index in tree t with DFT order.
//     v: original velocity index in tree_topology.
// @param[out] body_to_tree_map
//   This map stores what tree each body belongs to. It is built such that
//   t = body_to_tree_map[body_index] is the tree to which body_index belongs.
//   t < 0 if body_index is anchored to the world. body_to_tree_map[0] < 0
//   always since body_index = 0 corresponds to the world body.
void ComputeDfsPermutation(
    const MultibodyTreeTopology& tree_topology,
    std::vector<std::vector<int>>* velocity_permutation,
    std::vector<int>* body_to_tree_map);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
