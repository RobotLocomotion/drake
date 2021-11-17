#include "drake/multibody/plant/contact_permutation.h"

#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/multibody/tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {
namespace internal {

// Helper to deep traverse a tree from its base at base_node_index.
void TreeDepthFirstTraversal(const MultibodyTreeTopology& tree_topology,
                             BodyNodeIndex base_node_index,
                             std::vector<int>* tree_velocity_permutation,
                             std::vector<BodyIndex>* tree_bodies) {
  const BodyNodeTopology& node = tree_topology.get_body_node(base_node_index);
  tree_bodies->push_back(node.body);

  for (BodyNodeIndex child_index : node.child_nodes) {
    TreeDepthFirstTraversal(tree_topology, child_index,
                            tree_velocity_permutation, tree_bodies);
  }

  // Push the velocity indexes for this node.
  for (int i = 0; i < node.num_mobilizer_velocities; ++i) {
    const int m = node.mobilizer_velocities_start_in_v + i;
    tree_velocity_permutation->push_back(m);
  }
}

// Recall, v = velocity_permutation[t][vt]
// where:
//   t: tree index.
//   vt: local velocity index in tree t with DFS order.
//   v: original velocity index in tree_topology.
// and t = body_to_tree_map[body_index]. t < 0 if body_index
// is anchored to the world. Therefore body_to_tree_map[0] < 0 always.
void ComputeDfsPermutation(const MultibodyTreeTopology& tree_topology,
                           std::vector<std::vector<int>>* velocity_permutation,
                           std::vector<int>* body_to_tree_map) {
  DRAKE_DEMAND(velocity_permutation != nullptr);
  DRAKE_DEMAND(body_to_tree_map != nullptr);

  velocity_permutation->clear();
  body_to_tree_map->clear();

  const BodyNodeTopology& world_node =
      tree_topology.get_body_node(BodyNodeIndex(0));

  // Invalid (negative) index for the world. It does not belong to any tree in
  // particular, but it's the "floor" of the forest. Also any "tree" that is
  // anchored to the world is labeled with -1.
  body_to_tree_map->resize(tree_topology.num_bodies(), -1);

  // We deep traverse one tree at a time.
  for (BodyNodeIndex base_index : world_node.child_nodes) {
    // Bodies of the tree with base_index at the base.
    std::vector<BodyIndex> tree_bodies;
    // The permutation for the tree with base_index at its base.
    std::vector<int> tree_permutation;
    TreeDepthFirstTraversal(tree_topology, base_index, &tree_permutation,
                            &tree_bodies);
    const int tree_num_velocities = tree_permutation.size();
    // Trees with zero dofs are not considered.
    if (tree_num_velocities != 0) {
      const int t = velocity_permutation->size();
      velocity_permutation->push_back(tree_permutation);
      for (BodyIndex body_index : tree_bodies) {
        (*body_to_tree_map)[body_index] = t;
      }
    }
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
