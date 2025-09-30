#include "drake/multibody/tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {
namespace internal {

bool MultibodyTreeTopology::operator==(
    const MultibodyTreeTopology& other) const {
  if (is_valid_ != other.is_valid_) return false;
  if (forest_height_ != other.forest_height_) return false;

  if (num_positions_ != other.num_positions_) return false;
  if (num_velocities_ != other.num_velocities_) return false;
  if (num_states_ != other.num_states_) return false;
  if (num_actuated_dofs_ != other.num_actuated_dofs_) return false;

  if (num_tree_velocities_ != other.num_tree_velocities_) return false;
  if (tree_velocities_start_in_v_ != other.tree_velocities_start_in_v_)
    return false;
  // Each velocity should have a valid tree index, so it is ok to compare these
  // directly.
  if (velocity_to_tree_index_ != other.velocity_to_tree_index_) return false;
  // The world body (BodyIndex(0)) does not have a valid tree index so we skip
  // it when comparing for equality.
  DRAKE_DEMAND(!other.rigid_body_to_tree_index_[0].is_valid());
  return std::equal(rigid_body_to_tree_index_.begin() + 1,
                    rigid_body_to_tree_index_.end(),
                    other.rigid_body_to_tree_index_.begin() + 1,
                    other.rigid_body_to_tree_index_.end());
}

MultibodyTreeTopology::~MultibodyTreeTopology() = default;

// TODO(sherm1) For historical reasons we're extracting information from the
//  Forest and distributing it but we should work directly from the Forest
//  without duplication.
void MultibodyTreeTopology::FinalizeTopology(const LinkJointGraph& graph) {
  // If the topology is valid it means that it was already finalized.
  // Re-compilation is not allowed.
  if (is_valid()) {
    throw std::logic_error(
        "Attempting to call MultibodyTree::Finalize() on an already "
        "finalized MultibodyTree.");
  }
  DRAKE_DEMAND(graph.forest_is_valid());

  const SpanningForest& forest = graph.forest();

  num_positions_ = forest.num_positions();
  num_velocities_ = forest.num_velocities();
  num_states_ = num_positions_ + num_velocities_;

  ExtractForestInfo(graph);

  // We are done with a successful Finalize() and we mark it as so.
  // Do not add any more code after this!
  is_valid_ = true;
}

// TODO(sherm1) Currently this copies from the graph and forest into the
//  previous data structures, to establish that we are computing the same
//  quantities. Once this works, should switch to using the forest data
//  directly and cut out the redundant data structures.
void MultibodyTreeTopology::ExtractForestInfo(const LinkJointGraph& graph) {
  const SpanningForest& forest = graph.forest();
  forest_height_ = forest.height();

  const SpanningForest::Mobod& root = forest.mobods(MobodIndex(0));
  DRAKE_DEMAND(ssize(root.outboards()) == ssize(forest.trees()));

  tree_velocities_start_in_v_.resize(ssize(forest.trees()), -1);
  num_tree_velocities_.resize(ssize(forest.trees()), -1);
  for (const auto& tree : forest.trees()) {
    const TreeIndex index = tree.index();
    tree_velocities_start_in_v_.at(index) = tree.v_start();
    num_tree_velocities_[index] = tree.nv();
  }

  velocity_to_tree_index_.reserve(forest.num_velocities());
  for (int v = 0; v < forest.num_velocities(); ++v)
    velocity_to_tree_index_.emplace_back(forest.v_to_tree(v));

  // Map each Link in the graph (including shadows added to break loops) to
  // the tree to which its modeling Mobod belongs.
  // TODO(sherm1) Really should be graph.num_link_indexes() which is private.
  //  Doesn't matter though since we don't currently remove Links and this code
  //  is going away soon.
  rigid_body_to_tree_index_.resize(graph.num_links());
  for (const auto& link : graph.links()) {
    // The tree index will be invalid for World.
    rigid_body_to_tree_index_[link.index()] = forest.link_to_tree(link.index());
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
