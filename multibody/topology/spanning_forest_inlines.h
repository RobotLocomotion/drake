#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_FOREST_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/forest.h".
#endif

#include <vector>

namespace drake {
namespace multibody {
// TODO(sherm1) Promote from internal once API has stabilized: issue #11307.
namespace internal {

// SpanningForest definitions deferred until Mobod defined.

inline auto SpanningForest::mobods(MobodIndex mobod_index) const
    -> const Mobod& {
  DRAKE_ASSERT(mobod_index.is_valid() && mobod_index < ssize(mobods()));
  return mobods()[mobod_index];
}

inline BodyIndex SpanningForest::mobod_to_link(MobodIndex mobod_index) const {
  return mobods(mobod_index).link();
}

inline const std::vector<BodyIndex>& SpanningForest::mobod_to_links(
    MobodIndex mobod_index) const {
  return mobods(mobod_index).follower_links();
}

inline TreeIndex SpanningForest::q_to_tree(int q_index) const {
  return mobods(q_to_mobod(q_index)).tree();
}

inline TreeIndex SpanningForest::v_to_tree(int v_index) const {
  return mobods(v_to_mobod(v_index)).tree();
}

// SpanningForest definitions deferred until Tree available.

inline auto SpanningForest::trees(TreeIndex tree_index) const -> const Tree& {
  DRAKE_ASSERT(tree_index.is_valid() && tree_index < ssize(trees()));
  return trees()[tree_index];
}

// SpanningForest definitions deferred until LoopConstraint defined.

inline auto SpanningForest::loop_constraints(LoopConstraintIndex index) const
    -> const LoopConstraint& {
  DRAKE_ASSERT(index.is_valid() && index < ssize(loop_constraints()));
  return loop_constraints()[index];
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
