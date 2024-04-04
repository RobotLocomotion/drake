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
