#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_FOREST_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/forest.h".
#endif

#include <vector>

namespace drake {
namespace multibody {
// TODO(sherm1) Promote from internal once API has stabilized: issue #11307.
namespace internal {

/** Weld constraints added during modeling to close loops. */
class SpanningForest::LoopConstraint {
 public:
  LoopConstraint(MobodIndex parent_mobod_index, MobodIndex child_mobod_index,
                 ConstraintIndex graph_constraint_index)
      : parent_mobod_index_(parent_mobod_index),
        child_mobod_index_(child_mobod_index),
        graph_constraint_index_(graph_constraint_index) {}

  /** (Internal use only) Copy/Move constructor & assignment. */
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LoopConstraint)

 private:
  friend class SpanningForest;

  void FixupAfterReordering(const std::vector<MobodIndex>& old_to_new) {
    parent_mobod_index_ = old_to_new[parent_mobod_index_];
    child_mobod_index_ = old_to_new[child_mobod_index_];
  }

  MobodIndex parent_mobod_index_;
  MobodIndex child_mobod_index_;

  ConstraintIndex graph_constraint_index_;  // As added to the graph.
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
