#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_FOREST_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/forest.h".
#endif

#include <vector>

namespace drake {
namespace multibody {
// TODO(sherm1) Promote from internal once API has stabilized: issue #11307.
namespace internal {

/* Weld constraints added during modeling to close loops. */
class SpanningForest::LoopConstraint {
 public:
  /* (Internal use only) Copy/Move constructor & assignment. */
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LoopConstraint);

  /* (Internal use only) Created in SpanningForest::BuildForest() when loops
  are cut. */
  LoopConstraint(LoopConstraintIndex loop_constraint_index,
                 MobodIndex primary_mobod_index, MobodIndex shadow_mobod_index)
      : loop_constraint_index_(loop_constraint_index),
        primary_mobod_index_(primary_mobod_index),
        shadow_mobod_index_(shadow_mobod_index) {
    DRAKE_DEMAND(loop_constraint_index.is_valid() &&
                 primary_mobod_index.is_valid() &&
                 shadow_mobod_index.is_valid());
    DRAKE_DEMAND(primary_mobod_index != shadow_mobod_index);
  }

  /* Returns the index assigned to this LoopConstraint. */
  LoopConstraintIndex index() const { return loop_constraint_index_; }

  /* Returns the primary Mobod associated with the cut Link. This will
  be the parent of the implementing weld constraint. */
  MobodIndex primary_mobod() const { return primary_mobod_index_; }

  /* Returns the shadow Mobod associated with the cut Link. This will
  be the child of the implementing weld constraint. */
  MobodIndex shadow_mobod() const { return shadow_mobod_index_; }

 private:
  friend class SpanningForest;

  void FixupAfterReordering(const std::vector<MobodIndex>& old_to_new) {
    primary_mobod_index_ = old_to_new[primary_mobod_index_];
    shadow_mobod_index_ = old_to_new[shadow_mobod_index_];
  }

  LoopConstraintIndex loop_constraint_index_;

  MobodIndex primary_mobod_index_;
  MobodIndex shadow_mobod_index_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
