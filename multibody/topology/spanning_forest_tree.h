#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_FOREST_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/forest.h".
#endif

#include <vector>

namespace drake {
namespace multibody {
// TODO(sherm1) Promote from internal once API has stabilized: issue #11307.
namespace internal {

/** Everything you might want to know about an individual tree in the forest.
A Tree consists of consecutively numbered Mobod nodes in depth first order.
The first node is its base body (root node) and the last node is its
"rightmost" terminal body (assuming you draw the tree with the root at the
bottom, and children consistently left-to-right). Duplication is avoided by
keeping a back pointer to the owning Forest. */
class SpanningForest::Tree {
 public:
  Tree(const SpanningForest* forest, TreeIndex index, MobodIndex base_mobod)
      : index_(index), base_mobod_(base_mobod), height_(1), forest_(forest) {}

  TreeIndex index() const { return index_; }
  int height() const { return height_; }
  MobodIndex base_mobod() const { return base_mobod_; }
  MobodIndex last_mobod() const { return last_mobod_; }

  const SpanningForest::Mobod* begin() const {
    return &forest_->mobods()[base_mobod_];
  }
  const SpanningForest::Mobod* end() const {
    return &forest_->mobods()[last_mobod_] + 1;
  }
  const SpanningForest::Mobod& front() const {
    return forest_->mobods()[base_mobod_];
  }
  const SpanningForest::Mobod& back() const {
    return forest_->mobods()[last_mobod_];
  }

  int num_mobods() const { return last_mobod_ - base_mobod_ + 1; }

  int q_start() const { return front().q_start(); }
  int v_start() const { return front().v_start(); }
  int nq() const {
    // We depend on Weld Mobods to set their q_start to where it
    // would be if they had qs.
    const int last_q_plus_1 = back().q_start() + back().nq();
    return last_q_plus_1 - q_start();
  }
  int nv() const {
    // We depend on Weld Mobods to set their v_start to where it
    // would be if they had vs.
    const int last_v_plus_1 = back().v_start() + back().nv();
    return last_v_plus_1 - v_start();
  }

  /** (Internal use only) Copy/Move constructor & assignment. Back pointer
  requires fixup afterwards. */
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Tree)

 private:
  friend class SpanningForest;

  // Update all MobodIndex entries to use the new numbering.
  void FixupAfterReordering(const std::vector<MobodIndex>& old_to_new) {
    base_mobod_ = old_to_new[base_mobod_];
    // last_mobod_ is always set using the new ordering
  }

  TreeIndex index_;
  MobodIndex base_mobod_;  // The tree's root node that connects it to World.
  MobodIndex last_mobod_;  // The tree's highest numbered node.
  int height_{-1};         // 1 if just base body.
  const SpanningForest* forest_{nullptr};  // The containing forest.
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
