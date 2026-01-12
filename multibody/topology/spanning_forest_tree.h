#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_FOREST_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/forest.h".
#endif

#include <vector>

namespace drake {
namespace multibody {
// TODO(sherm1) Promote from internal once API has stabilized: issue #11307.
namespace internal {

/* Everything you might want to know about an individual tree in the forest.
A Tree consists of consecutively numbered Mobod nodes in depth-first order.
The first node is its base Mobod (root node) and the last node is its
"rightmost" terminal Mobod (assuming you draw the tree with the root at the
bottom, and children consistently left-to-right). Duplication is avoided by
keeping a back pointer to the owning SpanningForest. */
class SpanningForest::Tree {
 public:
  /* (Internal use only) Copy/Move constructor & assignment. Back pointer
  requires fixup afterwards. */
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Tree);

  /* (Internal use only) Trees are constructed during BuildForest(). Note
  that we know the base_mobod at construction but can't know the last_mobod
  until much later. */
  Tree(const SpanningForest* forest, TreeIndex index, MobodIndex base_mobod)
      : forest_(forest), index_(index), base_mobod_(base_mobod), height_(1) {}

  /* The TreeIndex of this Tree within the SpanningForest. */
  TreeIndex index() const { return index_; }

  /* The length of the longest branch of this %Tree, counting the base (root)
  body of the %Tree but not counting World. */
  int height() const { return height_; }

  /* The level 1 Mobod that is the base (root) Mobod of this %Tree. This is
  the lowest-numbered Mobod in this %Tree. */
  MobodIndex base_mobod() const { return base_mobod_; }

  /* The highest-numbered Mobod that is part of this %Tree. The Mobods in
  this tree are numbered consecutively from base_mobod() to last_mobod(). */
  MobodIndex last_mobod() const { return last_mobod_; }

  /* An iterator pointing to the base_mobod(), suitable for range iteration. */
  const SpanningForest::Mobod* begin() const {
    return &forest_->mobods()[base_mobod_];
  }

  /* An iterator pointing one entry past the last_mobod(), suitable for range
  iteration. Don't dereference this! */
  const SpanningForest::Mobod* end() const {
    return &forest_->mobods()[last_mobod_] + 1;
  }

  /* The Mobod whose index is returned by base_mobod(). */
  const SpanningForest::Mobod& front() const {
    return forest_->mobods()[base_mobod_];
  }

  /* The Mobod whose index is returned by last_mobod(). */
  const SpanningForest::Mobod& back() const {
    return forest_->mobods()[last_mobod_];
  }

  /* The number of Mobods in this %Tree, counting the base (root) body.
  (World is not considered to be part of any %Tree.) */
  int num_mobods() const { return last_mobod_ - base_mobod_ + 1; }

  /* The lowest numbered generalized position coordinate q assigned to
  this %Tree. */
  int q_start() const { return front().q_start(); }

  /* The lowest numbered generalized velocity coordinate v assigned to
  this %Tree. */
  int v_start() const { return front().v_start(); }

  /* The number of generalized position coordinates q assigned to
  this %Tree. They are numbered sequentially from q_start() to
  q_start() + nq() - 1. */
  int nq() const {
    // We depend on Weld Mobods to set their q_start to where it
    // would be if they had qs.
    const int last_q_plus_1 = back().q_start() + back().nq();
    return last_q_plus_1 - q_start();
  }

  /* The number of generalized velocity coordinates v assigned to
  this %Tree. They are numbered sequentially from v_start() to
  v_start() + nv() - 1. */
  int nv() const {
    // We depend on Weld Mobods to set their v_start to where it
    // would be if they had vs.
    const int last_v_plus_1 = back().v_start() + back().nv();
    return last_v_plus_1 - v_start();
  }

  /* Convenience method equivalent to (nv() > 0). */
  bool has_dofs() const { return nv() > 0; }

 private:
  friend class SpanningForest;

  // Update all MobodIndex entries to use the new numbering.
  void FixupAfterReordering(const std::vector<MobodIndex>& old_to_new) {
    base_mobod_ = old_to_new[base_mobod_];
    // last_mobod_ is always set using the new ordering
  }

  const SpanningForest* forest_{nullptr};  // The containing forest.

  TreeIndex index_;
  MobodIndex base_mobod_;  // The tree's root node that connects it to World.
  MobodIndex last_mobod_;  // The tree's highest numbered node.
  int height_{-1};         // 1 if just base body.
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
