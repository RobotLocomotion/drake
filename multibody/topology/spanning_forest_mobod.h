#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_FOREST_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/forest.h".
#endif

#include <utility>
#include <vector>

namespace drake {
namespace multibody {
// TODO(sherm1) Promote from internal once API has stabilized: issue #11307.
namespace internal {

/** Everything you might want to know about a mobilized body. */
class SpanningForest::Mobod {
 public:
  /** (Internal use only) Copy/Move constructor & assignment. */
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Mobod)

  /** (Internal use only) This constructor is just for World, the only %Mobod
  with no mobilizer. (Or you can consider it welded to the universe at the
  World origin.) */
  Mobod(MobodIndex mobod_index, BodyIndex world_link);

  /** (Internal use only) The general constructor for any non-World %Mobod. */
  Mobod(MobodIndex mobod_index, BodyIndex link_index, JointIndex joint_index,
        int level, bool is_reversed);

  /** Returns `true` if this %Mobod is the World body, that is, the one with
  MobodIndex 0. Returns `false` for anything else, even if this %Mobod is
  welded to World. */
  bool is_world() const { return level_ == 0; }

  /** A %Mobod is a "base body" if its inboard mobilizer connects it directly to
  World. That means it is at level 1 in the Forest. */
  bool is_base_body() const { return level_ == 1; }

  /** A %Mobod is "anchored" if it is the World %Mobod or if it is a %Mobod
  directly or indirectly connected to the World %Mobod by Weld mobilizers.
  All such %Mobods are part of the 0th WeldedMobods group. */
  bool is_anchored() const {
    return welded_mobods_index_.is_valid() &&
           welded_mobods_index_ == WeldedMobodsIndex(0);
  }

  /** Returns true if there is no %Mobod that claims this one as its
  inboard %Mobod.
  @note Even if this %Mobod is not a leaf of its tree, it could still
  be the case that there are no outboard degrees of freedom if all the
  outboard joints are welds (recursively). That isn't taken into account here;
  use nq_outboard() for that information.
  @see nq_outboard() */
  bool is_leaf_mobod() const { return outboard_mobods_.empty(); }

  /** Returns true if the inboard/outboard relation here is opposite the
  parent/child relation of the Joint that this Mobod is modeling. */
  bool is_reversed() const { return use_reverse_mobilizer_; }

  /** Returns true if this %Mobod's inboard mobilizer has no degrees of
  freedom (or if this is World). */
  bool is_weld() const { return nq() == 0; }

  /** Returns the index of this %Mobod within the SpanningForest. World has
  index 0 and the rest are in depth-first order. */
  MobodIndex index() const { return index_; }

  /** Returns the index of this %Mobod's unique inboard %Mobod. The index is
  invalid if this is the World %Mobod. The inboard %Mobod's level()
  is one less that this %Mobod's level(). */
  MobodIndex inboard() const { return inboard_mobod_; }

  /** Returns the indices of all %Mobods for which this %Mobod serves as the
  inboard body. Each of the outboard %Mobods has a level() one higher than this
  %Mobod's level(). */
  const std::vector<MobodIndex>& outboards() const { return outboard_mobods_; }

  /** Returns the index of the Link mobilized by this %Mobod. If this %Mobod
  models a LinkComposite (collection of welded-together Links), this is the
  "active" Link of that composite, the one whose (non-Weld) Joint connects
  the whole LinkComposite to its inboard %Mobod.
  @see joint() */
  BodyIndex link() const { return follower_links()[0]; }

  /** Returns all the Links that are mobilized by this %Mobod. If this %Mobod
  represents a LinkComposite, the first Link returned is the "active" Link as
  returned by link(). There is always at least one Link. */
  const std::vector<BodyIndex>& follower_links() const {
    DRAKE_ASSERT(!follower_links_.empty());
    return follower_links_;
  }

  /** Returns the Joint represented by this %Mobod. If this %Mobod represents
  a LinkComposite (which has internal weld Joints), the Joint returned here
  is the "active" Joint that connects the Composite's active Link to a Link
  that follows this %Mobod's inboard %Mobod in the forest. The returned
  index is invalid if and only if this is the World %Mobod. */
  JointIndex joint() const { return joint_index_; }

  /** Returns the index of the Tree of which this %Mobod is a member. The
  index is invalid if and only if this is the World %Mobod. */
  TreeIndex tree() const { return tree_index_; }

  /** Returns the index of the WeldedMobods group of which this %Mobod is a
  part, if any. If this is the World %Mobod, we always return
  WeldedMobodIndex(0). Otherwise, the returned index is invalid if there are no
  %Mobods welded to this one. */
  WeldedMobodsIndex welded_mobods_group() const { return welded_mobods_index_; }

  /** Returns the level of this %Mobod within the SpanningForest. World is
  level 0, the root (base body) of each Tree is level 1, %Mobods connected to
  the base body are level 2, etc. */
  int level() const { return level_; }

  /** Starting offset within the contiguous q vector. */
  int q_start() const { return q_start_; }

  /** The number of position coordinates q used by this %Mobod's mobilizer. */
  int nq() const { return nq_; }

  /** Starting offset within the contiguous v vector. */
  int v_start() const { return v_start_; }

  /** The number of velocity coordinates v used by this %Mobod's mobilizer. */
  int nv() const { return nv_; }

  /** The count of generalized position coordinates q on the path between this
  and World. Includes this %Mobod's qs. */
  int nq_inboard() const { return nq_inboard_; }

  /** The count of generalized velocity coordinates v on the path between this
  and World. Includes this %Mobod's vs. */
  int nv_inboard() const { return nv_inboard_; }

  /** The count of generalized position coordinates q in the outboard subtree
  that has this %Mobod as its root. Does _not_ include this %Mobod's qs. */
  int nq_outboard() const { return nq_outboard_; }

  /** The count of generalized velocity coordinates v in the outboard subtree
  that has this %Mobod as its root. Does _not_ include this %Mobod's vs. */
  int nv_outboard() const { return nv_outboard_; }

  /** The number of %Mobods in the subtree with this %Mobod as its root,
  including this %Mobod in the count. These are numbered consecutively starting
  with this %Mobod's index, so the indices of the subtree %Mobods are [i, i+n)
  where i is this %Mobod's index and n is the return value of this method. */
  int num_subtree_mobods() const { return num_subtree_mobods_; }

  /** Returns the velocity coordinates for the subtree rooted at this %Mobod,
  _including_ this %Mobod's velocities. Because velocities are assigned
  depth-first, all the velocities in a subtree are consecutive so we need only
  return the index of the first one and the number of velocities. We return the
  pair {i, n} where i is the index of this %Mobod's first velocity coordinate
  vᵢ and n is the number of subtree velocity coordinates. So the subtree
  coordinates are [vᵢ..vᵢ₊ₙ).
  @see outboard_velocities() */
  std::pair<int, int> subtree_velocities() const {
    return std::make_pair(v_start(), nv() + nv_outboard());
  }

  /** Returns the velocity coordinates v that are outboard of this %Mobod,
  _not including_ its own velocities. This is the same as subtree_velocities()
  but with this Mobod's velocities removed.
  @see subtree_velocities(). */
  std::pair<int, int> outboard_velocities() const {
    return std::make_pair(v_start() + nv(), nv_outboard());
  }

 private:
  friend class SpanningForest;

  // Update all MobodIndex entries to use the new numbering.
  void FixupAfterReordering(const std::vector<MobodIndex>& old_to_new);

  // Switch the contents of `this` and `other` mobilized bodies.
  void Swap(Mobod& other);

  // Given a mapping from old MobodIndex to new MobodIndex, repair an
  // existing vector of old MobodIndexes to use the new numbering. Any invalid
  // indexes are left untouched.
  static void RenumberMobodIndexVector(
      const std::vector<MobodIndex>& old_to_new,
      std::vector<MobodIndex>* to_be_renumbered);

  // CAREFUL: if you add any members here, update Swap!

  // Links represented by this Mobod. The first one is always present and is
  // the active Link if we're mobilizing a LinkComposite.
  std::vector<BodyIndex> follower_links_;

  // Corresponding Joint (user or modeling joint). If this Mobod represents
  // a LinkComposite, this is the Joint that mobilizes the whole Composite.
  JointIndex joint_index_;

  // For an already-existing Joint, must we use a reverse mobilizer? If true,
  // we know tree inboard/outboard order is opposite graph parent/child order.
  bool use_reverse_mobilizer_{false};

  int level_{-1};  // Path length from World to this Mobod.

  // These references must be renumbered when we reorder the mobods.
  MobodIndex index_;          // Index of this mobilized body.
  MobodIndex inboard_mobod_;  // Tree parent at level-1 (invalid for World)
  std::vector<MobodIndex> outboard_mobods_;  // Tree children at level+1

  TreeIndex tree_index_;                   // Which Tree is this Mobod part of?
  WeldedMobodsIndex welded_mobods_index_;  // Part of a WeldedMobods group?

  // Coordinate assignments (done last). For welds (and for World), q/v_start
  // is still set to where coordinates would have started if there were any,
  // with nq/v==0.
  int q_start_{-1};  // within the full q vector
  int nq_{-1};
  int v_start_{-1};  // within the full v vector
  int nv_{-1};

  // Positioning within the coordinates of the containing tree. For World there
  // are zero inboard, num_positions and num_velocities outboard.
  int nq_inboard_{-1};  // Coordinates along the path to World.
  int nv_inboard_{-1};
  int nq_outboard_{-1};  // All coordinates outboard of this Mobod.
  int nv_outboard_{-1};

  // Number of Mobods in the subtree for which this is the root.
  int num_subtree_mobods_{-1};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
