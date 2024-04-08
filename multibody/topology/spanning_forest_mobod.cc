// NOLINTNEXTLINE(build/include): prevent complaint re spanning_forest_mobod.h
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/multibody/topology/forest.h"

namespace drake {
namespace multibody {
namespace internal {

// Constructor for the World mobilized body.
SpanningForest::Mobod::Mobod(MobodIndex mobod_index, BodyIndex world_link)
    : level_(0),
      index_(mobod_index),
      q_start_(0),
      nq_(0),
      v_start_(0),
      nv_(0),
      nq_inboard_(0),
      nv_inboard_(0) {
  DRAKE_DEMAND(mobod_index.is_valid() && world_link.is_valid());
  DRAKE_DEMAND(world_link == 0 && mobod_index == 0);
  follower_links_.push_back(world_link);
}

// Constructor for mobilized bodies other than World.
SpanningForest::Mobod::Mobod(MobodIndex mobod_index, BodyIndex link_index,
                             JointIndex joint_index, int level,
                             bool is_reversed)
    : joint_index_(joint_index),
      use_reverse_mobilizer_(is_reversed),
      level_(level),
      index_(mobod_index) {
  DRAKE_DEMAND(mobod_index.is_valid() && link_index.is_valid() &&
               joint_index.is_valid());
  DRAKE_DEMAND(mobod_index != 0 && link_index != 0 && level > 0);
  follower_links_.push_back(link_index);
}

void SpanningForest::Mobod::FixupAfterReordering(
    const std::vector<MobodIndex>& old_to_new) {
  index_ = old_to_new[index_];
  if (!is_world()) inboard_mobod_ = old_to_new[inboard_mobod_];
  RenumberMobodIndexVector(old_to_new, &outboard_mobods_);
}

void SpanningForest::Mobod::Swap(Mobod& other) {
  std::swap(follower_links_, other.follower_links_);
  std::swap(joint_index_, other.joint_index_);
  std::swap(use_reverse_mobilizer_, other.use_reverse_mobilizer_);
  std::swap(level_, other.level_);
  std::swap(index_, other.index_);
  std::swap(inboard_mobod_, other.inboard_mobod_);
  std::swap(outboard_mobods_, other.outboard_mobods_);
  std::swap(tree_index_, other.tree_index_);
  std::swap(welded_mobods_index_, other.welded_mobods_index_);
  std::swap(q_start_, other.q_start_);
  std::swap(nq_, other.nq_);
  std::swap(v_start_, other.v_start_);
  std::swap(nv_, other.nv_);
  std::swap(nq_inboard_, other.nq_inboard_);
  std::swap(nv_inboard_, other.nv_inboard_);
  std::swap(nq_outboard_, other.nq_outboard_);
  std::swap(nv_outboard_, other.nv_outboard_);
  std::swap(num_subtree_mobods_, other.num_subtree_mobods_);
}

void SpanningForest::Mobod::RenumberMobodIndexVector(
    const std::vector<MobodIndex>& old_to_new,
    std::vector<MobodIndex>* to_be_renumbered) {
  for (MobodIndex& index : *to_be_renumbered) {
    if (!index.is_valid()) continue;
    DRAKE_ASSERT(index < old_to_new.size());
    index = old_to_new[index];
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
