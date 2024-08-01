// NOLINTNEXTLINE(build/include): prevent complaint re spanning_forest_mobod.h
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/multibody/topology/forest.h"

namespace drake {
namespace multibody {
namespace internal {

// Constructor for the World mobilized body.
SpanningForest::Mobod::Mobod(MobodIndex mobod_index,
                             LinkOrdinal world_link_ordinal)
    : level_(0),
      index_(mobod_index),
      q_start_(0),
      nq_(0),
      v_start_(0),
      nv_(0),
      nq_inboard_(0),
      nv_inboard_(0) {
  DRAKE_DEMAND(mobod_index.is_valid() && world_link_ordinal.is_valid());
  DRAKE_DEMAND(world_link_ordinal == 0 && mobod_index == 0);
  follower_link_ordinals_.push_back(world_link_ordinal);
}

// Constructor for mobilized bodies other than World.
SpanningForest::Mobod::Mobod(MobodIndex mobod_index, LinkOrdinal link_ordinal,
                             JointOrdinal joint_ordinal, int level,
                             bool is_reversed)
    : joint_ordinal_(joint_ordinal),
      use_reverse_mobilizer_(is_reversed),
      level_(level),
      index_(mobod_index) {
  DRAKE_DEMAND(mobod_index.is_valid() && link_ordinal.is_valid() &&
               joint_ordinal.is_valid());
  DRAKE_DEMAND(mobod_index != 0 && link_ordinal != 0 && level > 0);
  follower_link_ordinals_.push_back(link_ordinal);
}

void SpanningForest::Mobod::FixupAfterReordering(
    const std::vector<MobodIndex>& old_to_new) {
  index_ = old_to_new[index_];
  if (!is_world()) inboard_mobod_ = old_to_new[inboard_mobod_];
  RenumberMobodIndexVector(old_to_new, &outboard_mobods_);
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
