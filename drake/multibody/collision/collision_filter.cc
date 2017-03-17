#include "drake/multibody/collision/collision_filter.h"

#include <string>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/text_logging.h"

namespace DrakeCollision {

using drake::AutoDiffXd;

template <typename T>
CollisionFilterGroup<T>::CollisionFilterGroup() : name_(""), mask_id_(-1) {}

template <typename T>
CollisionFilterGroup<T>::CollisionFilterGroup(const std::string& name, int id)
    : name_(name), mask_id_(id) {
  DRAKE_ASSERT(mask_id_ >= 0 && mask_id_ < kMaxNumCollisionFilterGroups);
}

template <typename T>
const int CollisionFilterGroupManager<T>::kInvalidGroupId = -1;

template <typename T>
void CollisionFilterGroupManager<T>::DefineCollisionFilterGroup(
    const std::string& name) {
  auto itr = collision_filter_groups_.find(name);
  if (itr == collision_filter_groups_.end()) {
    int id = acquire_next_group_id();
    collision_filter_groups_[name] = CollisionFilterGroup<T>(name, id);
  } else {
    throw std::runtime_error(
        "Attempting to create duplicate collision filter group: " + name + ".");
  }
}

template <typename T>
void CollisionFilterGroupManager<T>::CompileGroups() {
  for (auto& pair : collision_filter_groups_) {
    CollisionFilterGroup<T>& group = pair.second;
    bitmask group_mask = kDefaultGroup, ignore_mask = kNoneMask;
    group_mask.set(group.get_mask_id());
    for (const auto& ignore_name : group.get_ignore_groups()) {
      const auto& itr = collision_filter_groups_.find(ignore_name);
      if (itr != collision_filter_groups_.end()) {
        ignore_mask.set(itr->second.get_mask_id());
      } else {
        drake::log()->warn(
            "Collision filter group '{}' was specified to ignore"
            " group '{}', but '{}' was not defined.",
            pair.first, ignore_name, ignore_name);
      }
    }
    for (const RigidBody<T>* body : group.get_bodies()) {
      auto itr = body_groups_.find(body);
      if (itr == body_groups_.end()) {
        body_groups_[body] = std::make_pair(group_mask, ignore_mask);
      } else {
        body_groups_[body].first |= group_mask;
        body_groups_[body].second |= ignore_mask;
      }
    }
  }
}

template <typename T>
bool CollisionFilterGroupManager<T>::AddCollisionFilterGroupMember(
    const std::string& group_name, const RigidBody<T>& body) {
  auto itr = collision_filter_groups_.find(group_name);
  if (itr == collision_filter_groups_.end()) {
    return false;
  }
  CollisionFilterGroup<T>& group = itr->second;
  group.add_body(body);
  return true;
}

template <typename T>
void CollisionFilterGroupManager<T>::AddCollisionFilterIgnoreTarget(
    const std::string& group_name, const std::string& target_group_name) {
  auto itr = collision_filter_groups_.find(group_name);
  if (itr == collision_filter_groups_.end()) {
    throw std::runtime_error(
        "Attempting to add an ignored collision filter group to an undefined "
        "collision filter group: Ignoring " +
        target_group_name + " by " + group_name + ".");
  }
  itr->second.add_ignore_group(target_group_name);
}

template <typename T>
int CollisionFilterGroupManager<T>::GetGroupId(const std::string& group_name) {
  auto itr = collision_filter_groups_.find(group_name);
  if (itr == collision_filter_groups_.end()) {
    return kInvalidGroupId;
  }
  return itr->second.get_mask_id();
}

template <typename T>
const bitmask& CollisionFilterGroupManager<T>::get_group_mask(
    const RigidBody<T>& body) {
  auto itr = body_groups_.find(&body);
  if (itr != body_groups_.end()) {
    return itr->second.first;
  }
  return kNoneMask;
}

template <typename T>
const bitmask& CollisionFilterGroupManager<T>::get_ignore_mask(
    const RigidBody<T>& body) {
  auto itr = body_groups_.find(&body);
  if (itr != body_groups_.end()) {
    return itr->second.second;
  }
  return kNoneMask;
}

template <typename T>
void CollisionFilterGroupManager<T>::SetBodyCollisionFilters(
    const RigidBody<T>& body, const bitmask& group, const bitmask& ignores) {
  body_groups_[&body] = std::make_pair(group, ignores);
}

template <typename T>
void CollisionFilterGroupManager<T>::Clear() {
  collision_filter_groups_.clear();
  body_groups_.clear();
}

template <typename T>
int CollisionFilterGroupManager<T>::acquire_next_group_id() {
  if (next_id_ >= kMaxNumCollisionFilterGroups) {
    throw std::runtime_error(
        "Requesting a collision filter group id when"
        " there are no more available. Drake only supports " +
        std::to_string(kMaxNumCollisionFilterGroups) +
        " collision filter groups per RigidBodyTree instance.");
  }
  return next_id_++;
}

// Explicitly instantiates on the most common scalar types.
template class CollisionFilterGroup<double>;
template class CollisionFilterGroup<AutoDiffXd>;
template class CollisionFilterGroupManager<double>;
template class CollisionFilterGroupManager<AutoDiffXd>;

}  // namespace DrakeCollision
