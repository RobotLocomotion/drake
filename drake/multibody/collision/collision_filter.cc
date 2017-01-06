#include "drake/multibody/collision/collision_filter.h"

#include "drake/common/eigen_autodiff_types.h"

namespace DrakeCollision {

using drake::AutoDiffXd;

template <typename T>
CollisionFilterGroup<T>::CollisionFilterGroup() : name_(""), model_id_(-1) {}

template <typename T>
CollisionFilterGroup<T>::CollisionFilterGroup(const std::string& name,
                                              int model_id)
    : name_(name), model_id_(model_id) {}

template <typename T>
void CollisionFilterGroupManager<T>::DefineCollisionFilterGroup(
    const std::string& name, int model_id) {
  auto itr = collision_filter_groups_.find(name);
  if (itr == collision_filter_groups_.end()) {
    collision_filter_groups_[name] = CollisionFilterGroup<T>(name, model_id);
  }
  throw std::runtime_error(
      "Attempting to create duplicate collision filter group: " + name);
}

template <typename T>
bool CollisionFilterGroupManager<T>::AddCollisionFilterGroupMember(
    const std::string& group_name, RigidBody<T>* body) {
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
int CollisionFilterGroupManager<T>::GetGroupModelInstanceId(
    const std::string& group_name) {
  auto itr = collision_filter_groups_.find(group_name);
  if (itr == collision_filter_groups_.end()) {
    return -1;
  }
  return itr->second.get_model_id();
}

// Explicitly instantiates on the most common scalar types.
template class CollisionFilterGroup<double>;
template class CollisionFilterGroup<AutoDiffXd>;
template class CollisionFilterGroupManager<double>;
template class CollisionFilterGroupManager<AutoDiffXd>;

}  // namespace DrakeCollision
