#include "drake/multibody/parsing/collision_filter_groups.h"

#include <memory>

#include "drake/multibody/parsing/detail_collision_filter_groups_impl.h"

namespace drake {
namespace multibody {

using internal::CollisionFilterGroupsImpl;

CollisionFilterGroups::CollisionFilterGroups()
    : impl_(std::make_unique<CollisionFilterGroupsImpl<std::string>>()) {}
CollisionFilterGroups::~CollisionFilterGroups() = default;

CollisionFilterGroups::CollisionFilterGroups(const CollisionFilterGroups&) =
    default;
CollisionFilterGroups& CollisionFilterGroups::operator=(
    const CollisionFilterGroups&) = default;

// Move operations must be careful to preserve the non-null invariant.
CollisionFilterGroups::CollisionFilterGroups(CollisionFilterGroups&& other) {
  impl_ = std::move(other.impl_);
  other.impl_ = std::make_unique<CollisionFilterGroupsImpl<std::string>>();
}
CollisionFilterGroups& CollisionFilterGroups::operator=(
    CollisionFilterGroups&& other) {
  std::swap(impl_, other.impl_);
  return *this;
}

// The "internal use only" move constructor.
CollisionFilterGroups::CollisionFilterGroups(
    CollisionFilterGroupsImpl<std::string>&& impl)
    : impl_(std::make_unique<CollisionFilterGroupsImpl<std::string>>(
          std::move(impl))) {}

bool CollisionFilterGroups::operator==(
    const CollisionFilterGroups& that) const {
  return *impl_ == *that.impl_;
}

void CollisionFilterGroups::AddGroup(const std::string& name,
                                     const std::set<std::string>& members) {
  impl_->AddGroup(name, members);
}

void CollisionFilterGroups::AddExclusionPair(
    const SortedPair<std::string>& pair) {
  impl_->AddExclusionPair(pair);
}

bool CollisionFilterGroups::empty() const {
  return impl_->empty();
}

const std::map<std::string, std::set<std::string>>&
CollisionFilterGroups::groups() const {
  return impl_->groups();
}

const std::set<SortedPair<std::string>>&
CollisionFilterGroups::exclusion_pairs() const {
  return impl_->exclusion_pairs();
}

std::string CollisionFilterGroups::to_string() const {
  return impl_->to_string();
}

}  // namespace multibody
}  // namespace drake
