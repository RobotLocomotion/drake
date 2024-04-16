#include "drake/multibody/parsing/collision_filter_groups.h"

#include "drake/multibody/parsing/detail_collision_filter_groups_impl.h"

namespace drake {
namespace multibody {

using internal::CollisionFilterGroupsImpl;

CollisionFilterGroups::CollisionFilterGroups()
    : impl_(new CollisionFilterGroupsImpl<std::string>) {}
CollisionFilterGroups::~CollisionFilterGroups() {}

CollisionFilterGroups::CollisionFilterGroups(const CollisionFilterGroups&) =
    default;
CollisionFilterGroups& CollisionFilterGroups::operator=(
    const CollisionFilterGroups&) = default;
CollisionFilterGroups::CollisionFilterGroups(CollisionFilterGroups&&) = default;
CollisionFilterGroups& CollisionFilterGroups::operator=(
    CollisionFilterGroups&&) = default;

bool CollisionFilterGroups::operator==(
    const CollisionFilterGroups& that) const {
  if (impl_.empty() != that.impl_.empty()) {
    return false;
  }
  if (impl_.empty()) {
    return true;
  }
  return *impl_ == *that.impl_;
}
std::weak_ordering CollisionFilterGroups::operator<=>(
    const CollisionFilterGroups& that) const {
  if (auto cmp = (!impl_.empty() <=> !that.impl_.empty()); cmp != 0) {
    return cmp;
  }
  if (impl_.empty()) {
    return std::weak_ordering::equivalent;
  }
  return *impl_ <=> *that.impl_;
}

void CollisionFilterGroups::AddGroup(const std::string& name,
                                     const std::set<std::string>& members) {
  DRAKE_THROW_UNLESS(!is_moved_from());
  impl_->AddGroup(name, members);
}

void CollisionFilterGroups::AddExclusionPair(
    const SortedPair<std::string>& pair) {
  impl_->AddExclusionPair(pair);
  DRAKE_THROW_UNLESS(!is_moved_from());
}

bool CollisionFilterGroups::empty() const {
  return impl_.empty() || impl_->empty();
}

const std::map<std::string, std::set<std::string>>&
CollisionFilterGroups::groups() const {
  DRAKE_THROW_UNLESS(!is_moved_from());
  return impl_->groups();
}

const std::set<SortedPair<std::string>>&
CollisionFilterGroups::exclusion_pairs() const {
  DRAKE_THROW_UNLESS(!is_moved_from());
  return impl_->exclusion_pairs();
}

bool CollisionFilterGroups::is_moved_from() const {
  return impl_.empty();
}

std::ostream& operator<<(std::ostream& os, const CollisionFilterGroups& g) {
  DRAKE_THROW_UNLESS(!g.is_moved_from());
  os << *g.impl_;
  return os;
}

}  // namespace multibody
}  // namespace drake
