#include "drake/geometry/proximity/collision_filter.h"

#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"

namespace drake {
namespace geometry {
namespace internal {

using std::unordered_set;

void CollisionFilter::Apply(const CollisionFilterDeclaration& declaration,
                            const CollisionFilter::ExtractIds& extract_ids,
                            bool is_permanent) {
  using Operation = CollisionFilterDeclaration::StatementOp;
  for (const auto& statement : declaration.statements()) {
    switch (statement.operation) {
      case Operation::kAllowBetween:
        RemoveFiltersBetween(statement.set_A, statement.set_B, extract_ids);
        break;
      case Operation::kAllowWithin:
        RemoveFiltersBetween(statement.set_A, statement.set_A, extract_ids);
        break;
      case Operation::kExcludeWithin:
        AddFiltersBetween(statement.set_A, statement.set_A, extract_ids,
                          is_permanent);
        break;
      case Operation::kExcludeBetween:
        AddFiltersBetween(statement.set_A, statement.set_B, extract_ids,
                          is_permanent);
        break;
    }
  }
}

void CollisionFilter::AddGeometry(GeometryId new_id) {
  DRAKE_DEMAND(filter_state_.count(new_id) == 0);
  GeometryMap& new_map = filter_state_[new_id] = {};
  for (auto& [other_id, other_map] : filter_state_) {
    /* Whichever id is *smaller* tracks the relationship with the other.
     That relationship defaults to unfiltered (i.e., "can collide").

     Note: we're iterating over filter_state_ and assigning to either new_map
     or other_map. This doesn't invalidate the implicit iterator of the range
     operator. We never change the *keys* of filter_state_ in this loop, only
     its values. And we don't do any iteration in the new_map or other_map
     so don't depend on their iterators. */
    if (other_id < new_id) {
      other_map[new_id] = kUnfiltered;
    } else {
      new_map[other_id] = kUnfiltered;
    }
  }
}

void CollisionFilter::RemoveGeometry(GeometryId remove_id) {
  DRAKE_DEMAND(filter_state_.count(remove_id) == 1);
  filter_state_.erase(remove_id);
  for (auto& [other_id, other_map] : filter_state_) {
    /* remove_id will only be found in maps belonging to geometries with ids
     that are smaller than remove_id. Those that are larger were deleted when we
     removed remove_id's entry. */
    if (other_id < remove_id) {
      other_map.erase(remove_id);
    }
  }
}

bool CollisionFilter::CanCollideWith(GeometryId id_A, GeometryId id_B) const {
  if (id_A == id_B) return false;
  if (id_A < id_B) {
    return filter_state_.at(id_A).at(id_B) == kUnfiltered;
  } else {
    return filter_state_.at(id_B).at(id_A) == kUnfiltered;
  }
}

void CollisionFilter::AddFiltersBetween(
    const GeometrySet& set_A, const GeometrySet& set_B,
    const CollisionFilter::ExtractIds& extract_ids, bool is_permanent) {
  const std::unordered_set<GeometryId> ids_A = extract_ids(set_A);
  const std::unordered_set<GeometryId>& ids_B =
      &set_A == &set_B ? ids_A : extract_ids(set_B);
  for (GeometryId id_A : ids_A) {
    for (GeometryId id_B : ids_B) {
      AddFilteredPair(id_A, id_B, is_permanent);
    }
  }
}

void CollisionFilter::RemoveFiltersBetween(
    const GeometrySet& set_A, const GeometrySet& set_B,
    const CollisionFilter::ExtractIds& extract_ids) {
  const unordered_set<GeometryId> ids_A = extract_ids(set_A);
  const unordered_set<GeometryId>& ids_B =
      &set_A == &set_B ? ids_A : extract_ids(set_B);
  for (GeometryId id_A : ids_A) {
    for (GeometryId id_B : ids_B) {
      RemoveFilteredPair(id_A, id_B);
    }
  }
}

void CollisionFilter::AddFilteredPair(GeometryId id_A, GeometryId id_B,
                                      bool is_permanent) {
  DRAKE_ASSERT(filter_state_.count(id_A) == 1 &&
               filter_state_.count(id_B) == 1);

  if (id_A == id_B) return;
  PairFilterState& pair_state =
      id_A < id_B ? filter_state_[id_A][id_B] : filter_state_[id_B][id_A];
  if (pair_state == kLockedFiltered) return;
  pair_state = is_permanent ? kLockedFiltered : kFiltered;
}

void CollisionFilter::RemoveFilteredPair(GeometryId id_A, GeometryId id_B) {
  DRAKE_ASSERT(filter_state_.count(id_A) == 1 &&
               filter_state_.count(id_B) == 1);
  if (id_A == id_B) return;
  PairFilterState& pair_state =
      id_A < id_B ? filter_state_[id_A][id_B] : filter_state_[id_B][id_A];
  if (pair_state == kLockedFiltered) return;
  pair_state = kUnfiltered;
}

bool CollisionFilter::operator==(const CollisionFilter& other) const {
  if (this == &other) return true;
  if (filter_state_.size() != other.filter_state_.size()) return false;
  for (const auto& [this_id, this_map] : filter_state_) {
    if (!other.HasGeometry(this_id)) return false;
    for (const auto& [this_pair_id, can_collide] : this_map) {
      unused(can_collide);
      if (!other.HasGeometry(this_pair_id)) return false;
      if (CanCollideWith(this_id, this_pair_id) !=
          other.CanCollideWith(this_id, this_pair_id)) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
