#include "drake/geometry/proximity/collision_filter.h"

#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"

namespace drake {
namespace geometry {
namespace internal {

using std::unordered_set;

CollisionFilter::CollisionFilter() {
  /* The filter history always has at least one entry -- entry[0] is the
   persistent base. We associate it with a geometry id that can never be
   accessed via ApplyTransient() so that it can't accidentally be removed. */
  filter_history_.emplace_back(FilterState{}, FilterId::get_new_id());
}

// TODO(SeanCurtis-TRI): Multiple calls to Apply will lead to multiple
//  constructions of std::unordered_set from GeometrySet (as opposed to
//  re-using a single set). If this is a performance issue, revisit this so
//  that I can operate on multiple filter states *per statement*.
void CollisionFilter::Apply(const CollisionFilterDeclaration& declaration,
                            const CollisionFilter::ExtractIds& extract_ids,
                            bool is_permanent) {
  if (filter_history_.size() > 1) {
    throw std::runtime_error(
        "You cannot attempt to modify the persistent collision filter "
        "configuration when there are active, transient filter declarations");
  }
  /* Keep current configuration and persistent base in sync. */
  Apply(declaration, extract_ids, is_permanent, &filter_state_);
  Apply(declaration, extract_ids, is_permanent,
        &filter_history_[0].filter_state);
}

FilterId CollisionFilter::ApplyTransient(
    const CollisionFilterDeclaration& declaration,
    const CollisionFilter::ExtractIds& extract_ids) {
  /* Apply to the current cached state and add a history entry. */
  const bool is_permanent = false;
  Apply(declaration, extract_ids, is_permanent, &filter_state_);
  filter_history_.emplace_back(
      InitializeTransientState(filter_state_, kUndefined),
      FilterId::get_new_id());
  Apply(declaration, extract_ids, is_permanent,
        &filter_history_.back().filter_state);
  return filter_history_.back().id;
}

bool CollisionFilter::IsActive(FilterId id) const {
  for (const auto& delta : filter_history_) {
    if (id == delta.id) return true;
  }
  return false;
}

bool CollisionFilter::RemoveDeclaration(FilterId id) {
  /* We skip the first entry, that is always the persistent base and can't be
   removed. */
  auto itr = ++filter_history_.begin();
  while (itr != filter_history_.end()) {
    if (itr->id == id) break;
    ++itr;
  }
  if (itr != filter_history_.end()) {
    /* We found a declaration to remove. We simply pop it out, relying on the
     std::vector to use move semantics to slide the subsequent entries down.
     Then we just copy the persistent base and "replay" the transient
     declarations. */
    filter_history_.erase(itr);
    filter_state_ = filter_history_[0].filter_state;
    for (size_t i = 1; i < filter_history_.size(); ++i) {
      const FilterState& state = filter_history_[i].filter_state;
      /* Note: If the filtered state in transient declarations was sparse, this
       application algorithm would directly benefit. It only requires that the
       persistent state has all of the required pairs. See the TODO in
       AddGeometry() for more discussion. */
      for (const auto& [src_id, src_map] : state) {
        for (const auto& [pair_id, pair_state] : src_map) {
          if (pair_state != kUndefined) {
            if (filter_state_[src_id][pair_id] != kLockedFiltered) {
              filter_state_[src_id][pair_id] = pair_state;
            }
          }
        }
      }
    }
    return true;
  }
  return false;
}

void CollisionFilter::Flatten() {
  if (filter_history_.size() > 1) {
    filter_history_.resize(1);
    filter_history_[0].filter_state = filter_state_;
  }
}

void CollisionFilter::AddGeometry(GeometryId new_id) {
  /* Current and persistent configurations should simply add the id with
   unfiltered status. */
  AddGeometry(new_id, &filter_state_, kUnfiltered);
  AddGeometry(new_id, &filter_history_[0].filter_state, kUnfiltered);
  // TODO(SeanCurtis): Can I skip this work by allowing delta filter state to be
  //  incomplete? If each transient delta *only* contains explicitly declared
  //  changes, iterating through it would be faster. More complex, but faster.
  /* Active transient history should add the id with undefined status. */
  for (size_t i = 1; i < filter_history_.size(); ++i) {
    AddGeometry(new_id, &filter_history_[i].filter_state, kUndefined);
  }
}

void CollisionFilter::RemoveGeometry(GeometryId remove_id) {
  /* Simply remove the geometry from everything. */
  RemoveGeometry(remove_id, &filter_state_);
  for (auto& delta : filter_history_) {
    RemoveGeometry(remove_id, &delta.filter_state);
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
    const CollisionFilter::ExtractIds& extract_ids, bool is_permanent,
    FilterState* state_ptr) {
  const std::unordered_set<GeometryId> ids_A = extract_ids(set_A);
  const std::unordered_set<GeometryId>& ids_B =
      &set_A == &set_B ? ids_A : extract_ids(set_B);
  for (GeometryId id_A : ids_A) {
    for (GeometryId id_B : ids_B) {
      AddFilteredPair(id_A, id_B, is_permanent, state_ptr);
    }
  }
}

void CollisionFilter::RemoveFiltersBetween(
    const GeometrySet& set_A, const GeometrySet& set_B,
    const CollisionFilter::ExtractIds& extract_ids, FilterState* state_ptr) {
  const std::unordered_set<GeometryId> ids_A = extract_ids(set_A);
  const std::unordered_set<GeometryId>& ids_B =
      &set_A == &set_B ? ids_A : extract_ids(set_B);
  for (GeometryId id_A : ids_A) {
    for (GeometryId id_B : ids_B) {
      RemoveFilteredPair(id_A, id_B, state_ptr);
    }
  }
}

void CollisionFilter::AddFilteredPair(GeometryId id_A, GeometryId id_B,
                                      bool is_permanent,
                                      FilterState* state_ptr) {
  FilterState& filter_state = *state_ptr;
  DRAKE_DEMAND(filter_state.count(id_A) == 1 &&
               filter_state.count(id_B) == 1);

  if (id_A == id_B) return;
  PairFilterState& pair_state =
      id_A < id_B ? filter_state[id_A][id_B] : filter_state[id_B][id_A];
  if (pair_state == kLockedFiltered) return;
  pair_state = is_permanent ? kLockedFiltered : kFiltered;
}

void CollisionFilter::RemoveFilteredPair(GeometryId id_A, GeometryId id_B,
                                         FilterState* state_ptr) {
  FilterState& filter_state = *state_ptr;
  DRAKE_DEMAND(filter_state.count(id_A) == 1 &&
               filter_state.count(id_B) == 1);
  if (id_A == id_B) return;
  PairFilterState& pair_state =
      id_A < id_B ? filter_state[id_A][id_B] : filter_state[id_B][id_A];
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

void CollisionFilter::Apply(const CollisionFilterDeclaration& declaration,
                            const CollisionFilter::ExtractIds& extract_ids,
                            bool is_permanent, FilterState* filter_state) {
  using Operation = CollisionFilterDeclaration::StatementOp;
  for (const auto& statement : declaration.statements()) {
    switch (statement.operation) {
      case Operation::kAllowBetween:
        RemoveFiltersBetween(statement.set_A, statement.set_B, extract_ids,
                             filter_state);
        break;
      case Operation::kAllowWithin:
        RemoveFiltersBetween(statement.set_A, statement.set_A, extract_ids,
                             filter_state);
        break;
      case Operation::kExcludeWithin:
        AddFiltersBetween(statement.set_A, statement.set_A, extract_ids,
                          is_permanent, filter_state);
        break;
      case Operation::kExcludeBetween:
        AddFiltersBetween(statement.set_A, statement.set_B, extract_ids,
                          is_permanent, filter_state);
        break;
    }
  }
}

void CollisionFilter::AddGeometry(GeometryId new_id,
                                  FilterState* filter_state_ptr,
                                  PairFilterState initial_state) {
  FilterState& filter_state = *filter_state_ptr;
  DRAKE_DEMAND(filter_state.count(new_id) == 0);
  GeometryMap& new_map = filter_state[new_id] = {};
  for (auto& [other_id, other_map] : filter_state) {
    /* Whichever id is *smaller* tracks the relationship with the other.
     That relationship defaults to unfiltered (i.e., "can collide").

     Note: we're iterating over filter_state_ and assigning to either new_map
     or other_map. This doesn't invalidate the implicit iterator of the range
     operator. We never change the *keys* of filter_state_ in this loop, only
     its values. And we don't do any iteration in the new_map or other_map
     so don't depend on their iterators. */
    if (other_id < new_id) {
      other_map[new_id] = initial_state;
    } else {
      new_map[other_id] = initial_state;
    }
  }
}

void CollisionFilter::RemoveGeometry(GeometryId id,
                                     FilterState* filter_state_ptr) {
  FilterState& filter_state = *filter_state_ptr;
  DRAKE_DEMAND(filter_state.count(id) == 1);
  filter_state.erase(id);
  for (auto& [other_id, other_map] : filter_state) {
    /* remove_id will only be found in maps belonging to geometries with ids
     that are smaller than remove_id. Those that are larger were deleted when we
     removed remove_id's entry. */
    if (other_id < id) {
      other_map.erase(id);
    }
  }
}

CollisionFilter::FilterState CollisionFilter::InitializeTransientState(
    const FilterState& reference, PairFilterState default_state) {
  FilterState new_state;
  for (const auto& [id, _] : reference) {
    unused(_);
    AddGeometry(id, &new_state, default_state);
  }
  return new_state;
}
}  // namespace internal
}  // namespace geometry
}  // namespace drake
