#include "drake/geometry/proximity/collision_filter.h"

#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"

namespace drake {
namespace geometry {
namespace internal {

using std::unordered_set;

CollisionFilter::CollisionFilter() {
  /* The filter history always has at least one entry -- entry[0] is the
   persistent base. We associate it with a filter id that can never be
   accessed via ApplyTransient() so that it can't accidentally be removed. */
  filter_history_.emplace_back(FilterState{}, FilterId::get_new_id());
}

// TODO(SeanCurtis-TRI): Multiple calls to Apply will lead to multiple
//  constructions of std::unordered_set from GeometrySet (as opposed to
//  re-using a single set). If this is a performance issue, revisit this so
//  that I can operate on multiple filter states *per statement*.
void CollisionFilter::Apply(const CollisionFilterDeclaration& declaration,
                            const CollisionFilter::ExtractIds& extract_ids,
                            bool is_invariant) {
  if (has_transient_history()) {
    throw std::runtime_error(
        "You cannot attempt to modify the persistent collision filter "
        "configuration when there are active, transient filter declarations");
  }
  // TODO(SeanCurtis-TRI): We're using a brute-force approach to update the
  //  cached composite state and the persistent state. We assume it's cheaper
  //  to "apply" the declaration twice than to apply once and copy. If this
  //  introduces too much cost during configuration, we have a couple of options
  //    - Find out if copying is faster.
  //    - In the case where there is *only* persistent state, don't use the
  //      copy as the persistent state *is* its own composite state.
  /* Keep current configuration and persistent base in sync. */
  Apply(declaration, extract_ids, is_invariant, &filter_state_);
  Apply(declaration, extract_ids, is_invariant,
        &filter_history_[0].filter_state);
}

FilterId CollisionFilter::ApplyTransient(
    const CollisionFilterDeclaration& declaration,
    const CollisionFilter::ExtractIds& extract_ids) {
  /* By its very definition, transient declarations *cannot* be invariant. They
   are never used by the system to implement invariants and users cannot declare
   a filter to be invariant. */
  const bool is_invariant = false;

  /* Using the same brute-force approach as in Apply(), we need to apply the
   declaration to our cached, composite result (filter_state_). Then we need
   to add it to the history by:

     1. Creating a new FilterState instance (with all the registered geometry
        in final_state_).
     2. Apply the declaration to that new state in the history. */
  // TODO(SeanCurtis-TRI): The brute force approach introduces several costs.
  //  In addition to the cost of converting GeometrySet --> set<GeometryId>
  //  twice, we are instantiating a full FilterState for what might be a small
  //  declaration and applying the declaration twice. If this cost is too high,
  //  even for out-of-the-loop configuration, revisit how we represent the
  //  history and maintain the composite copy.
  Apply(declaration, extract_ids, is_invariant, &filter_state_);
  filter_history_.emplace_back(
      InitializeTransientState(filter_state_, kUndefined),
      FilterId::get_new_id());
  Apply(declaration, extract_ids, is_invariant,
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
  for (auto it = filter_history_.begin() + 1; it != filter_history_.end();
       ++it) {
    if (it->id != id) continue;

    /* We found a declaration to remove. We simply pop it out, relying on the
     std::vector to use move semantics to slide the subsequent entries down.
     Then we just copy the persistent base and "replay" the transient
     declarations. */
    filter_history_.erase(it);
    filter_state_ = filter_history_[0].filter_state;
    for (size_t i = 1; i < filter_history_.size(); ++i) {
      const FilterState& state = filter_history_[i].filter_state;
      /* Note: If the filtered state in transient declarations was sparse, this
       application algorithm would directly benefit. It only requires that the
       persistent state has all of the required pairs. See the TODO in
       AddGeometry() for more discussion. */
      for (const auto& [source_id, source_map] : state) {
        for (const auto& [pair_id, pair_relation] : source_map) {
          if (pair_relation != kUndefined) {
            if (filter_state_[source_id][pair_id] != kInvariantFilter) {
              filter_state_[source_id][pair_id] = pair_relation;
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
    const CollisionFilter::ExtractIds& extract_ids, bool is_invariant,
    FilterState* state_out) {
  const std::unordered_set<GeometryId> ids_A = extract_ids(set_A);
  const std::unordered_set<GeometryId>& ids_B =
      &set_A == &set_B ? ids_A : extract_ids(set_B);
  for (GeometryId id_A : ids_A) {
    for (GeometryId id_B : ids_B) {
      AddFilteredPair(id_A, id_B, is_invariant, state_out);
    }
  }
}

void CollisionFilter::RemoveFiltersBetween(
    const GeometrySet& set_A, const GeometrySet& set_B,
    const CollisionFilter::ExtractIds& extract_ids, FilterState* state_out) {
  const std::unordered_set<GeometryId> ids_A = extract_ids(set_A);
  const std::unordered_set<GeometryId>& ids_B =
      &set_A == &set_B ? ids_A : extract_ids(set_B);
  for (GeometryId id_A : ids_A) {
    for (GeometryId id_B : ids_B) {
      RemoveFilteredPair(id_A, id_B, state_out);
    }
  }
}

void CollisionFilter::AddFilteredPair(GeometryId id_A, GeometryId id_B,
                                      bool is_invariant,
                                      FilterState* state_out) {
  FilterState& filter_state = *state_out;
  DRAKE_DEMAND(filter_state.count(id_A) == 1 &&
               filter_state.count(id_B) == 1);

  if (id_A == id_B) return;
  PairRelationship& pair_relation =
      id_A < id_B ? filter_state[id_A][id_B] : filter_state[id_B][id_A];
  if (pair_relation == kInvariantFilter) return;
  pair_relation = is_invariant ? kInvariantFilter : kFiltered;
}

void CollisionFilter::RemoveFilteredPair(GeometryId id_A, GeometryId id_B,
                                         FilterState* state_out) {
  FilterState& filter_state = *state_out;
  DRAKE_DEMAND(filter_state.count(id_A) == 1 &&
               filter_state.count(id_B) == 1);
  if (id_A == id_B) return;
  PairRelationship& pair_relation =
      id_A < id_B ? filter_state[id_A][id_B] : filter_state[id_B][id_A];
  if (pair_relation == kInvariantFilter) return;
  pair_relation = kUnfiltered;
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

CollisionFilter CollisionFilter::MakeClearCopy() const {
  auto clear_state = CollisionFilter::InitializeTransientState(
      filter_state_, CollisionFilter::kUnfiltered);
  CollisionFilter new_filter;
  new_filter.filter_state_ = clear_state;
  new_filter.filter_history_[0].filter_state = clear_state;
  return new_filter;
}

void CollisionFilter::Apply(const CollisionFilterDeclaration& declaration,
                            const CollisionFilter::ExtractIds& extract_ids,
                            bool is_invariant, FilterState* filter_state) {
  using Operation = CollisionFilterDeclaration::StatementOp;
  for (const auto& statement : declaration.statements()) {
    switch (statement.operation) {
      case Operation::kAllowBetween:
        // Note: GeometryState should never be declaring is_invariant is true
        // while removing collision filters.
        DRAKE_DEMAND(!is_invariant);
        RemoveFiltersBetween(statement.set_A, statement.set_B, extract_ids,
                             filter_state);
        break;
      case Operation::kAllowWithin:
        DRAKE_DEMAND(!is_invariant);
        RemoveFiltersBetween(statement.set_A, statement.set_A, extract_ids,
                             filter_state);
        break;
      case Operation::kExcludeWithin:
        AddFiltersBetween(statement.set_A, statement.set_A, extract_ids,
                          is_invariant, filter_state);
        break;
      case Operation::kExcludeBetween:
        AddFiltersBetween(statement.set_A, statement.set_B, extract_ids,
                          is_invariant, filter_state);
        break;
    }
  }
}

void CollisionFilter::AddGeometry(GeometryId new_id,
                                  FilterState* filter_state_out,
                                  PairRelationship relationship) {
  FilterState& filter_state = *filter_state_out;
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
      other_map[new_id] = relationship;
    } else {
      new_map[other_id] = relationship;
    }
  }
}

void CollisionFilter::RemoveGeometry(GeometryId remove_id,
                                     FilterState* filter_state_out) {
  FilterState& filter_state = *filter_state_out;
  DRAKE_DEMAND(filter_state.count(remove_id) == 1);
  filter_state.erase(remove_id);
  for (auto& [other_id, other_map] : filter_state) {
    /* remove_id will only be found in maps belonging to geometries with ids
     that are smaller than remove_id. Those that are larger were deleted when we
     removed remove_id's entry. */
    if (other_id < remove_id) {
      other_map.erase(remove_id);
    }
  }
}

CollisionFilter::FilterState CollisionFilter::InitializeTransientState(
    const FilterState& reference, PairRelationship default_relationship) {
  FilterState new_state;
  for (const auto& [id, _] : reference) {
    unused(_);
    AddGeometry(id, &new_state, default_relationship);
  }
  return new_state;
}
}  // namespace internal
}  // namespace geometry
}  // namespace drake
