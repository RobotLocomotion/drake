#include "drake/geometry/proximity/collision_filter.h"

#include <algorithm>
#include <stdexcept>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"

namespace drake {
namespace geometry {
namespace internal {

CollisionFilter::CollisionFilter() = default;

void CollisionFilter::Apply(const CollisionFilterDeclaration& declaration,
                            const CollisionFilter::ExtractIds& extract_ids,
                            bool is_invariant) {
  if (has_transient_history()) {
    throw std::runtime_error(
        "You cannot attempt to modify the persistent collision filter "
        "configuration when there are active, transient filter declarations");
  }
  /* Resolve and validate all IDs once, before touching any state. */
  const std::vector<ResolvedStatement> resolved =
      ResolveStatementsOrThrow(declaration, extract_ids, is_invariant);
  const StateDelta delta{resolved, FilterId{}};

  ApplyStatements(delta, &persistent_base_);
  /* Keep the cached composite and the persistent base in sync. We *could*
   simply copy the persistent base to the filter state, but we assume that the
   application of a declaration is a generally cheaper operation than a
   wholesale copy. */
  ApplyStatements(delta, &filter_state_);
}

FilterId CollisionFilter::ApplyTransient(
    const CollisionFilterDeclaration& declaration,
    const CollisionFilter::ExtractIds& extract_ids) {
  /* Transient declarations are never invariant. Resolve and validate all IDs
   before touching state, so the callback is not re-invoked at replay time. */
  std::vector<ResolvedStatement> resolved = ResolveStatementsOrThrow(
      declaration, extract_ids, /*is_invariant=*/false);

  const FilterId new_id = FilterId::get_new_id();
  StateDelta delta{std::move(resolved), new_id};

  /* Apply to the cached composite first, then store the delta. */
  ApplyStatements(delta, &filter_state_);
  transient_history_.push_back(std::move(delta));
  return new_id;
}

bool CollisionFilter::IsActive(FilterId id) const {
  for (const StateDelta& delta : transient_history_) {
    if (id == delta.id) return true;
  }
  return false;
}

bool CollisionFilter::RemoveDeclaration(FilterId id) {
  for (auto it = transient_history_.begin(); it != transient_history_.end();
       ++it) {
    if (it->id != id) continue;

    /* Remove the target delta, then rebuild the composite from the persistent
     base plus the remaining transient deltas. Because multiple deltas can
     touch the same pair, we cannot simply invert the removed delta in place --
     we must replay from scratch to get the correct result. */
    transient_history_.erase(it);
    RebuildComposite();
    return true;
  }
  return false;
}

void CollisionFilter::SetActiveStatus(
    const GeometrySet& geometry_set, bool active,
    const CollisionFilter::ExtractIds& extract_ids,
    const CollisionFilter::ActiveStatusChangeCallback& on_change) {
  const std::unordered_set<GeometryId> ids =
      extract_ids(geometry_set, CollisionFilterScope::kAll);
  /* Build the net change directly from the ids whose membership actually flips;
   the inactive set is independent of the pairwise sets and the transient
   history, so it is edited in place. */
  ActiveStatusChange change;
  if (active) {
    for (GeometryId id : ids) {
      if (inactive_.erase(id) > 0) {
        change.activated.push_back(id);
      }
    }
  } else {
    for (GeometryId id : ids) {
      if (inactive_.insert(id).second) {
        change.deactivated.push_back(id);
      }
    }
  }
  if (!change.empty()) {
    on_change(change);
  }
}

void CollisionFilter::Flatten() {
  if (!transient_history_.empty()) {
    /* The composite already reflects all transient deltas applied, so just
     promote it to the persistent base and clear the history. */
    persistent_base_ = filter_state_;
    transient_history_.clear();
  }
}

void CollisionFilter::AddGeometry(GeometryId new_id) {
  DRAKE_DEMAND(!geometries_.contains(new_id));
  geometries_.insert(new_id);
  /* No pair entries are needed: the sparse representation stores only filtered
   pairs, and new geometry is unfiltered by default. Transient deltas store
   only resolved GeometryId sets, so they also require no update. */
}

void CollisionFilter::RemoveGeometry(GeometryId remove_id) {
  DRAKE_DEMAND(geometries_.contains(remove_id));
  geometries_.erase(remove_id);
  inactive_.erase(remove_id);

  RemovePairsFor(remove_id, &filter_state_);
  RemovePairsFor(remove_id, &persistent_base_);

  /* Purge the removed id from all resolved statement sets in transient history
   so that future replays do not reference a geometry that no longer exists. */
  for (StateDelta& delta : transient_history_) {
    for (ResolvedStatement& statement : delta.statements) {
      std::erase(statement.set_A, remove_id);
      std::erase(statement.set_B, remove_id);
    }
  }

  /* Rebuild composite from the updated persistent base + purged transients. */
  if (has_transient_history()) {
    RebuildComposite();
  }
}

bool CollisionFilter::CanCollideWith(GeometryId id_A, GeometryId id_B) const {
  if (id_A == id_B) return false;
  /* Inactive geometries first: an inactive geometry collides with nothing,
   including geometries registered after it was deactivated. This function runs
   once per broadphase candidate pair, so the empty() guard keeps the common
   (all-active) case at a single branch. */
  if (!inactive_.empty() &&
      (inactive_.contains(id_A) || inactive_.contains(id_B))) {
    return false;
  }
  const PairKey key(id_A, id_B);
  return !filter_state_.blocked.contains(key);
}

bool CollisionFilter::IsEquivalent(const CollisionFilter& other) const {
  if (this == &other) return true;
  if (geometries_ != other.geometries_) return false;
  if (inactive_ != other.inactive_) {
    return false;
  }
  /* Two filters are equal iff CanCollideWith() agrees on every pair. A pair is
   blocked iff it is in filtered OR invariant, so we check that every blocked
   pair in either filter is also blocked in the other. */
  auto is_blocked = [](const FilterState& fs, const PairKey& k) {
    return fs.filtered.contains(k) || fs.invariant.contains(k);
  };
  for (const PairKey& key : filter_state_.filtered) {
    if (!is_blocked(other.filter_state_, key)) return false;
  }
  for (const PairKey& key : filter_state_.invariant) {
    if (!is_blocked(other.filter_state_, key)) return false;
  }
  for (const PairKey& key : other.filter_state_.filtered) {
    if (!is_blocked(filter_state_, key)) return false;
  }
  for (const PairKey& key : other.filter_state_.invariant) {
    if (!is_blocked(filter_state_, key)) return false;
  }
  return true;
}

CollisionFilter CollisionFilter::MakeClearCopy() const {
  CollisionFilter copy;
  copy.geometries_ = geometries_;
  /* filtered, invariant, and inactive_ are intentionally left empty. */
  return copy;
}

void CollisionFilter::AddPairsBetween(const std::vector<GeometryId>& set_A,
                                      const std::vector<GeometryId>& set_B,
                                      bool is_invariant, FilterState* state) {
  for (GeometryId id_A : set_A) {
    for (GeometryId id_B : set_B) {
      if (id_A == id_B) continue;
      const PairKey key(id_A, id_B);
      /* Never downgrade an invariant pair. */
      if (state->invariant.contains(key)) continue;
      if (is_invariant) {
        state->filtered.erase(key);
        state->invariant.insert(key);
      } else {
        state->filtered.insert(key);
      }
      state->blocked.insert(key);
    }
  }
}

void CollisionFilter::RemovePairsBetween(const std::vector<GeometryId>& set_A,
                                         const std::vector<GeometryId>& set_B,
                                         FilterState* state) {
  for (GeometryId id_A : set_A) {
    for (GeometryId id_B : set_B) {
      if (id_A == id_B) continue;
      const PairKey key(id_A, id_B);
      if (state->filtered.erase(key) > 0) {
        /* filtered and invariant are disjoint, so removal from filtered means
         the pair is no longer blocked. */
        state->blocked.erase(key);
      }
    }
  }
}

void CollisionFilter::RemovePairsFor(GeometryId id, FilterState* state) {
  auto pair_has_id = [id](const PairKey& k) {
    return k.first() == id || k.second() == id;
  };
  std::erase_if(state->filtered, pair_has_id);
  std::erase_if(state->invariant, pair_has_id);
  std::erase_if(state->blocked, pair_has_id);
}

void CollisionFilter::ApplyStatement(const ResolvedStatement& statement,
                                     FilterState* state) {
  using Op = CollisionFilterDeclaration::StatementOp;
  switch (statement.operation) {
    case Op::kExcludeBetween:
      AddPairsBetween(statement.set_A, statement.set_B, statement.is_invariant,
                      state);
      break;
    case Op::kExcludeWithin:
      AddPairsBetween(statement.set_A, statement.set_A, statement.is_invariant,
                      state);
      break;
    case Op::kAllowBetween:
      DRAKE_DEMAND(!statement.is_invariant);
      RemovePairsBetween(statement.set_A, statement.set_B, state);
      break;
    case Op::kAllowWithin:
      DRAKE_DEMAND(!statement.is_invariant);
      RemovePairsBetween(statement.set_A, statement.set_A, state);
      break;
  }
}

void CollisionFilter::ApplyStatements(const StateDelta& delta,
                                      FilterState* state) {
  for (const ResolvedStatement& statement : delta.statements) {
    ApplyStatement(statement, state);
  }
}

std::vector<CollisionFilter::ResolvedStatement>
CollisionFilter::ResolveStatementsOrThrow(
    const CollisionFilterDeclaration& declaration,
    const CollisionFilter::ExtractIds& extract_ids, bool is_invariant) const {
  using Statement = CollisionFilterDeclaration::Statement;
  using Op = CollisionFilterDeclaration::StatementOp;
  const CollisionFilterScope scope = declaration.scope();

  std::vector<ResolvedStatement> resolved;
  resolved.reserve(declaration.statements().size());
  for (const Statement& statement : declaration.statements()) {
    ResolvedStatement rs;
    rs.operation = statement.operation;
    rs.is_invariant = is_invariant;

    const std::unordered_set<GeometryId> ids_A =
        extract_ids(statement.set_A, scope);
    ThrowIfAnyUnregistered(ids_A);
    // Note: here and on rs.set_B below, we convert unordered_set to vector.
    // This is simply because we may iterate through the set multiple times
    // (particularly if it's part of a transient declaration) and prefer
    // memory-contiguous representation to keep that running smoothly.
    rs.set_A.assign(ids_A.begin(), ids_A.end());

    if (statement.operation == Op::kExcludeBetween ||
        statement.operation == Op::kAllowBetween) {
      const std::unordered_set<GeometryId> ids_B =
          extract_ids(statement.set_B, scope);
      ThrowIfAnyUnregistered(ids_B);
      rs.set_B.assign(ids_B.begin(), ids_B.end());
    }
    resolved.push_back(std::move(rs));
  }
  return resolved;
}

void CollisionFilter::ThrowIfAnyUnregistered(
    const std::unordered_set<GeometryId>& ids) const {
  for (GeometryId id : ids) {
    if (!geometries_.contains(id)) {
      throw std::runtime_error(fmt::format(
          "Collision filter declaration references geometry id {} which has "
          "not been registered with this filter system.",
          id));
    }
  }
}

void CollisionFilter::RebuildComposite() {
  /* Start from the persistent base and replay all transient deltas in order. */
  filter_state_ = persistent_base_;
  for (const StateDelta& delta : transient_history_) {
    ApplyStatements(delta, &filter_state_);
  }
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
