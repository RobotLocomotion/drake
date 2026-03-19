#include "drake/geometry/proximity/collision_filter.h"

#include <algorithm>

#include "drake/common/drake_assert.h"

namespace drake {
namespace geometry {
namespace internal {

CollisionFilter::CollisionFilter() = default;

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
  /* Keep the cached composite and the persistent base in sync. */
  ApplyDeclarationToState(declaration, extract_ids, is_invariant,
                          &filter_state_);
  ApplyDeclarationToState(declaration, extract_ids, is_invariant,
                          &persistent_base_);
}

FilterId CollisionFilter::ApplyTransient(
    const CollisionFilterDeclaration& declaration,
    const CollisionFilter::ExtractIds& extract_ids) {
  /* Transient declarations are never invariant. */
  using Statement = CollisionFilterDeclaration::Statement;
  using Op = CollisionFilterDeclaration::StatementOp;
  const CollisionFilterScope scope = declaration.scope();

  /* Resolve every statement's GeometrySet to explicit GeometryId vectors now,
   while we have the extract_ids callback. The resolved statements are stored
   compactly in the StateDelta and replayed cheaply in RebuildComposite()
   without needing the callback again. */
  std::vector<ResolvedStatement> resolved;
  resolved.reserve(declaration.statements().size());
  for (const Statement& statement : declaration.statements()) {
    ResolvedStatement rs;
    rs.operation = statement.operation;
    const std::unordered_set<GeometryId> ids_A =
        extract_ids(statement.set_A, scope);
    rs.set_A.assign(ids_A.begin(), ids_A.end());
    if (statement.operation == Op::kExcludeBetween ||
        statement.operation == Op::kAllowBetween) {
      const std::unordered_set<GeometryId> ids_B =
          extract_ids(statement.set_B, scope);
      rs.set_B.assign(ids_B.begin(), ids_B.end());
    }
    resolved.push_back(std::move(rs));
  }

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

  RemovePairsFor(remove_id, &filter_state_);
  RemovePairsFor(remove_id, &persistent_base_);

  /* Purge the removed id from all resolved statement sets in transient history
   so that future replays do not reference a geometry that no longer exists. */
  for (StateDelta& delta : transient_history_) {
    for (ResolvedStatement& statement : delta.statements) {
      statement.set_A.erase(std::remove(statement.set_A.begin(),
                                        statement.set_A.end(), remove_id),
                            statement.set_A.end());
      statement.set_B.erase(std::remove(statement.set_B.begin(),
                                        statement.set_B.end(), remove_id),
                            statement.set_B.end());
    }
  }

  /* Rebuild composite from the updated persistent base + purged transients. */
  if (has_transient_history()) {
    RebuildComposite();
  }
}

bool CollisionFilter::CanCollideWith(GeometryId id_A, GeometryId id_B) const {
  if (id_A == id_B) return false;
  const PairKey key(id_A, id_B);
  return !filter_state_.filtered.contains(key) &&
         !filter_state_.invariant.contains(key);
}

bool CollisionFilter::operator==(const CollisionFilter& other) const {
  if (this == &other) return true;
  if (geometries_ != other.geometries_) return false;
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
  /* filtered and invariant sets are intentionally left empty. */
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
      /* Invariant pairs cannot be removed by Allow* declarations. */
      if (state->invariant.contains(key)) continue;
      state->filtered.erase(key);
    }
  }
}

void CollisionFilter::RemovePairsFor(GeometryId id, FilterState* state) {
  auto erase_matching = [&id](CollisionFilter::FilteredPairs& pair_set) {
    for (auto it = pair_set.begin(); it != pair_set.end();) {
      if (it->first() == id || it->second() == id) {
        it = pair_set.erase(it);
      } else {
        ++it;
      }
    }
  };
  erase_matching(state->filtered);
  erase_matching(state->invariant);
}

void CollisionFilter::ApplyStatement(const ResolvedStatement& statement,
                                     FilterState* state) {
  using Op = CollisionFilterDeclaration::StatementOp;
  switch (statement.operation) {
    case Op::kExcludeBetween:
      AddPairsBetween(statement.set_A, statement.set_B, /*is_invariant=*/false,
                      state);
      break;
    case Op::kExcludeWithin:
      AddPairsBetween(statement.set_A, statement.set_A, /*is_invariant=*/false,
                      state);
      break;
    case Op::kAllowBetween:
      RemovePairsBetween(statement.set_A, statement.set_B, state);
      break;
    case Op::kAllowWithin:
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

/* static */
void CollisionFilter::ApplyDeclarationToState(
    const CollisionFilterDeclaration& declaration,
    const CollisionFilter::ExtractIds& extract_ids, bool is_invariant,
    FilterState* state) {
  using Statement = CollisionFilterDeclaration::Statement;
  using Op = CollisionFilterDeclaration::StatementOp;
  const CollisionFilterScope scope = declaration.scope();
  for (const Statement& statement : declaration.statements()) {
    const std::unordered_set<GeometryId> ids_A_set =
        extract_ids(statement.set_A, scope);
    const std::vector<GeometryId> ids_A(ids_A_set.begin(), ids_A_set.end());

    switch (statement.operation) {
      case Op::kExcludeBetween: {
        const std::unordered_set<GeometryId> ids_B_set =
            extract_ids(statement.set_B, scope);
        const std::vector<GeometryId> ids_B(ids_B_set.begin(), ids_B_set.end());
        AddPairsBetween(ids_A, ids_B, is_invariant, state);
        break;
      }
      case Op::kExcludeWithin:
        AddPairsBetween(ids_A, ids_A, is_invariant, state);
        break;
      case Op::kAllowBetween: {
        DRAKE_DEMAND(!is_invariant);
        const std::unordered_set<GeometryId> ids_B_set =
            extract_ids(statement.set_B, scope);
        const std::vector<GeometryId> ids_B(ids_B_set.begin(), ids_B_set.end());
        RemovePairsBetween(ids_A, ids_B, state);
        break;
      }
      case Op::kAllowWithin:
        DRAKE_DEMAND(!is_invariant);
        RemovePairsBetween(ids_A, ids_A, state);
        break;
    }
  }
}

void CollisionFilter::RebuildComposite() {
  /* Start from the persistent base and replay all transient deltas in order. */
  filter_state_.filtered = persistent_base_.filtered;
  filter_state_.invariant = persistent_base_.invariant;
  for (const StateDelta& delta : transient_history_) {
    ApplyStatements(delta, &filter_state_);
  }
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
