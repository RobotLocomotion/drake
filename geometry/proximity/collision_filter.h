#pragma once

#include <functional>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/common/sorted_pair.h"
#include "drake/geometry/collision_filter_declaration.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {
namespace internal {

/* Implementation of a collision filter policy. Collision filters operate on
 pairs of geometries. A pair is either filtered or not. Most proximity queries
 rely on the collision filter policy to determine if they should compute results
 for a given geometry pair. Filtered pairs do not contribute to the results.

 In addition to pairwise filters, a geometry can be marked *inactive* (see
 CollisionFilterDeclaration::Deactivate()). An inactive geometry cannot collide
 with *any* other geometry, including geometries added to this filter system
 after it was deactivated. Inactivity is stored as a set of geometry ids (the
 "inactive set") and evaluated at query time in CanCollideWith() rather than
 being resolved to explicit pairs; that is what makes it robust to later
 geometry registration (and cheap: one set entry instead of O(N) pairs). The
 inactive set is independent of the pairwise sets: the pairwise Allow*()
 statements never reactivate a geometry, and reactivating one leaves all
 pairwise filters in force.

 Geometries are identified by their geometry ids. The filter status of the
 geometry pair (g, h) can only be modified if both g and h have already been
 added to this filter system (via AddGeometry()). */
class CollisionFilter {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CollisionFilter);

  CollisionFilter();

  /* The net change to the inactive set (see the class documentation) produced
   by applying or removing a declaration. Clients that derive data structures
   from the inactive set (e.g., ProximityEngine culls inactive geometries from
   its broadphase trees) use this to update incrementally, in O(delta), without
   inspecting the full filter state. The two vectors are disjoint; ids appear
   at most once. A declaration whose statements have no net effect on the
   inactive set (including all purely pairwise declarations) produces an empty
   change. */
  struct ActiveStatusChange {
    /* Geometries that became inactive (were active before the operation). */
    std::vector<GeometryId> deactivated;
    /* Geometries that became active. */
    std::vector<GeometryId> activated;
    bool empty() const { return deactivated.empty() && activated.empty(); }
  };

  /* The callback function type for resolving a GeometrySet in a declaration
   into an explicitly enumerated set of GeometryIds. All ids in the resultant
   set must have previously been added to this filter system.

   This callback should throw if the GeometrySet contains FrameId values that
   cannot be mapped to GeometryIds. */
  using ExtractIds = std::function<std::unordered_set<GeometryId>(
      const GeometrySet&, CollisionFilterScope)>;

  /* Applies the collision filter declaration to the persistent state. The
   callback `extract_ids` provides a means to convert the GeometrySet into an
   explicit, unique set of GeometryIds for each GeometrySet defined in the
   given `declaration`. The callback is *not* stored. It is used to process
   the given `declaration` and then forgotten. The persistent configuration can
   only be modified if there are no transient declarations active.

   Note that adding invariant filters is a very explicit act. It is the
   expectation that SceneGraph (and related classes) will never attempt to add
   invariant filters at the same time as removing filters -- the only time a
   statement adds and removes filters should be from a user. As such,
   `is_invariant` should be `true` iff the declaration consists solely of
   `Exclude*()` statements.

   @param declaration       The declaration to apply.
   @param extract_ids       A callback to convert a GeometrySet into the
                            explicit set of geometry ids.
   @param is_invariant      If `true` a filter added (via an Exclude* API) will
                            be treated as a system invariant -- a filter that
                            cannot be removed.
   @param active_status_change  If non-null, receives the net change to the
                            inactive set (replacing any previous value).
   @throws std::exception if any GeometryId referenced by the declaration has
                          not previously been added to `this` filter system.
   @throws std::exception if there are any transient declarations active.
   @pre If `is_invariant` is `true`, declaration contains no `Allow*()`,
        `Activate()`, or `Deactivate()` statements. */
  void Apply(const CollisionFilterDeclaration& declaration,
             const ExtractIds& extract_ids, bool is_invariant,
             ActiveStatusChange* active_status_change);

  /* Applies the collision filter declaration as a transient declaration. The
   callback `extract_ids` provides a means to convert the GeometrySet into an
   explicit, unique set of GeometryIds for each Geometry set defined in the
   given `declaration`. The callback is *not* stored. It is used to process
   the given `declaration` and then forgotten.

   @param declaration       The declaration to apply.
   @param extract_ids       A callback to convert a GeometrySet into the
                            explicit set of geometry ids.
   @param active_status_change  If non-null, receives the net change to the
                            inactive set (replacing any previous value).
   @returns A unique FilterId for this transient declaration.
   @throws std::exception if any GeometryId referenced by the declaration has
                          not previously been added to the system. */
  FilterId ApplyTransient(const CollisionFilterDeclaration& declaration,
                          const ExtractIds& extract_ids,
                          ActiveStatusChange* active_status_change);

  /* Reports true if there are any active transient declarations.  */
  bool has_transient_history() const { return !transient_history_.empty(); }

  /* Reports if the given filter id is part of the active, transient history. */
  bool IsActive(FilterId id) const;

  /* Attempts to remove the transient declaration associated with the given id.
   If the id isn't valid, no action is taken.

   @param active_status_change  If non-null, receives the net change to the
                      inactive set (replacing any previous value).
   @returns true if a declaration is actually removed. */
  bool RemoveDeclaration(FilterId id, ActiveStatusChange* active_status_change);

  /* Flattens the history. The current configuration becomes the persistent
   base configuration and all transient history is removed. */
  void Flatten();

  /* Adds a geometry to the filter system. When added, it will not be part of
   any filtered pairs.
   @pre `new_id` has not been previously added to this filter system. */
  void AddGeometry(GeometryId new_id);

  /* Removes a geometry from the filter system. All filtered collision pairs
   including `remove_id` will be removed, and the geometry is dropped from the
   inactive set (if present). Removing a geometry never changes any *other*
   geometry's active status.
   @pre `remove_id` is part of this filter system. */
  void RemoveGeometry(GeometryId remove_id);

  /* Reports true if the geometry pair (`id_A`, `id_B`) is considered to be
   unfiltered. A pair is filtered if either geometry is inactive (see
   CollisionFilterDeclaration::Deactivate()) or if the pair itself is filtered.
   @pre `id_A` and `id_B` are both part of this filter system. */
  bool CanCollideWith(GeometryId id_A, GeometryId id_B) const;

  /* Reports if two collision filters are "equivalent" -- in that they are
   defined over the same set of geometry ids, have the same inactive set, and
   report the same pairs as being filtered (or not). (The inactive sets are
   compared directly rather than via their current pair closure because an
   inactive geometry also affects geometries added in the future.) This says
   nothing about how the two filters are articulated (e.g., different
   declarations of invariance, transient versus permanent declarations,
   etc.). */
  bool IsEquivalent(const CollisionFilter& other) const;

  /* Reports if the given `id` has been added to this filter system. */
  bool HasGeometry(GeometryId id) const { return geometries_.contains(id); }

 private:
  friend class CollisionFilterTest;

  /* Test utility so that the unit tests can create a CollisionFilter with all
   of the registered geometry but no collision filters. */
  CollisionFilter MakeClearCopy() const;

  /* The representation of a pair of geometry ids in a hash set. The presence of
   this key in a FilteredPairs set implies filtering of the pair. */
  using PairKey = SortedPair<GeometryId>;

  using FilteredPairs = std::unordered_set<PairKey>;

  /* Sparse representation of the collision filter state. Only filtered pairs
   are stored; an absent pair is unfiltered by default.

   The `filtered` and `invariant` sets are always disjoint: a pair is in at
   most one of them. Membership in either set means CanCollideWith returns
   false. The distinction is that `invariant` pairs cannot be removed by the
   user via Allow*() declarations. */
  struct FilterState {
    /* User-removable filtered pairs. */
    FilteredPairs filtered;
    /* SceneGraph-invariant filtered pairs (cannot be removed by Allow*). */
    FilteredPairs invariant;
    /* The inactive set `Nₚ` (see the class documentation): membership is
     edited by Deactivate() (insert) and Activate() (erase). It is deliberately
     a set of ids and never a tally, so its effect against the live geometry
     set lives entirely in CanCollideWith()'s membership test. */
    std::unordered_set<GeometryId> inactive;
  };

  /* A resolved statement: the GeometrySets of the original declaration have
   been expanded to explicit vectors of GeometryIds at the time the transient
   was applied. Storing resolved IDs means the extract_ids callback is not
   needed at replay time.

   For Within operations, set_B is empty and the cross product is set_A × set_A.
   For Between operations, the cross product is set_A × set_B.

   Note: Although the geometry ids are stored in vectors, they are _not_ ordered
   and not duplicated. No operation on these sets should depend on the ordering.
   */
  struct ResolvedStatement {
    using Operation = CollisionFilterDeclaration::StatementOp;
    Operation operation{};
    std::vector<GeometryId> set_A;
    std::vector<GeometryId> set_B;
  };

  /* A transient declaration stored as its resolved statements. */
  struct StateDelta {
    StateDelta() = default;
    StateDelta(std::vector<ResolvedStatement> statements_in, FilterId id_in)
        : statements(std::move(statements_in)), id(id_in) {}

    std::vector<ResolvedStatement> statements;
    FilterId id{};
  };

  /* Applies the resolved statements in `delta` to `state`. Used when first
   applying a transient and when replaying history in RebuildComposite(). */
  static void ApplyStatements(const StateDelta& delta, FilterState* state);

  /* Applies a single resolved statement to `state`. */
  static void ApplyStatement(const ResolvedStatement& statement,
                             FilterState* state);

  /* For each pair (a, b) with a ∈ set_A, b ∈ set_B (a ≠ b), adds the pair
   to `state->filtered` or `state->invariant` depending on `is_invariant`.
   Pairs already in `invariant` are never downgraded. */
  static void AddPairsBetween(const std::vector<GeometryId>& set_A,
                              const std::vector<GeometryId>& set_B,
                              bool is_invariant, FilterState* state);

  /* For each pair (a, b) with a ∈ set_A, b ∈ set_B (a ≠ b), removes the pair
   from `state->filtered`. Pairs in `state->invariant` are not affected. */
  static void RemovePairsBetween(const std::vector<GeometryId>& set_A,
                                 const std::vector<GeometryId>& set_B,
                                 FilterState* state);

  /* Removes all entries in `state->filtered` and `state->invariant` that
   involve `id`, and `id`'s entry in `state->inactive` (if any). Called
   when a geometry is unregistered. */
  static void RemovePairsFor(GeometryId id, FilterState* state);

  /* Computes the net active-status change from `before` to `state->inactive`
   and writes it to `active_status_change` (which must be non-null; any
   previous value is replaced).

   Diffing the before/after snapshots (rather than recording individual
   statement effects) is what makes a self-cancelling declaration like
   Deactivate({g}).Activate({g}) produce an empty change, and -- more
   importantly -- it is the only way to recover the net change after
   RemoveDeclaration(), which replays the surviving transient history from
   scratch (see RebuildComposite()) instead of inverting the removed
   declaration. Cost is O(#inactive geometries), not O(#filtered pairs). */
  static void ComputeActiveStatusChange(
      const std::unordered_set<GeometryId>& before, const FilterState& state,
      ActiveStatusChange* active_status_change);

  /* Applies the given declaration (using the extract_ids callback) to `state`.
   Used by the public Apply(). */
  static void ApplyDeclarationToState(
      const CollisionFilterDeclaration& declaration,
      const ExtractIds& extract_ids, bool is_invariant, FilterState* state);

  /* Rebuilds filter_state_ from persistent_base_ plus all entries in
   transient_history_ replayed in order. Called after RemoveDeclaration() or
   RemoveGeometry() when transient history is active. */
  void RebuildComposite();

  /* The cached composite filter state: persistent_base_ with all transient
   declarations applied in order. CanCollideWith() reads only from this, keeping
   query cost at O(1) regardless of history depth. */
  FilterState filter_state_;

  /* The persistent base configuration. Apply() modifies this (and keeps
   filter_state_ in sync). Transient declarations do not touch this directly. */
  FilterState persistent_base_;

  /* The set of geometry IDs registered with this filter system. Geometry
   add/remove is never part of a transient declaration, so this is owned
   directly by CollisionFilter. */
  std::unordered_set<GeometryId> geometries_;

  /* The ordered list of active transient declarations. Each entry stores
   resolved statements (not a full NxN copy), so memory per entry is O(|A|+|B|)
   rather than O(N²). */
  std::vector<StateDelta> transient_history_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
