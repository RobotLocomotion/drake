#pragma once

#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/geometry/collision_filter_declaration.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {
namespace internal {

/* Implementation of a collision filter policy. Collision filters operate on
 pairs of geometries. A pair is either filtered or not. Most proximity queries
 rely on the collision filter policy to determine if they should compute results
 for a given geometry pair. Filtered pairs do not contribute to the results.

 Geometries are identified by their geometry ids. The filter status of the
 geometry pair (g, h) can only be modified if both g and h have already been
 added to this filter system (via AddGeometry()). */
class CollisionFilter {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CollisionFilter)

  CollisionFilter();

  /* The callback function type for resolving a GeometrySet in a declaration
   into an explicitly enumerated set of GeometryIds. All ids in the resultant
   set must have previously been added to this filter system.

   This callback should throw if the GeometrySet contains FrameId values that
   cannot be mapped to GeometryIds. */
  using ExtractIds =
      std::function<std::unordered_set<GeometryId>(const GeometrySet&)>;

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
   @throws std::exception if any GeometryId referenced by the declaration has
                          not previously been added to `this` filter system.
   @throws std::exception if there are any transient declarations active.
   @pre If `is_invariant` is `true`, declaration contains no `Allow*()`
        statements. */
  void Apply(const CollisionFilterDeclaration& declaration,
             const ExtractIds& extract_ids, bool is_invariant = false);

  /* Applies the collision filter declaration as a transient declaration. The
   callback `extract_ids` provides a means to convert the GeometrySet into an
   explicit, unique set of GeometryIds for each Geometry set defined in the
   given `declaration`. The callback is *not* stored. It is used to process
   the given `declaration` and then forgotten.

   @param declaration       The declaration to apply.
   @param extract_ids       A callback to convert a GeometrySet into the
                            explicit set of geometry ids.
   @returns A unique FilterId for this transient declaration.
   @throws std::exception if any GeometryId referenced by the declaration has
                          not previously been added to the system. */
  FilterId ApplyTransient(const CollisionFilterDeclaration& declaration,
                          const ExtractIds& extract_ids);

  /* Reports true if there are any active transient declarations.  */
  bool has_transient_history() const {
    /* Remember: filter_history_[0] is the persistent state. */
    return filter_history_.size() > 1;
  }

  /* Reports if the given filter id is part of the active, transient history. */
  bool IsActive(FilterId id) const;

  /* Attempts to remove the transient declaration associated with the given id.
   If the id isn't valid, no action is taken.

   @returns true if a declaration is actually removed. */
  bool RemoveDeclaration(FilterId id);

  /* Flattens the history. The current configuration becomes the persistent
   base configuration and all transient history is removed. */
  void Flatten();

  /* Adds a geometry to the filter system. When added, it will not be part of
   any filtered pairs.
   @pre `new_id` has not been previously added to this filter system. */
  void AddGeometry(GeometryId new_id);

  /* Removes a geometry from the filter system. All filtered collision pairs
   including `remove_id` will be removed.
   @pre `remove_id` is part of this filter system. */
  void RemoveGeometry(GeometryId remove_id);

  /* Reports true if the geometry pair (`id_A`, `id_B`) is considered to be
   unfiltered.
   @pre `id_A` and `id_B` are both part of this filter system. */
  bool CanCollideWith(GeometryId id_A, GeometryId id_B) const;

  /* Reports if two collision filters are configured the same. They are
   considered the same if the two filter systems report the same results for
   all calls to `CanCollideWith()`. This implies they have the same geometries
   registered, and equivalent filter state for each pair. */
  bool operator==(const CollisionFilter& other) const;

  /* Reports if two collision filters are configured differently.  See
   operator==() for what this means. */
  bool operator!=(const CollisionFilter& other) const {
    return !(*this == other);
  }

  /* Reports if the given `id` has been added to this filter system. */
  bool HasGeometry(GeometryId id) const { return filter_state_.count(id) > 0; }

 private:
  friend class CollisionFilterTest;

  /* Test utility so that the unit tests can create a CollisionFilter with all
   of the registered geometry but no collision filters. */
  CollisionFilter MakeClearCopy() const;

  /* The collision filter state between a pair of geometries. */
  enum PairRelationship {
    kUndefined,        // No relationship has been defined; used for transient
                       // declarations.
    kUnfiltered,       // No filter has been declared.
    kFiltered,         // A user-declared filter exists, the user can remove it.
    kInvariantFilter,  // The filter supports a SceneGraph filter invariant and
                       // cannot be removed by the user.
  };

  /* The "filter state" is a 2d table. For N registered geometries, it is
   an NxN table where cell (i, j) reports the filter status of the ith and jth
   geometries (although i and j are actually GeometryIds).

   Because a pair's filter status is symmetric (e.g., (a, b) and (b, a) are
   equivalent), we only encode a triangular portion of the table. We have two
   types that work together to define the table: GeometryMap and FilterState.

   FilterState maps a GeometryId to the filter relationship with all other
   registered geometries with *greater* GeometryId values. That set of
   geometries is contained in a GeometryMap, where each entry on that "row"
   is the key-value pair (other_id, filter_pair_state).

   So, given a FilterState instance (filter_state) and two GeometryIds, a and b,
   we could set the filtered state as (assuming a < b):

      filter_state[a][b] = kFiltered;

   As a concrete example, if we've registered GeometryId values 10, 20, 30, 40,
   the filter state would have keys {10, 20, 30, 40} which each maps to a
   corresponding GeometryMap as illustrated below.
     10 -> {(20, PairRelationship), (30, PairRelationship),
            (40, PairRelationship)}
     20 -> {(30, PairRelationship), (40, PairRelationship)}
     30 -> {(40, PairRelationship)}
     40 -> {} */
  using GeometryMap = std::unordered_map<GeometryId, PairRelationship>;

  using FilterState = std::unordered_map<GeometryId, GeometryMap>;

  /* Applies the given declaration to an arbitrary `filter_state`. */
  static void Apply(const CollisionFilterDeclaration& declaration,
                    const ExtractIds& extract_ids, bool is_invariant,
                    FilterState* filter_state);

  /* Adds the geometry with the given `id` to the given filter state with the
   given initial_state for all pairs including the new id. */
  static void AddGeometry(GeometryId id, FilterState* filter_state_out,
                          PairRelationship relationship);

  /* Removes the geometry with the given `id` from the given filter state. */
  static void RemoveGeometry(GeometryId id, FilterState* filter_state_out);

  /* Declares pairs (`id_A`, `id_B`) `∀ id_A ∈ set_A, id_B ∈ set_B` to be
   filtered. For each pair, if they are already filtered, no discernible change
   is made.

   The filtered pair can be made "invariant" such that subsequent calls to
   RemoveFiltersBetween will not remove the filter. This is intended to support
   SceneGraph invariants that geometries affixed to the same frame are filtered
   or pairs of anchored geometries are likewise filtered. GeometryState is
   responsible for determining invariance when adding filters.

   @pre All ids in `id_A` and `id_B` are part of this filter system.  */
  static void AddFiltersBetween(const GeometrySet& set_A,
                                const GeometrySet& set_B,
                                const ExtractIds& extract_ids,
                                bool is_invariant, FilterState* state_out);

  /* Declares pairs (`id_A`, `id_B`) `∀ id_A ∈ set_A, id_B ∈ set_B` to be
   unfiltered (if the filter isn't invariant). For each pair, if they are
   already unfiltered, no discernible change is made.

   @pre All ids `id_A` and `id_B` are part of the system.  */
  static void RemoveFiltersBetween(const GeometrySet& set_A,
                                   const GeometrySet& set_B,
                                   const ExtractIds& extract_ids,
                                   FilterState* state_out);

  /* Atomic operation in support of AddFiltersBetween().  */
  static void AddFilteredPair(GeometryId id_A, GeometryId id_B,
                              bool is_invariant, FilterState* state_out);

  /* Atomic operation in support of RemoveFilterBetween().  */
  static void RemoveFilteredPair(GeometryId id_A, GeometryId id_B,
                                 FilterState* state_out);

  /* Instantiates a FilterState such that all known pairs have given
   default relationship. */
  static FilterState InitializeTransientState(
      const FilterState& reference, PairRelationship default_relationship);

  /* The filter state of all pairs of geometry.

   This is neither the persistent base configuration nor the history. It is
   the cached result of the base with all transient declarations applied. We
   store this composite result in order to make collision filter lookups as
   fast as possible. The implication is that when we update the history, we
   must *also* update this so that it always reflects the final configuration.
   */
  FilterState filter_state_;

  /* The underlying data for transient history: the assigned filter id and
   the filter state instance which represents the applied transient declaration.
   The filter state has been initialized with all registered geometry and
   *no* collision filters and has the transient declaration applied directly
   to it. In other words, any geometry pair that isn't explicitly accounted
   for in the declaration is marked with PairRelationship::kUndefined. */
  struct StateDelta {
    /* Default constructor to support resize of vector<StateDelta>. */
    StateDelta() = default;

    StateDelta(FilterState state, FilterId id_in)
        : filter_state(std::move(state)), id(id_in) {}

    FilterState filter_state;
    FilterId id{};
  };
  std::vector<StateDelta> filter_history_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
