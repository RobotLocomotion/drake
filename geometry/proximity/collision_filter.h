#pragma once

#include <functional>
#include <unordered_map>
#include <unordered_set>
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

  CollisionFilter() = default;

  /* The callback function type for resolving a GeometrySet in a declaration
   into an explicitly enumerated set of GeometryIds. All ids in the resultant
   set must have previously been added to this filter system.

   This callback should throw if the GeometrySet contains FrameId values that
   cannot be mapped to GeometryIds. */
  using ExtractIds =
      std::function<std::unordered_set<GeometryId>(const GeometrySet&)>;

  /* Applies the collision filter declaration. The callback `extract_ids`
   provides a means to convert the GeometrySet into an explicit, unique set of
   GeometryIds for each GeometrySet defined in the given `declaration`.

   @param declaration       The declaration to apply.
   @param extract_ids       A callback to convert a GeometrySet into the
                            explicit set of geometry ids.
   @param is_permanent      If `true` a filter added (via an Exclude* API) will
                            be made permanent.
   @throws std::exception if any GeometryId referenced by the declaration has
                          not previously been added to `this` filter system. */
  void Apply(const CollisionFilterDeclaration& declaration,
             const ExtractIds& extract_ids, bool is_permanent = false);

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
  /* The collision filter state between a pair of geometries. */
  enum PairFilterState {
    kUnfiltered,      // No filter has been declared.
    kFiltered,        // A user-declared filter exists, the user can remove it.
    kLockedFiltered,  // A system filter has been created and can't be removed
                      // by the user.
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
     10 -> {(20, PairFilterState), (30, PairFilterState), (40, PairFilterState)}
     20 -> {(30, PairFilterState), (40, PairFilterState)}
     30 -> {(40, PairFilterState)}
     40 -> {} */
  using GeometryMap = std::unordered_map<GeometryId, PairFilterState>;

  using FilterState = std::unordered_map<GeometryId, GeometryMap>;

  /* Declares pairs (`id_A`, `id_B`) `∀ id_A ∈ set_A, id_B ∈ set_B` to be
   filtered. For each pair, if they are already filtered, no discernible change
   is made.

   The filtered pair can be made "permanent" such that subsequent calls to
   RemoveFiltersBetween will not remove the filter. This is intended to support
   SceneGraph invariants that geometries affixed to the same frame are filtered
   or pairs of anchored geometries are likewise filtered. GeometryState is
   responsible for determining permanence when adding filters.

   @pre All ids in `id_A` and `id_B` are part of this filter system.  */
  void AddFiltersBetween(const GeometrySet& set_A, const GeometrySet& set_B,
                         const ExtractIds& extract_ids, bool is_permanent);

  /* Declares pairs (`id_A`, `id_B`) `∀ id_A ∈ set_A, id_B ∈ set_B` to be
   unfiltered (if the filter isn't permanent). For each pair, if they are
   already unfiltered, no discernible change is made.

   @pre All ids in `id_A` and `id_B` are part of this filter system.  */
  void RemoveFiltersBetween(const GeometrySet& set_A, const GeometrySet& set_B,
                            const CollisionFilter::ExtractIds& extract_ids);

  /* Atomic operation in support of AddFiltersBetween(). */
  void AddFilteredPair(GeometryId id_A, GeometryId id_B, bool is_permanent);

  /* Atomic operation in support of RemoveFiltersBetween(). */
  void RemoveFilteredPair(GeometryId id_A, GeometryId id_B);

  /* The filter state of all pairs of geometry. */
  FilterState filter_state_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
