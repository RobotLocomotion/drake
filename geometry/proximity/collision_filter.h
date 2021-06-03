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
 added to this filter system (AddGeometry()). Every geometry with a proximity
 role must be added to the filter. */
class CollisionFilter {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CollisionFilter)

  CollisionFilter() = default;

  /* The callback function type for resolving a GeometrySet in a declaration
   into an explicitly enumerated set of GeometryIds. The geometries identified
   must all have the proximity role. Having the proximity role should be
   correlated with having been added to this filter system.

   This callback may throw if the GeometrySet contains FrameId values that
   cannot be mapped to GeometryIds. */
  using ExtractIds =
      std::function<std::unordered_set<GeometryId>(const GeometrySet&)>;

  /* Applies the collision filter declaration. The callback `extract_ids`
   provides a means to convert the GeometrySet into an explicit, unique set of
   GeometryIds for each Geometry set defined in the given `declaration`.

   @param declaration       The declaration to apply.
   @param extract_ids       A callback to convert a GeometrySet into the
                            explicit set of geometry ids.
   @throws std::exception if any GeometryId referenced by the declaration has
                          not previously been added to `this` system. */
  void Apply(const CollisionFilterDeclaration& declaration,
             const ExtractIds& extract_ids);

  /* Adds a geometry to the filter system. When added, it will not be part of
   any filtered pairs.
   @pre `new_id` has not been previously added to the system.  */
  void AddGeometry(GeometryId new_id);

  /* Removes a geometry from the filter system. All filtered collision pairs
   including `remove_id` will be removed.
   @pre `remove_id` is part of the system.  */
  void RemoveGeometry(GeometryId remove_id);

  /* Reports true if the geometry pair (`id_A`, `id_B`) is considered to be
   unfiltered.
   @pre `id_A` and `id_B` are both part of the system.  */
  bool CanCollideWith(GeometryId id_A, GeometryId id_B) const;

  /* Reports if two collision filters are configured the same. They are
   considered the same if the two filter systems report the same results for
   all calls to `CanCollideWith()`. This implies they have the same geometries
   registered, and equivalent filter state for all pairs. */
  bool operator==(const CollisionFilter& other) const;

  /* Reports if two collision filters are configured differently.  See
   operator==() for what this means. */
  bool operator!=(const CollisionFilter& other) const {
    return !(*this == other);
  }

 private:
  friend class CollisionFilterTest;

  /* The collision filter state between a pair of geometries. */
  enum PairFilterState {
    kUnfiltered,      // No filter has been declared.
    kFiltered         // A filter has been declared.
  };

  /* The filter state between an unknown geometry id and all ids in the map.
   For unknown id X, each key K in this map contributes to the geometry pair
   (X, K) having the associated pair filter state. */
  using GeometryMap = std::unordered_map<GeometryId, PairFilterState>;

  /* The table of all pair-wise filter configurations. Every added geometry is
   a key in this map. It maps to its pairwise relationship with every other
   geometry whose id is ordered *after* the key id. */
  using FilterState = std::unordered_map<GeometryId, GeometryMap>;

  /* Reports if the given `id` has been added to this filter system. */
  bool HasGeometry(GeometryId id) const { return filter_state_.count(id) > 0; }

  /* Declares pairs (`id_A`, `id_B`) `∀ id_A ∈ set_A, id_B ∈ set_B` to be
   filtered. For each pair, if they are already filtered, no discernible change
   is made.

   @pre All ids in `id_A` and `id_B` are part of the system.  */
  void AddFiltersBetween(const GeometrySet& set_A, const GeometrySet& set_B,
                         const ExtractIds& extract_ids);

  /* Atomic operation in support of AddFiltersBetween().  */
  void AddFilteredPair(GeometryId id_A, GeometryId id_B);

  /* A map from each geometry to all of the geometries with ids that follow it
   (and explicit information about whether the two geometry ids form a filtered
   pair). So, if we've registered GeometryId values 10, 20, 30, 40, we would
   have the map entries:
     10 -> {(20, PairFilterState), (30, PairFilterState), (40, PairFilterState)}
     20 -> {(30, PairFilterState), (40, PairFilterState)}
     30 -> {(40, PairFilterState)}
     40 -> {}  */
  FilterState filter_state_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
