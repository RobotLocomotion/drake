#pragma once

#include "drake/common/default_scalars.h"
#include "drake/geometry/geometry_set.h"

namespace drake {
namespace geometry {

// Forward declarations.
template <typename T>
class SceneGraph;
template <typename T>
class GeometryState;

/** Class for configuring "collision filters"; collision filters limit the scope
 of various proximity queries.

 The sole source of %CollisionFilterManager instances is SceneGraph. See
 @ref scene_graph_collision_filter_manager "SceneGraph's documentation" for
 details on acquiring an instance. It is important that the lifespan of a
 %CollisionFilterManager should *not* exceed the lifespan of its source -- a
 SceneGraph for the model manager or the systems::Context for the context
 manager.

 The scene graph consists of the set of geometry
 `G = D ⋃ A = {g₀, g₁, ..., gₙ}`, where D is the set of dynamic geometry and
 A is the set of anchored geometry (by definition `D ⋂ A = ∅`). Many proximity
 queries operate on pairs of geometries (e.g., (gᵢ, gⱼ)). The set of proximity
 candidate pairs is initially defined as `C = (G × G) - (A × A) - F - I`,
 where:
     - `G × G = {(gᵢ, gⱼ)}, ∀ gᵢ, gⱼ ∈ G` is the cartesian product of the set of
     %SceneGraph geometries.
     - `A × A` represents all pairs consisting only of anchored geometry;
     anchored geometry is never tested against other anchored geometry.
     - `F = (gᵢ, gⱼ)`, such that `frame(gᵢ) == frame(gⱼ)`; the pair where both
     geometries are rigidly affixed to the same frame. By implication,
     `gᵢ, gⱼ ∈ D` as only dynamic geometries are affixed to frames.
     - `I = {(g, g)}, ∀ g ∈ G` is the set of all pairs consisting of a geometry
     with itself; there is no collision between a geometry and itself.

 Only pairs contained in C will be included in pairwise proximity operations.
 These methods allow users to modify the set C. Additional pairs can be removed,
 but some pairs can never be re-introduced: e.g., the sets `A × A`, `F`, and `I`
 will *never* be part of C; that is a SceneGraph invariant.

 These manipulations take the form of declarations: e.g., exclude proximity
 operations between the geometry pair (g, h). There is no penalty for
 redundantly declaring that exclusion. After each declaration, the state of set
 C will be modified (to the extent possible) to reflect the declaration (see
 the earlier warning about pairs that are precluded from being a part of C).
 Any declaration can be undone by subsequent declarations. The final
 population of C is the aggregate result of a *sequence* of declarations. The
 order of declarations matters. The declaration is applied immediately to the
 current configuration of C.

 @warning All collision filtering is done based on geometry state at the time
 of the collision filtering calls. More concretely:
 - For a particular FrameId in a GeometrySet instance, only those geometries
    attached to the identified frame with the proximity role assigned at the
    time of the call will be included in the filter. If geometries are
    subsequently added or assigned the proximity role, they will not be
    retroactively added to the user-declared filter.
 - If the set includes geometries which have _not_ been assigned a proximity
    role, those geometries will be ignored. If a proximity role is
    subsequently assigned, those geometries will _still_ not be part of any
    user-declared collision filters.
 - In general, adding collisions and assigning proximity roles should
    happen prior to collision filter configuration.

 <h3>Collision filtering and geometry versions</h3>

 Declaring a change to the set C will increment the proximity version (see
 @ref scene_graph_versioning), even if applying the declaration doesn't actually
 change the set C.

 <!-- NOTE TO DEVELOPERS
   There is no collision_filter_manager_test.cc. As this can only be constructed
   by SceneGraph, the tests for this simple wrapper API are part of those tests
   (exploiting the infrastructure for populating SceneGraph).
 -->
  */
template <typename T>
class CollisionFilterManager {
 public:
  /** @name Copy and move semantics

   This class only implements MoveConstructible; copy construct and all
   assignment are not allowed. */
  //@{
  CollisionFilterManager(const CollisionFilterManager&) = delete;
  CollisionFilterManager& operator=(const CollisionFilterManager&) = delete;
  /* Note: move construction is added to support python bindings. */
  CollisionFilterManager(CollisionFilterManager&&) = default;
  CollisionFilterManager& operator=(CollisionFilterManager&&) = delete;
  //@}

  /** Excludes geometry pairs from proximity operations by updating the
   candidate pair set `C = C - P`, where `P = {(gᵢ, gⱼ)}, ∀ gᵢ, gⱼ ∈ G` and
   `G = {g₀, g₁, ..., gₘ}` is the input `set` of geometries.

   @throws std::logic_error if the set includes ids that don't exist in the
                            scene graph.  */
  void ExcludeCollisionsWithin(const GeometrySet& set);

  /** Excludes geometry pairs from collision evaluation by updating the
   candidate pair set `C = C - P`, where `P = {(a, b)}, ∀ a ∈ A, b ∈ B` and
   `A = {a₀, a₁, ..., aₘ}` and `B = {b₀, b₁, ..., bₙ}` are the input sets of
   geometries `setA` and `setB`, respectively. This does _not_ preclude
   collisions between members of the _same_ set.

   @throws std::logic_error if the groups include ids that don't exist in the
                            scene graph.  */
  void ExcludeCollisionsBetween(const GeometrySet& setA,
                                const GeometrySet& setB);

 private:
  // Only SceneGraph can construct a collision filter manager.
  template <typename>
  friend class SceneGraph;

  /* Constructs the manager with the underlying state instance that provides
   support for the operations. */
  explicit CollisionFilterManager(GeometryState<T>* state);

  GeometryState<T>* state_{};
};

}  // namespace geometry
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::CollisionFilterManager)
