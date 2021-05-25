#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/geometry/collision_filter_declaration.h"
#include "drake/geometry/geometry_ids.h"
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
 details on acquiring an instance.

 A SceneGraph instance contains the set of geometry
 `G = D ⋃ A = {g₀, g₁, ..., gₙ}`, where D is the set of dynamic geometries and
 A is the set of anchored geometries (by definition `D ⋂ A = ∅`). `Gₚ ⊂ G` is
 the subset of geometries that have a proximity role (with an analogous
 interpretation of `Dₚ` and `Aₚ`). Many proximity queries operate on pairs of
 geometries (e.g., (gᵢ, gⱼ)). The set of proximity candidate pairs is initially
 defined as `C = (Gₚ × Gₚ) - (Aₚ × Aₚ) - Fₚ - Iₚ`, where:

  - `Gₚ × Gₚ = {(gᵢ, gⱼ)}, ∀ gᵢ, gⱼ ∈ Gₚ` is the cartesian product of the set
    of SceneGraph geometries.
  - `Aₚ × Aₚ` represents all pairs consisting only of anchored geometries;
    anchored geometry is never tested against other anchored geometry.
  - `Fₚ = {(gᵢ, gⱼ)} ∀ i, j`, such that `gᵢ, gⱼ ∈ Dₚ` and
    `frame(gᵢ) == frame(gⱼ)`; the pairs where both geometries are rigidly
    affixed to the same frame.
  - `Iₚ = {(g, g)}, ∀ g ∈ Gₚ` is the set of all pairs consisting of a
    geometry with itself; there is no meaningful proximity query on a
    geometry with itself.

 Only pairs contained in C will be included in pairwise proximity operations.

 The manager provides an interface to modify the set C. Changes to C are
 articulated with CollisionFilterDeclaration. Once a change has been *declared*
 it is applied via the manager's API to change the configuration of C.

 There are limits to how C can be modified.

   - `∀ (gᵢ, gⱼ) ∈ C`, both gᵢ and gⱼ must be registered with SceneGraph; you
     can't inject arbitrary ids. Attempting to do so will result in an error.
   - No pairs in `Aₚ × Aₚ`, `Fₚ`, or `Iₚ` can ever be added to C. Excluding
     those pairs is a SceneGraph invariant. Attempts to do so will be ignored.

 The current configuration of C depends on the sequence of filter declarations
 that have been applied in the manager. Changing the order can change the end
 result.

 @warning The effect of applying a declaration is based on the state of
 SceneGraph's geometry data at the time of application. More concretely:
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
 change the set C.  */
template <typename T>
class CollisionFilterManager {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CollisionFilterManager)

  /** Applies the given `declaration` to the geometry state managed by `this`
   instance.

   The process of *applying* the collision filter data also validates it. The
   following polices are implemented during validation:

     - Referencing an invalid id (FrameId or GeometryId): throws.
     - Declaring a filtered pair that is already filtered: no discernible
       change.

   @throws std::exception if the declaration references invalid ids.  */
  void Apply(const CollisionFilterDeclaration& declaration);

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
