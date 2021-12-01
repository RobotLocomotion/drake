#pragma once

#include <limits>
#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/sorted_pair.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/internal_geometry.h"
#include "drake/geometry/proximity/collision_filter.h"
#include "drake/geometry/proximity/hydroelastic_internal.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/query_results/signed_distance_pair.h"
#include "drake/geometry/query_results/signed_distance_to_point.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

template <typename T> class GeometryState;

namespace internal {

/* The underlying engine for performing geometric _proximity_ queries.
 It owns the geometry instances and, once it has been provided with the poses
 of the geometry, it provides geometric queries on that geometry.

 Proximity queries span a range of types, including:

   - penetration
   - distance
   - ray-intersection

 Not all shape queries are fully supported. To add support for a shape:
 1. for fcl versions of the specification, modify CopyShapeOrThrow().
 2. add an instance of the new shape to the CopySemantics test in
    proximity_engine_test.cc.
 3. for penetration, test the new shape in the class BoxPenetrationTest of
    proximity_engine_test.cc and document its configuration.

 <!-- TODO(SeanCurtis-TRI): Fully document the semantics of the proximity
 properties that will affect the proximity engine -- hydroelastic semantics,
 required properties, etc.

 <h3>Geometry proximity properties</h3>
 -->

 @tparam_nonsymbolic_scalar

 @internal Historically, this replaces the DrakeCollision::Model class.  */
template <typename T>
class ProximityEngine {
 public:
  ProximityEngine();
  ~ProximityEngine();

  /* Construct a deep copy of the provided `other` engine.  */
  ProximityEngine(const ProximityEngine& other);

  /* Set `this` engine to be a deep copy of the `other` engine.  */
  ProximityEngine& operator=(const ProximityEngine& other);

  /* Construct an engine by moving the data of a source engine. The source
   engine will be returned to its default-initialized state.  */
  ProximityEngine(ProximityEngine&& other) noexcept;

  /* Move assign a source engine to this engine. The source
   engine will be returned to its default-initialized state.  */
  ProximityEngine& operator=(ProximityEngine&& other) noexcept;

  /* Returns an independent copy of this engine templated on the AutoDiffXd
   scalar type. If the engine is already an AutoDiffXd engine, it is equivalent
   to using the copy constructor to create a duplicate on the heap.  */
  std::unique_ptr<ProximityEngine<AutoDiffXd>> ToAutoDiffXd() const;

  /* Provides access to the mutable collision filter this engine uses. */
  CollisionFilter& collision_filter();

  /* @name Topology management */
  //@{

  /* Adds the given `shape` to the engine's _dynamic_ geometry.
   @param shape   The shape to add.
   @param X_WG    The pose of the shape in the world frame.
   @param id      The id of the geometry in SceneGraph to which this shape
                  belongs.
   @param props   The proximity properties for the shape.  */
  void AddDynamicGeometry(const Shape& shape, const math::RigidTransformd& X_WG,
                          GeometryId id, const ProximityProperties& props = {});

  /* Adds the given `shape` to the engine's _anchored_ geometry.
   @param shape   The shape to add.
   @param X_WG    The pose of the shape in the world frame.
   @param id      The id of the geometry in SceneGraph to which this shape
                  belongs.
   @param props   The proximity properties for the shape.  */
  void AddAnchoredGeometry(const Shape& shape,
                           const math::RigidTransformd& X_WG, GeometryId id,
                           const ProximityProperties& props = {});

  /* Possibly updates the proximity representation of the given `geometry`
   based on the relationship between its _current_ proximity properties and the
   given _new_ proximity properties. The underlying representation may not
   change if the change in proximity properties isn't of significance to the
   %ProximityEngine.

   @param geometry          The geometry to update.
   @param new_properties    The properties to associate with the given geometry.
   @throws std::exception   if `geometry` doesn't map to a known geometry in
                            the engine or if the new properties trigger work
                            that can't meaningfully be completed because of
                            incomplete or inconsistent property definitions.
   @pre `geometry` still has a copy of the original proximity properties that
         are to be replaced.  */
  void UpdateRepresentationForNewProperties(
      const InternalGeometry& geometry,
      const ProximityProperties& new_properties);

  // TODO(SeanCurtis-TRI): Decide if knowing whether something is dynamic or not
  //  is *actually* sufficiently helpful to justify this act.
  /* Removes the given geometry indicated by `id` from the engine.
   @param id          The id of the geometry to be removed.
   @param is_dynamic  True if the geometry is dynamic, false if anchored.
   @throws std::exception if `id` does not refer to a geometry in this engine.
  */
  void RemoveGeometry(GeometryId id, bool is_dynamic);

  /* Reports the _total_ number of geometries in the engine -- dynamic and
   anchored (spanning all sources).  */
  int num_geometries() const;

  /* Reports the number of _dynamic_ geometries (spanning all sources).  */
  int num_dynamic() const;

  /* Reports the number of _anchored_ geometries (spanning all sources).  */
  int num_anchored() const;

  /* The distance (signed/unsigned/penetration distance) is generally computed
   from an iterative process. The distance_tolerance determines when the
   iterative process will terminate.
   As a rule of rule of thumb, one can generally assume that the answer will
   be within 10 * tol to the true answer.  */
  void set_distance_tolerance(double tol);

  double distance_tolerance() const;

  //@}

  /* Updates the poses for all of the _dynamic_ geometries in the engine.
   @param X_WGs     The poses of each geometry `G` measured and expressed in the
                    world frame `W` (including geometries which may *not* be
                    registered with the proximity engine or may not be
                    dynamic).
  */
  // TODO(SeanCurtis-TRI): I could do things here differently a number of ways:
  //  1. I could make this move semantics (or swap semantics).
  //  2. I could simply have a method that returns a mutable reference to such
  //    a vector and the caller sets values there directly.
  void UpdateWorldPoses(
      const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs);

  // ----------------------------------------------------------------------
  /* @name              Signed Distance Queries
  See @ref signed_distance_query "Signed Distance Query" for more details.  */

  //@{
  // NOTE: This maps to Model::ClosestPointsAllToAll().
  /* Implementation of
   GeometryState::ComputeSignedDistancePairwiseClosestPoints().
   This includes `X_WGs`, the current poses of all geometries in World in the
   current scalar type, keyed on each geometry's GeometryId.  */
  std::vector<SignedDistancePair<T>>
  ComputeSignedDistancePairwiseClosestPoints(
      const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs,
      const double max_distance) const;

  /* Implementation of
   GeometryState::ComputeSignedDistancePairClosestPoints().
   This includes `X_WGs`, the current poses of all geometries in World in the
   current scalar type, keyed on each geometry's GeometryId.  */
  SignedDistancePair<T> ComputeSignedDistancePairClosestPoints(
      GeometryId id_A, GeometryId id_B,
      const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs)
      const;

  /* Implementation of GeometryState::ComputeSignedDistanceToPoint().
   This includes `X_WGs`, the current poses of all geometries in World in the
   current scalar type, keyed on each geometry's GeometryId.  */
  std::vector<SignedDistanceToPoint<T>>
  ComputeSignedDistanceToPoint(
      const Vector3<T>& p_WQ,
      const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs,
      const double threshold = std::numeric_limits<double>::infinity()) const;
  //@}


  //----------------------------------------------------------------------------
  /* @name                Collision Queries
  See @ref collision_queries "Collision Queries" for more details.  */

  //@{

  // NOTE: This maps to Model::ComputeMaximumDepthCollisionPoints().
  // The definition that touching is not penetrating is due to an FCL issue
  // described in https://github.com/flexible-collision-library/fcl/issues/375
  // and drake issue #10577. Once that is resolved, this definition can be
  // revisited (and ProximityEngineTest::Issue10577Regression_Osculation can
  // be updated).
  /* Implementation of GeometryState::ComputePointPairPenetration().
   This includes `X_WGs`, the current poses of all geometries in World in the
   current scalar type, keyed on each geometry's GeometryId.  */
  std::vector<PenetrationAsPointPair<T>> ComputePointPairPenetration(
      const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs)
      const;

  /* Implementation of GeometryState::ComputeContactSurfaces().
   @param X_WGs the current poses of all geometries in World in the
                current scalar type, keyed on each geometry's GeometryId.  */
  std::vector<ContactSurface<T>> ComputeContactSurfaces(
      HydroelasticContactRepresentation representation,
      const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs)
      const;

  /* Implementation of GeometryState::ComputeContactSurfacesWithFallback().
   @param X_WGs the current poses of all geometries in World in the
                current scalar type, keyed on each geometry's GeometryId.  */
  void ComputeContactSurfacesWithFallback(
      HydroelasticContactRepresentation representation,
      const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs,
      std::vector<ContactSurface<T>>* surfaces,
      std::vector<PenetrationAsPointPair<T>>* point_pairs) const;

  /* Implementation of GeometryState::FindCollisionCandidates().  */
  std::vector<SortedPair<GeometryId>> FindCollisionCandidates() const;

  /* Implementation of GeometryState::HasCollisions().  */
  bool HasCollisions() const;

  //@}

  /* The representation of every geometry that was successfully requested for
   use for hydroelastic contact surface computation. */
  const hydroelastic::Geometries& hydroelastic_geometries() const;

 private:
  // Testing utilities:
  // These functions facilitate *limited* introspection into the engine state.
  // This enables unit tests to make assertions about pre- and post-operation
  // state.

  // Reports true if other is detectably a deep copy of this engine.
  bool IsDeepCopy(const ProximityEngine<T>& other) const;

  // Reports the pose (X_WG) of the geometry with the given id.
  const math::RigidTransform<double> GetX_WG(GeometryId id,
                                             bool is_dynamic) const;

  ////////////////////////////////////////////////////////////////////////////

  // TODO(SeanCurtis-TRI): Pimpl + template implementation has proven
  // problematic. This gets around it but it isn't a reliable long-term
  // solution. Figure out how to make this work with unique_ptr or
  // copyable unique_ptr
  //
  // The implementation details.
  class Impl;
  Impl* impl_{};

  // Private constructor to use for scalar conversion.
  explicit ProximityEngine(Impl* impl);

  // Engine on one scalar can see the members of other engines.
  template <typename> friend class ProximityEngine;

  // Facilitate testing.
  friend class ProximityEngineTester;

  // Reports if the geometry with the given id is represented by an fcl::Convex.
  // This function exists solely for the purpose tracking the "represent Mesh
  // as Convex" logic in other unit tests. When we represent meshes as
  // non-convex entities in their own right, we can remove this method.
  // This very specifically does *not* answer the question of whether the
  // underlying shape is *mathematically* convex, just that it is implemented
  // as fcl::Convex.
  bool IsFclConvexType(GeometryId id) const;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
