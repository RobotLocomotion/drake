#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/query_results/signed_distance_pair.h"
#include "drake/geometry/query_results/signed_distance_to_point.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace geometry {

template <typename T>
class SceneGraph;

/** The %QueryObject serves as a mechanism to perform geometry queries on the
 world's geometry. The SceneGraph has an abstract-valued port that contains
 a  %QueryObject (i.e., a %QueryObject-valued output port).

 To perform geometry queries on SceneGraph:
   - a LeafSystem must have a %QueryObject-valued input port and connect it to
     the corresponding query output port on SceneGraph,
   - the querying LeafSystem can evaluate the input port, retrieving a `const
     QueryObject&` in return, and, finally,
   - invoke the appropriate method on the %QueryObject.

 The const reference returned by the input port is considered "live" - it is
 linked to the context, system, and cache (making full use of all of those
 mechanisms). This const reference should _never_ be persisted; doing so can
 lead to erroneous query results. It is simpler and more advisable to acquire it
 for evaluation in a limited scope (e.g., CalcTimeDerivatives()) and then
 discard it. If a %QueryObject is needed for many separate functions in a
 LeafSystem, each should re-evaluate the input port. The underlying caching
 mechanism should make the cost of this negligible.

 In addition to not persisting the reference from the output port, the
 %QueryObject shouldn't be copied. Strictly speaking, it is an allowed
 operation, but the result is not live, and any geometry query performed on the
 copy will throw an exception.

 <h2>Queries and scalar type</h2>

 A %QueryObject _cannot_ be converted to a different scalar type. A %QueryObject
 of scalar type T can only be acquired from the output port of a SceneGraph
 of type T evaluated on a corresponding Context, also of type T.

 %QueryObject's support for arbitrary scalar type is incomplete. Not all queries
 support all scalar types to the same degree. In some cases the level of support
 is obvious (such as when the query is declared *explicitly* in terms of a
 double-valued scalar -- see ComputePointPairPenetration()). In other cases,
 where the query is expressed in terms of scalar `T`, the query may have
 restrictions. If a query has restricted scalar support, it is included in
 the query's documentation.

 @tparam T The scalar type. Must be a valid Eigen scalar.

 Instantiated templates for the following kinds of T's are provided:

 - double
 - AutoDiffXd

 They are already available to link against in the containing library.
 No other values for T are currently supported.  */
template <typename T>
class QueryObject {
 public:
  /** Constructs a default QueryObject (all pointers are null). */
  QueryObject() = default;

#ifndef DRAKE_DOXYGEN_CXX
  // NOTE: The copy semantics are provided to be compatible with AbstractValue.
  // The result will always be a "default" QueryObject (i.e., all pointers are
  // null). The SceneGraph is responsible for guaranteeing the returned
  // QueryObject is "live" (via CalcQueryObject()).
  QueryObject(const QueryObject& other);
  QueryObject& operator=(const QueryObject&);
  // NOTE: The move semantics are implicitly deleted by the copy semantics.
  // There is no sense in "moving" a query object.
#endif  // DRAKE_DOXYGEN_CXX

  // Note to developers on adding queries:
  //  All queries should call ThrowIfDefault() before taking any action.
  //  Furthermore, an invocation of that query method should be included in
  //  query_object_test.cc in the DefaultQueryThrows test to confirm that the
  //  query *is* calling ThrowIfDefault().

  /** Provides an inspector for the topological structure of the underlying
   scene graph data (see SceneGraphInspector for details).  */
  const SceneGraphInspector<T>& inspector() const {
    return inspector_;
  }

  //----------------------------------------------------------------------------
  /** @name                Collision Queries

   These queries detect _collisions_ between geometry. Two geometries collide
   if they overlap each other and are not explicitly excluded through
   @ref collision_filter_concepts "collision filtering". These algorithms find
   those colliding cases, characterize them, and report the essential
   characteristics of that collision.

   For two colliding geometries g_A and g_B, it is guaranteed that they will
   map to `id_A` and `id_B` in a fixed, repeatable manner, where `id_A` and
   `id_B` are GeometryId's of geometries g_A and g_B respectively.

   These methods are affected by collision filtering; element pairs that
   have been filtered will not produce contacts, even if their collision
   geometry is penetrating.     */
  //@{

  /** Computes the penetrations across all pairs of geometries in the world.
   Only reports results for _penetrating_ geometries; if two geometries are
   separated, there will be no result for that pair. Pairs of _anchored_
   geometry are also not reported. The penetration between two geometries is
   characterized as a point pair (see PenetrationAsPointPair). This method is
   affected by collision filtering.

   <h3>Scalar support</h3>
   This method only provides double-valued penetration results.

   <!--
   TODO (SeanCurtis-TRI): This can/should be changed to offer at least partial
   AutoDiffXd support. At the very least, it should be declared on T and throw
   for AutoDiffXd. This is related to PR 11143
   https://github.com/RobotLocomotion/drake/pull/11143. In that PR, MBP is
   taking responsibility to know whether or not QueryObject supports AutoDiff
   penetration queries; MBP should not be responsible for that knowledge. By
   moving the exception into SceneGraph, it removes the false dependency and
   allows us to gradually increase the AutoDiff support for penetration.
   -->

   @returns A vector populated with all detected penetrations characterized as
            point pairs. */
  std::vector<PenetrationAsPointPair<double>> ComputePointPairPenetration()
      const;

  /**
   Reports pair-wise intersections and characterizes each non-empty
   intersection as a ContactSurface. The computation is subject to collision
   filtering.

   @returns A vector populated with contact surfaces of all detected
            intersecting pairs of geometries.
   @note  This function is not implemented yet. */
  std::vector<ContactSurface<T>> ComputeContactSurfaces() const;

  //@}

  //---------------------------------------------------------------------------
  // TODO(DamrongGuoy): Write a better documentation for Signed Distance
  // Queries.
  /**
   @anchor signed_distance_query
   @name                   Signed Distance Queries

   These queries provide the signed distance between two objects. Each query
   has a specific definition of the signed distance being positive, negative,
   or zero associated with some notions of being outside, inside, or on
   the boundary.

   These queries provide bookkeeping data like geometry id(s) of the geometries
   involved and the important locations on the boundaries of these geometries.

   The signed distance function is a continuous function. Its partial
   derivatives are continuous almost everywhere.
  */
  //@{

  // TODO(DamrongGuoy): Refactor documentation of
  // ComputeSignedDistancePairwiseClosestPoints(). Move the common sections
  // into Signed Distance Queries.
  /**
   Computes the signed distance together with the nearest points across all
   pairs of geometries in the world. Reports both the separating geometries
   and penetrating geometries.

   This query provides φ(A, B), the signed distance between two objects A and B.

   If the objects do not overlap (i.e., A ⋂ B = ∅), φ > 0 and represents the
   minimal distance between the two objects. More formally:
   φ = min(|Aₚ - Bₚ|)
   ∀ Aₚ ∈ A and Bₚ ∈ B.
   @note The pair (Aₚ, Bₚ) is a "witness" of the distance.
   The pair need not be unique (think of two parallel planes).

   If the objects touch or overlap (i.e., A ⋂ B ≠ ∅), φ ≤ 0 and can be
   interpreted as the negative penetration depth. It is the smallest length of
   the vector v, such that by shifting one object along that vector relative to
   the other, the two objects will no longer be overlapping. More formally,
   φ(A, B) = -min |v|.
   s.t (Tᵥ · A) ⋂ B = ∅
   where Tᵥ is a rigid transformation that displaces A by the vector v, namely
   Tᵥ · A = {u + v | ∀ u ∈ A}.
   By implication, there exist points Aₚ and Bₚ on the surfaces of objects A and
   B, respectively, such that Aₚ + v = Bₚ, Aₚ ∈ A ∩ B, Bₚ ∈ A ∩ B. These points
   are the witnesses to the penetration.

   This method is affected by collision filtering; geometry pairs that
   have been filtered will not produce signed distance query results.

   Notice that this is an O(N²) operation, where N
   is the number of geometries remaining in the world after applying collision
   filter. We report the distance between dynamic objects, and between dynamic
   and anchored objects. We DO NOT report the distance between two anchored
   objects.

   <h3>Scalar support</h3>
   This function does not support halfspaces. If an unfiltered pair contains
   a halfspace, an exception will be thrown for all scalar types. Otherwise,
   this query supports all other pairs of Drake geometry types for `double`.
   For `AutoDiffXd`, it only supports distance between sphere-box and
   sphere-sphere. If there are any unfiltered geometry pairs that include other
   geometries, the AutoDiff throws an exception.

   <!-- TODO(SeanCurtis-TRI): Document expected precision of answer based on
   members of shape pair. See
   https://github.com/RobotLocomotion/drake/issues/10907 -->
   <!-- TODO(SeanCurtis-TRI): Support queries of halfspace-A, where A is _not_ a
   halfspace. See https://github.com/RobotLocomotion/drake/issues/10905 -->

   @retval near_pairs The signed distance for all unfiltered geometry pairs.
  */
  // TODO(hongkai.dai): add a distance bound as an optional input, such that the
  // function doesn't return the pairs whose signed distance is larger than the
  // distance bound.
  std::vector<SignedDistancePair<T>>
  ComputeSignedDistancePairwiseClosestPoints() const;

  // TODO(DamrongGuoy): Improve and refactor documentation of
  // ComputeSignedDistanceToPoint(). Move the common sections into Signed
  // Distance Queries. Update documentation as we add more functionality.
  // Right now it only supports spheres and boxes.
  /**
   Computes the signed distances and gradients to a query point from each
   geometry in the scene.

   @warning Currently supports spheres, boxes, and cylinders only. Silently
   ignores other kinds of geometries, which will be added later.

   This query provides φᵢ(p), φᵢ:ℝ³→ℝ, the signed distance to the position
   p of a query point from geometry Gᵢ in the scene.  It returns an array of
   the signed distances from all geometries.

   Optionally you can specify a threshold distance that will filter out any
   object beyond the threshold. By default, we report distances from the query
   point to every object.

   This query also provides the gradient vector ∇φᵢ(p) of the signed distance
   function from geometry Gᵢ. Note that, in general, if p is outside Gᵢ, then
   ∇φᵢ(p) equals the unit vector in the direction from the nearest point Nᵢ on
   Gᵢ's surface to p. If p is inside Gᵢ, then ∇φᵢ(p) is in the direction from
   p to Nᵢ. This observation is written formally as:

   ∇φᵢ(p) = (p - Nᵢ)/|p - Nᵢ| if p is outside Gᵢ

   ∇φᵢ(p) = -(p - Nᵢ)/|p - Nᵢ| if p is inside Gᵢ

   Note that ∇φᵢ(p) is also defined on Gᵢ's surface, but we cannot use the
   above formula.

   <h3>Scalar support</h3>
   This query only supports computing distances from the point to spheres,
   boxes, and cylinders for both `double` and `AutoDiffXd` scalar types. If
   the SceneGraph contains any other geometry shapes, they will be silently
   ignored.

   @note For a sphere G, the signed distance function φᵢ(p) has an undefined
   gradient vector at the center of the sphere--every point on the sphere's
   surface has the same distance to the center.  In this case, we will assign
   ∇φᵢ(p) the unit vector Gx (x-directional vector of G's frame) expressed
   in World frame.

   @note For a box, at a point p on an edge or a corner of the box, the signed
   distance function φᵢ(p) has an undefined gradient vector.  In this case, we
   will assign a unit vector in the direction of the average of the outward
   face unit normals of the incident faces of the edge or the corner.
   A point p is considered being on a face, or an edge, or a corner of the
   box if it lies within a certain tolerance from them.

   @note For a box B, if a point p is inside the box, and it is equidistant to
   to multiple nearest faces, the signed distance function φᵢ(p) at p will have
   an undefined gradient vector. There is a nearest point candidate associated
   with each nearest face. In this case, we arbitrarily pick the point Nᵢ
   associated with one of the nearest faces.  Please note that, due to the
   possible round off error arising from applying a pose X_WG to B, there is no
   guarantee which of the nearest faces will be used.

   @note The signed distance function is a continuous function with respect to
   the position of the query point, but its gradient vector field may
   not be continuous. Specifically at a position equidistant to multiple
   nearest points, its gradient vector field is not continuous.

   @note For a convex object, outside the object at positive distance from
   the boundary, the signed distance function is smooth (having continuous
   first-order partial derivatives).

   @param[in] p_WQ            Position of a query point Q in world frame W.
   @param[in] threshold       We ignore any object beyond this distance.
                              By default, it is infinity, so we report
                              distances from the query point to every object.
   @retval signed_distances   A vector populated with per-object signed
                              distance values (and supporting data).
                              See SignedDistanceToPoint.
   */
  std::vector<SignedDistanceToPoint<T>>
  ComputeSignedDistanceToPoint(const Vector3<T> &p_WQ,
                               const double threshold
                               = std::numeric_limits<double>::infinity()) const;
  //@}

 private:
  // SceneGraph is the only class that may call set().
  friend class SceneGraph<T>;
  // Convenience class for testing.
  friend class QueryObjectTester;

  const GeometryState<T>& geometry_state() const;

  void set(const systems::Context<T>* context,
           const SceneGraph<T>* scene_graph) {
    DRAKE_DEMAND(context);
    DRAKE_DEMAND(scene_graph);
    context_ = context;
    scene_graph_ = scene_graph;
    inspector_.set(&geometry_state());
  }

  void ThrowIfDefault() const {
    if (!(context_ && scene_graph_)) {
      throw std::runtime_error(
          "Attempting to perform query on invalid QueryObject. "
          "Did you copy the QueryObject?");
    }
  }

  // TODO(SeanCurtis-TRI): Consider an alternate formulation. This stores
  // pointers to context and systems which raise some red flags in order to
  // enable other systems to evaluate queries without having a copy of the
  // geometry system or its context.
  //
  // Alternatively, this could store the *index* of the system in its parent
  // diagram. The context shares the same index in the parent diagram context.
  // Then the LeafSystem desiring to perform a query would pass itself and its
  // own context in (along with the query parameters). The QueryObject would
  // use those and the index to get the SceneGraph and Context.
  //
  // Several issues:
  //  1. Leads to a clunky API (passing self and context into *every* query).
  //  2. The index value would be insufficient if the SceneGraph were buried
  //     in a diagram with its query object port exported in the diagram.
  // This is documented for future consideration, and should not necessarily be
  // interpreted as a guaranteed task.

  // The contents of the "live" query object. It has pointers to the system and
  // context from which it spawned. It uses these to compute geometry queries
  // on the current context (fully-dependent on context). These pointers must
  // be null for "baked" contexts (e.g., the result of copying a "live"
  // context).
  const systems::Context<T>* context_{nullptr};
  const SceneGraph<T>* scene_graph_{nullptr};
  SceneGraphInspector<T> inspector_;
};

}  // namespace geometry
}  // namespace drake
