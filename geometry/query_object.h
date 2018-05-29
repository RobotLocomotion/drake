#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/geometry/geometry_context.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"

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

 A %QueryObject _cannot_ be converted to a different scalar type. A %QueryObject
 of scalar type S can only be acquired from the output port of a SceneGraph
 of type S evaluated on a corresponding GeometryContext, also of type S.

 @tparam T The scalar type. Must be a valid Eigen scalar.

 Instantiated templates for the following kinds of T's are provided:
 - double
 - AutoDiffXd

 They are already available to link against in the containing library.
 No other values for T are currently supported.  */
template <typename T>
class QueryObject {
 public:
  // NOTE: The copy semantics are provided to be compatible with AbstractValue.
  // The result will always be a "default" QueryObject (i.e., all pointers are
  // null). There is no public constructor, the assumption is that the only way
  // to acquire a reference/instance of QueryObject is through the
  // SceneGraph output port. The SceneGraph is responsible for
  // guaranteeing the returned QueryObject is "live" (via CalcQueryObject()).
  QueryObject(const QueryObject& other);
  QueryObject& operator=(const QueryObject&);
  // NOTE: The move semantics are implicitly deleted by the copy semantics.
  // There is no sense in "moving" a query object.

  // Note to developers on adding queries:
  //  All queries should call ThrowIfDefault() before taking any action.
  //  Furthermore, an invocation of that query method should be included in
  //  query_object_test.cc in the DefaultQueryThrows test to confirm that the
  //  query *is* calling ThrowIfDefault().

  //----------------------------------------------------------------------------
  /** @name                State queries */
  //@{

  /** Reports the name for the given source id.
   @throws  std::runtime_error if the %QueryObject is in default configuration.
   @throws  std::logic_error if the identifier is invalid. */
  const std::string& GetSourceName(SourceId id) const;

  /** Reports the id of the frame to which the given geometry id is registered.
   @throws  std::runtime_error if the %QueryObject is in default configuration.
   @throws  std::logic_error if the geometry id is invalid. */
  FrameId GetFrameId(GeometryId geometry_id) const;

  /** Returns the visual material of the geometry indicated by the given
   `geometry_id` (if it exists).
   @throws  std::runtime_error if the %QueryObject is in default configuration.
   @throws  std::logic_error if the geometry id is invalid. */
  const VisualMaterial* GetVisualMaterial(GeometryId geometry_id) const;

  //@}

  //----------------------------------------------------------------------------
  /** @name                Collision Queries

   These queries detect _collisions_ between geometry. Two geometries collide
   if they overlap each other and are not explicitly excluded through
   @ref collision_filter_concepts "collision filtering". These algorithms find
   those colliding cases, characterize them, and report the essential
   characteristics of that collision.  */
  //@{

  /** Computes the penetrations across all pairs of geometries in the world.
   Only reports results for _penetrating_ geometries; if two geometries are
   separated, there will be no result for that pair. Pairs of _anchored_
   geometry are also not reported. The penetration between two geometries is
   characterized as a point pair (see PenetrationAsPointPair).

   For two penetrating geometries g₁ and g₂, it is guaranteed that they will
   map to `id_A` and `id_B` in a fixed, repeatable manner.

   This method is affected by collision filtering; element pairs that
   have been filtered will not produce contacts, even if their collision
   geometry is penetrating.

   <!--
   NOTE: This is currently declared as double because we haven't exposed FCL's
   templated functionality yet. When that happens, double -> T.
   -->

   @returns A vector populated with all detected penetrations characterized as
            point pairs. */
  std::vector<PenetrationAsPointPair<double>> ComputePointPairPenetration()
      const;

  //@}

  //---------------------------------------------------------------------------
  /** @name                   Signed Distance Queries
   
   These queries represent _signed_distance_ querites -- queries to determine
   the signed distance. When two objects do not overlap, the signed distance
   is the actual distance between the two objects A and B, defined as the
   minimal distance between any pair of points Na and Nb, with Na on object A
   and Nb on object B. The nearest points are the pair of points that gives the
   minimal distance. When the two objects overlap, the signed distance is the
   penetration depth, a non-positive number. It is the smallest distance of a
   vector v, such that by shifting one object (for example, object A) along that
   vector to object A', the shifted object A' do not overlap with B (or they
   only overlap at the boundary). The nearest points between A' and B is Na' and
   Nb. The nearest points between A and B when the overlap, is defined as
   Na' - v and Nb. Notice that the signed distance function is a continuous
   function w.r.t the pose of the objects. */

  //@{

  /**
   * Computes the signed distance together with the nearest points across all
   * pairs of geometries in the world. Reports both the separating geometries
   * and penetrating geometries.
   * @retval near_pairs A vector of reporting the signed distance characterized
   * as point pairs. Notice that this is an O(N²) operation, where N is the 
   * number of geometries in the world. We report the distance between dynamic
   * objects, or between a dynamic object and a static object. We DO NOT report
   * the distance between two static objects.
   */
  std::vector<NearestPair<double>> ComputeSignedDistancePairwiseClosestPoints()
      const;
  //@}

 private:
  // SceneGraph is the only class that can instantiate QueryObjects.
  friend class SceneGraph<T>;
  // Convenience class for testing.
  friend class QueryObjectTester;

  // Only the SceneGraph<T> can instantiate this class - it gets
  // instantiated into a *copyable* default instance (to facilitate allocation
  // in contexts).
  QueryObject() = default;

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
  // use those and the index to get the SceneGraph and GeometryContext.
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
  const GeometryContext<T>* context_{nullptr};
  const SceneGraph<T>* scene_graph_{nullptr};
};

}  // namespace geometry
}  // namespace drake
