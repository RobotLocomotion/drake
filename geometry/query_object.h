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
#include "drake/math/rigid_transform.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace geometry {

template <typename T>
class SceneGraph;

/** The %QueryObject serves as a mechanism to query the state of the world's
 geometry. It can be used to introspect geometry properties, topology (via its
 SceneGraphInspector), or to query its current poses. The SceneGraph has an
 abstract-valued port that contains a %QueryObject (i.e., a %QueryObject-valued
 output port).

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

 The %QueryObject _can_ be copied. The copied instance is no longer "live"; it
 is now "baked". Essentially, it freezes the state of the live scene graph in
 its current configuration and disconnects it from the system and context. This
 means, even if the original context changes values, the copied/baked instance
 will always reproduce the same query results. This baking process is not cheap
 and should not be done without consideration.

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

 @note To perform proximity or perception queries, please see the
 ProximityQueryObject or PerceptionQueryObject classes, respectively.

 @tparam_nonsymbolic_scalar
*/
template <typename T>
class QueryObject {
 public:
  /** Constructs a default QueryObject (all pointers are null). */
  QueryObject() = default;

  /** @name Implements CopyConstructible, CopyAssignable, MoveConstructible, MoveAssignable

   Calling the copy constructor or assignment will turn a _live_ %QueryObject
   into a _baked_ %QueryObject (an expensive operation). Copying baked
   QueryObjects is cheap.  */
  //@{

  QueryObject(const QueryObject& other);
  QueryObject& operator=(const QueryObject&);
  QueryObject(QueryObject&&) = default;
  QueryObject& operator=(QueryObject&&) = default;

  //@}

  // Note to developers on adding queries:
  //  All queries should call ThrowIfNotCallable() before taking any action.
  //  Furthermore, an invocation of that query method should be included in
  //  query_object_test.cc in the DefaultQueryThrows test to confirm that the
  //  query *is* calling ThrowIfNotCallable().

  /** Provides an inspector for the topological structure of the underlying
   scene graph data (see SceneGraphInspector for details).  */
  const SceneGraphInspector<T>& inspector() const {
    return inspector_;
  }

  /** @name                Pose-dependent Introspection

   These methods provide access to introspect geometry and frame quantities that
   directly depend on the poses of the frames.  For geometry and frame
   quantities that do not depend on the poses of frames, such as  X_FG, use
   inspector() to access the SceneGraphInspector.  */
  //@{

  // TODO(SeanCurtis-TRI): When I have RigidTransform internally, make these
  // const references.
  /** Reports the position of the frame indicated by `id` relative to the world
   frame.
   @throws std::logic_error if the frame `id` is not valid.  */
  const math::RigidTransform<T>& X_WF(FrameId id) const;

  /** Reports the position of the frame indicated by `id` relative to its parent
   frame. If the frame was registered with the world frame as its parent frame,
   this value will be identical to that returned by X_WF().
   @note This is analogous to but distinct from SceneGraphInspector::X_PG().
   In this case, the pose will *always* be relative to another frame.
   @throws std::logic_error if the frame `id` is not valid.  */
  const math::RigidTransform<T>& X_PF(FrameId id) const;

  /** Reports the position of the geometry indicated by `id` relative to the
   world frame.
   @throws std::logic_error if the geometry `id` is not valid.  */
  const math::RigidTransform<T>& X_WG(GeometryId id) const;

  //@}

  /** See ProximityQueryObject::ComputePointPairPenetration().  */
  DRAKE_DEPRECATED("2020-08-01", "Use the ProximityQueryObject instead.")
  std::vector<PenetrationAsPointPair<double>> ComputePointPairPenetration()
      const;

  /** See ProximityQueryObject::ComputeContactSurfaces().  */
  DRAKE_DEPRECATED("2020-08-01", "Use the ProximityQueryObject instead.")
  std::vector<ContactSurface<T>> ComputeContactSurfaces() const;

  /** See ProximityQueryObject::ComputeContactSurfacesWithFallback().  */
  DRAKE_DEPRECATED("2020-08-01", "Use the ProximityQueryObject instead.")
  void ComputeContactSurfacesWithFallback(
      std::vector<ContactSurface<T>>* surfaces,
      std::vector<PenetrationAsPointPair<double>>* point_pairs) const;

  /** See ProximityQueryObject::FindCollisionCandidates().  */
  DRAKE_DEPRECATED("2020-08-01", "Use the ProximityQueryObject instead.")
  std::vector<SortedPair<GeometryId>> FindCollisionCandidates() const;

  /** See ProximityQueryObject::HasCollisions().  */
  DRAKE_DEPRECATED("2020-08-01", "Use the ProximityQueryObject instead.")
  bool HasCollisions() const;

  /** See ProximityQueryObject::ComputeSignedDistancePairwiseClosestPoints().
   */
  DRAKE_DEPRECATED("2020-08-01", "Use the ProximityQueryObject instead.")
  std::vector<SignedDistancePair<T>> ComputeSignedDistancePairwiseClosestPoints(
      const double max_distance =
          std::numeric_limits<double>::infinity()) const;

  /** See ProximityQueryObject::ComputeSignedDistancePairClosestPoints().  */
  DRAKE_DEPRECATED("2020-08-01", "Use the ProximityQueryObject instead.")
  SignedDistancePair<T> ComputeSignedDistancePairClosestPoints(
      GeometryId id_A, GeometryId id_B) const;

  /** See ProximityQueryObject::ComputeSignedDistanceToPoint().  */
  DRAKE_DEPRECATED("2020-08-01", "Use the ProximityQueryObject instead.")
  std::vector<SignedDistanceToPoint<T>>
  ComputeSignedDistanceToPoint(const Vector3<T> &p_WQ,
                               const double threshold
                               = std::numeric_limits<double>::infinity()) const;

  /** See PerceptionQueryObject::RenderColorImage().  */
  DRAKE_DEPRECATED("2020-08-01", "Use the PerceptionQueryObject instead.")
  void RenderColorImage(const render::CameraProperties& camera,
                        FrameId parent_frame,
                        const math::RigidTransformd& X_PC,
                        bool show_window,
                        systems::sensors::ImageRgba8U* color_image_out) const;

  /** See PerceptionQueryObject::RenderDepthImage().  */
  DRAKE_DEPRECATED("2020-08-01", "Use the PerceptionQueryObject instead.")
  void RenderDepthImage(const render::DepthCameraProperties& camera,
                        FrameId parent_frame,
                        const math::RigidTransformd& X_PC,
                        systems::sensors::ImageDepth32F* depth_image_out) const;

  /** See PerceptionQueryObject::RenderLabelImage().  */
  DRAKE_DEPRECATED("2020-08-01", "Use the PerceptionQueryObject instead.")
  void RenderLabelImage(const render::CameraProperties& camera,
                        FrameId parent_frame,
                        const math::RigidTransformd& X_PC,
                        bool show_window,
                        systems::sensors::ImageLabel16I* label_image_out) const;

 protected:
  /** Confirms that the %QueryObject (and its child classes) can operate on
   valid state. _Every_ query should invoke this method before doing any other
   work.
   @throws std::runtime_error if this query object is not valid for queries.  */
  void ValidateAndUpdate() const {
    ThrowIfNotCallable();
    FullPoseUpdate();
  }

  /** Access the GeometryState associated with this QueryObject.
   @pre ThrowIfNotCallable() has been invoked prior to this. */
  const GeometryState<T>& geometry_state() const;

 private:
  // SceneGraph is the only class that may call set().
  friend class SceneGraph<T>;
  // Convenience class for testing.
  friend class QueryObjectTest;

  // Sets the query object to be *live*. That means the `context` and
  // `scene_graph` cannot be null.
  void set(const systems::Context<T>* context,
           const SceneGraph<T>* scene_graph) {
    DRAKE_DEMAND(context);
    DRAKE_DEMAND(scene_graph);
    state_.reset();
    context_ = context;
    scene_graph_ = scene_graph;
    inspector_.set(&geometry_state());
  }

  // Update all poses. This method does no work if this is a "baked" query
  // object (see class docs for discussion).
  void FullPoseUpdate() const;

  // Reports true if this object is configured so that it can support a query.
  bool is_callable() const {
    const bool live_condition = context_ != nullptr && scene_graph_ != nullptr;
    const bool baked_condition = state_ != nullptr;
    // I.e., only one of the two conditions can be satisfied.
    return live_condition != baked_condition;
  }

  // Reports true if this object is in default configuration (not callable).
  bool is_default() const {
    return context_ == nullptr && scene_graph_ == nullptr && state_ == nullptr;
  }

  // Reports if the object can be copied; it must either be callable or default.
  bool is_copyable() const {
    return is_callable() || is_default();
  }

  // Throws an exception if the QueryObject is neither "live" nor "baked" (see
  // class docs for discussion).
  void ThrowIfNotCallable() const {
    if (!is_callable()) {
      throw std::runtime_error(
          "Attempting to perform query on invalid QueryObject.");
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
  // _both_ be non-null for a live QueryObject and both be null for a baked
  // QueryObject.
  const systems::Context<T>* context_{nullptr};
  const SceneGraph<T>* scene_graph_{nullptr};

  SceneGraphInspector<T> inspector_;

  // When a QueryObject is copied to a "baked" version, it contains a fully
  // updated GeometryState. Copies of bakes all share the same version.
  std::shared_ptr<const GeometryState<T>> state_{};
};

}  // namespace geometry
}  // namespace drake
