#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/geometry/dev/geometry_context.h"
#include "drake/geometry/dev/scene_graph_inspector.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/query_results/signed_distance_pair.h"

namespace drake {
namespace geometry {
namespace dev {

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

  /** @name               Context-dependent state

   These queries provide information about the context-dependent state --
   largely the poses of frames and geometries.   */
  //@{

  /** Reports the pose of the given frame relative to the world frame (i.e.,
   `X_WF`.
   @throws std::logic_error if `frame_id` is not a valid frame.   */
  const Isometry3<T>& GetPoseInWorld(FrameId frame_id) const;

  /** Reports the pose of the given geometry relative to the world frame (i.e.,
   `X_WG`.
   @throws std::logic_error if `geometry_id` is not a valid frame.   */
  const Isometry3<T>& GetPoseInWorld(GeometryId geometry_id) const;

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
  /**
   @anchor signed_distance_query
   @name                   Signed Distance Queries

   These queries provide φ(A, B), the signed distance between two objects A and
   B.

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

   @note the signed distance function is a continuous function with respect to
   the pose of the objects.
   */

  //@{

  // TODO(hongkai.dai): add a distance bound as an optional input, such that the
  // function doesn't return the pairs whose signed distance is larger than the
  // distance bound.
  /**
   * Computes the signed distance together with the nearest points across all
   * pairs of geometries in the world. Reports both the separating geometries
   * and penetrating geometries. Notice that this is an O(N²) operation, where N
   * is the number of geometries remaining in the world after applying collision
   * filter. We report the distance between dynamic objects, and between dynamic
   * and anchored objects. We DO NOT report the distance between two anchored
   * objects.
   * @retval near_pairs The signed distance for all unfiltered geometry pairs.
   */
  std::vector<SignedDistancePair<double>>
  ComputeSignedDistancePairwiseClosestPoints() const;
  //@}


  //---------------------------------------------------------------------------
  /** @name                Render Queries

   The methods support queries along the lines of "What do I see?" They support
   simulation of sensors. External entities define a sensor camera -- its
   extrinsic and intrinsic properties and %GeometryState renders into the
   provided image.

   Eventually, there will be multiple renderers that can be invoked which vary
   in the fidelity of the images they produce. Currently, only the low fidelity
   renderer is implemented. Invocation on a higher level of fidelity will throw
   an exception. As additional renderers get added, they will be engaged via
   this same interface.
   */
  //@{

  /** Renders and outputs the rendered color image.

   @param camera                The intrinsic properties of the camera.
   @param X_WC                  The pose of the camera in the world frame.
   @param[out] color_image_out  The rendered color image.
   @param show_window           If true, the render window will be displayed. */
  void RenderColorImage(const render::CameraProperties& camera,
                        const Isometry3<double>& X_WC,
                        systems::sensors::ImageRgba8U* color_image_out,
                        bool show_window) const;

  /** Overload for rendering a color image in which the camera's pose is defined
   relative to the given parent frame.  */
  void RenderColorImage(const render::CameraProperties& camera,
                        FrameId parent_frame,
                        const Isometry3<double>& X_PC,
                        systems::sensors::ImageRgba8U* color_image_out,
                        bool show_window) const;

  /** Renders and outputs the rendered depth image. In contrast to the other
   rendering operations, depth images don't have an option to display the
   window; generally, basic depth images are not readily communicative to
   humans.

   @param camera                The intrinsic properties of the camera.
   @param X_WC                  The pose of the camera in the world frame.
   @param[out] depth_image_out  The rendered depth image. */
  void RenderDepthImage(const render::DepthCameraProperties& camera,
                        const Isometry3<double>& X_WC,
                        systems::sensors::ImageDepth32F* depth_image_out) const;

  /** Overload for rendering a depth image in which the camera's pose is defined
   relative to the given parent frame.  */
  void RenderDepthImage(const render::DepthCameraProperties& camera,
                        FrameId parent_frame,
                        const Isometry3<double>& X_PC,
                        systems::sensors::ImageDepth32F* depth_image_out) const;

  /** Renders and outputs the rendered label image.

   @param camera                The intrinsic properties of the camera.
   @param X_WC                  The pose of the camera in the world frame.
   @param[out] label_image_out  The rendered label image.
   @param show_window           If true, the render window will be displayed. */
  void RenderLabelImage(const render::CameraProperties& camera,
                        const Isometry3<double>& X_WC,
                        systems::sensors::ImageLabel16I* label_image_out,
                        bool show_window) const;

  /** Overload for rendering a label image in which the camera's pose is defined
   relative to the given parent frame.  */
  void RenderLabelImage(const render::CameraProperties& camera,
                        FrameId parent_frame,
                        const Isometry3<double>& X_PC,
                        systems::sensors::ImageLabel16I* label_image_out,
                        bool show_window) const;

  //@}

 private:
  // SceneGraph is the only class that may call set().
  friend class SceneGraph<T>;
  // Convenience class for testing.
  friend class QueryObjectTester;

  const GeometryState<T>& geometry_state() const;

  void set(const GeometryContext<T>* context,
           const SceneGraph<T>* scene_graph) {
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
  SceneGraphInspector<T> inspector_;
};

}  // namespace dev
}  // namespace geometry
}  // namespace drake
