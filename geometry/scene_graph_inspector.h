#pragma once

#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/common/drake_deprecated.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/geometry_state.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

template <typename T>
class SceneGraph;
template <typename T>
class QueryObject;

/** The %SceneGraphInspector serves as a mechanism to query the topological
 structure of a SceneGraph instance. The topological structure consists of all
 of the SceneGraph data that does _not_ depend on input pose data. Including,
 but not limited to:

 - names of frames and geometries
 - hierarchies (parents of geometries, parents of frames, etc.)
 - geometry parameters (e.g., contact, rendering, visualization)
 - fixed poses of geometries relative to frames

 In contrast, the following pieces of data *do* depend on input pose data and
 _cannot_ be performed with the %SceneGraphInspector (see the QueryObject
 instead):

 - world pose of frames or geometry
 - collision queries
 - proximity queries

 A %SceneGraphInspector cannot be instantiated explicitly. Nor can it be copied
 or moved. A _reference_ to a %SceneGraphInspector instance can be acquired from

 - a SceneGraph instance (to inspect the state of the system's _model_), or
 - a QueryObject instance (to inspect the state of the scene graph data stored
   in the context).

 The reference should not be persisted (and, as previously indicated, cannot
 be copied). %SceneGraphInspector instances are cheap; they can be created,
 queried, and thrown out. If there is any doubt about the valid lifespan of
 a %SceneGraphInspector, throw out the old instance and request a new instance.

 @tparam T The scalar of the associated SceneGraph instance. The template
 parameter is provided for the sake of compatibility, although no queries (or
 their results) depend on the scalar.  */
template <typename T>
class SceneGraphInspector {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SceneGraphInspector)

  // NOTE: An inspector should never be released into the wild without having
  // set the state variable. Every query should start by demanding that state_
  // is defined.

  //----------------------------------------------------------------------------

  /** @name              Scene-graph wide data  */
  //@{

  /** Reports the number of registered sources -- whether they have registered
   frames/geometries or not. This will always be at least 1; the SceneGraph
   itself counts as a source.  */
  int num_sources() const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->get_num_sources();
  }

  /** Reports the _total_ number of frames registered in the scene graph
   (including the world frame).  */
  int num_frames() const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->get_num_frames();
  }

  /** Provides a range object for all of the frame ids in the scene graph. The
   order is not generally guaranteed; but it will be consistent as long as there
   are no changes to the topology. This is intended to be used as:
   @code
   for (FrameId id : inspector.all_frame_ids()) {
    ...
   }
   @endcode

   This includes the id for the world frame.  */
  typename GeometryState<T>::FrameIdRange all_frame_ids() const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->get_frame_ids();
  }

  /** Reports the _total_ number of geometries in the scene graph.  */
  int num_geometries() const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->get_num_geometries();
  }

  /** Returns the set of all ids for registered geometries. The order is _not_
   guaranteed to have any particular meaning. But the order is
   guaranteed to remain fixed until a topological change is made (e.g., removal
   or addition of geometry/frames).  */
  const std::vector<GeometryId> GetAllGeometryIds() const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetAllGeometryIds();
  }

  /** Reports the _total_ number of geometries in the scene graph with the
   indicated role.  */
  int NumGeometriesWithRole(Role role) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->NumGeometriesWithRole(role);
  }

  /** Reports the total number of _dynamic_ geometries in the scene graph.  */
  int NumDynamicGeometries() const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->NumDynamicGeometries();
  }

  DRAKE_DEPRECATED("2020-05-01", "Use NumDynamicGeometries() instead.")
  int GetNumDynamicGeometries() const {
    return NumDynamicGeometries();
  }

  /** Reports the total number of _anchored_ geometries. This should provide
   the same answer as calling NumGeometriesForFrame() with the world frame id.
   */
  int NumAnchoredGeometries() const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->NumAnchoredGeometries();
  }

  DRAKE_DEPRECATED("2020-05-01", "Use NumAnchoredGeometries() instead.")
  int GetNumAnchoredGeometries() const {
    return NumAnchoredGeometries();
  }

  /** Returns all pairs of geometries that are candidates for collision (in no
   particular order). See SceneGraph::ExcludeCollisionsBetween() or
   SceneGraph::ExcludeCollisionsWithin() for information on why a particular
   pair may _not_ be a candidate. For candidate pair (A, B), the candidate is
   always guaranteed to be reported in a fixed order (i.e., always (A, B) and
   _never_ (B, A)). This is the same ordering as would be returned by, e.g.,
   QueryObject::ComputePointPairPenetration().  */
  std::set<std::pair<GeometryId, GeometryId>> GetCollisionCandidates()
      const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetCollisionCandidates();
  }

  //@}

  /** @name                Sources and source-related data  */
  //@{

  /** Reports `true` if the given `id` maps to a registered source.  */
  bool SourceIsRegistered(SourceId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->SourceIsRegistered(id);
  }

  /** Reports the name for the source with the given `id`.
   @throws std::logic_error if `id` does not map to a registered source.  */
  const std::string& GetSourceName(SourceId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetName(id);
  }

  /** Reports the number of frames registered to the source with the given `id`.
   @throws std::logic_error if `id` does not map to a registered source.  */
  int NumFramesForSource(SourceId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->NumFramesForSource(id);
  }

  /** Reports the ids of all of the frames registered to the source with the
   given source `id`.
   @throws std::logic_error if `id` does not map to a registered source.  */
  const std::unordered_set<FrameId>& FramesForSource(SourceId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->FramesForSource(id);
  }

  //@}

  /** @name              Frames and their properties  */
  //@{

  /** Reports if the frame with given `frame_id` was registered to the source
   with the given `source_id`.
   @param frame_id      The query frame id.
   @param source_id     The query source id.
   @returns True if `frame_id` was registered on `source_id`.
   @throws std::logic_error  If `frame_id` does not map to a registered frame
                             or `source_id` does not map to a registered source.
   */
  bool BelongsToSource(FrameId frame_id, SourceId source_id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->BelongsToSource(frame_id, source_id);
  }

  /** Reports the _name_ of the geometry source that registered the frame with
   the given `id`.
   @throws std::logic_error  If `id` does not map to a registered frame.  */
  const std::string& GetOwningSourceName(FrameId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetOwningSourceName(id);
  }

  /** Reports the name of the frame with the given `id`.
   @throws std::logic_error if `id` does not map to a registered frame.
   @pydrake_mkdoc_identifier{1args_frame_id}
   */
  const std::string& GetName(FrameId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetName(id);
  }

  /** Reports the frame group for the frame with the given `id`.
   @throws  std::logic_error if `id` does not map to a registered frame.
   @internal This value is equivalent to the old "model instance id".  */
  int GetFrameGroup(FrameId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetFrameGroup(id);
  }

  /** Reports the number of geometries affixed to the frame with the given `id`.
   This count does _not_ include geometries attached to frames that are
   descendants of this frame.
   @throws std::logic_error if `id` does not map to a registered frame.  */
  int NumGeometriesForFrame(FrameId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->NumGeometriesForFrame(id);
  }

  /** Reports the total number of geometries with the given `role` directly
   registered to the frame with the given `id`. This count does _not_ include
   geometries attached to frames that are descendants of this frame.
   @throws std::logic_error if `id` does not map to a registered frame.  */
  int NumGeometriesForFrameWithRole(FrameId id, Role role) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->NumGeometriesForFrameWithRole(id, role);
  }

  /** Reports the id of the geometry with the given `name` and `role`, attached
   to the frame with the given frame `id`.
   @param id        The id of the frame whose geometry is being queried.
   @param role      The assigned role of the desired geometry.
   @param name      The name of the geometry to query for. The name will be
                    canonicalized prior to lookup (see
                    @ref canonicalized_geometry_names "GeometryInstance" for
                    details).
   @return The id of the queried geometry.
   @throws std::logic_error if no such geometry exists, multiple geometries have
                            that name, or if the `id` does not map to a
                            registered frame.  */
  GeometryId GetGeometryIdByName(FrameId id, Role role,
                                 const std::string& name) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetGeometryIdByName(id, role, name);
  }

  //@}

  /** @name           Geometries and their properties  */
  //@{

  /** Reports if the given geometry id was registered to the source with the
   given source id.
   @param geometry_id   The query geometry id.
   @param source_id     The query source id.
   @returns True if `geometry_id` was registered on `source_id`.
   @throws std::logic_error  If `geometry_id` does not map to a registered
                             geometry or `source_id` does not map to a
                             registered source.  */
  bool BelongsToSource(GeometryId geometry_id, SourceId source_id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->BelongsToSource(geometry_id, source_id);
  }

  /** Reports the _name_ of the geometry source that registered the geometry
   with the given `id`.
   @throws std::logic_error  If `id` does not map to a registered geometry.  */
  const std::string& GetOwningSourceName(GeometryId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetOwningSourceName(id);
  }

  /** Reports the id of the frame to which the given geometry with the given
   `id` is registered.
   @throws std::logic_error if `id` does not map to a registered geometry.  */
  FrameId GetFrameId(GeometryId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetFrameId(id);
  }

  /** Reports the stored, canonical name of the geometry with the given `id`
   (see  @ref canonicalized_geometry_names "GeometryInstance" for details).
   @throws std::logic_error if `id` does not map to a registered geometry.
   @pydrake_mkdoc_identifier{1args_geometry_id}
   */
  const std::string& GetName(GeometryId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetName(id);
  }

  /** Returns the shape specified for the geometry with the given `id`. In order
   to extract the details of the shape, it should be passed through an
   implementation of a ShapeReifier.  */
  const Shape& GetShape(GeometryId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetShape(id);
  }

  /** Reports the pose of the geometry G with the given `id` in its registered
   _topological parent_ P, `X_PG`. That topological parent may be a frame F or
   another geometry. If the geometry was registered directly to F, then
   `X_PG = X_FG`.
   @sa GetPoseInFrame()
   @throws std::logic_error if `id` does not map to a registered geometry.  */
  const math::RigidTransform<double>& GetPoseInParent(GeometryId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetPoseInParent(id);
  }

  /** Reports the pose of the geometry G with the given `id` in its registered
   frame F (regardless of whether its _topological parent_ is another geometry P
   or not). If the geometry was registered directly to the frame F, then
   `X_PG = X_FG`.
   @sa GetPoseInParent()
   @throws std::logic_error if `id` does not map to a registered geometry.  */
  const math::RigidTransform<double>& GetPoseInFrame(GeometryId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetPoseInFrame(id);
  }

  /** Returns a pointer to the const proximity properties of the geometry with
   the given `id`.
   @param id   The identifier for the queried geometry.
   @return A pointer to the properties (or nullptr if there are no such
           properties).
   @throws std::logic_error if `id` does not map to a registered geometry.  */
  const ProximityProperties* GetProximityProperties(
      GeometryId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetProximityProperties(id);
  }

  /** Returns a pointer to the const illustration properties of the geometry
   with the given `id`.
   @param id   The identifier for the queried geometry.
   @return A pointer to the properties (or nullptr if there are no such
           properties).
   @throws std::logic_error if `id` does not map to a registered geometry.  */
  const IllustrationProperties* GetIllustrationProperties(
      GeometryId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetIllustrationProperties(id);
  }

  /** Returns a pointer to the const perception properties of the geometry
   with the given `id`.
   @param id   The identifier for the queried geometry.
   @return A pointer to the properties (or nullptr if there are no such
           properties).
   @throws std::logic_error if `id` does not map to a registered geometry.  */
  const PerceptionProperties* GetPerceptionProperties(
      GeometryId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetPerceptionProperties(id);
  }

  /** Reports true if the two geometries with given ids `id1` and `id2`, define
   a collision pair that has been filtered out.
   @throws std::logic_error if either id does not map to a registered
                            geometry or if any of the geometries do not have
                            a proximity role.  */
  bool CollisionFiltered(GeometryId id1, GeometryId id2) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->CollisionFiltered(id1, id2);
  }

  /** Introspects the geometry indicated by the given `id`. The geometry will
   be passed into the provided `reifier`. This is the mechanism by which
   external code can discover and respond to the different types of geometries
   stored in SceneGraph. See ShapeToString as an example.
   @throws std::logic_error if the `id` does not refer to a valid geometry.  */
  void Reify(GeometryId id, ShapeReifier* reifier) const {
    DRAKE_DEMAND(state_ != nullptr);
    state_->GetShape(id).Reify(reifier);
  }

  //@}

 private:
  // Only SceneGraph and QueryObject instances can construct
  // SceneGraphInspector instances.
  friend class SceneGraph<T>;
  friend class QueryObject<T>;

  // Testing utility.
  friend class SceneGraphInspectorTester;

  // Simple default constructor to be used by SceneGraph and QueryObject.
  SceneGraphInspector() = default;

  // Sets the scene graph data to inspect -- the inspector does _not_ own the
  // data and expects that the lifespan of the provided data is longer than
  // the inspector.
  void set(const GeometryState<T>* state) { state_ = state; }

  const GeometryState<T>* state_{nullptr};
};

}  // namespace geometry
}  // namespace drake
