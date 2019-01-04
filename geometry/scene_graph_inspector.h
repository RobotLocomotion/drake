#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/geometry_state.h"

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
   guaranteed to remain fixed between topological changes (e.g., removal or
   addition of geometry/frames).  */
  const std::vector<GeometryId>& all_geometry_ids() const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->get_geometry_ids();
  }

  /** Reports the _total_ number of geometries in the scene graph with the
   indicated role.  */
  int NumGeometriesWithRole(Role role) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetNumGeometriesWithRole(role);
  }

  /** Reports the total number of _dynamic_ geometries in the scene graph.  */
  int GetNumDynamicGeometries() const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetNumDynamicGeometries();
  }

  /** Reports the total number of _anchored_ geometries. This should provide
   the same answer as calling GetNumFrameGeometries() with the world frame id.
   */
  int GetNumAnchoredGeometries() const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetNumAnchoredGeometries();
  }

  //@}

  /** @name                Sources and source-related data  */
  //@{

  /** Reports `true` if the given `id` maps to a registered source.  */
  bool SourceIsRegistered(SourceId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->source_is_registered(id);
  }

  /** Reports the name for the source with the given `id`.
   @throws std::logic_error if `id` does not map to a registered source.  */
  const std::string& GetSourceName(SourceId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->get_source_name(id);
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
    return state_->GetFramesForSource(id);
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

  /** Reports the name of the frame with the given `id`.
   @throws std::logic_error if `id` does not map to a registered frame.  */
  const std::string& GetName(FrameId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->get_frame_name(id);
  }

  /** Reports the frame group for the frame with the given `id`.
   @throws  std::logic_error if `id` does not map to a registered frame.
   @internal This value is equivalent to the old "model instance id".  */
  int GetFrameGroup(FrameId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->get_frame_group(id);
  }

  /** Reports the number of geometries affixed to the frame with the given `id`.
   This count does _not_ include geometries attached to frames that are
   descendants of this frame.
   @throws std::logic_error if `id` does not map to a registered frame.  */
  int NumGeometriesForFrame(FrameId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetNumFrameGeometries(id);
  }

  /** Reports the total number of geometries with the given `role` directly
   registered to the frame with the given `id`. This count does _not_ include
   geometries attached to frames that are descendants of this frame.
   @throws std::logic_error if `id` does not map to a registered frame.  */
  int NumGeometriesForFrameWithRole(FrameId id, Role role) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetNumFrameGeometriesWithRole(id, role);
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
    return state_->GetGeometryFromName(id, role, name);
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

  /** Reports the id of the frame to which the given geometry with the given
   `id` is registered.
   @throws std::logic_error if `id` does not map to a registered geometry.  */
  FrameId GetFrameId(GeometryId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetFrameId(id);
  }

  /** Reports the stored, canonical name of the geometry with the given `id`
   (see  @ref canonicalized_geometry_names "GeometryInstance" for details).
   @throws std::logic_error if `id` does not map to a registered geometry.  */
  const std::string& GetName(GeometryId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->get_name(id);
  }

  /** Reports the pose of the geometry G with the given `id` in its registered
   _topological parent_ P. That topological parent may be a frame F or another
   geometry. If the geometry was registered directly to F, then `X_PG = X_FG`.
   @throws std::logic_error if `id` does not map to a registered geometry.  */
  const Isometry3<double> X_PG(GeometryId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetPoseInParent(id);
  }

  /** Reports the pose of the geometry G with the given `id` in its registered
   frame F (regardless of whether its _topological parent_ is another geometry P
   or not). If the geometry was registered directly to the frame F, then
   `X_PG = X_FG`.
   @throws std::logic_error if `id` does not map to a registered geometry.  */
  const Isometry3<double> X_FG(GeometryId id) const {
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
    return state_->get_proximity_properties(id);
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
    return state_->get_illustration_properties(id);
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
