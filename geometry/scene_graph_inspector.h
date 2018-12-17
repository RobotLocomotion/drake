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
 of the SceneGraph data that does *not* depend on input pose data. Including,
 but not limited to:

 - names of frames and geometries
 - hierarchies (parents of geometries, parents of frames, etc.)
 - geometry parameters (e.g., contact, rendering, visualization)
 - fixed poses of geometries relative to frames

 In contrast, the following pieces of data *do* depend on input pose data and
 *cannot* be performed with the %SceneGraphInspector (see the QueryObject
 instead):

 - world pose of frames or geometry
 - collision queries
 - proximity queries

 A %SceneGraphInspector cannot be instantiated explicitly. Nor can it be copied
 or moved. A *reference* to a %SceneGraphInspector instance can be acquired from

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

  /** Reports the *total* number of frames registered in the scene graph
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

  /** Reports the *total* number of geometries in the scene graph.  */
  int num_geometries() const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->get_num_geometries();
  }

  /** The set of all ids for registered geometries. The order is_not_ guaranteed
   to have any particular semantic meaning. But the order is guaranteed to
   remain fixed between topological changes (e.g., removal or addition of
   geometry/frames).  */
  const std::vector<GeometryId>& all_geometry_ids() const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->get_geometry_ids();
  }

  /** Reports the *total* number of geometries in the scene graph with the
   indicated role.  */
  int NumGeometriesWithRole(Role role) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetNumGeometriesWithRole(role);
  }

  /** Reports the total number of *dynamic* geometries in the scene graph.  */
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

  /** Reports `true` if the given `id` references a registered source.  */
  bool SourceIsRegistered(SourceId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->source_is_registered(id);
  }

  /** Reports the name for the given source id.
   @throws std::logic_error if the identifier is invalid.  */
  const std::string& GetSourceName(SourceId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->get_source_name(id);
  }

  /** Reports the number of frames registered to the given source id. Returns
   zero for unregistered source ids.
   @throws std::logic_error if `source_id` is invalid.  */
  int NumFramesForSource(SourceId source_id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->NumFramesForSource(source_id);
  }

  /** Reports the ids of all of the frames registered to the source indicated by
   the given `source_id`.
   @throws std::logic_error if `source_id` is invalid.  */
  const std::unordered_set<FrameId>& FramesForSource(SourceId source_id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetFramesForSource(source_id);
  }

  //@}

  /** @name              Frames and their properties  */
  //@{

  /** Reports if the given frame id was registered to the given source id.
   @param frame_id      The query frame id.
   @param source_id     The query source id.
   @returns True if `frame_id` was registered on `source_id`.
   @throws std::logic_error  If the `frame_id` does _not_ map to a frame or the
                             identified source is not registered.  */
  bool BelongsToSource(FrameId frame_id, SourceId source_id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->BelongsToSource(frame_id, source_id);
  }

  /** Reports the name of the frame indicated by the given id.
   @throws std::logic_error if `frame_id` doesn't refer to a valid frame.  */
  const std::string& GetName(FrameId frame_id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->get_frame_name(frame_id);
  }

  /** Reports the frame group for the frame indicated by the given `id`.
   @throws  std::logic_error if `frame_id` doesn't refer to a valid frame.
   @internal This value is equivalent to the old "model instance id".  */
  int GetFrameGroup(FrameId frame_id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->get_frame_group(frame_id);
  }

  /** Reports the number of geometries affixed to the given frame id. This count
   does _not_ include geometries attached to frames that are descendants of this
   frame.
   @throws std::runtime_error if `frame_id` is invalid.  */
  int NumGeometriesForFrame(FrameId frame_id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetNumFrameGeometries(frame_id);
  }

  /** Reports the total number of geometries directly registered to the given
   frame. This count does _not_ include geometries attached to frames that are
   descendants of this frame.
   @throws std::runtime_error if the `frame_id` is invalid.  */
  int NumGeometriesForFrameWithRole(FrameId frame_id, Role role) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetNumFrameGeometriesWithRole(frame_id, role);
  }

  /** Reports the id of the geometry with the given name, attached to the
   indicated frame with the given role.
   @param frame_id  The frame whose geometry is being queried.
   @param role      The assigned role of the desired geometry.
   @param name      The name of the geometry to query for. The name will be
                    canonicalized prior to lookup (see
                    @ref canonicalized_geometry_names "GeometryInstance" for
                    details).
   @return The id of the queried geometry.
   @throws std::logic_error if no such geometry exists, multiple geometries have
                            that name, or if the frame doesn't exist.  */
  GeometryId GetGeometryIdByName(FrameId frame_id, Role role,
                                 const std::string& name) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetGeometryFromName(frame_id, role, name);
  }

  //@}

  /** @name           Geometries and their properties  */
  //@{

  /** Reports if the given geometry id was ultimately registered to the given
   source id.
   @param geometry_id   The query geometry id.
   @param source_id     The query source id.
   @returns True if `geometry_id` was registered on `source_id`.
   @throws std::logic_error  If the `geometry_id` does _not_ map to a valid
                             geometry or the identified source is not
                             registered  */
  bool BelongsToSource(GeometryId geometry_id, SourceId source_id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->BelongsToSource(geometry_id, source_id);
  }

  /** Reports the id of the frame to which the given geometry id is registered.
   @throws std::logic_error if the geometry id is invalid.  */
  FrameId GetFrameId(GeometryId geometry_id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetFrameId(geometry_id);
  }

  /** Reports the stored, canonical name of the geometry indicated by the given
   `id` (see  @ref canonicalized_geometry_names "GeometryInstance" for details).
   Reports the name of the geometry indicated by the given id.
   @throws std::logic_error if `geometry_id` doesn't refer to a valid geometry.
  */
  const std::string& GetName(GeometryId geometry_id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->get_name(geometry_id);
  }

  /** Reports the pose of the geometry G (indicated by `id`) in its registered
   _parent_ frame P. If the geometry was registered directly to the frame F,
   then `X_PG = X_FG`.
   @throws std::logic_error if the `id` is invalid.  */
  const Isometry3<double> X_PG(GeometryId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetPoseInParent(id);
  }

  /** Reports the pose of the geometry G (indicated by `id`) in its registered
   frame F (regardless of whether it was registered to another geometry or not).
   If the geometry was registered directly to the frame F, then X_PG = X_FG.
   @throws std::logic_error if the `id` is invalid.  */
  const Isometry3<double> X_FG(GeometryId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetPoseInFrame(id);
  }

  /** Returns a pointer to the const proximity properties of the geometry
   identified by `geometry_id`.
   @param geometry_id   The identifier for the queried geometry.
   @return A pointer to the properties (or nullptr if there are no such
           properties).  */
  const ProximityProperties* GetProximityProperties(
      GeometryId geometry_id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->get_proximity_properties(geometry_id);
  }

  /** Returns a pointer to the const illustration properties of the geometry
   identified by `geometry_id`.
   @param geometry_id   The identifier for the queried geometry.
   @return A pointer to the properties (or nullptr if there are no such
           properties.  */
  const IllustrationProperties* GetIllustrationProperties(
      GeometryId geometry_id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->get_illustration_properties(geometry_id);
  }

  /** Reports true if the collision pair (id1, id2) has been filtered out.
   @throws std::logic_error if either id does not reference a registered
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

  // Sets the scene graph data to inspect -- the inspector does *not* own the
  // data and expects that the lifespan of the provided data is longer than
  // the inspector.
  void set(const GeometryState<T>* state) { state_ = state; }

  const GeometryState<T>* state_{nullptr};
};

}  // namespace geometry
}  // namespace drake
