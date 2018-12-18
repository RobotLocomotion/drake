#pragma once

#include <string>

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

  //----------------------------------------------------------------------------

  /** @name              Scene-graph wide data  */

  /** Reports the *total* number of geometries in the scene graph.  */
  int num_geometries() const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->get_num_geometries();
  }

  /** Reports the *total* number of geometries in the scene graph with the
   indicated role.  */
  int NumGeometriesWithRole(Role role) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetNumGeometriesWithRole(role);
  }

  //@}

  /** @name                Sources and source-related data  */
  //@{

  // NOTE: An inspector should never be released into the wild without having
  // set the state variable. Every query should start by demanding that state_
  // is defined.

  /** Reports the name for the given source id.
   @throws std::logic_error if the identifier is invalid.  */
  const std::string& GetSourceName(SourceId id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->get_source_name(id);
  }

  /** Reports the number of frames registered to the given source id. Returns
   zero for unregistered source ids.  */
  int NumFramesForSource(SourceId source_id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->NumFramesForSource(source_id);
  }

  //@}

  /** @name              Frames and their properties  */
  //@{

  /** Reports the id of the frame to which the given geometry id is registered.
   @throws std::logic_error if the geometry id is invalid.  */
  FrameId GetFrameId(GeometryId geometry_id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetFrameId(geometry_id);
  }

  /** Reports the name of the frame indicated by the given id.
   @throws std::logic_error if `frame_id` doesn't refer to a valid frame.  */
  const std::string& GetName(FrameId frame_id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->get_frame_name(frame_id);
  }

  /** Reports the number of geometries affixed to the given frame id.
   @throws std::runtime_error if `frame_id` is invalid.  */
  int NumGeometriesForFrame(FrameId frame_id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetNumFrameGeometries(frame_id);
  }

  /** Reports the total number of geometries directly registered to the given
   frame. This count does _not_ include geometries attached to frames that are
   descendants of this frame.
   @throws std::runtime_error if the `frame_id` is invalid.  */
  int NumFrameGeometries(FrameId frame_id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetNumFrameGeometries(frame_id);
  }

  /** Reports the number of geometries assigned to the given frame with the
   given role.  */
  int NumFrameGeometriesWithRole(FrameId frame_id, Role role) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->GetNumFrameGeometriesWithRole(frame_id, role);
  }

  //@}

  /** @name           Geometries and their properties  */
  //@{

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

  /** Reports the name of the geometry indicated by the given id.
   @throws std::logic_error if `geometry_id` doesn't refer to a valid geometry.
  */
  const std::string& GetName(GeometryId geometry_id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->get_name(geometry_id);
  }

  /** Returns the visual material of the geometry indicated by the given
   `geometry_id` (if it exists).
   @throws  std::logic_error if the geometry id is invalid.  */
  const VisualMaterial& GetVisualMaterial(GeometryId geometry_id) const {
    DRAKE_DEMAND(state_ != nullptr);
    return state_->get_visual_material(geometry_id);
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

  /** Reports true if collision between the two indicated geometries has been
   filtered out.  */
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
