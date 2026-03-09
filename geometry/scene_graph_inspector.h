#pragma once

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/geometry_set.h"
#include "drake/geometry/geometry_version.h"
#include "drake/geometry/internal_frame.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/render/render_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

#ifndef DRAKE_DOXYGEN_CXX
template <typename T>
class GeometryState;
template <typename T>
class QueryObject;
template <typename T>
class SceneGraph;
#endif

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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SceneGraphInspector);

  // NOTE: An inspector should never be released into the wild without having
  // set the state variable. Every query should start by demanding that state_
  // is defined.

  //----------------------------------------------------------------------------

  /** @name              Scene-graph wide data  */
  //@{

  /** Reports the number of registered sources -- whether they have registered
   frames/geometries or not. This will always be at least 1; the SceneGraph
   itself counts as a source.  */
  int num_sources() const;

  /** Reports the _total_ number of frames registered in the scene graph
   (including the world frame).  */
  int num_frames() const;

  /** Returns all of the source ids in the scene graph. The order is guaranteed
   to be stable and consistent. The first element is the SceneGraph-internal
   source.  */
  std::vector<SourceId> GetAllSourceIds() const;

  /** Returns all of the frame ids in the scene graph. The order is guaranteed
   to be stable and consistent. The ids include the world frame's id.  */
  std::vector<FrameId> GetAllFrameIds() const;

  /** Reports the id for the world frame.  */
  FrameId world_frame_id() const {
    DRAKE_DEMAND(state_ != nullptr);
    return internal::InternalFrame::world_frame_id();
  }

  /** Reports the _total_ number of geometries in the scene graph.  */
  int num_geometries() const;

  /** Returns the set of all ids for registered geometries. The order is
   guaranteed to be stable and consistent.
   @param role  The requested role; if omitted, all geometries are returned.  */
  std::vector<GeometryId> GetAllGeometryIds(
      std::optional<Role> role = std::nullopt) const;

  /** Returns the geometry ids that are *implied* by the given GeometrySet and
   `role`. Remember that a GeometrySet can reference a FrameId in place of the
   ids of the individual geometries affixed to it. If a `role` is provided, only
   geometries with that role assigned will be returned, otherwise all geometries
   will be returned.

   @note Specifying `role` *can* have the effect of filtering geometries *from*
   the given `geometry_set` -- if a GeometryId is an explicit member of the
   geometry set but does not have the requested role, it will not be contained
   in the output.

   @param geometry_set    The encoding of the set of geometries.
   @param role            The requested role; if omitted, all geometries
                          registered to the frame are returned.
   @returns The requested unique geometry ids.

   @warning For C++ users: this returns an _unordered_ set, which means
   iteration order will be non-deterministic.  */
  std::unordered_set<GeometryId> GetGeometryIds(
      const GeometrySet& geometry_set,
      std::optional<Role> role = std::nullopt) const;

  /** Reports the _total_ number of geometries in the scene graph with the
   indicated role.  */
  int NumGeometriesWithRole(Role role) const;

  /** Reports the _total_ number of _deformable_ geometries in the scene graph
   with the indicated role.  */
  int NumDeformableGeometriesWithRole(Role role) const;

  /** Reports the total number of _dynamic_ geometries in the scene graph. This
   include all deformable geometries.  */
  int NumDynamicGeometries() const;

  /** Reports the total number of _anchored_ non-deformable geometries. This
   should provide the same answer as calling NumGeometriesForFrame() with the
   world frame id.  */
  int NumAnchoredGeometries() const;

  /** Returns all pairs of geometries that are candidates for collision (in no
   particular order). See CollisionFilterDeclaration and
   CollisionFilterManager::Apply() for information on why a particular pair may
   _not_ be a candidate.

   For candidate pair (A, B), the candidate is always guaranteed to be reported
   in a fixed order (i.e., always (A, B) and _never_ (B, A)). This is the same
   ordering as would be returned by, e.g.,
   QueryObject::ComputePointPairPenetration().  */
  std::set<std::pair<GeometryId, GeometryId>> GetCollisionCandidates() const;

  /** Returns the geometry version that can be used to detect changes
   to the geometry data associated with geometry roles. The reference returned
   should not be persisted. If it needs to be persisted, it should be copied. */
  const GeometryVersion& geometry_version() const;
  //@}

  /** @name                Sources and source-related data  */
  //@{

  /** Reports `true` if the given `source_id` maps to a registered source.  */
  bool SourceIsRegistered(SourceId source_id) const;

  /** Reports the name for the source with the given `source_id`.
   @throws std::exception if `source_id` does not map to a registered source. */
  const std::string& GetName(SourceId source_id) const;

  /** Reports the number of frames registered to the source with the given
   `source_id`.
   @throws std::exception if `source_id` does not map to a registered source.
   */
  int NumFramesForSource(SourceId source_id) const;

  /** Reports the ids of all of the frames registered to the source with the
   given source `source_id`.
   @throws std::exception if `source_id` does not map to a registered source.
   */
  const std::unordered_set<FrameId>& FramesForSource(SourceId source_id) const;

  //@}

  /** @name              Frames and their properties  */
  //@{

  /** Reports if the frame with given `frame_id` was registered to the source
   with the given `source_id`.
   @param frame_id      The query frame id.
   @param source_id     The query source id.
   @returns True if `frame_id` was registered on `source_id`.
   @throws std::exception  If `frame_id` does not map to a registered frame
                           or `source_id` does not map to a registered source.
   */
  bool BelongsToSource(FrameId frame_id, SourceId source_id) const;

  /** Reports the _name_ of the geometry source that registered the frame with
   the given `frame_id`.
   @throws std::exception  If `frame_id` does not map to a registered frame.
   */
  const std::string& GetOwningSourceName(FrameId frame_id) const;

  /** Reports the name of the frame with the given `frame_id`.
   @throws std::exception if `frame_id` does not map to a registered frame.
   */
  const std::string& GetName(FrameId frame_id) const;

  /** Reports the FrameId of the parent of `frame_id`.
   @throws std::exception if `frame_id` does not map to a registered frame.
   */
  FrameId GetParentFrame(FrameId frame_id) const;

  /** Reports the frame group for the frame with the given `frame_id`.
   @throws std::exception if `frame_id` does not map to a registered frame.
   @internal This value is equivalent to the old "model instance id".  */
  int GetFrameGroup(FrameId frame_id) const;

  /** Reports the number of geometries affixed to the frame with the given
   `frame_id`. This count does _not_ include geometries attached to frames that
   are descendants of this frame.
   @throws std::exception if `frame_id` does not map to a registered frame.
   */
  int NumGeometriesForFrame(FrameId frame_id) const;

  /** Reports the total number of geometries with the given `role` directly
   registered to the frame with the given `frame_id`. This count does _not_
   include geometries attached to frames that are descendants of this frame.
   @throws std::exception if `frame_id` does not map to a registered frame.
   */
  int NumGeometriesForFrameWithRole(FrameId frame_id, Role role) const;

  /** Returns geometry ids that have been registered directly to the frame
   indicated by `frame_id`. If a `role` is provided, only geometries with that
   role assigned will be returned, otherwise all geometries will be returned.
   The order of the ids is guaranteed to be stable and consistent.
   @param frame_id    The id of the frame in question.
   @param role  The requested role; if omitted, all geometries registered to the
                frame are returned.
   @returns The requested unique geometry ids in a consistent order.
   @throws std::exception if `id` does not map to a registered frame.  */
  std::vector<GeometryId> GetGeometries(
      FrameId frame_id, const std::optional<Role>& role = std::nullopt) const;

  /** Reports the id of the geometry with the given `name` and `role`, attached
   to the frame with the given frame `frame_id`.
   @param frame_id  The frame_id of the frame whose geometry is being
                    queried.
   @param role      The assigned role of the desired geometry.
   @param name      The name of the geometry to query for. The name will be
                    canonicalized prior to lookup (see
                    @ref canonicalized_geometry_names "GeometryInstance" for
                    details).
   @return The id of the queried geometry.
   @throws std::exception if no such geometry exists, multiple geometries have
                          that name, or if the `frame_id` does not map to a
                          registered frame.  */
  GeometryId GetGeometryIdByName(FrameId frame_id, Role role,
                                 const std::string& name) const;

  //@}

  /** @name           Geometries and their properties  */
  //@{

  /** Reports if the given geometry id was registered to the source with the
   given source id.
   @param geometry_id   The query geometry id.
   @param source_id     The query source id.
   @returns True if `geometry_id` was registered on `source_id`.
   @throws std::exception  If `geometry_id` does not map to a registered
                           geometry or `source_id` does not map to a
                           registered source.  */
  bool BelongsToSource(GeometryId geometry_id, SourceId source_id) const;

  /** Reports the _name_ of the geometry source that registered the geometry
   with the given `geometry_id`.
   @throws std::exception  If `geometry_id` does not map to a registered
   geometry. */
  const std::string& GetOwningSourceName(GeometryId geometry_id) const;

  /** Reports the id of the frame to which the given geometry with the given
   `geometry_id` is registered.
   @throws std::exception if `geometry_id` does not map to a registered
   geometry.  */
  FrameId GetFrameId(GeometryId geometry_id) const;

  /** Reports the stored, canonical name of the geometry with the given
   `geometry_id` (see  @ref canonicalized_geometry_names "GeometryInstance" for
   details).
   @throws std::exception if `geometry_id` does not map to a registered
   geometry.  */
  const std::string& GetName(GeometryId geometry_id) const;

  /** Returns the shape specified for the geometry with the given `geometry_id`.
   In order to extract the details of the shape, it should be passed through an
   implementation of a ShapeReifier.  */
  const Shape& GetShape(GeometryId geometry_id) const;

  /** Reports the pose of the geometry G with the given `geometry_id` in its
   registered frame F.
   @note For deformable geometries, this returns the pose of the reference mesh.
   @throws std::exception if `geometry_id` does not map to a registered
   geometry. */
  const math::RigidTransform<double>& GetPoseInFrame(
      GeometryId geometry_id) const;

  /** Returns the *mesh* used to represent this geometry in hydroelastic contact
   calculations, if it exists. Most primitives (sphere, cylinder, etc.) are
   actually represented by discrete approximations (i.e., the mesh). If there is
   no mesh, the returned variant will hold neither the
   TriangleSurfaceMesh<double> nor the VolumeMesh<double> alternatives. If
   either alternative is present, the pointer is guaranteed to be non-null.

   Just because hydroelastic properties have been assigned to a geometry does
   *not* mean there is necessarily a mesh associated with it. Some shape types
   (e.g., half space) have non-mesh representations.

   The result can be tested as follows:

   @code
     auto maybe_mesh = inspector.maybe_get_hydroelastic_mesh(id);

     // These two methods are equivalent, although testing index is more
     // brittle.
     const bool no_mesh1 = maybe_mesh.index() == 0;
     const bool no_mesh2 = std::holds_alternative<std::monostate>(maybe_mesh);
   @endcode

   @param geometry_id  The id of the geometry to query.
   @returns The associated mesh, if it exists. */
  std::variant<std::monostate, const TriangleSurfaceMesh<double>*,
               const VolumeMesh<double>*>
  maybe_get_hydroelastic_mesh(GeometryId geometry_id) const;

  /** Return a pointer to the const properties indicated by `role` of the
   geometry with the given `geometry_id`.
   @param geometry_id    The identifier for the queried geometry.
   @param role           The role whose properties are acquired.
   @return A pointer to the properties (or `nullptr` if there are no such
           properties).
   @throws std::exception if `geometry_id` does not map to a registered
           geometry.  */
  const GeometryProperties* GetProperties(GeometryId geometry_id,
                                          Role role) const;

  /** Returns a pointer to the const proximity properties of the geometry with
   the given `geometry_id`.
   @param geometry_id   The identifier for the queried geometry.
   @return A pointer to the properties (or `nullptr` if there are no such
           properties).
   @throws std::exception if `geometry_id` does not map to a registered
           geometry.  */
  const ProximityProperties* GetProximityProperties(
      GeometryId geometry_id) const;

  /** Returns a pointer to the const illustration properties of the geometry
   with the given `geometry_id`.
   @param geometry_id   The identifier for the queried geometry.
   @return A pointer to the properties (or `nullptr` if there are no such
           properties).
   @throws std::exception if `geometry_id` does not map to a registered
           geometry.  */
  const IllustrationProperties* GetIllustrationProperties(
      GeometryId geometry_id) const;

  /** Returns a pointer to the const perception properties of the geometry
   with the given `geometry_id`.
   @param geometry_id   The identifier for the queried geometry.
   @return A pointer to the properties (or `nullptr` if there are no such
           properties).
   @throws std::exception if `geometry_id` does not map to a registered
           geometry.  */
  const PerceptionProperties* GetPerceptionProperties(
      GeometryId geometry_id) const;

  /** Returns the reference mesh of the geometry with the given `geometry_id`,
   measured and expressed in the geometry's frame, G.
   @param geometry_id   The identifier for the queried geometry.
   @return A pointer to the reference mesh of the geometry if the geometry is
           deformable, or `nullptr` if the geometry is rigid.
   @note GetPoseInFrame() provides the transform necessary to measure and
         express the reference mesh's vertex positions in the geometry's
         registered frame F.
   @throws std::exception if `geometry_id` does not map to a registered
           geometry.  */
  const VolumeMesh<double>* GetReferenceMesh(GeometryId geometry_id) const;

  // TODO(xuchenhan-tri): This should cross reference the concept of driven
  // meshes when it is nicely written up somewhere (e.g., in the SceneGraph
  // documentation).
  /** Returns the render mesh representation of the driven meshes associated
   with the given `role` of the geometry with the given `geometry_id`.

   @param geometry_id      The identifier for the queried geometry.
   @param role             The role whose driven mesh representations are
                           acquired.
   @throws std::exception  if `geometry_id` does not map to a registered
                           deformable geometry with the given `role` or if
                           `role` is undefined.
   @experimental */
  const std::vector<internal::RenderMesh>& GetDrivenRenderMeshes(
      GeometryId geometry_id, Role role) const;

  /** Returns true if the geometry with the given `geometry_id` is deformable.
   @param geometry_id   The identifier for the queried geometry.
   @throws std::exception if `geometry_id` does not map to a registered
           geometry.  */
  bool IsDeformableGeometry(GeometryId geometry_id) const;

  /** Returns all geometry ids that correspond to deformable geometries. The
   order is guaranteed to be stable and consistent.  */
  std::vector<GeometryId> GetAllDeformableGeometryIds() const;

  /** Returns the convex hull (a polytope) associated with the given
   `geometry_id`, if it exists. Basic primitive shapes don't have convex hulls
   (including, arbitrarily, Box). But Mesh and Convex shapes do. Alternatively,
   if you already have a Mesh or Convex you can call Mesh::GetConvexHull() or
   Convex::GetConvexHull(), respectively. */
  const PolygonSurfaceMesh<double>* GetConvexHull(GeometryId geometry_id) const;

  /** Returns the oriented bounding box (OBB) associated with the given
   `geometry_id` in the geometry's frame, G. The OBB is defined in the
   geometry's canonical frame such that it tightly bounds the geometry. For
   primitive shapes, the OBB is computed analytically. For mesh-based shapes
   (Mesh and Convex), the OBB is computed using the mesh vertices.
   @param geometry_id   The identifier for the queried geometry.
   @note If geometry_id refers to a deformable geometry, the OBB is computed
         using the reference mesh. See QueryObject::ComputeObbInWorld() for
         computing the OBB of the deformed mesh in the world frame.
   @return The oriented bounding box (or `std::nullopt` if the geometry is an
           HalfSpace and doesn't have a bounding box).
   @throws std::exception if `geometry_id` does not map to a registered
           geometry.  */
  const std::optional<Obb>& GetObbInGeometryFrame(GeometryId geometry_id) const;

  /** Reports true if the two geometries with given ids `geometry_id1` and
   `geometry_id2`, define a collision pair that has been filtered out.
   @throws std::exception if either id does not map to a registered geometry
                          or if any of the geometries do not have a proximity
                          role.  */
  bool CollisionFiltered(GeometryId geometry_id1,
                         GeometryId geometry_id2) const;

  /** Obtains a new GeometryInstance that copies the geometry indicated by the
   given `geometry_id`.
   @return A new GeometryInstance that is ready to be added as a new geometry.
           All roles/properties will be copied, the shape will be cloned based
           off of the original, but the returned id() will completely unique.
   @throws std::exception if the `geometry_id` does not refer to a valid
   geometry.  */
  std::unique_ptr<GeometryInstance> CloneGeometryInstance(
      GeometryId geometry_id) const;
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
