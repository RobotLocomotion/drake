#pragma once

#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/geometry/collision_filter_manager.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/geometry_set.h"
#include "drake/geometry/geometry_version.h"
#include "drake/geometry/internal_frame.h"
#include "drake/geometry/internal_geometry.h"
#include "drake/geometry/kinematics_vector.h"
#include "drake/geometry/mesh_deformation_interpolator.h"
#include "drake/geometry/proximity_engine.h"
#include "drake/geometry/render/render_camera.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/scene_graph_config.h"
#include "drake/geometry/utilities.h"

namespace drake {
namespace geometry {

#ifndef DRAKE_DOXYGEN_CXX
namespace internal {

class GeometryVisualizationImpl;
using FrameNameSet = std::unordered_set<std::string>;

// Driven mesh data that depend on the configuration input values.
class DrivenMeshData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DrivenMeshData);

  DrivenMeshData();

  ~DrivenMeshData();

  // Updates the control mesh vertex positions for all driven meshes.
  // @param[in] q_WGs  q_WGs.at(id) contains control mesh vertex positions as a
  //                   flat Eigen vector.
  // @pre all driven meshes in `this` have their control mesh vertex positions
  //      specified in `q_WGs`.
  // @tparam_default_scalar
  template <typename T>
  void SetControlMeshPositions(
      const std::unordered_map<GeometryId, VectorX<T>>& q_WGs);

  const std::unordered_map<GeometryId, std::vector<DrivenTriangleMesh>>&
  driven_meshes() const {
    return driven_meshes_;
  }

  // Registers both driven meshes for the deformable geometry with the given id.
  // @pre driven_meshes is not empty.
  void SetMeshes(GeometryId id, std::vector<DrivenTriangleMesh> driven_meshes);

  // Removes the driven mesh associated with the given GeometryId.
  void Remove(GeometryId id) { driven_meshes_.erase(id); }

 private:
  std::unordered_map<GeometryId, std::vector<DrivenTriangleMesh>>
      driven_meshes_;
};

// TODO(xuchenhan-tri): These data should live in cache entries. Furthermore,
//  they should be broken up by source so that inputs can be pulled
//  independently. For now, they are big blobs of memory. Correct operation of
//  this ad hoc cache memory requires that we have a real cache entry that can
//  serve as a proxy and maintain appropriate dependencies on the pose &
//  configuration input ports. The Calc() method for the proxy entry then
//  handles calculation of the data here, followed by propagation to the
//  proximity and render engines. The values here should never be accessed
//  without a preceding call to the Eval() method of the proxy cache entry.
//
// Kinematics data that depend on time-dependent input values (e.g., current
// frame poses).
// @tparam_default_scalar
template <typename T>
struct KinematicsData {
  int num_frames() const { return X_WFs.size(); }

  int num_deformable_geometries() const { return q_WGs.size(); }

  // Map from a frame's index to the _current_ pose of the frame F it identifies
  // relative to its parent frame P, i.e., X_PF.
  std::vector<math::RigidTransform<T>> X_PFs;

  // The pose of every geometry relative to the _world_ frame (regardless of
  // roles) keyed by the corresponding geometry's id. After a complete state
  // update from input poses,
  //   X_WGs[i] == X_WFₙ · X_FₙFₙ₋₁ · ... · X_F₁F · G_i.X_FG()
  // Where F is the parent frame of geometry G_i, Fₖ₊₁ is the parent frame of
  // frame Fₖ, and the world frame W is the parent of frame Fₙ.
  // In other words, it is the full evaluation of the kinematic chain from the
  // geometry to the world frame.
  std::unordered_map<GeometryId, math::RigidTransform<T>> X_WGs;

  // The configuration of every deformable geometry relative to the _world_
  // frame (regardless of roles) keyed by the corresponding geometry's id.
  std::unordered_map<GeometryId, VectorX<T>> q_WGs;

  // The pose of each frame relative to the _world_ frame.
  // Furthermore, after a
  // complete state update from input poses,
  //   X_WFs[i] == X_WFₙ X_FₙFₙ₋₁ ... X_Fᵢ₊₂Fᵢ₊₁ X_PF[i]
  // Where Fᵢ₊₁ is the parent frame of frame i, Fₖ₊₁ is the parent frame of
  // frame Fₖ, and the world frame W is the parent of frame Fₙ.
  // In other words, it is the full evaluation of the kinematic chain from
  // frame i to the world frame.
  std::vector<math::RigidTransform<T>> X_WFs;

  // Mesh representations for deformable geometries that move passively with the
  // simulated control mesh, keyed by roles.
  std::map<Role, internal::DrivenMeshData> driven_mesh_data;
};

// A wrapper around a shared_ptr<T> where copying calls T::Clone() instead of
// bumping the ref_count with a new alias.
template <typename T>
class DeepCopySharedPtr {
 public:
  DeepCopySharedPtr() = default;
  explicit DeepCopySharedPtr(std::shared_ptr<T> value)
      : value_(std::move(value)) {}
  DeepCopySharedPtr(const DeepCopySharedPtr& other) {
    if (other.value_ != nullptr) {
      const T& other_value = *other.value_;
      // Use a static_cast<> to obtain a function pointer for Clone, in case it
      // is templated on the return type.
      auto Clone = static_cast<std::shared_ptr<T> (T::*)() const>(&T::Clone);
      value_ = (other_value.*Clone)();
    }
  }
  DeepCopySharedPtr& operator=(const DeepCopySharedPtr& other) {
    DeepCopySharedPtr other_copy(other);
    *this = std::move(other_copy);
    return *this;
  }
  DeepCopySharedPtr(DeepCopySharedPtr&& other) noexcept
      : value_(std::move(other.value_)) {}
  DeepCopySharedPtr& operator=(DeepCopySharedPtr&& other) noexcept {
    value_ = std::move(other.value_);
    return *this;
  }
  const T* get() const { return value_.get(); }
  T* get_mutable() const { return value_.get(); }

 private:
  std::shared_ptr<T> value_;
};

}  // namespace internal
#endif

class GeometryFrame;

class GeometryInstance;

template <typename T>
class SceneGraph;

/** @name Structures for maintaining the entity relationships  */
//@{

/** Collection of unique frame ids.  */
using FrameIdSet = std::unordered_set<FrameId>;
/** Collection of unique geometry ids.  */
using GeometryIdSet = std::unordered_set<GeometryId>;

//@}

// TODO(SeanCurtis-TRI): Move GeometryState into `internal` namespace (and then
//  I can kill the `@note` in the class documentation).

/**
 The context-dependent state of SceneGraph. This serves as an AbstractValue
 in the context. SceneGraph's time-dependent state includes more than just
 values; objects can be added to or removed from the world over time. Therefore,
 SceneGraph's context-dependent state includes values (the poses/configurations)
 and structure (the topology of the world).

 @note This is intended as an internal class only.

 @tparam_default_scalar
*/
template <typename T>
class GeometryState {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryState);

  /** An object that represents the range of FrameId values in the state. It
   is used in range-based for loops to iterate through registered frames.  */
  using FrameIdRange = internal::MapKeyRange<FrameId, internal::InternalFrame>;

  /** Default constructor.  */
  GeometryState();

  /** Allow assignment from a %GeometryState<double> to a %GeometryState<T>.
   @internal The SFINAE is required to prevent collision with the default
   defined assignment operator where T is double.  */
  template <class T1 = T>
  typename std::enable_if_t<!std::is_same_v<T1, double>, GeometryState<T>&>
  operator=(const GeometryState<double>& other) {
    // This reuses the private copy _conversion_ constructor. It is _not_
    // intended to be performant -- but no one should be copying SceneGraph's
    // state frequently anyways.
    GeometryState<T> temp{other};
    return *this = temp;
  }

  /** @name        Scene-graph wide introspection  */
  //@{

  /** Implementation of SceneGraphInspector::num_sources().  */
  int get_num_sources() const {
    return static_cast<int>(source_frame_id_map_.size());
  }

  /** Implementation of SceneGraphInspector::num_frames().  */
  int get_num_frames() const { return static_cast<int>(frames_.size()); }

  /** Implementation of SceneGraphInspector::GetAllFrameIds().  */
  FrameIdRange get_frame_ids() const { return FrameIdRange(&frames_); }

  /** Implementation of SceneGraphInspector::num_geometries().  */
  int get_num_geometries() const {
    return static_cast<int>(geometries_.size());
  }

  /** Implementation of SceneGraphInspector::GetAllGeometryIds().  */
  std::vector<GeometryId> GetAllGeometryIds(std::optional<Role> role) const;

  /** Implementation of SceneGraphInspector::GetGeometryIds().  */
  std::unordered_set<GeometryId> GetGeometryIds(const GeometrySet& geometry_set,
                                                std::optional<Role> role) const;

  /** Implementation of SceneGraphInspector::NumGeometriesWithRole().  */
  int NumGeometriesWithRole(Role role) const;

  /** Implementation of
   SceneGraphInspector::NumDeformableGeometriesWithRole().  */
  int NumDeformableGeometriesWithRole(Role role) const;

  /** Implementation of SceneGraphInspector::NumDynamicGeometries().  */
  int NumDynamicGeometries() const;

  /** Returns the total number of registered dynamic non-deformable geometries.
   */
  int NumDynamicNonDeformableGeometries() const;

  /** Returns the total number of registered deformable geometries. All
   deformable geometries are dynamic and _not_ anchored.  */
  int NumDeformableGeometries() const;

  /** Implementation of SceneGraphInspector::NumAnchoredGeometries().  */
  int NumAnchoredGeometries() const;

  /** Implementation of SceneGraphInspector::GetCollisionCandidates().  */
  std::set<std::pair<GeometryId, GeometryId>> GetCollisionCandidates() const;

  /** Implementation of SceneGraphInspector::GetGeometryVersion().  */
  const GeometryVersion& geometry_version() const { return geometry_version_; }

  //@}

  /** @name          Sources and source-related data  */
  //@{

  /** Returns all of the source ids in the scene graph. The order is guaranteed
   to be stable and consistent. The first element is the SceneGraph-internal
   source.  */
  std::vector<SourceId> GetAllSourceIds() const;

  /** Implementation of SceneGraphInspector::SourceIsRegistered().  */
  bool SourceIsRegistered(SourceId source_id) const;

  /** Implementation of SceneGraphInspector::GetName().  */
  const std::string& GetName(SourceId id) const;

  /** Implementation of SceneGraphInspector::NumFramesForSource().  */
  int NumFramesForSource(SourceId source_id) const;

  /** Implementation of SceneGraphInspector::FramesForSource().  */
  const FrameIdSet& FramesForSource(SourceId source_id) const;

  //@}

  /** @name              Frames and their properties  */
  //@{

  /** Implementation of
   SceneGraphInspector::BelongsToSource(FrameId, SourceId) const.  */
  bool BelongsToSource(FrameId frame_id, SourceId source_id) const;

  /** Implementation of
   SceneGraphInspector::GetOwningSourceName(FrameId) const.  */
  const std::string& GetOwningSourceName(FrameId id) const;

  /** Implementation of SceneGraphInspector::GetName(FrameId) const.  */
  const std::string& GetName(FrameId frame_id) const;

  /** Implementation of SceneGraphInspector::GetParentFrame(FrameId) const.  */
  FrameId GetParentFrame(FrameId frame_id) const;

  /** Implementation of SceneGraphInspector::GetFrameGroup().  */
  int GetFrameGroup(FrameId frame_id) const;

  /** Implementation of SceneGraphInspector::NumGeometriesForFrame().  */
  int NumGeometriesForFrame(FrameId frame_id) const;

  /** Implementation of SceneGraphInspector::NumGeometriesForFrameWithRole().
   */
  int NumGeometriesForFrameWithRole(FrameId frame_id, Role role) const;

  /** Implementation of SceneGraphInspector::GetGeometries.  */
  std::vector<GeometryId> GetGeometries(FrameId frame_id,
                                        std::optional<Role> role) const;

  // TODO(SeanCurtis-TRI): Redundant w.r.t. NumGeometriesForFrameWithRole().
  /** Reports the number of child geometries for this frame that have the
   indicated role assigned. This only includes the immediate child geometries of
   *this* frame, and not those of child frames.
   @throws std::exception if the `frame_id` does not map to a valid frame.  */
  int NumGeometriesWithRole(FrameId frame_id, Role role) const;

  /** Implementation of SceneGraphInspector::GetGeometryIdByName().  */
  GeometryId GetGeometryIdByName(FrameId frame_id, Role role,
                                 const std::string& name) const;

  //@}

  /** @name           Geometries and their properties  */
  //@{

  /** Implementation of
   SceneGraphInspector::BelongsToSource(GeometryId, SourceId) const.  */
  bool BelongsToSource(GeometryId geometry_id, SourceId source_id) const;

  /** Implementation of
   SceneGraphInspector::GetOwningSourceName(GeometryId) const.  */
  const std::string& GetOwningSourceName(GeometryId id) const;

  /** Implementation of SceneGraphInspector::GetFrameId().  */
  FrameId GetFrameId(GeometryId geometry_id) const;

  /** Implementation of SceneGraphInspector::GetName(GeometryId) const.  */
  const std::string& GetName(GeometryId geometry_id) const;

  /** Support for SceneGraphInspector::Reify().  */
  const Shape& GetShape(GeometryId id) const;

  /** Implementation of SceneGraphInspector::X_FG().  */
  const math::RigidTransform<double>& GetPoseInFrame(
      GeometryId geometry_id) const;

  /** Implementation of
   SceneGraphInspector::maybe_get_hydroelastic_mesh().  */
  std::variant<std::monostate, const TriangleSurfaceMesh<double>*,
               const VolumeMesh<double>*>
  maybe_get_hydroelastic_mesh(GeometryId geometry_id) const;

  /** Implementation of SceneGraphInspector::GetProximityProperties().  */
  const ProximityProperties* GetProximityProperties(GeometryId id) const;

  /** Implementation of SceneGraphInspector::GetIllustrationProperties().  */
  const IllustrationProperties* GetIllustrationProperties(GeometryId id) const;

  /** Implementation of SceneGraphInspector::GetPerceptionProperties().  */
  const PerceptionProperties* GetPerceptionProperties(GeometryId id) const;

  /** Implementation of SceneGraphInspector::GetReferenceMesh().  */
  const VolumeMesh<double>* GetReferenceMesh(GeometryId id) const;

  /** Implementation of
   SceneGraphInspector::GetDrivenRenderMeshes().  */
  const std::vector<internal::RenderMesh>& GetDrivenRenderMeshes(
      GeometryId id, Role role) const;

  /** Implementation of SceneGraphInspector::IsDeformableGeometry(). */
  bool IsDeformableGeometry(GeometryId id) const;

  /** Implementation of SceneGraphInspector::GetAllDeformableGeometryIds(). */
  std::vector<GeometryId> GetAllDeformableGeometryIds() const;

  /** Implementation of SceneGraphInspector::GetConvexHull(). */
  const PolygonSurfaceMesh<double>* GetConvexHull(GeometryId id) const;

  /** Implementation of SceneGraphInspector::GetObbInGeometryFrame(). */
  const std::optional<Obb>& GetObbInGeometryFrame(GeometryId id) const;

  /** Implementation of SceneGraphInspector::CollisionFiltered().  */
  bool CollisionFiltered(GeometryId id1, GeometryId id2) const;

  //@}

  /** @name                Configuration-dependent queries

   These quantities all depend on the most recent pose values assigned to the
   registered frames and configuration values assigned to deformable geometries.
  */
  //@{

  /** Implementation of QueryObject::GetPoseInWorld(FrameId).  */
  const math::RigidTransform<T>& get_pose_in_world(FrameId frame_id) const;

  /** Implementation of QueryObject::GetPoseInWorld(GeometryId).  */
  const math::RigidTransform<T>& get_pose_in_world(
      GeometryId geometry_id) const;

  /** Implementation of QueryObject::ComputeAabbInWorld(GeometryId).  */
  std::optional<Aabb> ComputeAabbInWorld(GeometryId geometry_id) const;

  /** Implementation of QueryObject::ComputeObbInWorld(GeometryId).  */
  std::optional<Obb> ComputeObbInWorld(GeometryId geometry_id) const;

  /** Implementation of QueryObject::GetPoseInParent().  */
  const math::RigidTransform<T>& get_pose_in_parent(FrameId frame_id) const;

  /** Implementation of QueryObject::GetConfigurationsInWorld().  */
  const VectorX<T>& get_configurations_in_world(GeometryId geometry_id) const;

  /** Implementation of QueryObject::GetDrivenMeshConfigurationsInWorld().  */
  std::vector<VectorX<T>> GetDrivenMeshConfigurationsInWorld(
      GeometryId geometry_id, Role role) const;
  //@}

  /** @name        State management

   The methods that modify the state including: adding/removing entities from
   the state, modifying values in the state, etc.  */
  //@{

  /** Implementation of SceneGraph::RegisterSource().
   The default logic is to define name as "Source_##" where the number is the
   value of the returned SourceId.  */
  SourceId RegisterNewSource(const std::string& name = "");

  /** Implementation of SceneGraph::RegisterFrame().  */
  FrameId RegisterFrame(SourceId source_id, const GeometryFrame& frame);

  /** Implementation of SceneGraph::RenameFrame().  */
  void RenameFrame(FrameId frame_id, const std::string& name);

  /** Implementation of
   @ref SceneGraph::RegisterFrame(SourceId,FrameId,const GeometryFrame&)
   "SceneGraph::RegisterFrame()" with parent FrameId.  */
  FrameId RegisterFrame(SourceId source_id, FrameId parent_id,
                        const GeometryFrame& frame);

  /** Implementation of
   @ref SceneGraph::RegisterGeometry(SourceId,FrameId,
   std::unique_ptr<GeometryInstance>) "SceneGraph::RegisterGeometry()" with
   parent FrameId.  */
  GeometryId RegisterGeometry(SourceId source_id, FrameId frame_id,
                              std::unique_ptr<GeometryInstance> geometry);

  // TODO(SeanCurtis-TRI): Consider deprecating this; it's now strictly a
  // wrapper for the more general `RegisterGeometry()`.
  /** Implementation of SceneGraph::RegisterAnchoredGeometry().  */
  GeometryId RegisterAnchoredGeometry(
      SourceId source_id, std::unique_ptr<GeometryInstance> geometry);

  /** Implementation of @ref SceneGraph::RegisterDeformableGeometry() */
  GeometryId RegisterDeformableGeometry(
      SourceId source_id, FrameId frame_id,
      std::unique_ptr<GeometryInstance> geometry, double resolution_hint);

  /** Implementation of SceneGraph::RenameGeometry().  */
  void RenameGeometry(GeometryId geometry_id, const std::string& name);

  /** Implementation of SceneGraph::ChangeShape().  */
  void ChangeShape(SourceId source_id, GeometryId geometry_id,
                   const Shape& shape,
                   std::optional<math::RigidTransform<double>> X_FG);

  /** Implementation of SceneGraph::RemoveGeometry().  */
  void RemoveGeometry(SourceId source_id, GeometryId geometry_id);

  /** Reports whether the canonicalized version of the given candidate geometry
   name is considered valid. This tests the requirements described in the
   documentation of @ref canonicalized_geometry_names "GeometryInstance". When
   adding a geometry to a frame, if there is doubt if a proposed name is valid,
   the name can be tested prior to registering the geometry.
   @param frame_id        The id of the frame to which the geometry would be
                          assigned.
   @param role            The role for the candidate name.
   @param candidate_name  The name to validate.
   @return true if the `candidate_name` can be given to a `GeometryInstance`
   assigned to the indicated frame with the indicated role.
   @throws std::exception if `frame_id` does not refer to a valid frame.  */
  bool IsValidGeometryName(FrameId frame_id, Role role,
                           const std::string& candidate_name) const;

  /** Implementation of
   @ref SceneGraph::AssignRole(SourceId, GeometryId, ProximityProperties)
   "SceneGraph::AssignRole()".  */
  void AssignRole(SourceId source_id, GeometryId geometry_id,
                  ProximityProperties properties,
                  RoleAssign assign = RoleAssign::kNew);

  /** Implementation of
   @ref SceneGraph::AssignRole(SourceId, GeometryId, PerceptionProperties)
   "SceneGraph::AssignRole()".  */
  void AssignRole(SourceId source_id, GeometryId geometry_id,
                  PerceptionProperties properties,
                  RoleAssign assign = RoleAssign::kNew);

  /** Implementation of
   @ref SceneGraph::AssignRole(SourceId, GeometryId, IllustrationProperties)
   "SceneGraph::AssignRole()".  */
  void AssignRole(SourceId source_id, GeometryId geometry_id,
                  IllustrationProperties properties,
                  RoleAssign assign = RoleAssign::kNew);

  /** Implementation of
   @ref SceneGraph::RemoveRole(SourceId, FrameId, Role)
   "SceneGraph::RemoveRole()".  */
  int RemoveRole(SourceId source_id, FrameId frame_id, Role role);

  /** Implementation of
   @ref SceneGraph::RemoveRole(SourceId, GeometryId, Role)
   "SceneGraph::RemoveRole()".  */
  int RemoveRole(SourceId source_id, GeometryId geometry_id, Role role);

  // TODO(SeanCurtis-TRI): These two "RemoveFromRenderer()" methods are not
  // currently exposed in the SceneGraph API. They've been left here so that
  // they (and their unit tests) don't have to be recreated. I need to
  // determine definitively if these methods *should* exist (in which case I
  // put them in the SceneGraph API or not, in which case I can remove them
  // entirely).

  /** For every geometry directly registered to the frame with the given
   `frame_id`, if it has been added to the renderer with the given
   `renderer_name` it is removed from that renderer.
   @return The number of geometries affected by the removal.
   @throws std::exception if a) `source_id` does not map to a registered
                          source,
                          b) `frame_id` does not map to a registered frame,
                          c) `frame_id` does not belong to `source_id` (unless
                          `frame_id` is the world frame id), or
                          d) the context has already been allocated.  */
  int RemoveFromRenderer(const std::string& renderer_name, SourceId source_id,
                         FrameId frame_id);

  /** Removes the geometry with the given `geometry_id` from the renderer with
   the given `renderer_name`, _if_ it has previously been added.
   @return The number of geometries affected by the removal (0 or 1).
   @throws std::exception if a) `source_id` does not map to a registered
                          source,
                          b) `geometry_id` does not map to a registered
                          geometry,
                          c) `geometry_id` does not belong to `source_id`, or
                          d) the context has already been allocated.  */
  int RemoveFromRenderer(const std::string& renderer_name, SourceId source_id,
                         GeometryId geometry_id);
  //@}

  //----------------------------------------------------------------------------
  /** @name                Collision Queries
   See @ref collision_queries "Collision Queries" for more details.  */
  //@{

  /** Implementation of QueryObject::ComputePointPairPenetration().  */
  std::vector<PenetrationAsPointPair<T>> ComputePointPairPenetration() const {
    return geometry_engine_->ComputePointPairPenetration(
        kinematics_data_.X_WGs);
  }

  /** Implementation of QueryObject::ComputeContactSurfaces().  */
  template <typename T1 = T>
  typename std::enable_if_t<scalar_predicate<T1>::is_bool,
                            std::vector<ContactSurface<T>>>
  ComputeContactSurfaces(
      HydroelasticContactRepresentation representation) const {
    return geometry_engine_->ComputeContactSurfaces(representation,
                                                    kinematics_data_.X_WGs);
  }

  /** Implementation of QueryObject::ComputeContactSurfacesWithFallback().  */
  template <typename T1 = T>
  typename std::enable_if_t<scalar_predicate<T1>::is_bool, void>
  ComputeContactSurfacesWithFallback(
      HydroelasticContactRepresentation representation,
      std::vector<ContactSurface<T>>* surfaces,
      std::vector<PenetrationAsPointPair<T>>* point_pairs) const {
    DRAKE_DEMAND(surfaces != nullptr);
    DRAKE_DEMAND(point_pairs != nullptr);
    return geometry_engine_->ComputeContactSurfacesWithFallback(
        representation, kinematics_data_.X_WGs, surfaces, point_pairs);
  }

  /** Implementation of QueryObject::ComputeDeformableContact().  */
  template <typename T1 = T>
  typename std::enable_if_t<std::is_same_v<T1, double>, void>
  ComputeDeformableContact(
      internal::DeformableContact<T>* deformable_contact) const {
    return geometry_engine_->ComputeDeformableContact(deformable_contact);
  }

  /** Implementation of QueryObject::FindCollisionCandidates().  */
  std::vector<SortedPair<GeometryId>> FindCollisionCandidates() const {
    return geometry_engine_->FindCollisionCandidates();
  }

  /** Implementation of QueryObject::HasCollisions().  */
  bool HasCollisions() const { return geometry_engine_->HasCollisions(); }

  //@}

  /** @name        Collision filtering    */

  /** Implementation of SceneGraph::collision_filter_manager(). */
  CollisionFilterManager collision_filter_manager() {
    geometry_version_.modify_proximity();
    return CollisionFilterManager(
        &geometry_engine_->collision_filter(),
        [this](const GeometrySet& set, CollisionFilterScope scope) {
          return this->CollectIds(set, Role::kProximity, scope);
        });
  }

  //---------------------------------------------------------------------------
  /** @name                Signed Distance Queries
   See @ref signed_distance_query "Signed Distance Queries" for more details.
   */

  //@{

  /** Implementation of
   QueryObject::ComputeSignedDistancePairwiseClosestPoints().  */
  std::vector<SignedDistancePair<T>> ComputeSignedDistancePairwiseClosestPoints(
      double max_distance) const {
    return geometry_engine_->ComputeSignedDistancePairwiseClosestPoints(
        kinematics_data_.X_WGs, max_distance);
  }

  /** Implementation of
   QueryObject::ComputeSignedDistancePairClosestPoints().  */
  SignedDistancePair<T> ComputeSignedDistancePairClosestPoints(
      GeometryId id_A, GeometryId id_B) const;

  /** Implementation of QueryObject::ComputeSignedDistanceToPoint().  */
  std::vector<SignedDistanceToPoint<T>> ComputeSignedDistanceToPoint(
      const Vector3<T>& p_WQ, double threshold) const {
    return geometry_engine_->ComputeSignedDistanceToPoint(
        p_WQ, kinematics_data_.X_WGs, threshold);
  }

  /** Implementation of QueryObject::ComputeSignedDistanceGeometryToPoint().  */
  std::vector<SignedDistanceToPoint<T>> ComputeSignedDistanceGeometryToPoint(
      const Vector3<T>& p_WQ, const GeometrySet& geometries) const;

  //@}

  //---------------------------------------------------------------------------
  /** @name                Render Queries
   See @ref render_queries "Render Queries" for more details.  */
  //@{

  /** Implementation of SceneGraph::AddRenderer().  */
  void AddRenderer(std::string name,
                   std::shared_ptr<render::RenderEngine> renderer);

  /** Implementation of SceneGraph::RemoveRenderer(). */
  void RemoveRenderer(const std::string& name);

  /** Implementation of SceneGraph::HasRenderer().  */
  bool HasRenderer(const std::string& name) const {
    return render_engines_.contains(name);
  }

  /** Implementation of QueryObject::GetRenderEngineByName.  */
  const render::RenderEngine* GetRenderEngineByName(
      const std::string& name) const {
    if (render_engines_.contains(name)) {
      return render_engines_.at(name).get();
    }
    return nullptr;
  }

  /** Implementation of SceneGraph::RendererCount().  */
  int RendererCount() const { return static_cast<int>(render_engines_.size()); }

  /** Implementation of SceneGraph::RegisteredRendererNames().  */
  std::vector<std::string> RegisteredRendererNames() const;

  /** Implementation of QueryObject::RenderColorImage().
   @pre All poses have already been updated.  */
  void RenderColorImage(const render::ColorRenderCamera& camera,
                        FrameId parent_frame, const math::RigidTransformd& X_PC,
                        systems::sensors::ImageRgba8U* color_image_out) const;

  /** Implementation of QueryObject::RenderDepthImage().
   @pre All poses have already been updated.  */
  void RenderDepthImage(const render::DepthRenderCamera& camera,
                        FrameId parent_frame, const math::RigidTransformd& X_PC,
                        systems::sensors::ImageDepth32F* depth_image_out) const;

  /** Implementation of QueryObject::RenderLabelImage().
   @pre All poses have already been updated.  */
  void RenderLabelImage(const render::ColorRenderCamera& camera,
                        FrameId parent_frame, const math::RigidTransformd& X_PC,
                        systems::sensors::ImageLabel16I* label_image_out) const;

  //@}

  /** @name Scalar conversion */
  //@{

  /** Returns a deep copy of this state using the AutoDiffXd scalar with all
   scalar values initialized from the current values. If this is invoked on an
   instance already instantiated on AutoDiffXd, it is equivalent to cloning
   the instance.  */
  std::unique_ptr<GeometryState<AutoDiffXd>> ToAutoDiffXd() const;

  //@}

  /** @name Default proximity properties */
  //@{

  /** Applies the default proximity values in `defaults` to the proximity
   properties of every currently registered geometry that has a proximity
   role. For detailed semantics, see the 2-argument overload.
  */
  void ApplyProximityDefaults(const DefaultProximityProperties& defaults);

  /** Applies the default proximity values in `defaults` to the proximity
   properties of the geometry with the given geometry_id as appropriate. For a
   given property, no value will be written if (a) `defaults` contains no value
   for it, or (b) a value has previously been set for that property.

   @pre geometry_id indicates a geometry with an assigned proximity role.
  */
  void ApplyProximityDefaults(const DefaultProximityProperties& defaults,
                              GeometryId geometry_id);

  //@}

 private:
  // GeometryState of one scalar type is friends with all other scalar types.
  template <typename>
  friend class GeometryState;

  // Scalar-converting copy constructor.
  template <typename U>
  explicit GeometryState(const GeometryState<U>& source);

  // Allow SceneGraph unique access to the state members to perform queries.
  friend class SceneGraph<T>;

  // Friend declaration so that the internals of the state can be confirmed in
  // unit tests.
  template <class U>
  friend class GeometryStateTester;

  // Defines the full set of geometry ids implied by the contents of the given
  // GeometrySet (which may be defined in terms of geometry ids *and* frame
  // ids). If GeometrySet only has GeometryIds, it is essentially a copy. Ids in
  // the set that can't be mapped to known geometries or frames will cause an
  // exception to be thrown. The ids can be optionally filtered based on role.
  // If `role` is nullopt, no filtering takes place. Otherwise, just those
  // geometries with the given role will be returned. Deformable geometries are
  // excluded from the results if `scope == kOmitDeformable`.
  std::unordered_set<GeometryId> CollectIds(const GeometrySet& geometry_set,
                                            std::optional<Role> role,
                                            CollisionFilterScope scope) const;

  // Sets the kinematic poses for the frames indicated by the given ids.
  // @param[in]  poses           The frame id and pose values.
  // @param[out] kinematics_data The updated kinematics data that contains the
  //                             frame poses.
  // @pre source_id is a registered source.
  // @throws std::exception  If the ids are invalid as defined by
  // ValidateFrameIds().
  void SetFramePoses(SourceId source_id, const FramePoseVector<T>& poses,
                     internal::KinematicsData<T>* kinematics_data) const;

  // Sets the kinematic configurations for the deformable geometries associated
  // with the given source.
  // @param[in]  configurations  The geometry id and configuration values.
  // @param[out] kinematics_data The updated kinematics data that contains the
  //                             deformable geometry configurations.
  // @pre source_id is a registered source.
  void SetGeometryConfiguration(
      SourceId source_id, const GeometryConfigurationVector<T>& configurations,
      internal::KinematicsData<T>* kinematics_data) const;

  // Confirms that the set of ids provided include _all_ of the frames
  // registered to the set's source id and that no extra frames are included.
  // @param values The kinematics values (ids and values) to validate.
  // @pre source_id is a registered source.
  // @throws std::exception if the set is inconsistent with known topology.
  template <typename ValueType>
  void ValidateFrameIds(
      SourceId source_id,
      const KinematicsVector<FrameId, ValueType>& values) const;

  // Helper for RegisterGeometry() and RegisterDeformableGeometry() that
  // validates the the source and frame ids (that they are registered) and
  // geometry id (that there are no duplicates), and configures the topology of
  // the geometry "tree".
  void ValidateRegistrationAndSetTopology(SourceId source_id, FrameId frame_id,
                                          GeometryId geometry_id);

  // Method that updates the proximity engine and the render engines with the
  // up-to-date _pose_ data in `kinematics_data`.
  void FinalizePoseUpdate(
      const internal::KinematicsData<T>& kinematics_data,
      internal::ProximityEngine<T>* proximity_engine,
      std::vector<render::RenderEngine*> render_engines) const;

  // Method that updates the proximity engine and the render engines with the
  // up-to-date configuration data in `kinematics_data`.
  void FinalizeConfigurationUpdate(
      const internal::KinematicsData<T>& kinematics_data,
      internal::ProximityEngine<T>* proximity_engine,
      std::vector<render::RenderEngine*> render_engines) const;

  // Gets the source id for the given frame id. Throws std::exception if the
  // frame belongs to no registered source.
  SourceId get_source_id(FrameId frame_id) const;

  // Gets the source id for the given frame id. Throws std::exception if the
  // geometry belongs to no registered source.
  SourceId get_source_id(GeometryId frame_id) const;

  // Recursively updates the frame and geometry _pose_ information for the tree
  // rooted at the given frame, whose parent's pose in the world frame is given
  // as `X_WP`.
  void UpdatePosesRecursively(
      const internal::InternalFrame& frame, const math::RigidTransform<T>& X_WP,
      const FramePoseVector<T>& poses,
      internal::KinematicsData<T>* kinematics_data) const;

  // Reports true if the given id refers to a _dynamic_ geometry. Assumes the
  // precondition that id refers to a valid geometry in the state.
  bool is_dynamic(GeometryId id) const {
    return geometries_.at(id).is_dynamic();
  }

  // Convenience function for accessing geometry whether dynamic or anchored.
  const internal::InternalGeometry* GetGeometry(GeometryId id) const;

  // Convenience function for accessing geometry whether dynamic or anchored.
  internal::InternalGeometry* GetMutableGeometry(GeometryId id);

  // Reports if the given name is unique in the given frame and role.
  bool NameIsUnique(FrameId id, Role role, const std::string& name) const;

  // If the given name exists in the geometries affixed to the indicated frame
  // for the given role, throws an exception.
  void ThrowIfNameExistsInRole(FrameId id, Role role,
                               const std::string& name) const;

  // Propagates all roles defined in geometry instance to `this` geometry state.
  void AssignAllDefinedRoles(SourceId source_id,
                             std::unique_ptr<GeometryInstance> geometry);

  // Confirms that the given role assignment is valid and return the geometry
  // if valid. Throws if not.
  internal::InternalGeometry& ValidateRoleAssign(SourceId source_id,
                                                 GeometryId geometry_id,
                                                 Role role, RoleAssign assign);

  // Attempts to remove the indicated `role` from the indicated geometry.
  // Returns true if removed (false doesn't imply "failure", just nothing to
  // remove). This does no checking on ownership.
  // @pre geometry_id maps to a registered geometry.
  bool RemoveRoleUnchecked(GeometryId geometry_id, Role role);

  // Handles adding the given geometry to the proximity engine. The only
  // GeometryState-level data structure modified is the proximity version. All
  // other changes to GeometryState data must happen elsewhere.
  void AddToProximityEngineUnchecked(
      const internal::InternalGeometry& geometry);

  // Handles removing the given geometry from the proximity engine. The only
  // GeometryState-level data structure modified is the proximity version. All
  // other changes to GeometryState data must happen elsewhere.
  void RemoveFromProximityEngineUnchecked(
      const internal::InternalGeometry& geometry);

  // Attempts to remove the geometry with the given `id` from the named
  // renderer. Returns true if removed (false doesn't imply "failure", just
  // nothing to remove). This does no checking on ownership.
  // @pre geometry_id maps to a registered geometry.
  bool RemoveFromRendererUnchecked(const std::string& renderer_name,
                                   GeometryId id);

  // Attempts to add the given `geometry` to all compatible render engines.
  // This helper function changes, at most, the state of registered render
  // engines and this geometry state's perception version.  The caller is
  // responsible for making sure that all other state associated with the
  // perception role of a geometry is properly maintained for this geometry
  // state to remain consistent.
  // @returns `true` if the geometry was added to *any* renderer.
  bool AddToCompatibleRenderersUnchecked(
      const internal::InternalGeometry& geometry);

  // Attempts to add the given `geometry` to all `candidate_renderers`
  // @pre `geometry` is not deformable.
  // @returns `true` if the geometry was added to *any* renderer.
  bool AddRigidToCompatibleRenderersUnchecked(
      const internal::InternalGeometry& geometry,
      std::vector<render::RenderEngine*>* candidate_renderers);

  // Attempts to add the given deformable `geometry` to all
  // `candidate_renderers`.
  // @pre `geometry` is deformable.
  // @returns `true` if the geometry was added to *any* renderer.
  bool AddDeformableToCompatibleRenderersUnchecked(
      const internal::InternalGeometry& geometry,
      std::vector<render::RenderEngine*>* candidate_renderers);

  // Adds the driven mesh for the deformable geometry with the given
  // `geometry_id` for the given role. The nature of the driven mesh depends on
  // the role and the geometry's properties for that role. Generally, it is
  // simply the surface triangle mesh of the control volume mesh. However,
  // the perception properties can define the ("deformable", "embedded_mesh")
  // property containing a valid path to a surface mesh. In that case, that
  // user-prescribed mesh will be the driven mesh.
  // @throws std::exception if role is kPerception and the
  // ("deformable", "embedded_mesh") property is present with a non-empty string
  // value and that value does not contain a path to a surface mesh that's
  // completely contained within the control volume mesh.
  // @pre The geometry associated with `geometry_id` is a deformable geometry
  // registered with `this` GeometryState with the given `role`.
  // @pre `role` is not Role::kUnassigned.
  // @note if driven meshes are already registered for the given `geometry_id`,
  // they will be replaced with the new driven mesh.
  void RegisterDrivenMesh(GeometryId geometry_id, Role role);

  // Attempts to remove the geometry with the given id from *all* render
  // engines. The only GeometryState-level data structure modified is the
  // perception version. All other changes to GeometryState data must happen
  // elsewhere.
  void RemoveFromAllRenderersUnchecked(GeometryId id);

  bool RemoveProximityRole(GeometryId geometry_id);
  bool RemoveIllustrationRole(GeometryId geometry_id);
  bool RemovePerceptionRole(GeometryId geometry_id);

  // When performing an operation on a frame, the caller provides its source id
  // and the id of the frame it owns as the operand. Generally, the validation
  // of the operation depends on two things:
  //  1. The source id must be valid.
  //  2. The source id must own the frame indicated.
  // However, there is an exception. Callers can operate on the *world frame*
  // (which merely affects the *geometries* that source owns that have been
  // attached to the world frame). But external geometry sources *can't* own the
  // world frame; it is owned by GeometryState/SceneGraph. Therefore, the
  // *requesting* source id may not be the same as the *owning* source id in
  // this one case.
  // This function handles the special case. It confirms all proper ownership
  // and, assuming the ids and relationships are valid, returns the frame
  // requested.
  const internal::InternalFrame& ValidateAndGetFrame(SourceId source_id,
                                                     FrameId frame_id) const;

  // Retrieves the requested renderer (if supported), throwing otherwise.
  const render::RenderEngine& GetRenderEngineOrThrow(
      const std::string& renderer_name) const;

  // Returns the world pose of the camera *sensor* based on camera properties,
  // parent frame, and pose in parent.
  math::RigidTransformd CalcCameraWorldPose(
      const render::RenderCameraCore& core, FrameId parent_frame,
      const math::RigidTransformd& X_PC) const;

  // Utility function to facilitate getting a double-valued pose for a frame,
  // regardless of T's actual type.
  math::RigidTransformd GetDoubleWorldPose(FrameId frame_id) const;

  // Returns a reference to the driven meshes with the given role in this
  // GeometryState.
  // @pre role is not Role::kUnassigned.
  const internal::DrivenMeshData& driven_mesh_data(Role role) const {
    DRAKE_DEMAND(role != Role::kUnassigned);
    return kinematics_data_.driven_mesh_data.at(role);
  }

  /* TODO(xuchenhan-tri) Dangerous mutable getters using const_cast.
   These data live in GeometryState (which is a Parameter in the system
   framework sense), but they are (or depend on) time-dependent position data
   which in turn depends on Scene Graph's input ports. These data should be
   separate cache entries in SceneGraph that get updated when an Eval notes that
   the input ports have changed. Instead, SceneGraph uses proxy `int` cache
   entries whose evaluation trigger the filling-in of these position data
   and subsequently update the proximity engine and the render engines that
   depend on these position data. As long as every reference to position data
   (and the proximity engine/render engines that depend on the position
   data) is preceded to an Eval of the proxy cache entry, these time-dependent
   members are just acting as awkwardly-placed cache memory. */
  //@{
  // Returns a mutable reference to the kinematics data in this GeometryState.
  internal::KinematicsData<T>& mutable_kinematics_data() const {
    GeometryState<T>* mutable_state = const_cast<GeometryState<T>*>(this);
    return mutable_state->kinematics_data_;
  }

  // Returns a mutable reference to the proximity engine in this GeometryState.
  internal::ProximityEngine<T>& mutable_proximity_engine() const {
    GeometryState<T>* mutable_state = const_cast<GeometryState<T>*>(this);
    return *mutable_state->geometry_engine_;
  }

  // Returns a vector of mutable pointers to all render engines in this
  // GeometryState.
  std::vector<render::RenderEngine*> GetMutableRenderEngines() const {
    std::vector<render::RenderEngine*> results;
    for (auto& [name, render_engine] : render_engines_) {
      unused(name);
      results.emplace_back(
          const_cast<render::RenderEngine*>(render_engine.get()));
    }
    return results;
  }
  //@}

  // NOTE: If adding a member it is important that it be _explicitly_ copied
  // in the converting copy constructor and likewise tested in the unit test
  // for that constructor.

  // The GeometryState gets its own source so it can own entities (such as the
  // world frame).
  SourceId self_source_;

  // Maps from registered source ids to the entities registered to those
  // sources (e.g., frames and geometries). This lives in the state to support
  // runtime topology changes. This data should only change at _discrete_
  // events where frames/geometries are introduced and removed. They do _not_
  // depend on time-dependent input values (e.g., systems::Context).

  // The registered geometry sources and the frame ids that have been registered
  // on them.
  std::unordered_map<SourceId, FrameIdSet> source_frame_id_map_;

  // The registered geometry sources and the deformable geometry ids that have
  // been registered on them.
  std::unordered_map<SourceId, GeometryIdSet>
      source_deformable_geometry_id_map_;

  // The registered geometry sources and the frame names that have been
  // registered on them. Only used to reject duplicate names.
  std::unordered_map<SourceId, internal::FrameNameSet> source_frame_name_map_;

  // The registered geometry sources and the frame ids that have the world frame
  // as the parent frame. For a completely flat hierarchy, this contains the
  // same values as the corresponding entry in source_frame_id_map_.
  std::unordered_map<SourceId, FrameIdSet> source_root_frame_map_;

  // The registered geometry source names. Each name is unique and the keys in
  // this map should be identical to those in source_frame_id_map_ and
  // source_root_frame_map_.
  std::unordered_map<SourceId, std::string> source_names_;

  // The registered geometry sources and the _anchored_ geometries that have
  // been registered on them. These don't fit in the frame hierarchy because
  // they do not belong to dynamic frames.
  std::unordered_map<SourceId, std::unordered_set<GeometryId>>
      source_anchored_geometry_map_;

  // The frame data, keyed on unique frame identifier.
  std::unordered_map<FrameId, internal::InternalFrame> frames_;

  // The geometry data, keyed on unique geometry identifiers.
  std::unordered_map<GeometryId, internal::InternalGeometry> geometries_;

  // This provides the look up from the internal index of a frame to its frame
  // id. It is constructed so that the index value of any position in the vector
  // _is_ the frame index of the corresponding frame.
  // It should be invariant that:
  //   1. frames_.size() == frame_index_to_id_map_.size();
  //   2. frame_index_to_id_map_.size() == biggest_index(frames_) + 1
  //      i.e. the largest pose index associated with frames_ is the last valid
  //      index of this vector.
  std::vector<FrameId> frame_index_to_id_map_;

  // Kinematics data for all frames and all deformable geometries.
  // frames_.size() == kinematics_data_.num_frames() and
  // NumDeformableGeometries() == kinematics_data_.num_deformable_geometries()
  // are two invariants.
  internal::KinematicsData<T> kinematics_data_;

  // deformable_render_meshes_[role][id] stores the render meshes for the
  // deformable geometry with the given id and role. Valid only for illustration
  // and perception roles.
  std::map<Role, std::map<GeometryId, std::vector<internal::RenderMesh>>>
      deformable_render_meshes_;

  // The underlying geometry engine. The topology of the engine does _not_
  // change with respect to time. But its values do. This straddles the two
  // worlds, maintaining its own persistent topological state and derived
  // time-dependent state. This _could_ be constructed from scratch at each
  // evaluation based on the previous data, but its internal data structures
  // rely on temporal coherency to speed up the calculations. Thus we persist
  // and copy it.
  copyable_unique_ptr<internal::ProximityEngine<T>> geometry_engine_;

  // The collection of all registered renderers. When copying a GeometryState,
  // we must ensure that it's a deep copy via DeepCopySharedPtr.
  std::unordered_map<std::string,
                     internal::DeepCopySharedPtr<render::RenderEngine>>
      render_engines_;

  // The version for this geometry data.
  GeometryVersion geometry_version_;
};
}  // namespace geometry
}  // namespace drake
