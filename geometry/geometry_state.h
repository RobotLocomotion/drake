#pragma once

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
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/geometry_set.h"
#include "drake/geometry/geometry_version.h"
#include "drake/geometry/internal_frame.h"
#include "drake/geometry/internal_geometry.h"
#include "drake/geometry/proximity_engine.h"
#include "drake/geometry/render/render_camera.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/utilities.h"

namespace drake {
namespace geometry {

#ifndef DRAKE_DOXYGEN_CXX
namespace internal {

class GeometryVisualizationImpl;

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

//@}

// TODO(SeanCurtis-TRI): Move GeometryState into `internal` namespace (and then
//  I can kill the `@note` in the class documentation).

/**
 The context-dependent state of SceneGraph. This serves as an AbstractValue
 in the context. SceneGraph's time-dependent state includes more than just
 values; objects can be added to or removed from the world over time. Therefore,
 SceneGraph's context-dependent state includes values (the poses) and
 structure (the topology of the world).

 @note This is intended as an internal class only.

 @tparam_nonsymbolic_scalar
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
  std::vector<GeometryId> GetAllGeometryIds() const {
    std::vector<GeometryId> ids;
    ids.reserve(geometries_.size());
    for (const auto& id_geometry_pair : geometries_) {
      ids.push_back(id_geometry_pair.first);
    }
    return ids;
  }

  /** Implementation of SceneGraphInspector::GetGeometryIds().  */
  std::unordered_set<GeometryId> GetGeometryIds(
      const GeometrySet& geometry_set, const std::optional<Role>& role) const;

  /** Implementation of SceneGraphInspector::NumGeometriesWithRole().  */
  int NumGeometriesWithRole(Role role) const;

  /** Implementation of SceneGraphInspector::NumDynamicGeometries().  */
  int NumDynamicGeometries() const;

  /** Implementation of SceneGraphInspector::NumAnchoredGeometries().  */
  int NumAnchoredGeometries() const;

  /** Implementation of SceneGraphInspector::GetCollisionCandidates().  */
  std::set<std::pair<GeometryId, GeometryId>> GetCollisionCandidates() const;

  /** Implementation of SceneGraphInspector::GetGeometryVersion().  */
  const GeometryVersion& geometry_version() const {
      return geometry_version_;
  }

  //@}

  /** @name          Sources and source-related data  */
  //@{

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

  /** Implementation of SceneGraphInspector::X_PG().  */
  const math::RigidTransform<double>& GetPoseInParent(
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

  /** Implementation of SceneGraphInspector::CollisionFiltered().  */
  bool CollisionFiltered(GeometryId id1, GeometryId id2) const;

  //@}

  /** @name                Pose-dependent queries

   These quantities all depend on the most recent pose values assigned to the
   registered frames.  */
  //@{

  /** Implementation of QueryObject::GetPoseInWorld(FrameId).  */
  const math::RigidTransform<T>& get_pose_in_world(FrameId frame_id) const;

  /** Implementation of QueryObject::GetPoseInWorld(GeometryId).  */
  const math::RigidTransform<T>& get_pose_in_world(
      GeometryId geometry_id) const;

  /** Implementation of QueryObject::GetPoseInParent().  */
  const math::RigidTransform<T>& get_pose_in_parent(FrameId frame_id) const;

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

  /** Implementation of
   @ref SceneGraph::RegisterGeometry(SourceId,GeometryId,
   std::unique_ptr<GeometryInstance>) "SceneGraph::RegisterGeometry()" with
   parent GeometryId.  */
  GeometryId RegisterGeometryWithParent(
      SourceId source_id, GeometryId parent_id,
      std::unique_ptr<GeometryInstance> geometry);

  // TODO(SeanCurtis-TRI): Consider deprecating this; it's now strictly a
  // wrapper for the more general `RegisterGeometry()`.
  /** Implementation of SceneGraph::RegisterAnchoredGeometry().  */
  GeometryId RegisterAnchoredGeometry(
      SourceId source_id, std::unique_ptr<GeometryInstance> geometry);

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
    return geometry_engine_->ComputePointPairPenetration(X_WGs_);
  }

  /** Implementation of QueryObject::ComputeContactSurfaces().  */
  std::vector<ContactSurface<T>> ComputeContactSurfaces(
      HydroelasticContactRepresentation representation) const {
    return geometry_engine_->ComputeContactSurfaces(representation, X_WGs_);
  }

  /** Implementation of QueryObject::ComputeContactSurfacesWithFallback().  */
  void ComputeContactSurfacesWithFallback(
      HydroelasticContactRepresentation representation,
      std::vector<ContactSurface<T>>* surfaces,
      std::vector<PenetrationAsPointPair<T>>* point_pairs) const {
    DRAKE_DEMAND(surfaces != nullptr);
    DRAKE_DEMAND(point_pairs != nullptr);
    return geometry_engine_->ComputeContactSurfacesWithFallback(
        representation, X_WGs_, surfaces, point_pairs);
  }

  /** Implementation of QueryObject::FindCollisionCandidates().  */
  std::vector<SortedPair<GeometryId>> FindCollisionCandidates() const {
    return geometry_engine_->FindCollisionCandidates();
  }

  /** Implementation of QueryObject::HasCollisions().  */
  bool HasCollisions() const {
    return geometry_engine_->HasCollisions();
  }

  //@}

  /** @name        Collision filtering    */

  /** Implementation of SceneGraph::collision_filter_manager(). */
  CollisionFilterManager collision_filter_manager() {
    geometry_version_.modify_proximity();
    return CollisionFilterManager(
        &geometry_engine_->collision_filter(), [this](const GeometrySet& set) {
          return this->CollectIds(set, Role::kProximity);
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
        X_WGs_, max_distance);
  }

  /** Implementation of
   QueryObject::ComputeSignedDistancePairClosestPoints().  */
  SignedDistancePair<T> ComputeSignedDistancePairClosestPoints(
      GeometryId id_A, GeometryId id_B) const {
    return geometry_engine_->ComputeSignedDistancePairClosestPoints(id_A, id_B,
                                                                    X_WGs_);
  }

  /** Implementation of QueryObject::ComputeSignedDistanceToPoint().  */
  std::vector<SignedDistanceToPoint<T>> ComputeSignedDistanceToPoint(
      const Vector3<T>& p_WQ, double threshold) const {
    return geometry_engine_->ComputeSignedDistanceToPoint(p_WQ, X_WGs_,
                                                          threshold);
  }

  //@}

  //---------------------------------------------------------------------------
  /** @name                Render Queries
   See @ref render_queries "Render Queries" for more details.  */
  //@{

  /** Implementation of SceneGraph::AddRenderer().  */
  void AddRenderer(std::string name,
                   std::unique_ptr<render::RenderEngine> renderer);

  /** Implementation of SceneGraph::HasRenderer().  */
  bool HasRenderer(const std::string& name) const {
    return render_engines_.count(name) > 0;
  }

  /** Implementation of QueryObject::GetRenderEngineByName.  */
  const render::RenderEngine* GetRenderEngineByName(
      const std::string& name) const {
    if (render_engines_.count(name) > 0) {
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

 private:
  // GeometryState of one scalar type is friends with all other scalar types.
  template <typename>
  friend class GeometryState;

  // Conversion constructor. In the initial implementation, this is only
  // intended to be used to clone an AutoDiffXd instance from a double instance.
  // It is _vitally_ important that all members are _explicitly_ accounted for
  // (either in the initialization list or in the body). Failure to do so will
  // lead to errors in the converted GeometryState instance.
  template <typename U>
  explicit GeometryState(const GeometryState<U>& source)
      : self_source_(source.self_source_),
        source_frame_id_map_(source.source_frame_id_map_),
        source_root_frame_map_(source.source_root_frame_map_),
        source_names_(source.source_names_),
        source_anchored_geometry_map_(source.source_anchored_geometry_map_),
        frames_(source.frames_),
        geometries_(source.geometries_),
        frame_index_to_id_map_(source.frame_index_to_id_map_),
        geometry_engine_(std::move(source.geometry_engine_->ToAutoDiffXd())),
        render_engines_(source.render_engines_),
        geometry_version_(source.geometry_version_) {
    auto convert_pose_vector = [](const std::vector<math::RigidTransform<U>>& s,
                                  std::vector<math::RigidTransform<T>>* d) {
      std::vector<math::RigidTransform<T>>& dest = *d;
      dest.resize(s.size());
      for (size_t i = 0; i < s.size(); ++i) {
        dest[i] = s[i].template cast<T>();
      }
    };
    convert_pose_vector(source.X_PF_, &X_PF_);
    convert_pose_vector(source.X_WF_, &X_WF_);

    // Now convert the id -> pose map.
    std::unordered_map<GeometryId, math::RigidTransform<T>>& dest = X_WGs_;
    const std::unordered_map<GeometryId, math::RigidTransform<U>>& s =
        source.X_WGs_;
    for (const auto& id_pose_pair : s) {
      const GeometryId id = id_pose_pair.first;
      const math::RigidTransform<U>& X_WG_source = id_pose_pair.second;
      dest.insert({id, X_WG_source.template cast<T>()});
    }
  }

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
  // geometries with the given role will be returned.
  std::unordered_set<GeometryId> CollectIds(
      const GeometrySet& geometry_set, std::optional<Role> role) const;

  // Sets the kinematic poses for the frames indicated by the given ids.
  // @param poses The frame id and pose values.
  // @pre source_id is a registered source.
  // @throws std::exception  If the ids are invalid as defined by
  // ValidateFrameIds().
  void SetFramePoses(SourceId source_id, const FramePoseVector<T>& poses);

  // Confirms that the set of ids provided include _all_ of the frames
  // registered to the set's source id and that no extra frames are included.
  // @param values The kinematics values (ids and values) to validate.
  // @pre source_id is a registered source.
  // @throws std::exception if the set is inconsistent with known topology.
  template <typename ValueType>
  void ValidateFrameIds(SourceId source_id,
                        const FrameKinematicsVector<ValueType>& values) const;

  // Method that performs any final book-keeping/updating on the state after
  // _all_ of the state's frames have had their poses updated.
  void FinalizePoseUpdate();

  // Gets the source id for the given frame id. Throws std::exception if the
  // frame belongs to no registered source.
  SourceId get_source_id(FrameId frame_id) const;

  // Gets the source id for the given frame id. Throws std::exception if the
  // geometry belongs to no registered source.
  SourceId get_source_id(GeometryId frame_id) const;

  // The origin from where an invocation of RemoveGeometryUnchecked was called.
  // The origin changes the work that is required.
  // TODO(SeanCurtis-TRI): Add `kFrame` when this can be invoked by removing
  // a frame.
  enum class RemoveGeometryOrigin {
    kGeometry,  // Invoked by RemoveGeometry().
    kRecurse    // Invoked by recursive call in RemoveGeometryUnchecked.
  };

  // Performs the work necessary to remove the identified geometry from
  // the world. The amount of work depends on the context from which this
  // method is invoked:
  //
  //  - RemoveGeometry(): A specific geometry (and its corresponding
  //    hierarchy) is being removed. In addition to recursively removing all
  //    child geometries, it must also remove this geometry id from its parent
  //    frame and, if it exists, its parent geometry.
  //   - RemoveGeometryUnchecked(): This is the recursive call; it's parent
  //    is already slated for removal, so parent references can be left alone.
  // @throws std::exception if `geometry_id` is not in `geometries_`.
  void RemoveGeometryUnchecked(GeometryId geometry_id,
                               RemoveGeometryOrigin caller);

  // Recursively updates the frame and geometry _pose_ information for the tree
  // rooted at the given frame, whose parent's pose in the world frame is given
  // as `X_WP`.
  void UpdatePosesRecursively(const internal::InternalFrame& frame,
                              const math::RigidTransform<T>& X_WP,
                              const FramePoseVector<T>& poses);

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

  // Attempts to remove the geometry with the given `id` from the named
  // renderer. Returns true if removed (false doesn't imply "failure", just
  // nothing to remove). This does no checking on ownership.
  // @pre geometry_id maps to a registered geometry.
  bool RemoveFromRendererUnchecked(const std::string& renderer_name,
                                   GeometryId id);

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

  // Utility function to facilitate getting a double-valued pose for a frame,
  // regardless of T's actual type.
  math::RigidTransformd GetDoubleWorldPose(FrameId frame_id) const;

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
  // depend on time-dependent input values (e.g., System::Context).

  // The registered geometry sources and the frame ids that have been registered
  // on them.
  std::unordered_map<SourceId, FrameIdSet> source_frame_id_map_;

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

  // ---------------------------------------------------------------------
  // These values depend on time-dependent input values (e.g., current frame
  // poses).

  // TODO(SeanCurtis-TRI): These values are place holders. Ultimately, they
  // will live in the cache. Furthermore, they will be broken up by source
  // so that inputs can be pulled independently. This work will be done when
  // the cache PR lands. For now, they are big blobs of memory.

  // Map from a frame's index to the _current_ pose of the frame F it identifies
  // relative to its parent frame P, i.e., X_PF.
  // TODO(SeanCurtis-TRI): Rename this to X_PFs_ to reflect multiplicity.
  std::vector<math::RigidTransform<T>> X_PF_;

  // The pose of every geometry relative to the _world_ frame (regardless of
  // roles) keyed by the corresponding geometry's id. After a complete state
  // update from input poses,
  //   X_WGs_[i] == X_WFₙ · X_FₙFₙ₋₁ · ... · X_F₁F · G_i.X_FG()
  // Where F is the parent frame of geometry G_i, Fₖ₊₁ is the parent frame of
  // frame Fₖ, and the world frame W is the parent of frame Fₙ.
  // In other words, it is the full evaluation of the kinematic chain from the
  // geometry to the world frame.
  std::unordered_map<GeometryId, math::RigidTransform<T>> X_WGs_;

  // The pose of each frame relative to the _world_ frame.
  // frames_.size() == X_WF_.size() is an invariant. Furthermore, after a
  // complete state update from input poses,
  //   X_WF_[i] == X_WFₙ X_FₙFₙ₋₁ ... X_Fᵢ₊₂Fᵢ₊₁ X_PF_[i]
  // Where Fᵢ₊₁ is the parent frame of frame i, Fₖ₊₁ is the parent frame of
  // frame Fₖ, and the world frame W is the parent of frame Fₙ.
  // In other words, it is the full evaluation of the kinematic chain from
  // frame i to the world frame.
  // TODO(SeanCurtis-TRI): Rename this to X_WFs_ to reflect multiplicity.
  std::vector<math::RigidTransform<T>> X_WF_;

  // The underlying geometry engine. The topology of the engine does _not_
  // change with respect to time. But its values do. This straddles the two
  // worlds, maintaining its own persistent topological state and derived
  // time-dependent state. This _could_ be constructed from scratch at each
  // evaluation based on the previous data, but its internal data structures
  // rely on temporal coherency to speed up the calculations. Thus we persist
  // and copy it.
  copyable_unique_ptr<internal::ProximityEngine<T>> geometry_engine_;

  // The collection of all registered renderers.
  std::unordered_map<std::string, copyable_unique_ptr<render::RenderEngine>>
      render_engines_;

  // The version for this geometry data.
  GeometryVersion geometry_version_;
};
}  // namespace geometry
}  // namespace drake
