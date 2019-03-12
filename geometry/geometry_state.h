#pragma once

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_index.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/geometry_set.h"
#include "drake/geometry/internal_frame.h"
#include "drake/geometry/internal_geometry.h"
#include "drake/geometry/proximity_engine.h"
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

/**
 The context-dependent state of SceneGraph. This serves as an AbstractValue
 in the context. SceneGraph's time-dependent state includes more than just
 values; objects can be added to or removed from the world over time. Therefore,
 SceneGraph's context-dependent state includes values (the poses) and
 structure (the topology of the world).

 @tparam T The scalar type. Must be a valid Eigen scalar.

 Instantiated templates for the following kinds of T's are provided:

 - double
 - AutoDiffXd

 They are already available to link against in the containing library.
 No other values for T are currently supported.  */
template <typename T>
class GeometryState {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryState)

 public:
  /** An object that represents the range of FrameId values in the state. It
   is used in range-based for loops to iterate through registered frames.  */
  using FrameIdRange = internal::MapKeyRange<FrameId, internal::InternalFrame>;

  /** Default constructor.  */
  GeometryState();

  /** Allow assignment from a %GeometryState<double> to a %GeometryState<T>.
   @internal The SFINAE is required to prevent collision with the default
   defined assignment operator where T is double.  */
  template <class T1 = T>
  typename std::enable_if<!std::is_same<T1, double>::value,
                          GeometryState<T>&>::type
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

  /** Implementation of SceneGraphInspector::all_frame_ids().  */
  FrameIdRange get_frame_ids() const {
    return FrameIdRange(&frames_);
  }

  /** Implementation of SceneGraphInspector::num_geometries().  */
  int get_num_geometries() const {
    return static_cast<int>(geometries_.size());
  }

  /** Implementation of SceneGraphInspector::all_geometry_ids().  */
  const std::vector<GeometryId>& get_geometry_ids() const {
    return geometry_index_to_id_map_;
  }

  /** Implementation of SceneGraphInspector::NumGeometriesWithRole().  */
  int GetNumGeometriesWithRole(Role role) const;

  /** Implementation of SceneGraphInspector::GetNumDynamicGeometries().  */
  int GetNumDynamicGeometries() const;

  /** Implementation of SceneGraphInspector::GetNumAnchoredGeometries().  */
  int GetNumAnchoredGeometries() const;

  /** Implementation of SceneGraphInspector::GetCollisionCandidates().  */
  std::set<std::pair<GeometryId, GeometryId>> GetCollisionCandidates() const;

  //@}

  /** @name          Sources and source-related data  */
  //@{

  /** Implementation of SceneGraphInspector::SourceIsRegistered().  */
  bool source_is_registered(SourceId source_id) const;

  /** Implementation of SceneGraphInspector::GetSourceName().  */
  const std::string& get_source_name(SourceId id) const;

  /** Implementation of SceneGraphInspector::NumFramesForSource().  */
  int NumFramesForSource(SourceId source_id) const;

  /** Implementation of SceneGraphInspector::FramesForSource().  */
  const FrameIdSet& GetFramesForSource(SourceId source_id) const;

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
  const std::string& get_frame_name(FrameId frame_id) const;

  /** Implementation of SceneGraphInspector::GetFrameGroup().  */
  int get_frame_group(FrameId frame_id) const;

  /** Implementation of SceneGraphInspector::NumGeometriesForFrame().  */
  int GetNumFrameGeometries(FrameId frame_id) const;

  /** Implementation of SceneGraphInspector::NumGeometriesForFrameWithRole().
   */
  int GetNumFrameGeometriesWithRole(FrameId frame_id, Role role) const;

  // TODO(SeanCurtis-TRI): Redundant w.r.t. GetNumFrameGeometriesWithRole().
  /** Reports the number of child geometries for this frame that have the
   indicated role assigned. This only includes the immediate child geometries of
   *this* frame, and not those of child frames.
   @throws std::logic_error if the `frame_id` does not map to a valid frame.  */
  int NumGeometriesWithRole(FrameId frame_id, Role role) const;

  /** Implementation of SceneGraphInspector::GetGeometryIdByName().  */
  GeometryId GetGeometryFromName(FrameId frame_id,
                                 Role role,
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
  const std::string& get_name(GeometryId geometry_id) const;

  /** Implementation of SceneGraphInspector::GetShape().  */
  const Shape& GetShape(GeometryId id) const;

  /** Implementation of SceneGraphInspector::X_FG().  */
  const Isometry3<double>& GetPoseInFrame(GeometryId geometry_id) const;

  /** Implementation of SceneGraphInspector::X_PG().  */
  const Isometry3<double>& GetPoseInParent(GeometryId geometry_id) const;

  /** Implementation of SceneGraphInspector::GetProximityProperties().  */
  const ProximityProperties* get_proximity_properties(GeometryId id) const;

  /** Implementation of SceneGraphInspector::GetIllustrationProperties().  */
  const IllustrationProperties* get_illustration_properties(
      GeometryId id) const;

  /** Implementation of SceneGraphInspector::CollisionFiltered().  */
  bool CollisionFiltered(GeometryId id1, GeometryId id2) const;

  //@}

  /** @name                Pose-dependent queries

   These quantities all depend on the most recent pose values assigned to the
   registered frames.  */
  //@{

  /** Reports the pose of the frame with the given id.
   @param frame_id  The identifier of the queried frame.
   @returns The pose in the world (X_WF) of the identified frame.
   @throws std::logic_error if the frame id is not valid.  */
  const Isometry3<T>& get_pose_in_world(FrameId frame_id) const;

  /** Reports the pose of the geometry with the given id.
   @param geometry_id  The identifier of the queried geometry.
   @returns The pose in the world (X_WG) of the identified geometry.
   @throws std::logic_error if the geometry id is not valid.  */
  const Isometry3<T>& get_pose_in_world(GeometryId geometry_id) const;

  /** Reports the pose of the frame with the given id relative to its parent
   frame. If the frame's parent is the world, the value should be the same as
   a call to get_pose_in_world().
   @param frame_id  The identifier of the queried frame.
   @returns The pose in the _parent_ frame (X_PF) of the identified frame.
   @throws std::logic_error if the frame id is not valid.  */
  const Isometry3<T>& get_pose_in_parent(FrameId frame_id) const;

  //@}

  /** @name        State management

   The methods that modify the state including: adding/removing entities from
   the state, modifying values in the state, etc.  */
  //@{

  /** Registers a new, named source into the state.
   @param name          The optional name of the source. If none or the empty
                        string is provided it will be named "Source_##" where
                        the number is the value of the returned SourceId.
   @throws std::logic_error is thrown if the name is _not_ unique.  */
  SourceId RegisterNewSource(const std::string& name = "");

  /** Registers a new frame for the given source, the id of the new frame is
   returned.
   @param source_id    The id of the source for which this frame is allocated.
   @param frame        The frame to register.
   @returns  A newly allocated frame id.
   @throws std::logic_error  If the `source_id` does _not_ map to a registered
                             source, or `frame` has an id that has already
                             been registered.  */
  FrameId RegisterFrame(SourceId source_id, const GeometryFrame& frame);

  /** Registers a new frame for the given source as a child of an existing
   frame. The id of the new frame is returned. A pose for this frame will need
   to be provided in the FramePoseVector parameter to SetFramePoses().
   @param source_id    The id of the source for which this frame is allocated.
   @param parent_id    The id of the parent frame.
   @param frame        The frame to register.
   @returns  A newly allocated frame id.
   @throws std::logic_error  1. If the `source_id` does _not_ map to a
                             registered source,
                             2. If the `parent_id` does _not_ map to a known
                             frame or does not belong to the source
                             3. `frame` has an id that has already been
                             registered.  */
  FrameId RegisterFrame(SourceId source_id, FrameId parent_id,
                        const GeometryFrame& frame);

  /** Implementation of
   @ref SceneGraph::RegisterGeometry(SourceId,FrameId,std::unique_ptr<GeometryInstance>)
   "SceneGraph::RegisterGeometry()" with parent FrameId.  */
  GeometryId RegisterGeometry(SourceId source_id, FrameId frame_id,
                              std::unique_ptr<GeometryInstance> geometry);

  /** Implementation of
   @ref SceneGraph::RegisterGeometry(SourceId,GeometryId,std::unique_ptr<GeometryInstance>)
   "SceneGraph::RegisterGeometry()" with parent GeometryId.  */
  GeometryId RegisterGeometryWithParent(
      SourceId source_id, GeometryId parent_id,
      std::unique_ptr<GeometryInstance> geometry);

  // TODO(SeanCurtis-TRI): Consider deprecating this; it's now strictly a
  // wrapper for the more general `RegisterGeometry()`.
  /** Implementation of SceneGraph::RegisterAnchoredGeometry().  */
  GeometryId RegisterAnchoredGeometry(
      SourceId source_id,
      std::unique_ptr<GeometryInstance> geometry);

  /** Removes the given geometry from the the indicated source's geometries. Any
   geometry that was hung from the indicated geometry will _also_ be removed.
   @param source_id     The identifier for the owner geometry source.
   @param geometry_id   The identifier of the geometry to remove (can be dynamic
                        or anchored).
   @throws std::logic_error  1. If the `source_id` does _not_ map to a
                             registered source, or
                             2. the `geometry_id` does not map to a valid
                             geometry, or
                             3. the `geometry_id` maps to a geometry that does
                             not belong to the indicated source.  */
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

  /** Assigns the given geometry id the proximity role by assigning it the given
   set of proximity properties. At this time, the geometry's name is tested for
   uniqueness in among geometries with the proximity role.

   @param source_id     The id of the geometry source that owns the geometry.
   @param geometry_id   The geometry to assign a role.
   @param properties    The proximity properties for this geometry.
   @throws std::logic_error if 1. source id is invalid,
                               2. geometry id is invalid,
                               3. geometry id is not owned by the source id,
                               4. geometry has already had a proximity role
                                  assigned,
                               5. the geometry's name is *not* unique in this
                                  role.  */
  void AssignRole(SourceId source_id, GeometryId geometry_id,
                  ProximityProperties properties);

  /** Assigns the given geometry id the illustration role by assigning it the
   given set of proximity properties. At this time, the geometry's name is
   tested for uniqueness in among geometries with the illustration role.

   @param source_id     The id of the geometry source that owns the geometry.
   @param geometry_id   The geometry to assign a role.
   @param properties    The illustration properties for this geometry.
   @throws std::logic_error if 1. source id is invalid,
                               2. geometry id is invalid,
                               3. geometry id is not owned by the source id,
                               4. geometry has already had a illustration role
                                  assigned,
                               5. the geometry's name is *not* unique in this
                                  role.    */
  void AssignRole(SourceId source_id, GeometryId geometry_id,
                  IllustrationProperties properties);

  //@}

  //----------------------------------------------------------------------------
  /** @name                Collision Queries

   These queries detect _collisions_ between geometry. Two geometries collide
   if they overlap each other and are not explicitly excluded through
   @ref collision_filter_concepts "collision filtering". These algorithms find
   those colliding cases, characterize them, and report the essential
   characteristics of that collision.  */
  //@{

  /** See QueryObject::ComputePointPairPenetration() for documentation.  */
  std::vector<PenetrationAsPointPair<double>> ComputePointPairPenetration()
      const {
    return geometry_engine_->ComputePointPairPenetration(
        geometry_index_to_id_map_);
  }

  /** See QueryObject::ComputeContactSurfaces() for documentation.  */
  std::vector<ContactSurface<T>> ComputeContactSurfaces() const {
    return geometry_engine_->ComputeContactSurfaces(
        geometry_index_to_id_map_);
  }

  //@}

  /** @name               Proximity filters

   This interface allows control over which pairs of geometries can even be
   considered for proximity queries. This affects all queries that depend on
   geometries with a proximity role.

   See @ref scene_graph_collision_filtering "Scene Graph Collision Filtering"
   for more details.   */
  //@{

  // TODO(SeanCurtis-TRI): Rename these functions to reflect the larger role
  // in proximity queries _or_ change the scope of the filters.

  /** Supporting function for SceneGraph::ExcludeCollisionsWithin().  */
  void ExcludeCollisionsWithin(const GeometrySet& set);

  /** Supporting function for SceneGraph::ExcludeCollisionsBetween().  */
  void ExcludeCollisionsBetween(const GeometrySet& setA,
                                const GeometrySet& setB);

  //@}

  //---------------------------------------------------------------------------
  /** @name                Signed Distance Queries

  Refer to @ref signed_distance_query "Signed Distance Queries" for more details.
  */

  //@{

  /**
   * Computes the signed distance together with the witness points across all
   * pairs of geometries in the world. Reports both the separating geometries
   * and penetrating geometries.
   * @retval witness_pairs A vector of reporting the signed distance
   * characterized as witness point pairs. Notice that this is an O(N²)
   * operation, where N is the number of geometries in the world. We report the
   * distance between dynamic objects, or between a dynamic object and an
   * anchored object. We DO NOT report the distance between two anchored
   * objects.
   */
  std::vector<SignedDistancePair<double>>
  ComputeSignedDistancePairwiseClosestPoints() const {
    return geometry_engine_->ComputeSignedDistancePairwiseClosestPoints(
        geometry_index_to_id_map_);
  }

  /** Performs work in support of QueryObject::ComputeSignedDistanceToPoint().
   */
  std::vector<SignedDistanceToPoint<T>>
  ComputeSignedDistanceToPoint(
      const Vector3<T> &p_WQ,
      const double threshold) const {
    return geometry_engine_->ComputeSignedDistanceToPoint(
        p_WQ, geometry_index_to_id_map_, X_WG_, threshold);
  }

  //@}

  /** @name Scalar conversion  */
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
  GeometryState(const GeometryState<U>& source)
      : self_source_(source.self_source_),
        source_frame_id_map_(source.source_frame_id_map_),
        source_root_frame_map_(source.source_root_frame_map_),
        source_names_(source.source_names_),
        source_anchored_geometry_map_(source.source_anchored_geometry_map_),
        frames_(source.frames_),
        geometries_(source.geometries_),
        geometry_index_to_id_map_(source.geometry_index_to_id_map_),
        frame_index_to_id_map_(source.frame_index_to_id_map_),
        dynamic_proximity_index_to_internal_map_(
            source.dynamic_proximity_index_to_internal_map_),
        geometry_engine_(std::move(source.geometry_engine_->ToAutoDiffXd())) {
    // NOTE: Can't assign Isometry3<double> to Isometry3<AutoDiff>. But we _can_
    // assign Matrix<double> to Matrix<AutoDiff>, so that's what we're doing.
    auto convert = [](const std::vector<Isometry3<U>>& s,
                      std::vector<Isometry3<T>>* d) {
      std::vector<Isometry3<T>>& dest = *d;
      dest.resize(s.size());
      for (size_t i = 0; i < s.size(); ++i) {
        dest[i].matrix() = s[i].matrix();
      }
    };

    convert(source.X_PF_, &X_PF_);
    convert(source.X_WG_, &X_WG_);
    convert(source.X_WF_, &X_WF_);
  }

  // NOTE: This friend class is responsible for evaluating the internals of
  // a GeometryState and translating it into the appropriate visualization
  // mechanism.
  friend class internal::GeometryVisualizationImpl;

  // Allow SceneGraph unique access to the state members to perform queries.
  friend class SceneGraph<T>;

  // Friend declaration so that the internals of the state can be confirmed in
  // unit tests.
  template <class U> friend class GeometryStateTester;

  // Function to facilitate testing.
  int peek_next_clique() const {
    return internal::GeometryStateCollisionFilterAttorney::peek_next_clique(
        *geometry_engine_);
  }

  // Takes the frame and geometry ids from the given geometry set and
  // populates the sets of geometry _indices_ for the dynamic and anchored
  // geometries implied by the group. Ids that can't be identified will cause
  // an exception to be thrown.
  // TODO(SeanCurtis-TRI): Because all geometries now only have a single index
  // type, we have two sets of the same index type. The compiler cannot know
  // that only anchored geometries go into the anchored set and only dynamic go
  // into the dynamic set. It relies on the correctness of the implementation.
  // This *could* be worked around: this function could simply provide
  // _unclassified_ geometry indices. However, the engine needs to know which
  // are dynamic and which are anchored and currently has no facility to do
  // so. So, it would have to, essentially, duplicate the data stored in the
  // InternalGeometry instances to classify the union of these two sets. For
  // now, we accept the *slightly* dissatisfying artifact of having the same
  // index type in both sets.
  void CollectIndices(const GeometrySet& geometry_set,
                      std::unordered_set<GeometryIndex>* dynamic,
                      std::unordered_set<GeometryIndex>* anchored);

  // Sets the kinematic poses for the frames indicated by the given ids.
  // @param poses The frame id and pose values.
  // @throws std::logic_error  If the ids are invalid as defined by
  // ValidateFrameIds().
  void SetFramePoses(const FramePoseVector<T>& poses);

  // Confirms that the set of ids provided include _all_ of the frames
  // registered to the set's source id and that no extra frames are included.
  // @param values The kinematics values (ids and values) to validate.
  // @throws std::runtime_error if the set is inconsistent with known topology.
  template <typename ValueType>
  void ValidateFrameIds(const FrameKinematicsVector<ValueType>& values) const;

  // Method that performs any final book-keeping/updating on the state after
  // _all_ of the state's frames have had their poses updated.
  void FinalizePoseUpdate();

  // Gets the source id for the given frame id. Throws std::logic_error if the
  // frame belongs to no registered source.
  SourceId get_source_id(FrameId frame_id) const;

  // Gets the source id for the given frame id. Throws std::logic_error if the
  // geometry belongs to no registered source.
  SourceId get_source_id(GeometryId frame_id) const;

  // The origin from where an invocation of RemoveGeometryUnchecked was called.
  // The origin changes the work that is required.
  // TODO(SeanCurtis-TRI): Add `kFrame` when this can be invoked by removing
  // a frame.
  enum class RemoveGeometryOrigin {
    kGeometry,   // Invoked by RemoveGeometry().
    kRecurse     // Invoked by recursive call in RemoveGeometryUnchecked.
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
  // @throws std::logic_error if `geometry_id` is not in `geometries_`.
  void RemoveGeometryUnchecked(GeometryId geometry_id,
                               RemoveGeometryOrigin caller);

  // Recursively updates the frame and geometry _pose_ information for the tree
  // rooted at the given frame, whose parent's pose in the world frame is given
  // as `X_WP`.
  void UpdatePosesRecursively(const internal::InternalFrame& frame,
                              const Isometry3<T>& X_WP,
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

  template <typename PropertyType>
  void AssignRoleInternal(SourceId source_id, GeometryId geometry_id,
                          PropertyType properties, Role role);

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

  // This provides the look up from internal index to geometry id for all
  // geometries. It is constructed so that the index value of any position in
  // the vector _is_ the geometry index of the corresponding geometry.
  // The following invariants should always be true:
  //   1. geometries_[geometry_index_to_id_map_[i]].index() == i.
  //   2. geometry_index_to_id_map_.size() == geometries_.size().
  std::vector<GeometryId> geometry_index_to_id_map_;

  // This provides the look up from the internal index of a frame to its frame
  // id. It is constructed so that the index value of any position in the vector
  // _is_ the frame index of the corresponding frame.
  // It should be invariant that:
  //   1. frames_.size() == frame_index_to_id_map_.size();
  //   2. frame_index_to_id_map_.size() == biggest_index(frames_) + 1
  //      i.e. the largest pose index associated with frames_ is the last valid
  //      index of this vector.
  std::vector<FrameId> frame_index_to_id_map_;

  // This contains internal indices into X_WG_. If a _dynamic_ geometry G has a
  // proximity role, in addition to its internal index, it will
  // also have a proximity index. It must be the case that
  // G.internal_index ==
  //      dynamic_proximity_index_to_internal_map_[G.proximity_index]
  // if it has a proximity role.
  // Generally, internal_index is not equal to the role index. This allows
  // just those geometries with the proximity role to be provided to
  // the proximity engine.
  // NOTE: There is no equivalent for anchored geometries because anchored
  // geometries do not need updating.
  std::vector<GeometryIndex> dynamic_proximity_index_to_internal_map_;

  // ---------------------------------------------------------------------
  // These values depend on time-dependent input values (e.g., current frame
  // poses).

  // TODO(SeanCurtis-TRI): These values are place holders. Ultimately, they
  // will live in the cache. Furthermore, they will be broken up by source
  // so that inputs can be pulled independently. This work will be done when
  // the cache PR lands. For now, they are big blobs of memory.

  // Map from a frame's index to the _current_ pose of the frame F it identifies
  // relative to its parent frame P, i.e., X_PF.
  std::vector<Isometry3<T>> X_PF_;

  // The pose of every geometry relative to the _world_ frame (regardless of
  // roles) indexed by GeometryIndex. After a complete state update from input
  // poses,
  //   X_WG_[i] == X_WFₙ · X_FₙFₙ₋₁ · ... · X_F₁F · G_i.X_FG()
  // Where F is the parent frame of geometry G_i, Fₖ₊₁ is the parent frame of
  // frame Fₖ, and the world frame W is the parent of frame Fₙ.
  // In other words, it is the full evaluation of the kinematic chain from the
  // geometry to the world frame.
  std::vector<Isometry3<T>> X_WG_;

  // The pose of each frame relative to the _world_ frame.
  // frames_.size() == X_WF_.size() is an invariant. Furthermore, after a
  // complete state update from input poses,
  //   X_WF_[i] == X_WFₙ X_FₙFₙ₋₁ ... X_Fᵢ₊₂Fᵢ₊₁ X_PF_[i]
  // Where Fᵢ₊₁ is the parent frame of frame i, Fₖ₊₁ is the parent frame of
  // frame Fₖ, and the world frame W is the parent of frame Fₙ.
  // In other words, it is the full evaluation of the kinematic chain from
  // frame i to the world frame.
  std::vector<Isometry3<T>> X_WF_;

  // The underlying geometry engine. The topology of the engine does _not_
  // change with respect to time. But its values do. This straddles the two
  // worlds, maintaining its own persistent topological state and derived
  // time-dependent state. This _could_ be constructed from scratch at each
  // evaluation based on the previous data, but its internal data structures
  // rely on temporal coherency to speed up the calculations. Thus we persist
  // and copy it.
  copyable_unique_ptr<internal::ProximityEngine<T>> geometry_engine_;
};
}  // namespace geometry
}  // namespace drake
