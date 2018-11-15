#pragma once

#include <memory>
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

/** @name Structures for maintaining the entity relationships */
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

  /** @name        State introspection

   Various methods that allow reading the state's properties and values.  */
  //@{

  /** Reports the number of registered sources -- whether they have frames or
   not.  */
  int get_num_sources() const {
    return static_cast<int>(source_frame_id_map_.size());
  }

  /** Reports the total number of frames -- across all sources.  */
  int get_num_frames() const { return static_cast<int>(frames_.size()); }

  /** Reports the total number of geometries.  */
  int get_num_geometries() const {
    return static_cast<int>(geometries_.size());
  }

  /** Reports the total number of geometries directly registered to the given
   frame. This count does _not_ include geometries attached to frames that are
   descendants of this frame.
   @throws std::runtime_error if the `frame_id` is invalid.  */
  int GetNumFrameGeometries(FrameId frame_id) const;

  /** Reports the total number of _dynamic_ geometries in the scene graph.  */
  int GetNumDynamicGeometries() const;

  /** Reports the total number of _anchored_ geometries. This should provide
   the same answer as calling GetNumFrameGeometries() with the world frame id.
   */
  int GetNumAnchoredGeometries() const;

  /** Reports true if the given `source_id` references a registered source.  */
  bool source_is_registered(SourceId source_id) const;

  /** The set of all dynamic geometries registered to the world. The order is
   _not_ guaranteed to have any particular semantic meaning. But the order is
   guaranteed to remain fixed between topological changes (e.g., removal or
   addition of geometry/frames).  */
  const std::vector<GeometryId>& get_geometry_ids() const {
    return geometry_index_to_id_map_;
  }

  /** Provides a range object for all of the frame ids in the scene graph. The
   order is not generally guaranteed; but it will be consistent as long as there
   are no changes to the topology. This is intended to be used as:
   @code
   for (FrameId id : state.get_frame_ids()) {
    ...
   }
   @endcode

   This will include the id for the world frame.  */
  FrameIdRange get_frame_ids() const {
    return FrameIdRange(&frames_);
  }

  /** Reports the frame group for the given frame.
   @param frame_id  The identifier of the queried frame.
   @returns The frame group of the identified frame.
   @throws std::logic_error if the frame id is not valid.
   @internal This is equivalent to the old "model instance id".  */
  int get_frame_group(FrameId frame_id) const;

  /** Reports the name of the frame.
   @param frame_id  The identifier of the queried frame.
   @returns The name of the identified frame.
   @throws std::logic_error if the frame id is not valid.  */
  const std::string& get_frame_name(FrameId frame_id) const;

  /** Reports the stored, canonical name of the geometry (see
   @ref canonicalized_geometry_names "GeometryInstance" for details).
   @param geometry_id  The identifier of the queried geometry.
   @returns The name of the identified geometry.
   @throws std::logic_error if the geometry id is not valid.  */
  const std::string& get_name(GeometryId geometry_id) const;

  /** Reports the id for the uniquely named geometry affixed to the indicated
   frame.
   @param frame_id  The id of the parent frame.
   @param name      The name of the geometry to query for. The name will be
                    canonicalized prior to lookup (see
                    @ref canonicalized_geometry_names "GeometryInstance" for
                    details).
   @return The id of the requested geometry.
   @throws std::logic_error if no such geometry exists, multiple geometries have
                            that name, or if the frame doesn't exist.  */
  // TODO(SeanCurtis-TRI): Account for role when geometry roles exist.
  GeometryId GetGeometryFromName(FrameId frame_id,
                                 const std::string& name) const;

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

  /** Reports the source name for the given source id.
   @param id  The identifier of the source.
   @return The name of the source.
   @throws std::logic_error if the id does _not_ map to a registered source.  */
  const std::string& get_source_name(SourceId id) const;

  /** Reports the pose, relative to the registered _frame_, for the geometry
   the given identifier refers to.
   @param geometry_id     The id of the queried geometry.
   @return The geometry's pose relative to its frame.
   @throws std::logic_error  If the `geometry_id` does _not_ map to a valid
                             GeometryInstance.  */
  const Isometry3<double>& GetPoseInFrame(GeometryId geometry_id) const;

  /** Reports the pose of identified dynamic geometry, relative to its
   registered parent. If the geometry was registered directly to a frame, this
   _must_ produce the same pose as GetPoseInFrame().
   @param geometry_id     The id of the queried geometry.
   @return The geometry's pose relative to its registered parent.
   @throws std::logic_error  If the `geometry_id` does _not_ map to a valid
                             GeometryInstance.  */
  const Isometry3<double>& GetPoseInParent(GeometryId geometry_id) const;

  /** Returns the visual material defined for geometry indicated by the given
   `geometry_id` (if defined).
   @throws std::logic_error If the `geometry_id` does _not_ map to a valid
                            GeometryInstance.  */
  const VisualMaterial& get_visual_material(GeometryId geometry_id) const;

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

  /** Registers a GeometryInstance with the state. The state takes ownership of
   the geometry and associates it with the given frame and source. Returns the
   new identifier for the successfully registered GeometryInstance.
   @param source_id    The id of the source to which the frame and geometry
                       belongs.
   @param frame_id     The id of the frame on which the geometry is to hang.
   @param geometry     The geometry to get the id for. The state takes
                       ownership of the geometry.
   @returns  A newly allocated geometry id.
   @throws std::logic_error  1. the `source_id` does _not_ map to a registered
                             source,
                             2. the `frame_id` doesn't belong to the source,
                             3. the `geometry` is equal to `nullptr`,
                             4. `geometry` has a previously registered id, or
                             5. the geometry's name doesn't satisfy the
                             requirements outlined in GeometryInstance.  */
  GeometryId RegisterGeometry(SourceId source_id, FrameId frame_id,
                              std::unique_ptr<GeometryInstance> geometry);

  /** Registers a GeometryInstance with the state. Rather than hanging directly
   from a _frame_, the instance hangs on another geometry instance. The input
   `geometry` instance's pose is assumed to be relative to that parent geometry
   instance. The state takes ownership of the geometry and associates it with
   the given geometry parent (and, ultimately, the parent geometry's frame) and
   source. Returns the new identifier for the successfully registered
   GeometryInstance.
   @param source_id    The id of the source on which the geometry is being
                       declared.
   @param parent_id    The parent geometry for this geometry.
   @param geometry     The geometry to get the id for. The state takes
                       ownership of the geometry.
   @returns  A newly allocated geometry id.
   @throws std::logic_error 1. the `source_id` does _not_ map to a registered
                            source,
                            2. the `parent_id` doesn't belong to the source,
                            3. the `geometry` is equal to `nullptr`,
                            4. `geometry` has a previously registered id,  or
                            5. the geometry's name doesn't satisfy the
                            requirements outlined in GeometryInstance.  */
  GeometryId RegisterGeometryWithParent(
      SourceId source_id, GeometryId parent_id,
      std::unique_ptr<GeometryInstance> geometry);

  // TODO(SeanCurtis-TRI): Consider deprecating this; it's now strictly a
  // wrapper for the more general `RegisterGeometry()`.
  /** Registers a GeometryInstance with the state as anchored geometry. This
   registers geometry which "hangs" from the world frame and never moves.
   The `geometry`'s pose value is relative to the world frame. The state takes
   ownership of the geometry and associates it with the given source. Returns
   the new identifier for the GeometryInstance.
   @param source_id    The id of the source on which the geometry is being
                       declared.
   @param geometry     The geometry to get the id for. The state takes
                       ownership of the geometry.
   @returns  A newly allocated geometry id.
   @throws std::logic_error  1. the `source_id` does _not_ map to a registered
                             source,
                             2. `geometry` has a previously registered id, or
                             3. the geometry's name doesn't satisfy the
                             requirements outlined in GeometryInstance.  */
  GeometryId RegisterAnchoredGeometry(
      SourceId source_id,
      std::unique_ptr<GeometryInstance> geometry);

  /** Reports whether the canonicalized version of the given candidate geometry
   name is considered valid. This tests the requirements described in the
   documentation of @ref canonicalized_geometry_names "GeometryInstance". When
   adding a geometry to a frame, if there is doubt if a proposed name is valid,
   the name can be tested prior to registering the geometry.
   @param frame_id        The id of the frame to which the geometry would be
                          assigned.
   @param candidate_name  The name to validate.
   @return true if the `candidate_name` can be given to a `GeometryInstance`
   assigned to the indicated frame.
   @throws std::exception if `frame_id` does not refer to a valid frame.  */
  bool IsValidGeometryName(FrameId frame_id,
                           const std::string& candidate_name) const;

  //@}

  /** @name       Relationship queries

   Various methods that map identifiers for one type of entity to its related
   entities.  */
  //@{

  /** Reports if the given frame id was registered to the given source id.
   @param frame_id      The query frame id.
   @param source_id     The query source id.
   @returns True if `frame_id` was registered on `source_id`.
   @throws std::logic_error  If the `frame_id` does _not_ map to a frame or the
                             identified source is not registered.  */
  bool BelongsToSource(FrameId frame_id, SourceId source_id) const;

  /** Reports if the given geometry id was ultimately registered to the given
   source id.
   @param geometry_id   The query geometry id.
   @param source_id     The query source id.
   @returns True if `geometry_id` was registered on `source_id`.
   @throws std::logic_error  If the `geometry_id` does _not_ map to a valid
                             geometry or the identified source is not
                             registered */
  bool BelongsToSource(GeometryId geometry_id, SourceId source_id) const;

  /** Retrieves the frame id on which the given geometry id is registered.
   @param geometry_id   The query geometry id.
   @returns An optional FrameId based on a successful lookup.
   @throws std::logic_error  If the `geometry_id` does _not_ map to a geometry
                             which belongs to an existing frame.*/
  FrameId GetFrameId(GeometryId geometry_id) const;

  /** Returns the set of frames registered to the given source.
   @param source_id     The identifier of the source to query.
   @return  The set of frames associated with the id.
   @throws std::logic_error If the `source_id` does _not_ map to a registered
                            source.  */
  const FrameIdSet& GetFramesForSource(SourceId source_id) const;

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

  //@}

  /** @name               Proximity filters

   This interface allows control over which pairs of geometries can even be
   considered for collision.

   See @ref scene_graph_collision_filtering "Scene Graph Collision Filtering"
   for more details.   */
  //@{

  // TODO(SeanCurtis-TRI): Rename these functions to reflect the larger role
  // in proximity queries _or_ change the scope of the filters.
  /** Excludes geometry pairs from collision evaluation by updating the
   candidate pair set `C = C - P`, where `P = {(gᵢ, gⱼ)}, ∀ gᵢ, gⱼ ∈ G` and
   `G = {g₀, g₁, ..., gₘ}` is the input `set` of geometries.

   @throws std::logic_error if the set includes ids that don't exist in the
                            scene graph.  */
  void ExcludeCollisionsWithin(const GeometrySet& set);

  /** Excludes geometry pairs from collision evaluation by updating the
   candidate pair set `C = C - P`, where `P = {(a, b)}, ∀ a ∈ A, b ∈ B` and
   `A = {a₀, a₁, ..., aₘ}` and `B = {b₀, b₁, ..., bₙ}` are the input sets of
   geometries `setA` and `setB`, respectively. This does _not_ preclude
   collisions between members of the _same_ set.

   @throws std::logic_error if the groups include ids that don't exist in the
                            scene graph.   */
  void ExcludeCollisionsBetween(const GeometrySet& setA,
                                const GeometrySet& setB);

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
  void FinalizePoseUpdate() { geometry_engine_->UpdateWorldPoses(X_WG_); }

  // Gets the source id for the given frame id. Throws std::logic_error if the
  // frame belongs to no registered source.
  SourceId get_source_id(FrameId frame_id) const;

  // Recursively updates the frame and geometry _pose_ information for the tree
  // rooted at the given frame, whose parent's pose in the world frame is given
  // as `X_WP`.
  void UpdatePosesRecursively(const internal::InternalFrame& frame,
                              const Isometry3<T>& X_WP,
                              const FramePoseVector<T>& poses);

  // Reports true if the given id refers to a _dynamic_ geometry. Assumes the
  // precondition that id refers to a valid geometry in the state.
  bool is_dynamic(GeometryId id) const {
    return geometries_.count(id) > 0;
  }

  // Convenience function for accessing geometry whether dynamic or anchored.
  const internal::InternalGeometry* GetGeometry(GeometryId id) const;

  // The GeometryState gets its own source so it can own entities (such as the
  // world frame).
  SourceId self_source_;

  // ---------------------------------------------------------------------
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

  // ---------------------------------------------------------------------
  // These values depend on time-dependent input values (e.g., current frame
  // poses).

  // TODO(SeanCurtis-TRI): These values are place holders. Ultimately, they
  // will live in the cache. Furthermore, they will be broken up by source
  // so that inputs can be pulled independently. This work will be done when
  // the cache PR lands. For now, they are big blobs of memory.

  // Map from the frame id to the _current_ pose of the frame it identifies, F,
  // relative to its parent frame, P: X_PF.
  std::vector<Isometry3<T>> X_PF_;

  // The pose of every _dynamic_ geometry relative to the _world_ frame.
  // After a complete state update from input poses,
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
