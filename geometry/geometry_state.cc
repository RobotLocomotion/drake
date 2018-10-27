#include "drake/geometry/geometry_state.h"

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/autodiff.h"
#include "drake/common/default_scalars.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/proximity_engine.h"
#include "drake/geometry/utilities.h"

namespace drake {
namespace geometry {

using internal::GeometryStateCollisionFilterAttorney;
using internal::InternalFrame;
using internal::InternalGeometry;
using internal::ProximityEngine;
using std::make_pair;
using std::make_unique;
using std::move;
using std::to_string;

//-----------------------------------------------------------------------------

// These utility methods help streamline the desired semantics of map lookups.
// We want to search for a key and throw an exception (with a meaningful
// message) if not found.

// Helper method for consistently determining the presence of a key in a
// container and throwing a consistent exception type if absent.
// Searches for a key value in a "findable" object. To be findable, the source
// must have find(const Key&) and end() methods that return types that can
// be equality compared, such that if they are equal, the key is *not* present
// in the source. The exception message is produced by the given functor,
// make_message().
template <class Key, class Findable>
void FindOrThrow(const Key& key, const Findable& source,
                 const std::function<std::string()>& make_message) {
  if (source.find(key) == source.end()) throw std::logic_error(make_message());
}
// Definition of error message for a missing key lookup.
template <class Key>
std::string get_missing_id_message(const Key& key) {
  // TODO(SeanCurtis-TRI): Use NiceTypeName to get the key name.
  return "Error in map look up of unexpected key type";
}

// The look up and error-throwing method for const values.
template <class Key, class Value>
const Value& GetValueOrThrow(const Key& key,
                             const std::unordered_map<Key, Value>& map) {
  auto itr = map.find(key);
  if (itr != map.end()) {
    return itr->second;
  }
  throw std::logic_error(get_missing_id_message(key));
}

// The look up and error-throwing method for mutable values.
template <class Key, class Value>
Value& GetMutableValueOrThrow(const Key& key,
                              std::unordered_map<Key, Value>* map) {
  auto itr = map->find(key);
  if (itr != map->end()) {
    return itr->second;
  }
  throw std::logic_error(get_missing_id_message(key));
}

// Specializations for missing key based on key types.
template <>
std::string get_missing_id_message<SourceId>(const SourceId& key) {
  std::stringstream ss;
  ss << "Referenced geometry source " << key << " is not registered.";
  return ss.str();
}

template <>
std::string get_missing_id_message<FrameId>(const FrameId& key) {
  std::stringstream ss;
  ss << "Referenced frame " << key << " has not been registered.";
  return ss.str();
}

template <>
std::string get_missing_id_message<GeometryId>(const GeometryId& key) {
  std::stringstream ss;
  ss << "Referenced geometry " << key << " has not been registered.";
  return ss.str();
}

//-----------------------------------------------------------------------------

template <typename T>
GeometryState<T>::GeometryState()
    : self_source_(SourceId::get_new_id()),
      geometry_engine_(make_unique<internal::ProximityEngine<T>>()) {
  source_names_[self_source_] = "SceneGraphInternal";

  const FrameId world = InternalFrame::world_frame_id();
  // As an arbitrary design choice, we'll say the world frame is its own parent.
  frames_[world] = InternalFrame(self_source_, world, "world",
                                 InternalFrame::world_frame_group(),
                                 FrameIndex(0), world,
                                 InternalFrame::world_frame_clique());
  frame_index_to_id_map_.push_back(world);
  X_WF_.push_back(Isometry3<T>::Identity());
  X_PF_.push_back(Isometry3<T>::Identity());

  source_frame_id_map_[self_source_] = {world};
  source_root_frame_map_[self_source_] = {world};
}

template <typename T>
int GeometryState<T>::GetNumFrameGeometries(FrameId frame_id) const {
  const InternalFrame& frame = GetValueOrThrow(frame_id, frames_);
  return static_cast<int>(frame.child_geometries().size());
}

template <typename T>
int GeometryState<T>::GetNumDynamicGeometries() const {
  int count = 0;
  for (const auto& pair : frames_) {
    const InternalFrame& frame = pair.second;
    if (frame.id() == InternalFrame::world_frame_id()) continue;
    count += frame.num_child_geometries();
  }
  return count;
}

template <typename T>
int GeometryState<T>::GetNumAnchoredGeometries() const {
  const InternalFrame& frame = frames_.at(InternalFrame::world_frame_id());
  return frame.num_child_geometries();
}

template <typename T>
bool GeometryState<T>::source_is_registered(SourceId source_id) const {
  return source_frame_id_map_.find(source_id) != source_frame_id_map_.end();
}

template <typename T>
int GeometryState<T>::get_frame_group(FrameId frame_id) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "No frame group available for invalid frame id: " +
           to_string(frame_id);
  });
  return frames_.at(frame_id).frame_group();
}

template <typename T>
const std::string& GeometryState<T>::get_frame_name(FrameId frame_id) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "No frame name available for invalid frame id: " +
           to_string(frame_id);
  });
  return frames_.at(frame_id).name();
}

template <typename T>
const std::string& GeometryState<T>::get_name(GeometryId geometry_id) const {
  const InternalGeometry* geometry = GetGeometry(geometry_id);
  if (geometry != nullptr) return geometry->name();

  throw std::logic_error("No geometry available for invalid geometry id: " +
      to_string(geometry_id));
}

template <typename T>
GeometryId GeometryState<T>::GetGeometryFromName(
    FrameId frame_id, const std::string& name) const {
  const std::string canonical_name = detail::CanonicalizeStringName(name);
  // TODO(SeanCurtis-TRI): Account for geometry role once implemented.

  GeometryId result;
  int count = 0;
  std::string frame_name;

  const InternalFrame& frame = GetValueOrThrow(frame_id, frames_);
  frame_name = frame.name();
  for (GeometryId geometry_id : frame.child_geometries()) {
    const InternalGeometry& geometry = geometries_.at(geometry_id);
    if (geometry.name() == canonical_name) {
      ++count;
      result = geometry_id;
    }
  }

  if (count == 1) return result;
  if (count < 1) {
    throw std::logic_error("The frame '" + frame_name + "' (" +
        to_string(frame_id) + ") has no geometry with the canonical name '" +
        canonical_name + "'");
  }
  throw std::logic_error("The frame '" + frame_name + "' (" +
      to_string(frame_id) + ") has multiple geometries with the canonical " +
      "name'" + canonical_name + "'");
}

template <typename T>
const Isometry3<T>& GeometryState<T>::get_pose_in_world(
    FrameId frame_id) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "No world pose available for invalid frame id: " +
           to_string(frame_id);
  });
  return X_WF_[frames_.at(frame_id).index()];
}

template <typename T>
const Isometry3<T>& GeometryState<T>::get_pose_in_world(
    GeometryId geometry_id) const {
  // TODO(SeanCurtis-TRI): This is a BUG! If you pass in the id of an
  // anchored geometry, this will throw an exception. See
  // https://github.com/RobotLocomotion/drake/issues/9145.
  FindOrThrow(geometry_id, geometries_, [geometry_id]() {
    return "No world pose available for invalid geometry id: " +
           to_string(geometry_id);
  });
  return X_WG_[geometries_.at(geometry_id).index()];
}

template <typename T>
const Isometry3<T>& GeometryState<T>::get_pose_in_parent(
    FrameId frame_id) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "No pose available for invalid frame id: " + to_string(frame_id);
  });
  return X_PF_[frames_.at(frame_id).index()];
}

template <typename T>
const std::string& GeometryState<T>::get_source_name(SourceId id) const {
  auto itr = source_names_.find(id);
  if (itr != source_names_.end()) return itr->second;
  throw std::logic_error(
      "Querying source name for an invalid source id: " + to_string(id) + ".");
}

template <typename T>
const Isometry3<double>& GeometryState<T>::GetPoseInFrame(
    GeometryId geometry_id) const {
  const auto& geometry = GetValueOrThrow(geometry_id, geometries_);
  return geometry.X_FG();
}

template <typename T>
const Isometry3<double>& GeometryState<T>::GetPoseInParent(
    GeometryId geometry_id) const {
  const auto& geometry = GetValueOrThrow(geometry_id, geometries_);
  return geometry.X_PG();
}

template <typename T>
const VisualMaterial& GeometryState<T>::get_visual_material(
    GeometryId geometry_id) const {
  const auto& geometry = GetValueOrThrow(geometry_id, geometries_);
  return geometry.visual_material();
}

template <typename T>
SourceId GeometryState<T>::RegisterNewSource(const std::string& name) {
  SourceId source_id = SourceId::get_new_id();
  const std::string final_name =
      name != "" ? name : "Source_" + to_string(source_id);

  // The user can provide bad names, _always_ test.
  for (const auto& pair : source_names_) {
    if (pair.second == final_name) {
      throw std::logic_error(
          "Registering new source with duplicate name: " + final_name + ".");
    }
  }

  source_frame_id_map_[source_id];
  source_root_frame_map_[source_id];
  source_anchored_geometry_map_[source_id];
  source_names_[source_id] = final_name;
  return source_id;
}

template <typename T>
FrameId GeometryState<T>::RegisterFrame(SourceId source_id,
                                        const GeometryFrame& frame) {
  return RegisterFrame(source_id, InternalFrame::world_frame_id(), frame);
}

template <typename T>
FrameId GeometryState<T>::RegisterFrame(SourceId source_id, FrameId parent_id,
                                        const GeometryFrame& frame) {
  FrameId frame_id = frame.id();

  if (frames_.count(frame_id) > 0) {
    throw std::logic_error(
        "Registering frame with an id that has already been registered: " +
            to_string(frame_id));
  }

  FrameIdSet& f_set = GetMutableValueOrThrow(source_id, &source_frame_id_map_);
  if (parent_id != InternalFrame::world_frame_id()) {
    FindOrThrow(parent_id, f_set, [parent_id, source_id]() {
      return "Indicated parent id " + to_string(parent_id) + " does not belong "
          "to the indicated source id " + to_string(source_id) + ".";
    });
    frames_[parent_id].add_child(frame_id);
  } else {
    // The parent is the world frame; register it as a root frame.
    source_root_frame_map_[source_id].insert(frame_id);
  }

  DRAKE_ASSERT(X_PF_.size() == frame_index_to_id_map_.size());
  FrameIndex index(X_PF_.size());
  X_PF_.emplace_back(frame.pose());
  X_WF_.emplace_back(Isometry3<double>::Identity());
  frame_index_to_id_map_.push_back(frame_id);
  f_set.insert(frame_id);
  int clique = GeometryStateCollisionFilterAttorney::get_next_clique(
      geometry_engine_.get_mutable());
  frames_.emplace(frame_id, InternalFrame(source_id, frame_id, frame.name(),
                                          frame.frame_group(), index,
                                          parent_id, clique));
  return frame_id;
}

template <typename T>
GeometryId GeometryState<T>::RegisterGeometry(
    SourceId source_id, FrameId frame_id,
    std::unique_ptr<GeometryInstance> geometry) {
  if (geometry == nullptr) {
    throw std::logic_error(
        "Registering null geometry to frame " + to_string(frame_id) +
            ", on source " + to_string(source_id) + ".");
  }

  GeometryId geometry_id = geometry->id();
  if (geometries_.count(geometry_id) > 0) {
    throw std::logic_error(
        "Registering geometry with an id that has already been registered: " +
            to_string(geometry_id));
  }

  SourceId frame_source_id = source_id;
  if (frame_id == InternalFrame::world_frame_id()) {
    // Explicitly validate the source id because it won't happen in acquiring
    // the world frame.
    FindOrThrow(source_id, source_frame_id_map_, [source_id]() {
      return get_missing_id_message(source_id);
    });
    frame_source_id = self_source_;
  }
  FrameIdSet& set = GetMutableValueOrThrow(frame_source_id,
                                           &source_frame_id_map_);

  FindOrThrow(frame_id, set, [frame_id, frame_source_id]() {
    return "Referenced frame " + to_string(frame_id) + " for source " +
        to_string(frame_source_id) +
        ", but the frame doesn't belong to the source.";
  });

  // Pass the geometry to the engine.

  GeometryIndex index(static_cast<int>(geometry_index_to_id_map_.size()));
  ProximityIndex engine_index;
  if (frame_id == InternalFrame::world_frame_id()) {
    engine_index = geometry_engine_->AddAnchoredGeometry(
        geometry->shape(), geometry->pose(), index);
  } else {
    engine_index =
        geometry_engine_->AddDynamicGeometry(geometry->shape(), index);
    DRAKE_DEMAND(engine_index == X_WG_.size());
    X_WG_.push_back(Isometry3<T>::Identity());
  }

  geometry_index_to_id_map_.push_back(geometry_id);

  // Configure topology.
  // TODO(SeanCurtis-TRI): Once geometry roles are implemented, test for
  // uniqueness of the canonical name in that role for the given frame.
  // NOTE: It is important to test for name validity *before* adding this
  // geometry to the frame.

  InternalFrame& frame = frames_[frame_id];
  frame.add_child(geometry_id);

  geometries_.emplace(
      geometry_id,
      InternalGeometry(source_id, geometry->release_shape(), frame_id,
                       geometry_id, geometry->name(), geometry->pose(),
                       index, geometry->visual_material()));
  geometries_[geometry_id].set_proximity_index(engine_index);

  if (!frame.is_world()) {
    // We only have to worry about applying collision filter on geometries that
    // share the same frame if the frame isn't the world; anchored geometries
    // are *implicitly* never collided against each other.
    int child_count = static_cast<int>(frame.child_geometries().size());
    if (child_count > 1) {
      // Filter collisions between geometries affixed to the same frame. We only
      // add a clique to a frame's geometries when there are *multiple* child
      // geometries.
      ProximityEngine<T>& engine = *geometry_engine_.get_mutable();
      if (child_count > 2) {
        // Assume all previous geometries have had the clique assigned.
        GeometryStateCollisionFilterAttorney::set_dynamic_geometry_clique(
            &engine, index, frame.clique());
      } else {  // child_count == 2.
        // We *now* have multiple child geometries -- assign to clique.
        for (GeometryId child_id : frame.child_geometries()) {
          GeometryIndex child_index = geometries_[child_id].index();
          GeometryStateCollisionFilterAttorney::set_dynamic_geometry_clique(
              &engine, child_index, frame.clique());
        }
      }
    }
  }

  return geometry_id;
}

template <typename T>
GeometryId GeometryState<T>::RegisterGeometryWithParent(
    SourceId source_id, GeometryId parent_id,
    std::unique_ptr<GeometryInstance> geometry) {
  // There are three error conditions in the doxygen:.
  //    1. geometry == nullptr,
  //    2. source_id is not a registered source, and
  //    3. parent_id doesn't belong to source_id.
  //
  // Only #1 is tested directly. #2 and #3 are tested implicitly during the act
  // of registering the geometry.

  if (geometry == nullptr) {
    throw std::logic_error(
        "Registering null geometry to geometry " + to_string(parent_id) +
            ", on source " + to_string(source_id) + ".");
  }

  // This confirms that parent_id exists at all.
  InternalGeometry& parent_geometry =
      GetMutableValueOrThrow(parent_id, &geometries_);
  FrameId frame_id = parent_geometry.frame_id();

  // This implicitly confirms that source_id is registered (condition #2) and
  // that frame_id belongs to source_id. By construction, parent_id must
  // belong to the same source as frame_id, so this tests condition #3.
  GeometryId new_id = RegisterGeometry(source_id, frame_id, move(geometry));

  // RegisterGeometry stores X_PG into X_FG_ (having assumed that  the
  // parent was a frame). This replaces the stored X_PG value with the
  // semantically correct value X_FG by concatenating X_FP with X_PG.

  // Transform pose relative to geometry, to pose relative to frame.
  InternalGeometry& new_geometry = geometries_[new_id];
  // The call to `RegisterGeometry()` above stashed the pose X_PG into the
  // X_FG_ vector assuming the parent was the frame. Replace it by concatenating
  // its pose in parent, with its parent's pose in frame. NOTE: the pose is no
  // longer available from geometry because of the `move(geometry)`.
  const Isometry3<double>& X_PG = new_geometry.X_FG();
  const Isometry3<double>& X_FP = parent_geometry.X_FG();
  new_geometry.set_geometry_parent(parent_id, X_FP * X_PG);
  parent_geometry.add_child(new_id);
  return new_id;
}

template <typename T>
GeometryId GeometryState<T>::RegisterAnchoredGeometry(
    SourceId source_id,
    std::unique_ptr<GeometryInstance> geometry) {
  return RegisterGeometry(source_id, InternalFrame::world_frame_id(),
                          std::move(geometry));
}

template <typename T>
bool GeometryState<T>::IsValidGeometryName(
    FrameId frame_id, const std::string& candidate_name) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "Given frame id is not valid: " + to_string(frame_id);
  });
  // TODO(SeanCurtis-TRI): Test for uniquness after geometry roles are added.
  return !detail::CanonicalizeStringName(candidate_name).empty();
}

template <typename T>
bool GeometryState<T>::BelongsToSource(FrameId frame_id,
                                       SourceId source_id) const {
  // Confirm that the source_id is valid; use the utility function to confirm
  // source_id is valid and throw an exception with a known message.
  GetValueOrThrow(source_id, source_frame_id_map_);
  // If valid, test the frame.
  return get_source_id(frame_id) == source_id;
}

template <typename T>
bool GeometryState<T>::BelongsToSource(GeometryId geometry_id,
                                       SourceId source_id) const {
  // Confirm valid source id.
  FindOrThrow(source_id, source_names_, [source_id](){
    return get_missing_id_message(source_id);
  });
  // If this fails, the geometry_id is not valid and an exception is thrown.
  const auto& geometry = GetValueOrThrow(geometry_id, geometries_);
  return geometry.belongs_to_source(source_id);
}

template <typename T>
FrameId GeometryState<T>::GetFrameId(GeometryId geometry_id) const {
  const auto& geometry = GetValueOrThrow(geometry_id, geometries_);
  return geometry.frame_id();
}

template <typename T>
const FrameIdSet& GeometryState<T>::GetFramesForSource(
    SourceId source_id) const {
  return GetValueOrThrow(source_id, source_frame_id_map_);
}

template <typename T>
void GeometryState<T>::ExcludeCollisionsWithin(const GeometrySet& set) {
  // There is no work to be done if:
  //   1. the set contains a single frame and no geometries -- geometries *on*
  //      that single frame have already been handled, or
  //   2. there are no frames and a single geometry.
  if ((set.num_frames() == 1 && set.num_geometries() == 0) ||
      (set.num_frames() == 0 && set.num_geometries() == 1)) {
    return;
  }

  std::unordered_set<GeometryIndex> dynamic;
  std::unordered_set<GeometryIndex> anchored;
  CollectIndices(set, &dynamic, &anchored);

  geometry_engine_->ExcludeCollisionsWithin(dynamic, anchored);
}

template <typename T>
void GeometryState<T>::ExcludeCollisionsBetween(const GeometrySet& setA,
                                                const GeometrySet& setB) {
  std::unordered_set<GeometryIndex> dynamic1;
  std::unordered_set<GeometryIndex> anchored1;
  CollectIndices(setA, &dynamic1, &anchored1);
  std::unordered_set<GeometryIndex> dynamic2;
  std::unordered_set<GeometryIndex> anchored2;
  CollectIndices(setB, &dynamic2, &anchored2);

  geometry_engine_->ExcludeCollisionsBetween(dynamic1, anchored1, dynamic2,
                                             anchored2);
}

template <typename T>
std::unique_ptr<GeometryState<AutoDiffXd>> GeometryState<T>::ToAutoDiffXd()
    const {
  return std::unique_ptr<GeometryState<AutoDiffXd>>(
      new GeometryState<AutoDiffXd>(*this));
}

template <typename T>
void GeometryState<T>::CollectIndices(
    const GeometrySet& geometry_set, std::unordered_set<GeometryIndex>* dynamic,
    std::unordered_set<GeometryIndex>* anchored) {
  std::unordered_set<GeometryIndex>* target;
  for (auto frame_id : geometry_set.frames()) {
    const auto& frame = GetValueOrThrow(frame_id, frames_);
    target = frame.is_world() ? anchored : dynamic;
    for (auto geometry_id : frame.child_geometries()) {
      InternalGeometry& geometry = geometries_[geometry_id];
        target->insert(geometry.index());
    }
  }

  for (auto geometry_id : geometry_set.geometries()) {
    const InternalGeometry* geometry = GetGeometry(geometry_id);
    if (geometry == nullptr) {
      throw std::logic_error(
          "Geometry set includes a geometry id that doesn't belong to the "
          "SceneGraph: " +
          to_string(geometry_id));
    }
    if (geometry->is_dynamic()) {
      dynamic->insert(geometry->index());
    } else {
      anchored->insert(geometry->index());
    }
  }
}

template <typename T>
void GeometryState<T>::SetFramePoses(const FramePoseVector<T>& poses) {
  // TODO(SeanCurtis-TRI): Down the road, make this validation depend on
  // ASSERT_ARMED.
  ValidateFrameIds(poses);
  const Isometry3<T> world_pose = Isometry3<T>::Identity();
  for (auto frame_id : source_root_frame_map_[poses.source_id()]) {
    UpdatePosesRecursively(frames_[frame_id], world_pose, poses);
  }
}

template <typename T>
template <typename ValueType>
void GeometryState<T>::ValidateFrameIds(
    const FrameKinematicsVector<ValueType>& kinematics_data) const {
  SourceId source_id = kinematics_data.source_id();
  auto& frames = GetFramesForSource(source_id);
  const int ref_frame_count = static_cast<int>(frames.size());
  if (ref_frame_count != kinematics_data.size()) {
    // TODO(SeanCurtis-TRI): Determine if more specific information is required.
    // e.g., which frames are missing/added.
    throw std::runtime_error(
        "Disagreement in expected number of frames (" +
        to_string(frames.size()) + ") and the given number of frames (" +
        to_string(kinematics_data.size()) + ").");
  }
  for (auto id : frames) {
    if (!kinematics_data.has_id(id)) {
      throw std::runtime_error(
          "Registered frame id (" + to_string(id) + ") belonging to source " +
          to_string(source_id) +
          " was not found in the provided kinematics data.");
    }
  }
}

template <typename T>
SourceId GeometryState<T>::get_source_id(FrameId frame_id) const {
  const auto& frame = GetValueOrThrow(frame_id, frames_);
  return frame.source_id();
}

template <typename T>
void GeometryState<T>::UpdatePosesRecursively(
    const internal::InternalFrame& frame, const Isometry3<T>& X_WP,
    const FramePoseVector<T>& poses) {
  const auto frame_id = frame.id();
  const auto& X_PF = poses.value(frame_id);
  // Cache this transform for later use.
  X_PF_[frame.index()] = X_PF;
  Isometry3<T> X_WF = X_WP * X_PF;
  // TODO(SeanCurtis-TRI): Replace this when we have a transform object that
  // allows proper multiplication between an AutoDiff type and a double type.
  // For now, it allows me to perform the multiplication by multiplying the
  // fully-defined transformation (with [0 0 0 1] on the bottom row).
  X_WF.makeAffine();
  X_WF_[frame.index()] = X_WF;
  // Update the geometry which belong to *this* frame.
  for (auto child_id : frame.child_geometries()) {
    auto& child_geometry = geometries_[child_id];
    auto child_index = child_geometry.proximity_index();
    // TODO(SeanCurtis-TRI): See note above about replacing this when we have a
    // transform that supports autodiff * double.
    Isometry3<double> X_FG(child_geometry.X_FG());
    X_FG.makeAffine();
    // TODO(SeanCurtis-TRI): These matrix() shenanigans are here because I can't
    // assign a an Isometry3<double> to an Isometry3<AutoDiffXd>. Replace this
    // when I can.
    X_WG_[child_index].matrix() = X_WF.matrix() * X_FG.matrix();
  }

  // Update each child frame.
  for (auto child_id : frame.child_frames()) {
    auto& child_frame = frames_[child_id];
    UpdatePosesRecursively(child_frame, X_WF, poses);
  }
}

template <typename T>
const InternalGeometry* GeometryState<T>::GetGeometry(GeometryId id) const {
  const auto& iterator = geometries_.find(id);
  if (iterator != geometries_.end()) {
    return &iterator->second;
  }
  return nullptr;
}

}  // namespace geometry
}  // namespace drake

// TODO(SeanCurtis-TRI): Currently assumes that "non-symbolic" implies
// AutoDiffXd. Update things appropriately when more non-symbolic scalars
// are available.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::GeometryState)
