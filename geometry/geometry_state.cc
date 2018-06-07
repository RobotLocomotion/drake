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

namespace drake {
namespace geometry {

using internal::GeometryStateCollisionFilterAttorney;
using internal::InternalAnchoredGeometry;
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
    : geometry_engine_(make_unique<internal::ProximityEngine<T>>()) {}

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
  return frames_.at(frame_id).get_frame_group();
}

template <typename T>
const std::string& GeometryState<T>::get_frame_name(FrameId frame_id) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "No frame name available for invalid frame id: " +
           to_string(frame_id);
  });
  return frames_.at(frame_id).get_name();
}

template <typename T>
const Isometry3<T>& GeometryState<T>::get_pose_in_world(
    FrameId frame_id) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "No world pose available for invalid frame id: " +
           to_string(frame_id);
  });
  return X_WF_[frames_.at(frame_id).get_pose_index()];
}

template <typename T>
const Isometry3<T>& GeometryState<T>::get_pose_in_world(
    GeometryId geometry_id) const {
  FindOrThrow(geometry_id, geometries_, [geometry_id]() {
    return "No world pose available for invalid geometry id: " +
           to_string(geometry_id);
  });
  return X_WG_[geometries_.at(geometry_id).get_engine_index()];
}

template <typename T>
const Isometry3<T>& GeometryState<T>::get_pose_in_parent(
    FrameId frame_id) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "No pose available for invalid frame id: " + to_string(frame_id);
  });
  return X_PF_[frames_.at(frame_id).get_pose_index()];
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
  return X_FG_[geometry.get_engine_index()];
}

template <typename T>
const Isometry3<double>& GeometryState<T>::GetPoseInParent(
    GeometryId geometry_id) const {
  const auto& geometry = GetValueOrThrow(geometry_id, geometries_);
  return geometry.get_pose_in_parent();
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
  return RegisterFrame(source_id, InternalFrame::get_world_frame_id(), frame);
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
  if (parent_id != InternalFrame::get_world_frame_id()) {
    FindOrThrow(parent_id, f_set, [parent_id, source_id]() {
      return "Indicated parent id " + to_string(parent_id) + " does not belong "
          "to the indicated source id " + to_string(source_id) + ".";
    });
    frames_[parent_id].add_child(frame_id);
  } else {
    // The parent is the world frame; register it as a root frame.
    source_root_frame_map_[source_id].insert(frame_id);
  }
  PoseIndex pose_index(X_PF_.size());
  X_PF_.emplace_back(frame.pose());
  X_WF_.emplace_back(Isometry3<double>::Identity());
  DRAKE_ASSERT(pose_index == static_cast<int>(pose_index_to_frame_map_.size()));
  pose_index_to_frame_map_.push_back(frame_id);
  f_set.insert(frame_id);
  int clique = GeometryStateCollisionFilterAttorney::get_next_clique(
      geometry_engine_.get_mutable());
  frames_.emplace(frame_id, InternalFrame(source_id, frame_id, frame.name(),
                                          frame.frame_group(), pose_index,
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

  FrameIdSet& set = GetMutableValueOrThrow(source_id, &source_frame_id_map_);

  FindOrThrow(frame_id, set, [frame_id, source_id]() {
    return "Referenced frame " + to_string(frame_id) + " for source " +
        to_string(source_id) + ", but the frame doesn't belong to the source.";
  });

  // Pass the geometry to the engine.
  GeometryIndex engine_index =
      geometry_engine_->AddDynamicGeometry(geometry->shape());
  DRAKE_DEMAND(engine_index == geometry_index_id_map_.size());
  geometry_index_id_map_.push_back(geometry_id);

  // Configure topology.
  InternalFrame& frame = frames_[frame_id];
  frame.add_child(geometry_id);

  // TODO(SeanCurtis-TRI): Get name from geometry instance (when available).
  geometries_.emplace(
      geometry_id,
      InternalGeometry(geometry->release_shape(), frame_id, geometry_id,
                       geometry->pose(), engine_index,
                       geometry->visual_material()));


  int child_count = static_cast<int>(frame.get_child_geometries().size());
  if (child_count > 1) {
    // Filter collisions between geometries affixed to the same frame. We only
    // add a clique to a frame's geometries when there are *multiple* child
    // geometries.
    ProximityEngine<T>& engine = *geometry_engine_.get_mutable();
    if (child_count > 2) {
      // Assume all previous geometries have had the clique assigned.
      GeometryStateCollisionFilterAttorney::set_dynamic_geometry_clique(
          &engine, engine_index, frame.clique());
    } else {  // child_count == 2.
      // We *now* have multiple child geometries -- assign to clique.
      for (GeometryId child_id : frame.get_child_geometries()) {
        GeometryIndex child_index = geometries_[child_id].get_engine_index();
        GeometryStateCollisionFilterAttorney::set_dynamic_geometry_clique(
            &engine, child_index, frame.clique());
      }
    }
  }

  // TODO(SeanCurtis-TRI): Enforcing the invariant that the indices are
  // compactly distributed. Is there a more robust way to do this?
  DRAKE_ASSERT(static_cast<int>(X_FG_.size()) == engine_index);
  DRAKE_ASSERT(static_cast<int>(geometry_index_id_map_.size()) - 1 ==
               engine_index);
  X_WG_.push_back(Isometry3<T>::Identity());
  X_FG_.emplace_back(geometry->pose());
  return geometry_id;
}

template <typename T>
GeometryId GeometryState<T>::RegisterGeometryWithParent(
    SourceId source_id, GeometryId geometry_id,
    std::unique_ptr<GeometryInstance> geometry) {
  // There are three error conditions in the doxygen:.
  //    1. geometry == nullptr,
  //    2. source_id is not a registered source, and
  //    3. geometry_id doesn't belong to source_id.
  //
  // Only #1 is tested directly. #2 and #3 are tested implicitly during the act
  // of registering the geometry.

  if (geometry == nullptr) {
    throw std::logic_error(
        "Registering null geometry to geometry " + to_string(geometry_id) +
            ", on source " + to_string(source_id) + ".");
  }

  // This confirms that geometry_id exists at all.
  InternalGeometry& parent_geometry =
      GetMutableValueOrThrow(geometry_id, &geometries_);
  FrameId frame_id = parent_geometry.get_frame_id();

  // This implicitly confirms that source_id is registered (condition #2) and
  // that frame_id belongs to source_id. By construction, geometry_id must
  // belong to the same source as frame_id, so this tests  condition #3.
  GeometryId new_id = RegisterGeometry(source_id, frame_id, move(geometry));

  // RegisterGeometry stores X_PG into X_FG_ (having assumed that  the
  // parent was a frame). This replaces the stored X_PG value with the
  // semantically correct value X_FG by concatenating X_FP with X_PG.

  // Transform pose relative to geometry, to pose relative to frame.
  const InternalGeometry& new_geometry = geometries_[new_id];
  // The call to `RegisterGeometry()` above stashed the pose X_PG into the
  // X_FG_ vector.
  const Isometry3<double>& X_PG = X_FG_[new_geometry.get_engine_index()];
  const Isometry3<double>& X_FP = X_FG_[parent_geometry.get_engine_index()];
  X_FG_[new_geometry.get_engine_index()] = X_FP * X_PG;

  geometries_[new_id].set_parent_id(geometry_id);
  parent_geometry.add_child(new_id);
  return new_id;
}

template <typename T>
GeometryId GeometryState<T>::RegisterAnchoredGeometry(
    SourceId source_id,
    std::unique_ptr<GeometryInstance> geometry) {
  if (geometry == nullptr) {
    throw std::logic_error(
        "Registering null anchored geometry on source "
        + to_string(source_id) + ".");
  }

  GeometryId geometry_id = geometry->id();
  if (anchored_geometries_.count(geometry_id) > 0) {
    throw std::logic_error(
        "Registering anchored geometry with an id that has already been "
        "registered: " +
        to_string(geometry_id));
  }

  auto& set = GetMutableValueOrThrow(source_id, &source_anchored_geometry_map_);

  set.emplace(geometry_id);

  // Pass the geometry to the engine.
  auto engine_index = geometry_engine_->AddAnchoredGeometry(geometry->shape(),
                                                            geometry->pose());
  DRAKE_ASSERT(static_cast<int>(anchored_geometry_index_id_map_.size()) ==
               engine_index);
  anchored_geometry_index_id_map_.push_back(geometry_id);
  anchored_geometries_.emplace(
      geometry_id,
      InternalAnchoredGeometry(
          geometry->release_shape(), geometry_id, geometry->pose(),
          engine_index, geometry->visual_material()));
  return geometry_id;
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
  // Geometry could be anchored. This also implicitly tests that source_id is
  // valid and throws an exception if not.
  const auto& anchored_geometries =
      GetValueOrThrow(source_id, source_anchored_geometry_map_);
  if (anchored_geometries.find(geometry_id) != anchored_geometries.end()) {
    return true;
  }
  // If not anchored, geometry must be dynamic. If this fails, the geometry_id
  // is not valid and an exception is thrown.
  const auto& geometry = GetValueOrThrow(geometry_id, geometries_);
  return BelongsToSource(geometry.get_frame_id(), source_id);
}

template <typename T>
FrameId GeometryState<T>::GetFrameId(GeometryId geometry_id) const {
  const auto& geometry = GetValueOrThrow(geometry_id, geometries_);
  return geometry.get_frame_id();
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
  std::unordered_set<AnchoredGeometryIndex> anchored;
  CollectIndices(set, &dynamic, &anchored);

  geometry_engine_->ExcludeCollisionsWithin(dynamic, anchored);
}

template <typename T>
void GeometryState<T>::ExcludeCollisionsBetween(const GeometrySet& setA,
                                                const GeometrySet& setB) {
  std::unordered_set<GeometryIndex> dynamic1;
  std::unordered_set<AnchoredGeometryIndex> anchored1;
  CollectIndices(setA, &dynamic1, &anchored1);
  std::unordered_set<GeometryIndex> dynamic2;
  std::unordered_set<AnchoredGeometryIndex> anchored2;
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
    std::unordered_set<AnchoredGeometryIndex>* anchored) {
  for (auto frame_id : geometry_set.frames()) {
    auto iterator = frames_.find(frame_id);
    if (iterator == frames_.end()) {
      throw std::logic_error(
          "Geometry set includes a frame id that doesn't belong to the "
          "SceneGraph: " + to_string(frame_id));
    }

    const auto& frame = iterator->second;
    for (auto geometry_id : frame.get_child_geometries()) {
      dynamic->insert(geometries_[geometry_id].get_engine_index());
    }
  }

  for (auto geometry_id : geometry_set.geometries()) {
    if (geometries_.count(geometry_id) == 1) {
      dynamic->insert(geometries_[geometry_id].get_engine_index());
    } else if (anchored_geometries_.count(geometry_id) == 1) {
      anchored->insert(anchored_geometries_[geometry_id].get_engine_index());
    } else {
      throw std::logic_error(
          "Geometry set includes a geometry id that doesn't belong to the "
          "SceneGraph: " + to_string(geometry_id));
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
  return frame.get_source_id();
}

template <typename T>
void GeometryState<T>::UpdatePosesRecursively(
    const internal::InternalFrame& frame, const Isometry3<T>& X_WP,
    const FramePoseVector<T>& poses) {
  const auto frame_id = frame.get_id();
  const auto& X_PF = poses.value(frame_id);
  // Cache this transform for later use.
  X_PF_[frame.get_pose_index()] = X_PF;
  Isometry3<T> X_WF = X_WP * X_PF;
  // TODO(SeanCurtis-TRI): Replace this when we have a transform object that
  // allows proper multiplication between an AutoDiff type and a double type.
  // For now, it allows me to perform the multiplication by multiplying the
  // fully-defined transformation (with [0 0 0 1] on the bottom row).
  X_WF.makeAffine();
  X_WF_[frame.get_pose_index()] = X_WF;

  // Update the geometry which belong to *this* frame.
  for (auto child_id : frame.get_child_geometries()) {
    auto& child_geometry = geometries_[child_id];
    auto child_index = child_geometry.get_engine_index();
    // TODO(SeanCurtis-TRI): See note above about replacing this when we have a
    // transform that supports autodiff * double.
    X_FG_[child_index].makeAffine();
    // TODO(SeanCurtis-TRI): These matrix() shenanigans are here because I can't
    // assign a an Isometry3<double> to an Isometry3<AutoDiffXd>. Replace this
    // when I can.
    X_WG_[child_index].matrix() = X_WF.matrix() * X_FG_[child_index].matrix();
  }

  // Update each child frame.
  for (auto child_id : frame.get_child_frames()) {
    auto& child_frame = frames_[child_id];
    UpdatePosesRecursively(child_frame, X_WF, poses);
  }
}

}  // namespace geometry
}  // namespace drake

// TODO(SeanCurtis-TRI): Currently assumes that "non-symbolic" implies
// AutoDiffXd. Update things appropriately when more non-symbolic scalars
// are available.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::GeometryState)
