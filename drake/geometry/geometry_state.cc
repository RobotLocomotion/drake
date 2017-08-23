#include "drake/geometry/geometry_state.h"

#include <algorithm>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"

namespace drake {
namespace geometry {

using internal::InternalAnchoredGeometry;
using internal::InternalFrame;
using internal::InternalGeometry;
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
GeometryState<T>::GeometryState() {}

template <typename T>
bool GeometryState<T>::source_is_registered(SourceId source_id) const {
  return source_frame_id_map_.find(source_id) != source_frame_id_map_.end();
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
  FrameId frame_id = FrameId::get_new_id();

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
  X_PF_.emplace_back(frame.get_pose());
  DRAKE_ASSERT(pose_index == static_cast<int>(pose_index_to_frame_map_.size()));
  pose_index_to_frame_map_.push_back(frame_id);
  f_set.insert(frame_id);
  frames_.emplace(
      frame_id, InternalFrame(source_id, frame_id, frame.get_name(),
                              frame.get_frame_group(), pose_index, parent_id));
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
  FrameIdSet& set = GetMutableValueOrThrow(source_id, &source_frame_id_map_);

  FindOrThrow(frame_id, set, [frame_id, source_id]() {
    return "Referenced frame " + to_string(frame_id) + " for source " +
        to_string(source_id) + ", but the frame doesn't belong to the source.";
  });

  GeometryId geometry_id = GeometryId::get_new_id();
  geometry_index_id_map_.push_back(geometry_id);

  // TODO(SeanCurtis-TRI): Replace this stub engine index with a call to the
  // geometry engine, storing the engine index it actually stores.
  GeometryIndex engine_index(X_FG_.size());

  // Configure topology.
  frames_[frame_id].add_child(geometry_id);
  // TODO(SeanCurtis-TRI): Get name from geometry instance (when available).
  geometries_.emplace(
      geometry_id,
      InternalGeometry(geometry->release_shape(), frame_id, geometry_id,
                       geometry->get_pose(), engine_index));
  // TODO(SeanCurtis-TRI): Enforcing the invariant that the indexes are
  // compactly distributed. Is there a more robust way to do this?
  DRAKE_ASSERT(static_cast<int>(X_FG_.size()) == engine_index);
  DRAKE_ASSERT(static_cast<int>(geometry_index_id_map_.size()) - 1 ==
               engine_index);
  X_WG_.push_back(Isometry3<T>::Identity());
  X_FG_.emplace_back(geometry->get_pose());
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
  Isometry3<T> X_PG = X_FG_[new_geometry.get_engine_index()];
  Isometry3<T> X_FG =
      X_FG_[parent_geometry.get_engine_index()] * X_PG;
  X_FG_[new_geometry.get_engine_index()] = X_FG;

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
  auto& set = GetMutableValueOrThrow(source_id, &source_anchored_geometry_map_);

  GeometryId geometry_id = GeometryId::get_new_id();
  set.emplace(geometry_id);

  // TODO(SeanCurtis-TRI): Replace this stub engine index with a call to the
  // geometry engine, storing the engine index it actually stores.
  AnchoredGeometryIndex engine_index(anchored_geometry_index_id_map_.size());

  // Note: This test will be more meaningful when the engine is included at this
  // enforces an invariant between *two* structures rather than being a
  // simple tautology.
  DRAKE_ASSERT(static_cast<int>(anchored_geometry_index_id_map_.size()) ==
               engine_index);
  anchored_geometry_index_id_map_.push_back(geometry_id);
  anchored_geometries_.emplace(
      geometry_id,
      InternalAnchoredGeometry(geometry->release_shape(), geometry_id,
                               geometry->get_pose(), engine_index));
  return geometry_id;
}

template <typename T>
void GeometryState<T>::ClearSource(SourceId source_id) {
  FrameIdSet& frames = GetMutableValueOrThrow(source_id, &source_frame_id_map_);
  for (auto frame_id : frames) {
    RemoveFrameUnchecked(frame_id, RemoveFrameOrigin::kSource);
  }
  source_frame_id_map_[source_id].clear();
  source_root_frame_map_[source_id].clear();
}

template <typename T>
void GeometryState<T>::RemoveFrame(SourceId source_id, FrameId frame_id) {
  if (!BelongsToSource(frame_id, source_id)) {
    throw std::logic_error("Trying to remove frame " + to_string(frame_id) +
        " from source " + to_string(source_id) +
        ", but the frame doesn't belong to that source.");
  }
  RemoveFrameUnchecked(frame_id, RemoveFrameOrigin::kFrame);
}

template <typename T>
void GeometryState<T>::RemoveGeometry(SourceId source_id,
                                      GeometryId geometry_id) {
  if (!BelongsToSource(geometry_id, source_id)) {
    throw std::logic_error(
        "Trying to remove geometry " + to_string(geometry_id) + " from "
            "source " + to_string(source_id) + ", but the geometry doesn't "
            "belong to that source.");
  }
  if (is_dynamic(geometry_id)) {
    RemoveGeometryUnchecked(geometry_id, RemoveGeometryOrigin::kGeometry);
  } else {
    RemoveAnchoredGeometryUnchecked(geometry_id);
  }
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
SourceId GeometryState<T>::get_source_id(FrameId frame_id) const {
  const auto& frame = GetValueOrThrow(frame_id, frames_);
  return frame.get_source_id();
}

template <typename T>
void GeometryState<T>::RemoveFrameUnchecked(FrameId frame_id,
                                            RemoveFrameOrigin caller) {
  auto& frame = GetMutableValueOrThrow(frame_id, &frames_);

  if (caller != RemoveFrameOrigin::kSource) {
    // Recursively delete the child frames.
    for (auto child_id : *frame.get_mutable_child_frames()) {
      RemoveFrameUnchecked(child_id, RemoveFrameOrigin::kRecurse);
    }

    // Remove the frames from the source.
    SourceId source_id = frame.get_source_id();
    auto& frame_set = GetMutableValueOrThrow(source_id, &source_frame_id_map_);
    frame_set.erase(frame_id);
    // This assumes that source_id in source_frame_id_map_ implies the existence
    // of an id set in source_root_frame_map_. It further relies on the
    // behavior that erasing a non-member of the set does nothing.
    source_root_frame_map_[source_id].erase(frame_id);
  }

  // Recursively delete the child geometries.
  for (auto child_id : *frame.get_mutable_child_geometries()) {
    RemoveGeometryUnchecked(child_id, RemoveGeometryOrigin::kFrame);
  }

  // Don't leave holes in the pose vectors. Rewire pose indices by taking the
  // frame with the last pose index and moving it to the newly vacated hold.
  // We do *not* copy the values in X_PF_ because changes to the topology
  // renders these values meaningless until recomputed.
  auto pose_index = frame.get_pose_index();
  PoseIndex last_index(static_cast<int>(X_PF_.size()) - 1);
  if (pose_index < last_index) {
    FrameId moved_id = pose_index_to_frame_map_[last_index];
    frames_[moved_id].set_pose_index(pose_index);
  }
  X_PF_.pop_back();

  if (caller == RemoveFrameOrigin::kFrame) {
    // Only the root needs to explicitly remove itself from a possible parent
    // frame.
    FrameId parent_frame_id = frame.get_parent_frame_id();
    if (parent_frame_id != InternalFrame::get_world_frame_id()) {
      auto& parent_frame = GetMutableValueOrThrow(parent_frame_id, &frames_);
      parent_frame.remove_child(frame_id);
    }
  }

  // Remove from the frames.
  frames_.erase(frame_id);
}

template <typename T>
void GeometryState<T>::RemoveGeometryUnchecked(GeometryId geometry_id,
                                               RemoveGeometryOrigin caller) {
  const InternalGeometry& geometry = GetValueOrThrow(geometry_id, geometries_);

  if (caller != RemoveGeometryOrigin::kFrame) {
    // Clear children
    for (auto child_id : geometry.get_child_geometry_ids()) {
      RemoveGeometryUnchecked(child_id, RemoveGeometryOrigin::kRecurse);
    }

    // Remove the geometry from its frame's list of geometries.
    auto& frame = GetMutableValueOrThrow(geometry.get_frame_id(), &frames_);
    frame.remove_child(geometry_id);
  }

  GeometryIndex engine_index = geometry.get_engine_index();

  // TODO(SeanCurtis-TRI): This simulates removal from the geometry engine where
  // the *last* geometry is swapped with the removed geometry to maintain
  // spatial coherency.
  optional<GeometryIndex> moved_index{
      GeometryIndex(static_cast<int>(geometry_index_id_map_.size()) - 1)};

  if (moved_index) {
    // The geometry engine moved a geometry into the removed `engine_index`.
    // Update the state's knowledge of this.
    GeometryId moved_id = geometry_index_id_map_[*moved_index];
    geometries_[moved_id].set_engine_index(engine_index);

    geometry_index_id_map_[engine_index] = moved_id;

    X_FG_[engine_index] = X_FG_[*moved_index];
  }
  // Trim the vectors for these removed geometries.
  X_FG_.pop_back();
  // NOTE: we are not obliged to copy the value from moved_index to
  // pose_index in X_WG_. This is a computed value that will live in the cache.
  // Changing the topology will dirty the cache so it will be recomputed
  // before being provided next.
  X_WG_.pop_back();

  // Always pop the last; we assume that either the geometry removed is already
  // the last, or has been swapped into the last.
  geometry_index_id_map_.pop_back();

  if (caller == RemoveGeometryOrigin::kGeometry) {
    // Only the root needs to explicitly remove itself from a possible parent
    // geometry.
    if (optional<GeometryId> parent_id = geometry.get_parent_id()) {
      auto& parent_geometry =
          GetMutableValueOrThrow(*parent_id, &geometries_);
      parent_geometry.remove_child(geometry_id);
    }
  }

  // Remove from the geometries.
  geometries_.erase(geometry_id);
}

template <typename T>
void GeometryState<T>::RemoveAnchoredGeometryUnchecked(GeometryId geometry_id) {
  const auto& geometry = GetValueOrThrow(geometry_id, anchored_geometries_);
  auto engine_index = geometry.get_engine_index();

  // TODO(SeanCurtis-TRI): This simulates removal from the geometry engine where
  // the *last* geometry is swapped with the removed geometry to maintain
  // spatial coherency.
  optional<AnchoredGeometryIndex> moved_index{AnchoredGeometryIndex(
      static_cast<int>(anchored_geometry_index_id_map_.size()) - 1)};

  if (moved_index) {
    GeometryId moved_id = anchored_geometry_index_id_map_[*moved_index];
    anchored_geometries_[moved_id].set_engine_index(engine_index);
    anchored_geometry_index_id_map_[engine_index] = moved_id;
  }
  anchored_geometry_index_id_map_.pop_back();
  anchored_geometries_.erase(geometry_id);
}

// Explicitly instantiates on the most common scalar types.
template class GeometryState<double>;

}  // namespace geometry
}  // namespace drake

