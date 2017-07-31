#include "drake/geometry/geometry_state.h"

#include <algorithm>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include "drake/common/unused.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"

namespace drake {
namespace geometry {

using internal::InternalFrame;
using std::make_pair;
using std::make_unique;
using std::move;

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
                 std::function<std::string()> make_message) {
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
                       const std::unordered_map<Key, Value>* map) {
  auto itr = map->find(key);
  if (itr != map->end()) {
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

//-----------------------------------------------------------------------------

template <typename T>
GeometryState<T>::GeometryState() {}

template <typename T>
bool GeometryState<T>::source_is_registered(SourceId source_id) const {
  return source_frame_id_map_.find(source_id) != source_frame_id_map_.end();
}

template <typename T>
const std::string& GeometryState<T>::get_source_name(SourceId id) const {
  using std::to_string;
  auto itr = source_names_.find(id);
  if (itr != source_names_.end()) return itr->second;
  throw std::logic_error(
      "Querying source name for an invalid source id: " + to_string(id) + ".");
}

template <typename T>
SourceId GeometryState<T>::RegisterNewSource(const std::string& name) {
  SourceId source_id = SourceId::get_new_id();
  using std::to_string;
  const std::string final_name =
      name != "" ? name : "Source_" + to_string(source_id);

  // The user can provide bad names, _always_ test.
  for (const auto &pair : source_names_) {
    if (pair.second == final_name) {
      throw std::logic_error(
          "Registering new source with duplicate name: " + final_name + ".");
    }
  }

  source_frame_id_map_[source_id];
  source_root_frame_map_[source_id];
  source_names_[source_id] = final_name;
  return source_id;
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
  using std::to_string;
  if (!BelongsToSource(frame_id, source_id)) {
    throw std::logic_error("Trying to remove frame " + to_string(frame_id) +
                           " from source " + to_string(source_id) +
                           ", but the frame doesn't belong to that source.");
  }
  RemoveFrameUnchecked(frame_id, RemoveFrameOrigin::kFrame);
}

template <typename T>
FrameId GeometryState<T>::RegisterFrame(SourceId source_id,
                                        const GeometryFrame<T>& frame) {
  return RegisterFrame(source_id, InternalFrame::get_world_frame_id(), frame);
}

template <typename T>
FrameId GeometryState<T>::RegisterFrame(SourceId source_id, FrameId parent_id,
                                        const GeometryFrame<T>&) {
  using std::to_string;
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
  f_set.insert(frame_id);
  frames_.emplace(frame_id,
                  InternalFrame(source_id, frame_id, parent_id));
  return frame_id;
}

template <typename T>
bool GeometryState<T>::BelongsToSource(FrameId frame_id,
                                       SourceId source_id) const {
  // Confirm that the source_id is valid.
  const auto& frames = GetValueOrThrow(source_id, &source_frame_id_map_);
  unused(frames);
  // If valid, test the frame.
  return get_source_id(frame_id) == source_id;
}

template <typename T>
const FrameIdSet& GeometryState<T>::GetFramesForSource(
    SourceId source_id) const {
  return GetValueOrThrow(source_id, &source_frame_id_map_);
}

template <typename T>
SourceId GeometryState<T>::get_source_id(FrameId frame_id) const {
  auto& frame = GetValueOrThrow(frame_id, &frames_);
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

  // TODO(SeanCurtis-TRI): Remove geometries.

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

// Explicitly instantiates on the most common scalar types.
template class GeometryState<double>;

}  // namespace geometry
}  // namespace drake

