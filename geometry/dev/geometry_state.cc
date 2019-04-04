#include "drake/geometry/dev/geometry_state.h"

#include <algorithm>
#include <functional>
#include <iterator>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/autodiff.h"
#include "drake/common/default_scalars.h"
#include "drake/geometry/dev/geometry_roles.h"
#include "drake/geometry/dev/proximity_engine.h"
#include "drake/geometry/dev/render/render_engine_vtk.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/internal_frame.h"
#include "drake/geometry/internal_geometry.h"
#include "drake/geometry/test_utilities/geometry_set_tester.h"
#include "drake/geometry/utilities.h"

namespace drake {
namespace geometry {

// This exploits the `GeometryStateTester` to dig into the internals of the
// geometry::GeometryState, in the name of converting from geometry to
// geometry::dev. This is, by its very nature, hacky, but it allows the
// existence of this dev folder without modifying the master branch to
// facilitate an otherwise temporary collection of code.
template <class T>
class GeometryStateTester {
 public:
  explicit GeometryStateTester(const geometry::GeometryState<T>& state)
      : state_(state) {}

  const std::unordered_map<SourceId, FrameIdSet>& source_frame_id_map() const {
    return state_.source_frame_id_map_;
  }

  const std::unordered_map<SourceId, std::string>& source_names() const {
    return state_.source_names_;
  }

  const std::unordered_map<SourceId, std::unordered_set<GeometryId>>&
  source_anchored_geometry_map() const {
    return state_.source_anchored_geometry_map_;
  }

  const std::unordered_map<FrameId, geometry::internal::InternalFrame>&
  frames() const {
    return state_.frames_;
  }

  const std::unordered_map<GeometryId, geometry::internal::InternalGeometry>&
  geometries() const {
    return state_.geometries_;
  }

  SourceId GetSourceId(GeometryId geometry_id) {
    return state_.get_source_id(state_.GetFrameId(geometry_id));
  }

 private:
  const geometry::GeometryState<T>& state_;
};

namespace dev {

using internal::GeometryStateCollisionFilterAttorney;
using internal::InternalFrame;
using internal::InternalGeometry;
using internal::ProximityEngine;
using std::make_pair;
using std::make_unique;
using std::move;
using std::to_string;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

namespace {

// TODO(SeanCurtis-TRI): This is copied from proximity_engine.cc. Refactor this
// into a single location for re-use.

// ADL-reliant helper functions for converting Isometry<T> to Isometry<double>.
const Isometry3<double>& convert(const Isometry3<double>& transform) {
  return transform;
}

template <class VectorType>
Isometry3<double> convert(
    const Isometry3<Eigen::AutoDiffScalar<VectorType>>& transform) {
  Isometry3<double> result;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      result.matrix()(r, c) = ExtractDoubleOrThrow(transform.matrix()(r, c));
    }
  }
  return result;
}

}  // namespace

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
  // Some arbitrary design choices:
  //   - The world frame is its own parent.
  //   - Its "frame group" (aka model instance id) is some big number.
  //   - the default clique group is also an arbitrarily large number.
  frames_[world] = InternalFrame(self_source_, world, "world", 1234567,
                                 InternalIndex(0), world, 1234567);
  frame_index_to_frame_map_.push_back(world);
  X_WF_.push_back(Isometry3<T>::Identity());
  X_PF_.push_back(Isometry3<T>::Identity());

  source_frame_id_map_[self_source_] = {world};
  source_root_frame_map_[self_source_] = {world};
}

template <typename T>
GeometryState<T>& GeometryState<T>::operator=(
    const geometry::GeometryState<T>& state) {
  geometry::GeometryStateTester<T> tester(state);

  // TODO(SeanCurtis-TRI): Consider refactoring this -- I have two pieces of
  // code which order entities in a top-down manner and two pieces of code that
  // register geometry. The duplication is left in place because this is a dev
  // folder that will die when 9540 is complete.

  // First register all sources and frames.
  for (const auto& pair : tester.source_frame_id_map()) {
    // First register the source. NOTE: This is a copy-and-paste of the
    // post-validation work done by RegisterNewSource.
    SourceId source_id = pair.first;
    RegisterValidSource(source_id, tester.source_names().at(source_id));

    auto& frame_set = source_frame_id_map_[source_id];

    // Register all of the frames for this source. This must be done in a "top"
    // down manner; i.e., before a frame is registered, its parent must be
    // registered. So, we'll order the frames appropriately.

    // Initialize by putting all of the frames *known* to be fixed to the world
    // first in the ordered set and all others in the unprocessed set.
    std::unordered_set<FrameId> unprocessed;
    std::vector<FrameId> ordered_frames;
    const auto& all_frame_ids = pair.second;
    for (FrameId frame_id : all_frame_ids) {
      const geometry::internal::InternalFrame& source_frame =
          tester.frames().at(frame_id);
      if (source_frame.parent_frame_id() == InternalFrame::world_frame_id()) {
        ordered_frames.push_back(frame_id);
      } else {
        unprocessed.insert(frame_id);
      }
    }

    // Now iterate through the unprocessed frames and order them.

    // Determine if the given frame id's parent has already been added to the
    // ordered set.
    auto has_ordered_parent = [&tester, &ordered_frames](FrameId frame_id) {
      const geometry::internal::InternalFrame& source_frame =
          tester.frames().at(frame_id);
      auto iter = std::find(ordered_frames.begin(), ordered_frames.end(),
          source_frame.parent_frame_id());
      return iter != ordered_frames.end();
    };

    while (!unprocessed.empty()) {
      std::unordered_set<FrameId> moved;
      for (FrameId frame_id : unprocessed) {
        if (has_ordered_parent(frame_id)) {
          ordered_frames.push_back(frame_id);
          moved.insert(frame_id);
        }
      }
      for (FrameId frame_id : moved) {
        unprocessed.erase(frame_id);
      }
      // *Some* progress must always be made.
      DRAKE_DEMAND(moved.size() > 0);
    }

    // Now register the frames in a top-down order.
    for (FrameId frame_id : ordered_frames) {
      const geometry::internal::InternalFrame& source_frame =
          tester.frames().at(frame_id);
      RegisterValidFrame(source_id, frame_id, source_frame.name(),
                         source_frame.frame_group(),
                         source_frame.parent_frame_id(), source_frame.clique(),
                         &frame_set);
    }
  }

  // Register *dynamic* geometries. Like frames, geometries can be registered on
  // other geometries and they need to be registered *in order*.
  std::unordered_set<GeometryId> unprocessed_geometries;
  std::vector<GeometryId> ordered_geometry;

  // Partition geometries between those that have parent geometries into the
  // unprocessed_geometries and those that don't into ordered_geometries.
  for (const auto& pair : tester.geometries()) {
    GeometryId id = pair.first;
    const geometry::internal::InternalGeometry& geometry =
        tester.geometries().at(id);
    if (!geometry.parent_id()) {
      ordered_geometry.push_back(id);
    } else {
      unprocessed_geometries.insert(id);
    }
  }

  // Determine if the given frame id's parent has already been added to the
  // ordered set.
  auto has_ordered_geometry_parent = [&tester,
                                      &ordered_geometry](GeometryId id) {
    const geometry::internal::InternalGeometry& geometry =
        tester.geometries().at(id);
    auto iter = std::find(ordered_geometry.begin(), ordered_geometry.end(),
                          geometry.parent_id());
    return iter != ordered_geometry.end();
  };

  while (!unprocessed_geometries.empty()) {
    std::unordered_set<GeometryId> moved;
    for (GeometryId id : unprocessed_geometries) {
      if (has_ordered_geometry_parent(id)) {
        ordered_geometry.push_back(id);
        moved.insert(id);
      }
    }
    for (GeometryId id : moved) {
      unprocessed_geometries.erase(id);
    }
    // *Some* progress must always be made.
    DRAKE_DEMAND(moved.size() > 0);
  }

  // Now register the geometries in a top-down order.
  for (GeometryId geometry_id : ordered_geometry) {
    const geometry::internal::InternalGeometry& geometry =
        tester.geometries().at(geometry_id);
    // NOTE: Geometry only gets registered into *this* scene graph if it is
    // "visualizable" (which means has an illustration role in the main
    // scene graph.
    if (geometry.has_illustration_role()) {
      internal::InternalGeometry* parent_geometry = nullptr;
      if (geometry.parent_id()) {
        GeometryId parent_id = *geometry.parent_id();
        parent_geometry = &geometries_.at(parent_id);
      }
      // NOTE: This *could* fail if a geometry with non-zero alpha is parented
      // to a geometry that has zero-alpha. We assume that for the lifespan of
      // this dev class, it won't come up. However, this is documented in
      // the accompanying README.md.
      IllustrationProperties properties;
      properties.AddGroup("phong");
      properties.AddProperty("phong", "diffuse",
          geometry.illustration_properties()->GetPropertyOrDefault(
              "phong", "diffuse", Vector4<double>(0.9, 0.9, 0.9, 1.0)));
      const SourceId source_id = tester.GetSourceId(geometry_id);
      RegisterValidGeometry(source_id,
                            geometry.frame_id(), geometry_id,
                            geometry.shape().Clone(), geometry.name(),
                            geometry.X_PG(), properties, parent_geometry);
    }
  }

  return *this;
}

template <typename T>
int GeometryState<T>::GetNumGeometriesWithRole(Role role) const {
  int count = 0;
  for (const auto& pair : geometries_) {
    if (pair.second.has_role(role)) ++count;
  }
  return count;
}

template <typename T>
int GeometryState<T>::GetNumFrameGeometries(FrameId frame_id) const {
  const InternalFrame& frame = GetValueOrThrow(frame_id, frames_);
  return static_cast<int>(frame.child_geometries().size());
}

template <typename T>
int GeometryState<T>::GetNumFrameGeometriesWithRole(FrameId frame_id,
                                                    Role role) const {
  const InternalFrame& frame = GetValueOrThrow(frame_id, frames_);
  int count = 0;
  for (GeometryId geometry_id : frame.child_geometries()) {
    if (geometries_.at(geometry_id).has_role(role)) ++count;
  }
  return count;
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
    FrameId frame_id, Role role, const std::string& name) const {
  const std::string canonical_name =
      geometry::internal::CanonicalizeStringName(name);

  GeometryId result;
  int count = 0;
  std::string frame_name;

  const InternalFrame& frame = GetValueOrThrow(frame_id, frames_);
  frame_name = frame.name();
  for (GeometryId geometry_id : frame.child_geometries()) {
    const InternalGeometry& geometry = geometries_.at(geometry_id);
    if (geometry.has_role(role) && geometry.name() == canonical_name) {
      ++count;
      result = geometry_id;
    }
  }

  if (count == 1) return result;
  if (count < 1) {
    throw std::logic_error("The frame '" + frame_name + "' (" +
        to_string(frame_id) + ") has no geometry with the role '" +
        to_string(role) + "' and the canonical name '" + canonical_name + "'");
  }
  // This case should only be possible for unassigned geometries.
  DRAKE_DEMAND(role == Role::kUnassigned);
  throw std::logic_error("The frame '" + frame_name + "' (" +
      to_string(frame_id) + ") has multiple geometries with the role '" +
      to_string(role) + "' and the canonical name '" + canonical_name + "'");
}

template <typename T>
const Isometry3<T>& GeometryState<T>::get_pose_in_world(
    FrameId frame_id) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "No world pose available for invalid frame id: " +
           to_string(frame_id);
  });
  return X_WF_[frames_.at(frame_id).internal_index()];
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
  return X_WG_[geometries_.at(geometry_id).internal_index()];
}

template <typename T>
const Isometry3<T>& GeometryState<T>::get_pose_in_parent(
    FrameId frame_id) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "No pose available for invalid frame id: " + to_string(frame_id);
  });
  return X_PF_[frames_.at(frame_id).internal_index()];
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
const ProximityProperties* GeometryState<T>::get_proximity_properties(
    GeometryId id) const {
  const InternalGeometry* geometry = GetGeometry(id);
  if (geometry != nullptr) return geometry->proximity_properties();
  return nullptr;
}

template <typename T>
const IllustrationProperties* GeometryState<T>::get_illustration_properties(
    GeometryId id) const {
  const InternalGeometry* geometry = GetGeometry(id);
  if (geometry != nullptr) return geometry->illustration_properties();
  return nullptr;
}

template <typename T>
const PerceptionProperties* GeometryState<T>::get_perception_properties(
    GeometryId id) const {
  const InternalGeometry* geometry = GetGeometry(id);
  if (geometry != nullptr) return geometry->perception_properties();
  return nullptr;
}

template <typename T>
int GeometryState<T>::NumGeometryWithRole(FrameId frame_id, Role role) const {
  int count = 0;
  FindOrThrow(frame_id, frames_, [frame_id, role]() {
    return "Cannot report number of geometries with the " + to_string(role) +
        " role for invalid frame id: " + to_string(frame_id);
  });
  const InternalFrame& frame = frames_.at(frame_id);
  for (GeometryId id : frame.child_geometries()) {
    if (geometries_.at(id).has_role(role)) ++count;
  }
  return count;
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

  RegisterValidSource(source_id, final_name);
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

  int clique = GeometryStateCollisionFilterAttorney::get_next_clique(
      geometry_engine_.get_mutable());
  RegisterValidFrame(source_id, frame_id, frame.name(), frame.frame_group(),
                     parent_id, clique, &f_set);
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

  IllustrationProperties properties;
  if (geometry->illustration_properties() != nullptr) {
    properties.AddGroup("phong");
    properties.AddProperty(
        "phong", "diffuse",
        geometry->illustration_properties()->GetPropertyOrDefault(
            "phong", "diffuse", Vector4<double>(0.9, 0.9, 0.9, 1.0)));
  }
  RegisterValidGeometry(source_id, frame_id, geometry_id,
                        geometry->release_shape(), geometry->name(),
                        geometry->pose(), properties, nullptr);

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

  GeometryId new_id = geometry->id();
  IllustrationProperties properties;
  if (geometry->illustration_properties() != nullptr) {
    properties.AddGroup("phong");
    properties.AddProperty(
        "phong", "diffuse",
        geometry->illustration_properties()->GetPropertyOrDefault(
            "phong", "diffuse", Vector4<double>(0.9, 0.9, 0.9, 1.0)));
  }
  RegisterValidGeometry(source_id, frame_id, new_id,
                        geometry->release_shape(), geometry->name(),
                        geometry->pose(), properties, &parent_geometry);

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
    FrameId frame_id, Role role, const std::string& candidate_name) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "Given frame id is not valid: " + to_string(frame_id);
  });
  const std::string name =
      geometry::internal::CanonicalizeStringName(candidate_name);
  if (name.empty()) return false;
  return NameIsUnique(frame_id, role, name);
}

template <typename T>
void GeometryState<T>::AssignRole(SourceId source_id,
                                  GeometryId geometry_id,
                                  ProximityProperties properties) {
  AssignRoleInternal(source_id, geometry_id, std::move(properties),
                     Role::kProximity);

  InternalGeometry* geometry = GetMutableGeometry(geometry_id);
  // This *must* be no-null, otherwise the role assignment would have failed.
  DRAKE_DEMAND(geometry != nullptr);

  if (geometry->is_dynamic()) {
    // Pass the geometry to the engine.
    const InternalIndex internal_index = geometry->internal_index();
    ProximityIndex index =
        geometry_engine_->AddDynamicGeometry(geometry->shape(), internal_index);
    geometry->set_proximity_index(index);
    DRAKE_DEMAND(static_cast<int>(X_WG_proximity_.size()) == index);
    X_WG_proximity_.push_back(geometry->internal_index());

    InternalFrame& frame = frames_[geometry->frame_id()];

    int child_count = static_cast<int>(frame.child_geometries().size());
    if (child_count > 1) {
      // Multiple children does *not* imply the need for collision filtering.
      // Only operate on those children which have a proximity role.
      // TODO(SeanCurtis-TRI): Perhaps refactor this elsewhere?
      std::vector<GeometryId> proximity_geometries;
      proximity_geometries.reserve(child_count);
      std::copy_if(frame.child_geometries().begin(),
                   frame.child_geometries().end(),
                   std::back_inserter(proximity_geometries),
                   [this](GeometryId id) {
                     return geometries_[id].has_proximity_role();
                   });
      int proximity_count = static_cast<int>(proximity_geometries.size());

      if (proximity_count > 1) {
        // Filter collisions between geometries affixed to the same frame. We
        // only add a clique to a frame's geometries when there are *multiple*
        // child geometries.
        ProximityEngine<T>& engine = *geometry_engine_.get_mutable();
        if (proximity_count > 2) {
          // Assume all previous geometries have already had the clique
          // assigned.
          GeometryStateCollisionFilterAttorney::set_dynamic_geometry_clique(
              &engine, internal_index, frame.clique());
        } else {  // proximity_count == 2.
          // We *now* have multiple child geometries with proximity role --
          // assign to clique.
          for (GeometryId child_id : proximity_geometries) {
            InternalIndex child_index = geometries_[child_id].internal_index();
            GeometryStateCollisionFilterAttorney::set_dynamic_geometry_clique(
                &engine, child_index, frame.clique());
          }
        }
      }
    }
  } else {
    // If it's not dynamic, it must be anchored.
    ProximityIndex index = geometry_engine_->AddAnchoredGeometry(
        geometry->shape(), geometry->X_FG(),
        geometry->internal_index());
    geometry->set_proximity_index(index);
  }
}

template <typename T>
void GeometryState<T>::AssignRole(SourceId source_id,
                                  GeometryId geometry_id,
                                  PerceptionProperties properties) {
  AssignRoleInternal(source_id, geometry_id, std::move(properties),
                     Role::kPerception);

  InternalGeometry* geometry = GetMutableGeometry(geometry_id);
  // This *must* be non-null, otherwise the role assignment would have failed.
  DRAKE_DEMAND(geometry != nullptr);

  for (auto& pair : render_engines_) {
    const std::string& renderer_name = pair.first;
    auto& engine = pair.second;
    optional<RenderIndex> index =
        engine->RegisterVisual(geometry->internal_index(), geometry->shape(),
                               *geometry->perception_properties(),
                               geometry->X_FG(), geometry->is_dynamic());
    if (index) geometry->set_render_index(renderer_name, *index);
  }
}

template <typename T>
void GeometryState<T>::AssignRole(SourceId source_id,
                                  GeometryId geometry_id,
                                  IllustrationProperties properties) {
  AssignRoleInternal(source_id, geometry_id, std::move(properties),
                     Role::kIllustration);
  // NOTE: No need to assign to any engines.
}

template <typename T>
int GeometryState<T>::RemoveRole(SourceId source_id, GeometryId geometry_id,
                                 Role role) {
  if (!BelongsToSource(geometry_id, source_id)) {
    throw std::logic_error(
        "Trying to remove the role " + to_string(role) + " from the geometry " +
        to_string(geometry_id) + " from source " + to_string(source_id) +
        ", but the geometry doesn't belong to that source.");
  }

  // dev/SceneGraph doesn't work with proximity, and one can't "remove" the
  // unassigned role state.
  if (role == Role::kUnassigned || role == Role::kProximity) return 0;

  return RemoveRoleUnchecked(geometry_id, role);
}

template <typename T>
int GeometryState<T>::RemoveRole(SourceId source_id, FrameId frame_id,
                                 Role role) {
  int count = 0;
  SourceId frame_source_id = source_id;
  if (frame_id == InternalFrame::world_frame_id()) {
    // Explicitly validate the source id because it won't happen in acquiring
    // the world frame.
    FindOrThrow(source_id, source_frame_id_map_, [source_id]() {
      return get_missing_id_message(source_id);
    });
    frame_source_id = self_source_;
  }
  const FrameIdSet& set = GetMutableValueOrThrow(frame_source_id,
                                           &source_frame_id_map_);
  FindOrThrow(frame_id, set, [frame_id, frame_source_id]() {
    return "Referenced frame " + to_string(frame_id) + " for source " +
        to_string(frame_source_id) +
        ", but the frame doesn't belong to the source.";
  });

  // dev/SceneGraph doesn't work with proximity, and one can't "remove" the
  // unassigned role state.
  if (role == Role::kUnassigned || role == Role::kProximity) return 0;

  const InternalFrame& frame = frames_[frame_id];
  for (GeometryId geometry_id : frame.child_geometries()) {
    // If the frame is the world frame, then the specific geometry needs to be
    // tested to see if it belongs to the source. Otherwise, by definition, the
    // geometry must belong to the same source as the parent frame.
    if (frame_id != InternalFrame::world_frame_id() ||
        BelongsToSource(geometry_id, source_id)) {
      count += RemoveRoleUnchecked(geometry_id, role);
    }
  }
  return count;
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
  GeometrySetTester tester(&set);
  // There is no work to be done if:
  //   1. the set contains a single frame and no geometries -- geometries *on*
  //      that single frame have already been handled, or
  //   2. there are no frames and a single geometry.
  if ((tester.num_frames() == 1 && tester.num_geometries() == 0) ||
      (tester.num_frames() == 0 && tester.num_geometries() == 1)) {
    return;
  }

  std::unordered_set<InternalIndex> dynamic;
  std::unordered_set<InternalIndex> anchored;
  CollectIndices(set, &dynamic, &anchored);

  geometry_engine_->ExcludeCollisionsWithin(dynamic, anchored);
}

template <typename T>
void GeometryState<T>::ExcludeCollisionsBetween(const GeometrySet& setA,
                                                const GeometrySet& setB) {
  std::unordered_set<InternalIndex> dynamic1;
  std::unordered_set<InternalIndex> anchored1;
  CollectIndices(setA, &dynamic1, &anchored1);
  std::unordered_set<InternalIndex> dynamic2;
  std::unordered_set<InternalIndex> anchored2;
  CollectIndices(setB, &dynamic2, &anchored2);

  geometry_engine_->ExcludeCollisionsBetween(dynamic1, anchored1, dynamic2,
                                             anchored2);
}

template <typename T>
void GeometryState<T>::AddRenderer(
    std::string name, std::unique_ptr<render::RenderEngine> renderer) {
  if (geometries_.size() > 0) {
    throw std::logic_error(fmt::format(
        "Error adding renderer '{}'; geometries have already been registered",
        name));
  }
  if (render_engines_.count(name) > 0) {
    throw std::logic_error(
        fmt::format("A renderer with the name '{}' already exists", name));
  }
  render_engines_[move(name)] = move(renderer);
}

template <typename T>
std::vector<std::string> GeometryState<T>::RegisteredRendererNames() const {
  std::vector<std::string> names;
  names.reserve(render_engines_.size());
  for (const auto& pair : render_engines_) {
    names.push_back(pair.first);
  }
  return names;
}

template <typename T>
void GeometryState<T>::RenderColorImage(const render::CameraProperties& camera,
                                        FrameId parent_frame,
                                        const Isometry3<double>& X_PC,
                                        ImageRgba8U* color_image_out,
                                        bool show_window) const {
  // This assumes that the poses in the engine have already been updated.
  render::RenderEngine* engine = GetRenderEngineOrThrow(camera.renderer_name);
  const Isometry3<double> X_WC = GetDoubleWorldPose(parent_frame) * X_PC;
  engine->UpdateViewpoint(X_WC);
  engine->RenderColorImage(camera, color_image_out, show_window);
}

template <typename T>
void GeometryState<T>::RenderDepthImage(
    const render::DepthCameraProperties& camera,
    FrameId parent_frame, const Isometry3<double>& X_PC,
    ImageDepth32F* depth_image_out) const {
  // This assumes that the poses in the engine have already been updated.
  render::RenderEngine* engine = GetRenderEngineOrThrow(camera.renderer_name);
  const Isometry3<double> X_WC = GetDoubleWorldPose(parent_frame) * X_PC;
  engine->UpdateViewpoint(X_WC);
  engine->RenderDepthImage(camera, depth_image_out);
}

template <typename T>
void GeometryState<T>::RenderLabelImage(const render::CameraProperties& camera,
                                        FrameId parent_frame,
                                        const Isometry3<double>& X_PC,
                                        ImageLabel16I* label_image_out,
                                        bool show_window) const {
  // This assumes that the poses in the engine have already been updated.
  render::RenderEngine* engine = GetRenderEngineOrThrow(camera.renderer_name);
  const Isometry3<double> X_WC = GetDoubleWorldPose(parent_frame) * X_PC;
  engine->UpdateViewpoint(X_WC);
  engine->RenderLabelImage(camera, label_image_out, show_window);
}

template <typename T>
std::unique_ptr<GeometryState<AutoDiffXd>> GeometryState<T>::ToAutoDiffXd()
    const {
  return std::unique_ptr<GeometryState<AutoDiffXd>>(
      new GeometryState<AutoDiffXd>(*this));
}

template <typename T>
void GeometryState<T>::CollectIndices(
    const GeometrySet& geometry_set, std::unordered_set<InternalIndex>* dynamic,
    std::unordered_set<InternalIndex>* anchored) {
  GeometrySetTester tester(&geometry_set);
  std::unordered_set<InternalIndex>* target;
  for (auto frame_id : tester.frames()) {
    const auto& frame = GetValueOrThrow(frame_id, frames_);
    target = frame.is_world() ? anchored : dynamic;
    for (auto geometry_id : frame.child_geometries()) {
      InternalGeometry& geometry = geometries_[geometry_id];
      if (geometry.has_proximity_role()) {
        target->insert(geometry.internal_index());
      }
    }
  }

  for (auto geometry_id : tester.geometries()) {
    const InternalGeometry* geometry = GetGeometry(geometry_id);
    if (geometry == nullptr) {
      throw std::logic_error(
          "Geometry set includes a geometry id that doesn't belong to the "
          "SceneGraph: " + to_string(geometry_id));
    }
    if (geometry->has_proximity_role()) {
      if (geometry->is_dynamic()) {
        dynamic->insert(geometry->internal_index());
      } else {
        anchored->insert(geometry->internal_index());
      }
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
void GeometryState<T>::FinalizePoseUpdate() {
  geometry_engine_->UpdateWorldPoses(X_WG_, X_WG_proximity_);
  for (auto& pair : render_engines_) {
    pair.second->UpdatePoses(X_WG_);
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
  X_PF_[frame.internal_index()] = X_PF;
  Isometry3<T> X_WF = X_WP * X_PF;
  // TODO(SeanCurtis-TRI): Replace this when we have a transform object that
  // allows proper multiplication between an AutoDiff type and a double type.
  // For now, it allows me to perform the multiplication by multiplying the
  // fully-defined transformation (with [0 0 0 1] on the bottom row).
  X_WF.makeAffine();
  X_WF_[frame.internal_index()] = X_WF;
  // Update the geometry which belong to *this* frame.
  for (auto child_id : frame.child_geometries()) {
    auto& child_geometry = geometries_[child_id];
    auto child_index = child_geometry.internal_index();
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

template <typename T>
InternalGeometry* GeometryState<T>::GetMutableGeometry(
    GeometryId id) {
  const InternalGeometry* geometry = GetGeometry(id);
  return const_cast<InternalGeometry*>(geometry);
}

template <typename T>
bool GeometryState<T>::NameIsUnique(FrameId id, Role role,
                                    const std::string& name) const {
  bool unique = true;
  const InternalFrame& frame = GetValueOrThrow(id, frames_);
  for (GeometryId geometry_id : frame.child_geometries()) {
    const InternalGeometry& geometry = geometries_.at(geometry_id);
    if (geometry.has_role(role) && geometry.name() == name) {
      unique = false;
      break;
    }
  }
  return unique;
}

template <typename T>
void GeometryState<T>::ThrowIfNameExistsInRole(FrameId id, Role role,
                                               const std::string& name) const {
  if (!NameIsUnique(id, role, name)) {
    throw std::logic_error("The name " + name + " has already been used by "
        "a geometry with the '" + to_string(role) + "' role.");
  }
}

template <typename T>
template <typename PropertyType>
void GeometryState<T>::AssignRoleInternal(SourceId source_id,
                                          GeometryId geometry_id,
                                          PropertyType properties, Role role) {
  if (!BelongsToSource(geometry_id, source_id)) {
    throw std::logic_error("Given geometry id " + to_string(geometry_id) +
        " does not belong to the given source id " +
        to_string(source_id));
  }
  InternalGeometry* geometry = GetMutableGeometry(geometry_id);
  // Must be non-null, otherwise, we never would've gotten past the
  // `BelongsToSource()` call.
  DRAKE_DEMAND(geometry != nullptr);

  if (!geometry->has_role(role)) {
    // Only test for name uniqueness if this geometry doesn't already have the
    // specified role. This is here for two reasons:
    //   1. If the role has already been assigned, we want that error to
    //      have precedence -- i.e., the name is irrelevant if the role has
    //      already been assigned. We rely on SetRole() to detect and throw.
    //   2. We don't want this to *follow* SetRole(), because we only want to
    //      set the role if the name is unique -- testing after would leave the
    //      role assigned.
    ThrowIfNameExistsInRole(geometry->frame_id(), role, geometry->name());
  }
  geometry->SetRole(std::move(properties));
}

template <typename T>
int GeometryState<T>::RemoveRoleUnchecked(GeometryId geometry_id, Role role) {
  switch (role) {
    case Role::kUnassigned:
      // Can't remove unassigned; it's a no op.
      return 0;
    case Role::kProximity:
      // This dev/SceneGraph doesn't support proximity roles.
      return 0;
    case Role::kIllustration:
      return RemoveIllustrationRole(geometry_id);
    case Role::kPerception:
      return RemovePerceptionRole(geometry_id);
  }
  return 0;
}

template <typename T>
int GeometryState<T>::RemoveFromRendererUnchecked(
    const std::string& renderer_name, GeometryId id) {
  internal::InternalGeometry* geometry = GetMutableGeometry(id);
  optional<RenderIndex> render_index = geometry->render_index(renderer_name);
  if (render_index) {
    render::RenderEngine* engine = render_engines_[renderer_name].get_mutable();
    geometry->ClearRenderIndex(renderer_name);
    optional<InternalIndex> moved_geometry_index =
        engine->RemoveGeometry(*render_index);
    if (moved_geometry_index) {
      GeometryId moved_id = geometry_index_id_map_[*moved_geometry_index];
      InternalGeometry& moved_geometry = geometries_.at(moved_id);
      optional<RenderIndex> old_render_index =
          moved_geometry.render_index(renderer_name);
      // This must be the case, or else the renderer would _not_ have been
      // able to move this geometry.
      DRAKE_DEMAND(old_render_index.has_value());
      moved_geometry.ClearRenderIndex(renderer_name);
      moved_geometry.set_render_index(renderer_name, *render_index);
    }
    return 1;
  }
  return 0;
}

template <typename T>
int GeometryState<T>::RemoveIllustrationRole(GeometryId geometry_id) {
  internal::InternalGeometry* geometry = GetMutableGeometry(geometry_id);
  DRAKE_DEMAND(geometry != nullptr);
  if (geometry->has_illustration_role()) {
    geometry->RemoveIllustrationRole();
    return 1;
  }
  return 0;
}

template <typename T>
int GeometryState<T>::RemovePerceptionRole(GeometryId geometry_id) {
  internal::InternalGeometry* geometry = GetMutableGeometry(geometry_id);
  DRAKE_DEMAND(geometry != nullptr);
  if (geometry->has_perception_role()) {
    int count = 0;
    for (auto& pair : render_engines_) {
      const std::string& engine_name = pair.first;
      count = RemoveFromRendererUnchecked(engine_name, geometry_id);
    }
    geometry->RemovePerceptionRole();
    return count;
  }
  return 0;
}

template <typename T>
render::RenderEngine* GeometryState<T>::GetRenderEngineOrThrow(
    const std::string& renderer_name) const {
  auto iter = render_engines_.find(renderer_name);
  if (iter != render_engines_.end()) {
    // NOTE: The render engines are contained in copyable unique pointers so
    // that the geometry state can be copied. However, getting a mutable pointer
    // can't be done via a `const copyable_unique_ptr`. So, we const cast it as
    // a short-term hack until we work out caching issues -- i.e. where this is
    // called in a cache calc method.
    return const_cast<render::RenderEngine*>(iter->second.get());
  }

  throw std::logic_error(
      fmt::format("No renderer exists with name: '{}'", renderer_name));
}

template <typename T>
Isometry3<double> GeometryState<T>::GetDoubleWorldPose(FrameId frame_id) const {
  if (frame_id == InternalFrame::world_frame_id()) {
    return Isometry3<double>::Identity();
  }
  const internal::InternalFrame& frame = GetValueOrThrow(frame_id, frames_);
  return convert(X_WF_[frame.internal_index()]);
}

template <typename T>
void GeometryState<T>::RegisterValidSource(SourceId source_id,
                                           const std::string& name) {
  source_frame_id_map_[source_id];
  source_root_frame_map_[source_id];
  source_anchored_geometry_map_[source_id];
  source_names_[source_id] = name;
}

template <typename T>
void GeometryState<T>::RegisterValidFrame(SourceId source_id, FrameId frame_id,
                                          const std::string& name,
                                          int frame_group, FrameId parent_id,
                                          int clique, FrameIdSet* frame_set) {
  if (parent_id != InternalFrame::world_frame_id()) {
    FindOrThrow(parent_id, *frame_set, [parent_id, source_id]() {
      return "Indicated parent id " + to_string(parent_id) + " does not belong "
             "to the indicated source id " + to_string(source_id) + ".";
    });
    frames_[parent_id].add_child(frame_id);
  } else {
    // The parent is the world frame; register it as a root frame.
    source_root_frame_map_[source_id].insert(frame_id);
  }

  DRAKE_ASSERT(X_PF_.size() == frame_index_to_frame_map_.size());
  InternalIndex internal_index(X_PF_.size());
  X_PF_.emplace_back(Isometry3<double>::Identity());
  X_WF_.emplace_back(Isometry3<double>::Identity());
  frame_index_to_frame_map_.push_back(frame_id);
  frame_set->insert(frame_id);
  frames_.emplace(frame_id, InternalFrame(source_id, frame_id, name,
                                          frame_group, internal_index,
                                          parent_id, clique));
}

template <typename T>
void GeometryState<T>::RegisterValidGeometry(
    SourceId source_id, FrameId frame_id, GeometryId geometry_id,
    std::unique_ptr<Shape> shape, const std::string& name,
    const Isometry3<double>& X_PG, const IllustrationProperties& properties,
    internal::InternalGeometry* parent_geometry) {
  InternalFrame& frame = frames_[frame_id];
  frame.add_child(geometry_id);

  // TODO(SeanCurtis-TRI): Enforcing the invariant that the indices are
  // compactly distributed. Is there a more robust way to do this?
  DRAKE_ASSERT(geometry_index_id_map_.size() == X_WG_.size());
  InternalIndex internal_index(static_cast<int>(X_WG_.size()));
  X_WG_.push_back(Isometry3<T>::Identity());
  geometry_index_id_map_.push_back(geometry_id);

  geometries_.emplace(
      geometry_id, InternalGeometry(source_id, std::move(shape), frame_id,
                                    geometry_id, name, X_PG, internal_index));

  // If there's a parent geometry
  if (parent_geometry != nullptr) {
    // RegisterGeometry stores X_PG into X_FG_ (having assumed that  the
    // parent was a frame). This replaces the stored X_PG value with the
    // semantically correct value X_FG by concatenating X_FP with X_PG.

    // Transform pose relative to geometry, to pose relative to frame.
    InternalGeometry& new_geometry = geometries_[geometry_id];
    // The call to `RegisterGeometry()` above stashed the pose X_PG into the
    // X_FG_ vector assuming the parent was the frame. Replace it by
    // concatenating its pose in parent, with its parent's pose in frame.
    // NOTE: the pose is no longer available from geometry because of the
    // `move(geometry)`.
    const Isometry3<double>& X_FP = parent_geometry->X_FG();
    new_geometry.set_geometry_parent(parent_geometry->id(), X_FP * X_PG);
    parent_geometry->add_child(geometry_id);
  }

  AssignRole(source_id, geometry_id, properties);

  PerceptionProperties p;
  if (properties.HasProperty("phong", "diffuse")) {
    p.AddGroup("phong");
    p.AddProperty("phong", "diffuse",
                  properties.GetProperty<Vector4<double>>("phong", "diffuse"));
  }
  p.AddGroup("label");
  // NOTE: These render labels are *not* being returned and there is currently
  // no meaningful facility for looking up by render label.
  static const auto kNonTerrainLabel = render::RenderLabel::new_label();
  p.AddProperty("label", "id", kNonTerrainLabel);
  AssignRole(source_id, geometry_id, p);
}

}  // namespace dev
}  // namespace geometry
}  // namespace drake

// TODO(SeanCurtis-TRI): Currently assumes that "non-symbolic" implies
// AutoDiffXd. Update things appropriately when more non-symbolic scalars
// are available.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::dev::GeometryState)
