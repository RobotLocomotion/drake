#include "drake/geometry/geometry_state.h"

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <fmt/format.h>

#include "drake/common/autodiff.h"
#include "drake/common/default_scalars.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/proximity_engine.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/utilities.h"

namespace drake {
namespace geometry {

using internal::GeometryStateCollisionFilterAttorney;
using internal::InternalFrame;
using internal::InternalGeometry;
using internal::ProximityEngine;
using math::RigidTransform;
using math::RigidTransformd;
using std::make_pair;
using std::make_unique;
using std::move;
using std::swap;
using std::to_string;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

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
  X_WF_.push_back(RigidTransform<T>::Identity());
  X_PF_.push_back(RigidTransform<T>::Identity());

  source_frame_id_map_[self_source_] = {world};
  source_root_frame_map_[self_source_] = {world};
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
std::set<std::pair<GeometryId, GeometryId>>
GeometryState<T>::GetCollisionCandidates() const {
  std::set<std::pair<GeometryId, GeometryId>> pairs;
  for (const auto& pairA : geometries_) {
    const GeometryId idA = pairA.first;
    const InternalGeometry& geometryA = pairA.second;
    if (!geometryA.has_proximity_role()) continue;
    for (const auto& pairB : geometries_) {
      const GeometryId idB = pairB.first;
      if (idB < idA) continue;  // Only consider the pair (A, B) and not (B, A).
      const InternalGeometry& geometryB = pairB.second;
      if (!geometryB.has_proximity_role()) continue;
      // This relies on CollisionFiltered() to handle if A == B, if they
      // are both anchored, etc.
      if (!CollisionFiltered(idA, idB)) {
        pairs.insert({idA, idB});
      }
    }
  }
  return pairs;
}

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
int GeometryState<T>::NumFramesForSource(SourceId source_id) const {
  const auto& frame_set = GetValueOrThrow(source_id, source_frame_id_map_);
  return static_cast<int>(frame_set.size());
}

template <typename T>
const FrameIdSet& GeometryState<T>::GetFramesForSource(
    SourceId source_id) const {
  return GetValueOrThrow(source_id, source_frame_id_map_);
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
const std::string& GeometryState<T>::GetOwningSourceName(FrameId id) const {
  SourceId source_id = get_source_id(id);
  return source_names_.at(source_id);
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
int GeometryState<T>::get_frame_group(FrameId frame_id) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "No frame group available for invalid frame id: " +
        to_string(frame_id);
  });
  return frames_.at(frame_id).frame_group();
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
int GeometryState<T>::NumGeometriesWithRole(FrameId frame_id, Role role) const {
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
GeometryId GeometryState<T>::GetGeometryFromName(
    FrameId frame_id, Role role, const std::string& name) const {
  const std::string canonical_name = internal::CanonicalizeStringName(name);

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
  // This case should only be possible for unassigned geometries - internal
  // invariants require unique names for actual geometries with the _same_
  // role on the same frame.
  DRAKE_DEMAND(role == Role::kUnassigned);
  throw std::logic_error("The frame '" + frame_name + "' (" +
      to_string(frame_id) + ") has multiple geometries with the role '" +
      to_string(role) + "' and the canonical name '" + canonical_name + "'");
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
const std::string& GeometryState<T>::GetOwningSourceName(GeometryId id) const {
  SourceId source_id = get_source_id(id);
  return source_names_.at(source_id);
}

template <typename T>
FrameId GeometryState<T>::GetFrameId(GeometryId geometry_id) const {
  const auto& geometry = GetValueOrThrow(geometry_id, geometries_);
  return geometry.frame_id();
}

template <typename T>
const std::string& GeometryState<T>::GetName(GeometryId geometry_id) const {
  const InternalGeometry* geometry = GetGeometry(geometry_id);
  if (geometry != nullptr) return geometry->name();

  throw std::logic_error("No geometry available for invalid geometry id: " +
      to_string(geometry_id));
}

template <typename T>
const Shape& GeometryState<T>::GetShape(GeometryId id) const {
  const InternalGeometry* geometry = GetGeometry(id);
  if (geometry != nullptr) return geometry->shape();

  throw std::logic_error("No geometry available for invalid geometry id: " +
      to_string(id));
}

template <typename T>
const math::RigidTransform<double>& GeometryState<T>::GetPoseInFrame(
    GeometryId geometry_id) const {
  const auto& geometry = GetValueOrThrow(geometry_id, geometries_);
  return geometry.X_FG();
}

template <typename T>
const math::RigidTransform<double>& GeometryState<T>::GetPoseInParent(
    GeometryId geometry_id) const {
  const auto& geometry = GetValueOrThrow(geometry_id, geometries_);
  return geometry.X_PG();
}

template <typename T>
const ProximityProperties* GeometryState<T>::GetProximityProperties(
    GeometryId id) const {
  const InternalGeometry* geometry = GetGeometry(id);
  if (geometry) return geometry->proximity_properties();
  throw std::logic_error(
      fmt::format("Referenced geometry {} has not been registered", id));
}

template <typename T>
const IllustrationProperties* GeometryState<T>::GetIllustrationProperties(
    GeometryId id) const {
  const InternalGeometry* geometry = GetGeometry(id);
  if (geometry) return geometry->illustration_properties();
  throw std::logic_error(
      fmt::format("Referenced geometry {} has not been registered", id));
}

template <typename T>
const PerceptionProperties* GeometryState<T>::GetPerceptionProperties(
    GeometryId id) const {
  const InternalGeometry* geometry = GetGeometry(id);
  if (geometry) return geometry->perception_properties();
  throw std::logic_error(
      fmt::format("Referenced geometry {} has not been registered", id));
}

template <typename T>
bool GeometryState<T>::CollisionFiltered(GeometryId id1, GeometryId id2) const {
  std::string base_message =
      "Can't report collision filter status between geometries " +
          to_string(id1) + " and " + to_string(id2) + "; ";
  const internal::InternalGeometry* geometry1 = GetGeometry(id1);
  const internal::InternalGeometry* geometry2 = GetGeometry(id2);
  if (geometry1 != nullptr && geometry2 != nullptr) {
    if (geometry1->has_proximity_role() && geometry2->has_proximity_role()) {
      return geometry_engine_->CollisionFiltered(
          geometry1->id(), geometry1->is_dynamic(),
          geometry2->id(), geometry2->is_dynamic());
    }
    if (geometry1->has_proximity_role()) {
      throw std::logic_error(base_message + to_string(id2) +
          " does not have a proximity role");
    } else if (geometry2->has_proximity_role()) {
      throw std::logic_error(base_message + to_string(id1) +
          " does not have a proximity role");
    } else {
      throw std::logic_error(base_message + " neither id has a proximity role");
    }
  }
  if (geometry1 != nullptr) {
    throw std::logic_error(base_message + to_string(id2) +
        " is not a valid geometry");
  } else if (geometry2 != nullptr) {
    throw std::logic_error(base_message + to_string(id1) +
        " is not a valid geometry");
  } else {
    throw std::logic_error(base_message + "neither id is a valid geometry");
  }
}

template <typename T>
const math::RigidTransform<T>& GeometryState<T>::get_pose_in_world(
    FrameId frame_id) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "No world pose available for invalid frame id: " +
           to_string(frame_id);
  });
  return X_WF_[frames_.at(frame_id).index()];
}

template <typename T>
const math::RigidTransform<T>& GeometryState<T>::get_pose_in_world(
    GeometryId geometry_id) const {
  FindOrThrow(geometry_id, geometries_, [geometry_id]() {
    return "No world pose available for invalid geometry id: " +
           to_string(geometry_id);
  });
  return X_WGs_.at(geometry_id);
}

template <typename T>
const math::RigidTransform<T>& GeometryState<T>::get_pose_in_parent(
    FrameId frame_id) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "No pose available for invalid frame id: " + to_string(frame_id);
  });
  return X_PF_[frames_.at(frame_id).index()];
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
  X_PF_.emplace_back(RigidTransform<T>::Identity());
  X_WF_.emplace_back(RigidTransform<T>::Identity());
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

  // Configure topology.
  // NOTE: Names are not validated here -- there are no roles. The names are
  // validated when roles are assigned.

  InternalFrame& frame = frames_[frame_id];
  frame.add_child(geometry_id);

  // pose() is always RigidTransform<double>. To account for
  // GeometryState<AutoDiff>, we need to cast it to the common type T.
  X_WGs_[geometry_id] = geometry->pose().cast<T>();

  geometries_.emplace(
      geometry_id,
      InternalGeometry(source_id, geometry->release_shape(), frame_id,
                       geometry_id, geometry->name(), geometry->pose()));

  // Any roles defined on the geometry instance propagate through automatically.
  if (geometry->illustration_properties()) {
    AssignRole(source_id, geometry_id,
               std::move(*geometry->mutable_illustration_properties()));
  }

  if (geometry->proximity_properties()) {
    AssignRole(source_id, geometry_id,
               std::move(*geometry->mutable_proximity_properties()));
  }

  if (geometry->perception_properties()) {
    AssignRole(source_id, geometry_id,
               std::move(*geometry->mutable_perception_properties()));
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
  const RigidTransform<double>& X_PG = new_geometry.X_FG();
  const RigidTransform<double>& X_FP = parent_geometry.X_FG();
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
void GeometryState<T>::RemoveGeometry(SourceId source_id,
                                      GeometryId geometry_id) {
  if (!BelongsToSource(geometry_id, source_id)) {
    throw std::logic_error(
        "Trying to remove geometry " + to_string(geometry_id) + " from "
            "source " + to_string(source_id) + ", but the geometry doesn't "
            "belong to that source.");
  }
  RemoveGeometryUnchecked(geometry_id, RemoveGeometryOrigin::kGeometry);
}

template <typename T>
bool GeometryState<T>::IsValidGeometryName(
    FrameId frame_id, Role role, const std::string& candidate_name) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "Given frame id is not valid: " + to_string(frame_id);
  });
  const std::string name = internal::CanonicalizeStringName(candidate_name);
  if (name.empty()) return false;
  return NameIsUnique(frame_id, role, name);
}

namespace {
// Small class for identifying mesh geometries.
class MeshIdentifier final : public ShapeReifier {
 public:
  bool is_mesh() const { return is_mesh_; }

  // Implementation of ShapeReifier interface.
  void ImplementGeometry(const Sphere&, void*) final {}
  void ImplementGeometry(const Cylinder&, void*) final {}
  void ImplementGeometry(const HalfSpace&, void*) final {}
  void ImplementGeometry(const Box&, void*) final {}
  void ImplementGeometry(const Mesh& mesh, void*) final {
    is_mesh_ = true;
    drake::log()->warn("Meshes are _not_ supported for proximity: ({})",
                       mesh.filename());
  }
  void ImplementGeometry(const Convex&, void*) final {}

 private:
  bool is_mesh_{false};
};
}  // namespace

template <typename T>
void GeometryState<T>::AssignRole(SourceId source_id, GeometryId geometry_id,
                                  ProximityProperties properties,
                                  RoleAssign assign) {
  // TODO(SeanCurtis-TRI): When meshes are supported for proximity roles, remove
  // this test and the MeshIdentifier class.
  {
    const InternalGeometry* g = GetGeometry(geometry_id);
    if (g) {
      MeshIdentifier identifier;
      g->shape().Reify(&identifier);
      if (identifier.is_mesh()) return;
    }
  }

  InternalGeometry& geometry =
      ValidateRoleAssign(source_id, geometry_id, Role::kProximity, assign);

  // TODO(SeanCurtis-TRI): Before setting the properties, if this is kReplace I
  //  may need to address the changes between properties (possibly undoing
  //  something).

  geometry.SetRole(std::move(properties));

  if (assign == RoleAssign::kNew) {
    if (geometry.is_dynamic()) {
      // Pass the geometry to the engine.
      geometry_engine_->AddDynamicGeometry(geometry.shape(), geometry_id);

      InternalFrame& frame = frames_[geometry.frame_id()];

      int child_count = static_cast<int>(frame.child_geometries().size());
      if (child_count > 1) {
        // Having multiple children is _necessary_ but not _sufficient_ to
        // require collision filtering. Only if there are multiple children with
        // the proximity role do we engage filtering.
        // TODO(SeanCurtis-TRI): Perhaps refactor this elsewhere?
        std::vector<GeometryId> proximity_geometries;
        proximity_geometries.reserve(child_count);
        for (GeometryId child_id : frame.child_geometries()) {
          if (geometries_[child_id].has_proximity_role()) {
            proximity_geometries.push_back(child_id);
          }
        }
        const int proximity_count =
            static_cast<int>(proximity_geometries.size());

        if (proximity_count > 1) {
          // Filter collisions between geometries affixed to the same frame. We
          // only add a clique to a frame's geometries when there are *multiple*
          // child geometries.
          ProximityEngine<T>& engine = *geometry_engine_.get_mutable();
          if (proximity_count > 2) {
            // Assume all previous geometries have already had the clique
            // assigned.
            GeometryStateCollisionFilterAttorney::set_dynamic_geometry_clique(
                &engine, geometry_id, frame.clique());
          } else {  // proximity_count == 2.
            // This geometry tips us over to the point where we need to assign
            // the clique to the new (and previous) geometries.
            // NOTE: this is an optimization based on the clunky nature of the
            // current collision filtering -- we're benefited in limiting the
            // number of cliques assigned to a geometry.
            for (GeometryId child_id : proximity_geometries) {
              GeometryStateCollisionFilterAttorney::set_dynamic_geometry_clique(
                  &engine, child_id, frame.clique());
            }
          }
        }
      }
    } else {
      // If it's not dynamic, it must be anchored. No clique madness required;
      // anchored geometries are not tested against each other by the process.
      geometry_engine_->AddAnchoredGeometry(geometry.shape(), geometry.X_FG(),
                                            geometry_id);
    }
  }
  // TODO(SeanCurtis-TRI): Handle the assign == kReplace branch for when
  //  ProximityEngine depends on the properties.
}

template <typename T>
void GeometryState<T>::AssignRole(SourceId source_id, GeometryId geometry_id,
                                  PerceptionProperties properties,
                                  RoleAssign assign) {
  InternalGeometry& geometry = ValidateRoleAssign(source_id, geometry_id,
                     Role::kPerception, assign);

  // TODO(SeanCurtis-TRI): To support RoleAssign::kReplace, the render engines
  //  need to handle these changes.

  geometry.SetRole(std::move(properties));

  for (auto& pair : render_engines_) {
    auto& engine = pair.second;
    engine->RegisterVisual(
        geometry_id, geometry.shape(), *geometry.perception_properties(),
        RigidTransformd(geometry.X_FG()), geometry.is_dynamic());
  }
}

template <typename T>
void GeometryState<T>::AssignRole(SourceId source_id, GeometryId geometry_id,
                                  IllustrationProperties properties,
                                  RoleAssign assign) {
  InternalGeometry& geometry =
      ValidateRoleAssign(source_id, geometry_id, Role::kIllustration, assign);
  // TODO(SeanCurtis-TRI): Ideally, if assign == RoleAssign::kReplace, this
  //  should cause the visualization to change. I.e., if I've loaded an object
  //  then I change its color here, it would be great if the visualization
  //  reflected this. That is a *huge* issue that is not easily resolved.
  //  Alternatively, I need to document that this *doesn't* happen and that it
  //  is up to the visualizer to re-initialize itself.
  geometry.SetRole(std::move(properties));
}

template <typename T>
int GeometryState<T>::RemoveRole(SourceId source_id, FrameId frame_id,
                                 Role role) {
  int count = 0;

  // Note: attempting any operation with invalid ids is "bad", even if it were
  // to be a no-op. Callers shouldn't try to manipulate things they don't own.
  const InternalFrame& frame = ValidateAndGetFrame(source_id, frame_id);

  // One can't "remove" the unassigned role.
  if (role == Role::kUnassigned) return 0;

  for (GeometryId geometry_id : frame.child_geometries()) {
    // If the frame is the world frame, then the specific geometry needs to be
    // tested to see if it belongs to the source. Otherwise, by definition, the
    // geometry must belong to the same source as the parent frame.
    if (frame_id != InternalFrame::world_frame_id() ||
        BelongsToSource(geometry_id, source_id)) {
      if (RemoveRoleUnchecked(geometry_id, role)) ++count;
    }
  }
  return count;
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

  // One can't "remove" the unassigned role state.
  if (role == Role::kUnassigned) return 0;

  return RemoveRoleUnchecked(geometry_id, role) ? 1 : 0;
}

template <typename T>
int GeometryState<T>::RemoveFromRenderer(const std::string& renderer_name,
                                         SourceId source_id, FrameId frame_id) {
  int count = 0;

  const InternalFrame& frame = ValidateAndGetFrame(source_id, frame_id);

  for (GeometryId geometry_id : frame.child_geometries()) {
    // If the frame is the world frame, then the specific geometry needs to be
    // tested to see if it belongs to the source. Otherwise, by definition, the
    // geometry must belong to the same source as the parent frame.
    if (frame_id != InternalFrame::world_frame_id() ||
        BelongsToSource(geometry_id, source_id)) {
      if (RemoveFromRendererUnchecked(renderer_name, geometry_id)) ++count;
    }
  }
  return count;
}

template <typename T>
int GeometryState<T>::RemoveFromRenderer(const std::string& renderer_name,
                                         SourceId source_id,
                                         GeometryId geometry_id) {
  if (!BelongsToSource(geometry_id, source_id)) {
    throw std::logic_error(
        "Trying to remove geometry " + to_string(geometry_id) + " from the "
        "renderer '" + renderer_name + "', but the geometry doesn't belong to "
        "given source " + to_string(source_id) + ".");
  }

  return RemoveFromRendererUnchecked(renderer_name, geometry_id) ? 1 : 0;
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

  std::unordered_set<GeometryId> dynamic;
  std::unordered_set<GeometryId> anchored;
  CollectIds(set, &dynamic, &anchored);

  geometry_engine_->ExcludeCollisionsWithin(dynamic, anchored);
}

template <typename T>
void GeometryState<T>::ExcludeCollisionsBetween(const GeometrySet& setA,
                                                const GeometrySet& setB) {
  std::unordered_set<GeometryId> dynamic1;
  std::unordered_set<GeometryId> anchored1;
  CollectIds(setA, &dynamic1, &anchored1);
  std::unordered_set<GeometryId> dynamic2;
  std::unordered_set<GeometryId> anchored2;
  CollectIds(setB, &dynamic2, &anchored2);
  geometry_engine_->ExcludeCollisionsBetween(dynamic1, anchored1, dynamic2,
                                             anchored2);
}

template <typename T>
void GeometryState<T>::AddRenderer(
    std::string name, std::unique_ptr<render::RenderEngine> renderer) {
  if (render_engines_.count(name) > 0) {
    throw std::logic_error(fmt::format(
        "AddRenderer(): A renderer with the name '{}' already exists", name));
  }
  render::RenderEngine* render_engine = renderer.get();
  render_engines_[name] = move(renderer);
  for (auto& id_geo_pair : geometries_) {
    InternalGeometry& geometry = id_geo_pair.second;
    if (geometry.has_perception_role()) {
      const GeometryId id = id_geo_pair.first;
      const PerceptionProperties* properties = geometry.perception_properties();
      DRAKE_DEMAND(properties != nullptr);
      render_engine->RegisterVisual(id, geometry.shape(), *properties,
                                    RigidTransformd(geometry.X_FG()),
                                    geometry.is_dynamic());
    }
  }
}

template <typename T>
std::vector<std::string> GeometryState<T>::RegisteredRendererNames() const {
  std::vector<std::string> names;
  names.reserve(render_engines_.size());
  for (const auto& name_engine_pair : render_engines_) {
    names.push_back(name_engine_pair.first);
  }
  return names;
}

template <typename T>
void GeometryState<T>::RenderColorImage(const render::CameraProperties& camera,
                                        FrameId parent_frame,
                                        const RigidTransformd& X_PC,
                                        bool show_window,
                                        ImageRgba8U* color_image_out) const {
  const RigidTransformd X_WC = GetDoubleWorldPose(parent_frame) * X_PC;
  const render::RenderEngine& engine =
      GetRenderEngineOrThrow(camera.renderer_name);
  // TODO(SeanCurtis-TRI): Invoke UpdateViewpoint() as part of a calc cache
  //  entry. Challenge: how to do that with a parameter passed here?
  const_cast<render::RenderEngine&>(engine).UpdateViewpoint(X_WC);
  engine.RenderColorImage(camera, show_window, color_image_out);
}

template <typename T>
void GeometryState<T>::RenderDepthImage(
    const render::DepthCameraProperties& camera,
    FrameId parent_frame, const RigidTransformd& X_PC,
    ImageDepth32F* depth_image_out) const {
  const RigidTransformd X_WC = GetDoubleWorldPose(parent_frame) * X_PC;
  const render::RenderEngine& engine =
      GetRenderEngineOrThrow(camera.renderer_name);
  // See note in RenderColorImage() about this const cast.
  const_cast<render::RenderEngine&>(engine).UpdateViewpoint(X_WC);
  engine.RenderDepthImage(camera, depth_image_out);
}

template <typename T>
void GeometryState<T>::RenderLabelImage(const render::CameraProperties& camera,
                                        FrameId parent_frame,
                                        const RigidTransformd& X_PC,
                                        bool show_window,
                                        ImageLabel16I* label_image_out) const {
  const RigidTransformd X_WC = GetDoubleWorldPose(parent_frame) * X_PC;
  const render::RenderEngine& engine =
      GetRenderEngineOrThrow(camera.renderer_name);
  // See note in RenderColorImage() about this const cast.
  const_cast<render::RenderEngine&>(engine).UpdateViewpoint(X_WC);
  engine.RenderLabelImage(camera, show_window, label_image_out);
}

template <typename T>
std::unique_ptr<GeometryState<AutoDiffXd>> GeometryState<T>::ToAutoDiffXd()
    const {
  return std::unique_ptr<GeometryState<AutoDiffXd>>(
      new GeometryState<AutoDiffXd>(*this));
}

template <typename T>
void GeometryState<T>::CollectIds(
    const GeometrySet& geometry_set, std::unordered_set<GeometryId>* dynamic,
    std::unordered_set<GeometryId>* anchored) {
  // TODO(SeanCurtis-TRI): Consider expanding this to include Role if it proves
  // that collecting ids for *other* role-related tasks prove necessary.
  std::unordered_set<GeometryId>* target;
  for (auto frame_id : geometry_set.frames()) {
    const auto& frame = GetValueOrThrow(frame_id, frames_);
    target = frame.is_world() ? anchored : dynamic;
    for (auto geometry_id : frame.child_geometries()) {
      InternalGeometry& geometry = geometries_[geometry_id];
      if (geometry.has_proximity_role()) {
        target->insert(geometry_id);
      }
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
    if (geometry->has_proximity_role()) {
      if (geometry->is_dynamic()) {
        dynamic->insert(geometry_id);
      } else {
        anchored->insert(geometry_id);
      }
    }
  }
}

template <typename T>
void GeometryState<T>::SetFramePoses(
    const SourceId source_id, const FramePoseVector<T>& poses) {
  // TODO(SeanCurtis-TRI): Down the road, make this validation depend on
  // ASSERT_ARMED.
  ValidateFrameIds(source_id, poses);
  const RigidTransform<T> world_pose = RigidTransform<T>::Identity();
  for (auto frame_id : source_root_frame_map_[source_id]) {
    UpdatePosesRecursively(frames_[frame_id], world_pose, poses);
  }
}

template <typename T>
template <typename ValueType>
void GeometryState<T>::ValidateFrameIds(
    const SourceId source_id,
    const FrameKinematicsVector<ValueType>& kinematics_data) const {
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
  geometry_engine_->UpdateWorldPoses(X_WGs_);
  for (auto& pair : render_engines_) {
    pair.second->UpdatePoses(X_WGs_);
  }
}

template <typename T>
SourceId GeometryState<T>::get_source_id(FrameId frame_id) const {
  const auto& frame = GetValueOrThrow(frame_id, frames_);
  return frame.source_id();
}

template <typename T>
SourceId GeometryState<T>::get_source_id(GeometryId id) const {
  const InternalGeometry* geometry = GetGeometry(id);
  if (geometry == nullptr) {
    throw std::logic_error("Geometry id " + to_string(id) +
                           " does not map to a registered geometry");
  }
  return geometry->source_id();
}

template <typename T>
void GeometryState<T>::RemoveGeometryUnchecked(GeometryId geometry_id,
                                               RemoveGeometryOrigin caller) {
  const InternalGeometry& geometry = GetValueOrThrow(geometry_id, geometries_);

  // TODO(SeanCurtis-TRI): When this gets invoked by RemoveFrame(), this
  // recursive action will not be necessary, as all child geometries will
  // automatically get removed. I've put it into a block so for future
  // reference; simply add an if statement to determine if this is coming from
  // frame removal.
  {
    for (auto child_id : geometry.child_geometry_ids()) {
      RemoveGeometryUnchecked(child_id, RemoveGeometryOrigin::kRecurse);
    }
    // Remove the geometry from its frame's list of geometries.
    auto& frame = GetMutableValueOrThrow(geometry.frame_id(), &frames_);
    frame.remove_child(geometry_id);
  }

  RemoveProximityRole(geometry_id);
  RemovePerceptionRole(geometry_id);
  RemoveIllustrationRole(geometry_id);

  if (caller == RemoveGeometryOrigin::kGeometry) {
    // Only the geometry that this function is *directly* invoked on needs to
    // remove itself from its possible parent geometry. If called recursively,
    // it is because the parent geometry is being deleted anyways and removal
    // is implicit in the deletion of that parent geometry.
    if (optional<GeometryId> parent_id = geometry.parent_id()) {
      auto& parent_geometry =
          GetMutableValueOrThrow(*parent_id, &geometries_);
      parent_geometry.remove_child(geometry_id);
    }
  }

  // Clean up state collections.
  X_WGs_.erase(geometry_id);

  // Remove from the geometries.
  geometries_.erase(geometry_id);
}

template <typename T>
void GeometryState<T>::UpdatePosesRecursively(
    const internal::InternalFrame& frame, const RigidTransform<T>& X_WP,
    const FramePoseVector<T>& poses) {
  const auto frame_id = frame.id();
  const auto& X_PF = poses.value(frame_id);
  // Cache this transform for later use.
  X_PF_[frame.index()] = X_PF;
  RigidTransform<T> X_WF = X_WP * X_PF;
  X_WF_[frame.index()] = X_WF;
  // Update the geometry which belong to *this* frame.
  for (auto child_id : frame.child_geometries()) {
    auto& child_geometry = geometries_[child_id];
    // X_FG() is always RigidTransform<double>, to account for
    // GeometryState<AutoDiff>, we need to cast it to the common type T.
    RigidTransform<double> X_FG(child_geometry.X_FG());
    X_WGs_[child_id] = X_WF * X_FG.cast<T>();
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
InternalGeometry* GeometryState<T>::GetMutableGeometry(GeometryId id) {
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
    throw std::logic_error("The name '" + name + "' has already been used by "
        "a geometry with the '" + to_string(role) + "' role.");
  }
}

template <typename T>
InternalGeometry& GeometryState<T>::ValidateRoleAssign(SourceId source_id,
                                                       GeometryId geometry_id,
                                                       Role role,
                                                       RoleAssign assign) {
  if (assign == RoleAssign::kReplace &&
      (role == Role::kPerception || role == Role::kIllustration)) {
    throw std::logic_error(
        "AssignRole() for updating properties currently only supports "
        "proximity properties");
  }
  if (!BelongsToSource(geometry_id, source_id)) {
    throw std::logic_error("Given geometry id " + to_string(geometry_id) +
        " does not belong to the given source id " +
        to_string(source_id));
  }
  InternalGeometry* geometry = GetMutableGeometry(geometry_id);
  // Must be non-null, otherwise, we never would've gotten past the
  // `BelongsToSource()` call.
  DRAKE_DEMAND(geometry != nullptr);

  // For now, we only have "new" and "replace" as operations. Therefore, the
  //  validity of this operation is simply expressed.
  const bool has_role = geometry->has_role(role);
  if (has_role && assign == RoleAssign::kNew) {
    throw std::logic_error(
        "Trying to assign the '" + to_string(role)
        + "' role to geometry id " + to_string(geometry_id)
        + " for the first time; it already has the role assigned");
  } else if (!has_role && assign == RoleAssign::kReplace) {
    throw std::logic_error(
        "Trying to replace the properties on geometry id "
        + to_string(geometry_id) + " for the '" + to_string(role)
        + "' role; it has not had the role initially assigned");
  }

  if (!has_role && assign == RoleAssign::kNew) {
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
  return *geometry;
}

template <typename T>
bool GeometryState<T>::RemoveRoleUnchecked(GeometryId geometry_id, Role role) {
  switch (role) {
    case Role::kUnassigned:
      // Can't remove unassigned; it's a no op.
      return false;
    case Role::kProximity:
      return RemoveProximityRole(geometry_id);
    case Role::kIllustration:
      return RemoveIllustrationRole(geometry_id);
    case Role::kPerception:
      return RemovePerceptionRole(geometry_id);
  }
  return false;
}

template <typename T>
bool GeometryState<T>::RemoveFromRendererUnchecked(
    const std::string& renderer_name, GeometryId id) {
  render::RenderEngine* engine = render_engines_[renderer_name].get_mutable();
  if (engine->has_geometry(id)) {
    // The engine has reported the belief that it has geometry `id`. Therefore,
    // removal should report true.
    DRAKE_DEMAND(engine->RemoveGeometry(id) == true);
    return true;
  }
  return false;
}

template <typename T>
bool GeometryState<T>::RemoveProximityRole(GeometryId geometry_id) {
  internal::InternalGeometry* geometry = GetMutableGeometry(geometry_id);
  DRAKE_DEMAND(geometry != nullptr);

  // Geometry is not registered with the proximity engine.
  if (!geometry->has_proximity_role()) return false;

  // Geometry *is* registered; do the work to remove it.
  geometry_engine_->RemoveGeometry(geometry_id, geometry->is_dynamic());
  geometry->RemoveProximityRole();
  return true;
}

template <typename T>
bool GeometryState<T>::RemoveIllustrationRole(GeometryId geometry_id) {
  internal::InternalGeometry* geometry = GetMutableGeometry(geometry_id);
  DRAKE_DEMAND(geometry != nullptr);

  // Geometry has no illustration role.
  if (!geometry->has_illustration_role()) return false;

  geometry->RemoveIllustrationRole();
  return true;
}

template <typename T>
bool GeometryState<T>::RemovePerceptionRole(GeometryId geometry_id) {
  internal::InternalGeometry* geometry = GetMutableGeometry(geometry_id);
  DRAKE_DEMAND(geometry != nullptr);

  // Geometry has no perception role.
  if (!geometry->has_perception_role()) return false;

  // Geometry has a perception role; do the work to remove it from whichever
  // render engines it happens to present in.
  for (auto& name_engine_pair : render_engines_) {
    const std::string& engine_name = name_engine_pair.first;
    RemoveFromRendererUnchecked(engine_name, geometry_id);
  }
  geometry->RemovePerceptionRole();
  return true;
}

template <typename T>
const InternalFrame& GeometryState<T>::ValidateAndGetFrame(
    SourceId source_id, FrameId frame_id) const {
  // Handle the special case of the world frame; source_id will *not* own it.
  if (frame_id == InternalFrame::world_frame_id()) {
    FindOrThrow(source_id, source_frame_id_map_, [source_id]() {
      return get_missing_id_message(source_id);
    });
    return frames_.at(frame_id);
  } else {
    // The generic test that the frame_id is owned by the source_id.
    const FrameIdSet& set =
        GetValueOrThrow(source_id, source_frame_id_map_);
    FindOrThrow(frame_id, set, [frame_id, source_id]() {
      return "Referenced frame " + to_string(frame_id) + " for source " +
          to_string(source_id) +
          ", but the frame doesn't belong to the source.";
    });
  }
  return frames_.at(frame_id);
}

template <typename T>
const render::RenderEngine& GeometryState<T>::GetRenderEngineOrThrow(
    const std::string& renderer_name) const {
  auto iter = render_engines_.find(renderer_name);
  if (iter != render_engines_.end()) {
    return *iter->second;
  }

  throw std::logic_error(
      fmt::format("No renderer exists with name: '{}'", renderer_name));
}

template <typename T>
RigidTransformd GeometryState<T>::GetDoubleWorldPose(FrameId frame_id) const {
  if (frame_id == InternalFrame::world_frame_id()) {
    return RigidTransformd::Identity();
  }
  const internal::InternalFrame& frame = GetValueOrThrow(frame_id, frames_);
  return internal::convert_to_double(X_WF_[frame.index()]);
}

}  // namespace geometry
}  // namespace drake

// TODO(SeanCurtis-TRI): Currently assumes that "non-symbolic" implies
// AutoDiffXd. Update things appropriately when more non-symbolic scalars
// are available.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::GeometryState)
