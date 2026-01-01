#include "drake/geometry/geometry_state.h"

#include <algorithm>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <fmt/format.h>
#include <fmt/ranges.h>

#include "drake/common/autodiff.h"
#include "drake/common/default_scalars.h"
#include "drake/common/extract_double.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/proximity/calc_obb.h"
#include "drake/geometry/proximity/make_convex_hull_mesh.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/geometry/proximity_engine.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/utilities.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace geometry {

using internal::convert_to_double;
using internal::ConvertVolumeToSurfaceMeshWithBoundaryVertices;
using internal::DrivenTriangleMesh;
using internal::FrameNameSet;
using internal::HydroelasticType;
using internal::InternalFrame;
using internal::InternalGeometry;
using internal::kComplianceType;
using internal::kElastic;
using internal::kFriction;
using internal::kHcDissipation;
using internal::kHydroGroup;
using internal::kMargin;
using internal::kMaterialGroup;
using internal::kPointStiffness;
using internal::kRelaxationTime;
using internal::kRezHint;
using internal::kSlabThickness;
using internal::MakeRenderMeshFromTriangleSurfaceMesh;
using internal::ProximityEngine;
using internal::RenderMesh;
using internal::VertexSampler;
using math::RigidTransform;
using math::RigidTransformd;
using render::ColorRenderCamera;
using render::DepthRenderCamera;
using std::make_pair;
using std::make_unique;
using std::set;
using std::string;
using std::swap;
using std::to_string;
using std::unordered_set;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

namespace internal {

DrivenMeshData::DrivenMeshData() = default;

DrivenMeshData::~DrivenMeshData() = default;

template <typename T>
void DrivenMeshData::SetControlMeshPositions(
    const std::unordered_map<GeometryId, VectorX<T>>& q_WGs) {
  for (auto& [id, meshes] : driven_meshes_) {
    DRAKE_DEMAND(q_WGs.contains(id));
    // To prevent unnecessary copying, this returns a reference for T=double and
    // returns a copy otherwise.
    const VectorX<double>& q_WG =
        geometry::internal::convert_to_double(q_WGs.at(id));
    // The meshes are partitions of the overall geometry and each of them knows
    // how to locate its own coordinates from within the full set of q_WG.
    for (auto& mesh : meshes) {
      mesh.SetControlMeshPositions(q_WG);
    }
  }
}

void DrivenMeshData::SetMeshes(GeometryId id,
                               std::vector<DrivenTriangleMesh> driven_meshes) {
  DRAKE_DEMAND(!driven_meshes.empty());
  driven_meshes_.emplace(id, std::move(driven_meshes));
}

}  // namespace internal

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
  return fmt::format("Referenced geometry source {} is not registered.", key);
}

template <>
std::string get_missing_id_message<FrameId>(const FrameId& key) {
  return fmt::format("Referenced frame {} has not been registered.", key);
}

template <>
std::string get_missing_id_message<GeometryId>(const GeometryId& key) {
  return fmt::format("Referenced geometry {} has not been registered.", key);
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
                                 InternalFrame::world_frame_group(), 0, world);
  frame_index_to_id_map_.push_back(world);
  kinematics_data_.X_WFs.push_back(RigidTransform<T>::Identity());
  kinematics_data_.X_PFs.push_back(RigidTransform<T>::Identity());
  kinematics_data_.driven_mesh_data[Role::kPerception] = {};
  kinematics_data_.driven_mesh_data[Role::kIllustration] = {};
  kinematics_data_.driven_mesh_data[Role::kProximity] = {};
  deformable_render_meshes_[Role::kPerception] = {};
  deformable_render_meshes_[Role::kIllustration] = {};
  deformable_render_meshes_[Role::kProximity] = {};

  source_frame_id_map_[self_source_] = {world};
  source_deformable_geometry_id_map_[self_source_] = {};
  source_frame_name_map_[self_source_] = {"world"};
  source_root_frame_map_[self_source_] = {world};
}

namespace {

// Helper for the scalar-converting copy constructor.
// Copies the argument to use a different scalar type U => T.
// See #13618 and related for a possible generic replacement.
template <typename T, typename U>
static VectorX<T> ChangeScalarType(const VectorX<U>& other) {
  if constexpr (std::is_same_v<T, U>) {
    return other;
  } else if constexpr (std::is_same_v<U, AutoDiffXd>) {
    return math::DiscardZeroGradient(other);
  } else {
    return ExtractDoubleOrThrow(other);
  }
}

// Helper for the scalar-converting copy constructor.
// Copies the argument to use a different scalar type U => T.
// See #13618 and related for a possible generic replacement.
template <typename T, typename U>
static RigidTransform<T> ChangeScalarType(const RigidTransform<U>& other) {
  if constexpr (std::is_same_v<T, U>) {
    return other;
  } else if constexpr (std::is_same_v<U, AutoDiffXd>) {
    return RigidTransform<T>(math::DiscardZeroGradient(other.GetAsMatrix34()));
  } else {
    return RigidTransform<T>(ExtractDoubleOrThrow(other.GetAsMatrix34()));
  }
}

// Helper for ApplyProximityDefaults(). Adds any proximity properties that are
// (a) missing in `properties`, and (b) not nullopt in `defaults`.
//
// @returns true if any properties were modified.
bool BackfillDefaults(ProximityProperties* properties,
                      const DefaultProximityProperties& defaults) {
  auto backfill = [&](const std::string& group_name, const std::string& name,
                      const auto& default_value) -> bool {
    if (properties->HasProperty(group_name, name)) {
      return false;
    }
    if (!default_value.has_value()) {
      return false;
    }
    properties->AddProperty(group_name, name, *default_value);
    return true;
  };

  bool result = false;
  std::optional<HydroelasticType> wrapped_compliance(
      internal::GetHydroelasticTypeFromString(defaults.compliance_type));
  result |= backfill(kHydroGroup, kComplianceType, wrapped_compliance);

  result |= backfill(kHydroGroup, kElastic, defaults.hydroelastic_modulus);
  result |= backfill(kHydroGroup, kRezHint, defaults.resolution_hint);
  result |= backfill(kHydroGroup, kSlabThickness, defaults.slab_thickness);

  result |= backfill(kMaterialGroup, kHcDissipation,
                     defaults.hunt_crossley_dissipation);
  result |= backfill(kMaterialGroup, kRelaxationTime, defaults.relaxation_time);
  result |= backfill(kMaterialGroup, kPointStiffness, defaults.point_stiffness);
  result |= backfill(kHydroGroup, kMargin, defaults.margin);
  if (defaults.static_friction.has_value()) {
    // DefaultProximityProperties::ValidateOrThrow() enforces invariants on
    // friction quantities.
    DRAKE_DEMAND(defaults.dynamic_friction.has_value());
    const auto wrapped_friction =
        std::make_optional<multibody::CoulombFriction<double>>(
            *defaults.static_friction, *defaults.dynamic_friction);
    result |= backfill(kMaterialGroup, kFriction, wrapped_friction);
  }
  return result;
}

// Helper data structure for computing the bounding box of a deformable geometry
// that is fed to AabbMaker/ObbMaker.
struct BoundingBoxInput {
  VolumeMesh<double> mesh;
  std::set<int> vertices;
};

// Helper function to compute the ingredients for AabbMaker and ObbMaker for
// a deformable geometry given its current configuration q_WG.
BoundingBoxInput GetBoundingBoxInputFromDeformableGeometry(
    const InternalGeometry& geometry, const VectorX<double>& q_WG) {
  // TODO(SeanCurtis-TRI): Currently, each site that requires the current,
  //  deformed state of a deformable mesh must perform the computation by
  //  itself. We should provide a mechanism to compute the deformed mesh once
  //  and then provide a mechanism to query the deformed mesh.
  const VolumeMesh<double>& reference_mesh = *geometry.reference_mesh();
  std::vector<Vector3<double>> deformed_vertices(reference_mesh.num_vertices());
  for (int i = 0; i < reference_mesh.num_vertices(); ++i) {
    deformed_vertices[i] = q_WG.template segment<3>(3 * i);
  }
  // AabbMaker doesn't depend on tets, only vertices. So, we'll create a single
  // dummy tet to satisfy VolumeMesh's requirements, but otherwise ignore them.
  std::vector<VolumeElement> tets{VolumeElement(0, 1, 2, 3)};
  const VolumeMesh<double> deformed_mesh(std::move(tets),
                                         std::move(deformed_vertices));
  std::set<int> vertex_indices;
  for (int i = 0; i < deformed_mesh.num_vertices(); ++i) {
    vertex_indices.insert(i);
  }
  return BoundingBoxInput{std::move(deformed_mesh), std::move(vertex_indices)};
}

}  // namespace

// It is _vitally_ important that all members are _explicitly_ accounted for
// (either in the initialization list or in the body). Failure to do so will
// lead to errors in the converted GeometryState instance.
template <typename T>
template <typename U>
GeometryState<T>::GeometryState(const GeometryState<U>& source)
    : self_source_(source.self_source_),
      source_frame_id_map_(source.source_frame_id_map_),
      source_deformable_geometry_id_map_(
          source.source_deformable_geometry_id_map_),
      source_frame_name_map_(source.source_frame_name_map_),
      source_root_frame_map_(source.source_root_frame_map_),
      source_names_(source.source_names_),
      source_anchored_geometry_map_(source.source_anchored_geometry_map_),
      frames_(source.frames_),
      geometries_(source.geometries_),
      frame_index_to_id_map_(source.frame_index_to_id_map_),
      deformable_render_meshes_(source.deformable_render_meshes_),
      geometry_engine_(
          std::move(source.geometry_engine_->template ToScalarType<T>())),
      render_engines_(source.render_engines_),
      geometry_version_(source.geometry_version_) {
  auto convert_pose_vector = [](const std::vector<RigidTransform<U>>& s,
                                std::vector<RigidTransform<T>>* d) {
    std::vector<RigidTransform<T>>& dest = *d;
    dest.resize(s.size());
    for (size_t i = 0; i < s.size(); ++i) {
      dest[i] = ChangeScalarType<T>(s[i]);
    }
  };
  // TODO(xuchenhan-tri): The scalar conversion of KinematicsData should be
  // handled by the KinematicsData class.
  convert_pose_vector(source.kinematics_data_.X_PFs, &kinematics_data_.X_PFs);
  convert_pose_vector(source.kinematics_data_.X_WFs, &kinematics_data_.X_WFs);

  // Now convert the id -> pose map.
  {
    std::unordered_map<GeometryId, RigidTransform<T>>& dest =
        kinematics_data_.X_WGs;
    const std::unordered_map<GeometryId, RigidTransform<U>>& s =
        source.kinematics_data_.X_WGs;
    for (const auto& id_pose_pair : s) {
      const GeometryId id = id_pose_pair.first;
      const RigidTransform<U>& X_WG_source = id_pose_pair.second;
      dest.insert({id, ChangeScalarType<T>(X_WG_source)});
    }
  }

  // Now convert the id -> configuration map.
  {
    std::unordered_map<GeometryId, VectorX<T>>& dest = kinematics_data_.q_WGs;
    const std::unordered_map<GeometryId, VectorX<U>>& s =
        source.kinematics_data_.q_WGs;
    for (const auto& id_configuration_pair : s) {
      const GeometryId id = id_configuration_pair.first;
      const VectorX<U>& q_WG_source = id_configuration_pair.second;
      dest.insert({id, ChangeScalarType<T>(q_WG_source)});
    }
  }
}

template <typename T>
std::vector<GeometryId> GeometryState<T>::GetAllGeometryIds(
    std::optional<Role> role) const {
  std::vector<GeometryId> result;
  if (role.has_value()) {
    for (const auto& [geometry_id, internal_geometry] : geometries_) {
      if (internal_geometry.has_role(*role)) {
        result.push_back(geometry_id);
      }
    }
  } else {
    result.reserve(geometries_.size());
    for (const auto& [geometry_id, _] : geometries_) {
      result.push_back(geometry_id);
    }
  }
  std::sort(result.begin(), result.end());
  return result;
}

template <typename T>
unordered_set<GeometryId> GeometryState<T>::GetGeometryIds(
    const GeometrySet& geometry_set, std::optional<Role> role) const {
  return CollectIds(geometry_set, role, CollisionFilterScope::kAll);
}

template <typename T>
int GeometryState<T>::NumGeometriesWithRole(Role role) const {
  int count = 0;
  for (const auto& pair : geometries_) {
    if (pair.second.has_role(role)) ++count;
  }
  return count;
}

template <typename T>
int GeometryState<T>::NumDeformableGeometriesWithRole(Role role) const {
  int count = 0;
  for (const auto& pair : geometries_) {
    if (pair.second.has_role(role) && pair.second.is_deformable()) ++count;
  }
  return count;
}

template <typename T>
int GeometryState<T>::NumDynamicGeometries() const {
  return NumDeformableGeometries() + NumDynamicNonDeformableGeometries();
}

template <typename T>
int GeometryState<T>::NumDynamicNonDeformableGeometries() const {
  int count = 0;
  for (const auto& pair : frames_) {
    const InternalFrame& frame = pair.second;
    if (frame.id() != InternalFrame::world_frame_id()) {
      count += frame.num_child_geometries();
    }
  }
  return count;
}

template <typename T>
int GeometryState<T>::NumDeformableGeometries() const {
  int count = 0;
  for (const auto& [source_id, deformable_geometry_ids] :
       source_deformable_geometry_id_map_) {
    unused(source_id);
    count += deformable_geometry_ids.size();
  }
  return count;
}

template <typename T>
int GeometryState<T>::NumAnchoredGeometries() const {
  const InternalFrame& frame = frames_.at(InternalFrame::world_frame_id());
  int count = 0;
  const std::unordered_set<GeometryId>& child_geometries =
      frame.child_geometries();
  for (auto geometry_id : child_geometries) {
    if (!geometries_.at(geometry_id).is_deformable()) {
      ++count;
    }
  }
  return count;
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
std::vector<SourceId> GeometryState<T>::GetAllSourceIds() const {
  std::vector<SourceId> result;
  result.reserve(source_frame_id_map_.size());
  result.push_back(self_source_);
  for (const auto& [source_id, _] : source_names_) {
    if (source_id != self_source_) {
      result.push_back(source_id);
    }
  }
  std::sort(result.begin() + 1, result.end());
  return result;
}

template <typename T>
bool GeometryState<T>::SourceIsRegistered(SourceId source_id) const {
  return source_frame_id_map_.find(source_id) != source_frame_id_map_.end();
}

template <typename T>
const std::string& GeometryState<T>::GetName(SourceId id) const {
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
const FrameIdSet& GeometryState<T>::FramesForSource(SourceId source_id) const {
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
const std::string& GeometryState<T>::GetName(FrameId frame_id) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "No frame name available for invalid frame id: " +
           to_string(frame_id);
  });
  return frames_.at(frame_id).name();
}

template <typename T>
FrameId GeometryState<T>::GetParentFrame(FrameId frame_id) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "No frame name available for invalid frame id: " +
           to_string(frame_id);
  });
  const FrameId parent_frame_id = frames_.at(frame_id).parent_frame_id();
  if (parent_frame_id == frame_id) {
    // If there's no parent frame then world frame is implicitly the parent
    return InternalFrame::world_frame_id();
  }
  return parent_frame_id;
}

template <typename T>
int GeometryState<T>::GetFrameGroup(FrameId frame_id) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "No frame group available for invalid frame id: " +
           to_string(frame_id);
  });
  return frames_.at(frame_id).frame_group();
}

template <typename T>
int GeometryState<T>::NumGeometriesForFrame(FrameId frame_id) const {
  const InternalFrame& frame = GetValueOrThrow(frame_id, frames_);
  return static_cast<int>(frame.child_geometries().size());
}

template <typename T>
int GeometryState<T>::NumGeometriesForFrameWithRole(FrameId frame_id,
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
std::vector<GeometryId> GeometryState<T>::GetGeometries(
    FrameId frame_id, std::optional<Role> role) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return fmt::format(
        "Cannot report geometries associated with invalid frame id: {}",
        frame_id);
  });
  const InternalFrame& frame = frames_.at(frame_id);

  std::vector<GeometryId> ids;
  ids.reserve(frame.child_geometries().size());
  for (GeometryId g_id : frame.child_geometries()) {
    if (role.has_value()) {
      if (!geometries_.at(g_id).has_role(*role)) continue;
    }
    ids.push_back(g_id);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

template <typename T>
GeometryId GeometryState<T>::GetGeometryIdByName(
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
    std::vector<std::string_view> names;
    for (GeometryId geometry_id : frame.child_geometries()) {
      const InternalGeometry& geometry = geometries_.at(geometry_id);
      if (geometry.has_role(role)) {
        names.emplace_back(geometry.name());
      }
    }
    throw std::logic_error(fmt::format(
        "The frame '{}' ({}) has no geometry with the role '{}' and the "
        "canonical name '{}'. The names associated with this frame/role are "
        "{{{}}}.",
        frame_name, frame_id, role, canonical_name, fmt::join(names, ", ")));
  }
  // This case should only be possible for unassigned geometries - internal
  // invariants require unique names for actual geometries with the _same_
  // role on the same frame.
  DRAKE_DEMAND(role == Role::kUnassigned);
  throw std::logic_error(
      fmt::format("The frame '{}' ({}) has multiple geometries with the role "
                  "'{}' and the canonical name '{}'",
                  frame_name, frame_id, role, canonical_name));
}

template <typename T>
bool GeometryState<T>::BelongsToSource(GeometryId geometry_id,
                                       SourceId source_id) const {
  // Confirm valid source id.
  FindOrThrow(source_id, source_names_, [source_id]() {
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
std::variant<std::monostate, const TriangleSurfaceMesh<double>*,
             const VolumeMesh<double>*>
GeometryState<T>::maybe_get_hydroelastic_mesh(GeometryId geometry_id) const {
  const auto& hydro_geometries = geometry_engine_->hydroelastic_geometries();
  switch (hydro_geometries.hydroelastic_type(geometry_id)) {
    case HydroelasticType::kUndefined:
      break;
    case HydroelasticType::kRigid: {
      const auto& rigid = hydro_geometries.rigid_geometry(geometry_id);
      if (!rigid.is_half_space()) {
        return &rigid.mesh();
      }
      break;
    }
    case HydroelasticType::kSoft: {
      const auto& soft = hydro_geometries.soft_geometry(geometry_id);
      if (!soft.is_half_space()) {
        return &soft.mesh();
      }
      break;
    }
  }
  return {};
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
const VolumeMesh<double>* GeometryState<T>::GetReferenceMesh(
    GeometryId id) const {
  const InternalGeometry* geometry = GetGeometry(id);
  if (geometry == nullptr) {
    throw std::logic_error(
        fmt::format("Referenced geometry {} has not been registered", id));
  }
  return geometry->reference_mesh();
}

template <typename T>
const std::vector<RenderMesh>& GeometryState<T>::GetDrivenRenderMeshes(
    GeometryId id, Role role) const {
  const InternalGeometry* geometry = GetGeometry(id);
  DRAKE_THROW_UNLESS(role != Role::kUnassigned);
  if (geometry == nullptr || !geometry->has_role(role) ||
      !geometry->is_deformable()) {
    throw std::logic_error(
        fmt::format("Referenced geometry {} is not a registered deformable "
                    "geometry with specified role {}",
                    id, role));
  }
  return deformable_render_meshes_.at(role).at(id);
}

template <typename T>
bool GeometryState<T>::IsDeformableGeometry(GeometryId id) const {
  const InternalGeometry& geometry = GetValueOrThrow(id, geometries_);
  return geometry.is_deformable();
}

template <typename T>
std::vector<GeometryId> GeometryState<T>::GetAllDeformableGeometryIds() const {
  std::vector<GeometryId> ids;
  for (const auto& it : source_deformable_geometry_id_map_) {
    ids.insert(ids.end(), it.second.begin(), it.second.end());
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

namespace {

// Extracts a convex hull from the two shapes that support it (returning
// nullptr for everything else). Essentially, this merely serves as a mechanism
// for finding a pointer to a convex hull that is stored with the corresponding
// InternalGeometry. Therefore, the lifespan of a HullExtractor has no bearing
// on the lifespan of the pointer returned.
class HullExtractor final : public ShapeReifier {
 public:
  HullExtractor() = default;
  ~HullExtractor() = default;

  const PolygonSurfaceMesh<double>* GetConvexHull() const { return hull_; }

 private:
  // Don't throw for the rest, nullptr is fine.
  void ThrowUnsupportedGeometry(const std::string&) final {}

  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Mesh& mesh, void*) final {
    hull_ = &mesh.GetConvexHull();
  }

  void ImplementGeometry(const Convex& convex, void*) final {
    hull_ = &convex.GetConvexHull();
  }

  const PolygonSurfaceMesh<double>* hull_{nullptr};
};

}  // namespace

template <typename T>
const PolygonSurfaceMesh<double>* GeometryState<T>::GetConvexHull(
    GeometryId id) const {
  const InternalGeometry& geometry = GetValueOrThrow(id, geometries_);
  HullExtractor extractor;
  geometry.shape().Reify(&extractor);
  return extractor.GetConvexHull();
}

template <typename T>
const std::optional<Obb>& GeometryState<T>::GetObbInGeometryFrame(
    GeometryId id) const {
  const InternalGeometry& geometry = GetValueOrThrow(id, geometries_);
  return geometry.GetObb();
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
      return !geometry_engine_->collision_filter().CanCollideWith(
          geometry1->id(), geometry2->id());
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
  return kinematics_data_.X_WFs[frames_.at(frame_id).index()];
}

template <typename T>
const math::RigidTransform<T>& GeometryState<T>::get_pose_in_world(
    GeometryId geometry_id) const {
  FindOrThrow(geometry_id, geometries_, [geometry_id]() {
    return "No world pose available for invalid geometry id: " +
           to_string(geometry_id);
  });
  const auto& geometry = GetValueOrThrow(geometry_id, geometries_);
  if (geometry.is_deformable()) {
    throw std::logic_error(
        "Deformable geometries are characterized by vertex positions. Use "
        "get_configurations_in_world() instead.");
  }
  return kinematics_data_.X_WGs.at(geometry_id);
}

template <typename T>
std::optional<Aabb> GeometryState<T>::ComputeAabbInWorld(
    GeometryId geometry_id) const {
  FindOrThrow(geometry_id, geometries_, [geometry_id]() {
    return fmt::format("No AABB available for invalid geometry id: {}.",
                       geometry_id);
  });
  const auto& geometry = GetValueOrThrow(geometry_id, geometries_);
  // For non-deformable geometries, we don't support computing the AABB (yet).
  // TODO(SeanCurtis-TRI): Support computing AABB for shapes (#15121).
  if (!geometry.is_deformable()) {
    return std::nullopt;
  }
  // For deformable geometries with proximity role, the proximity engine
  // already keeps track of the AABB.
  if (geometry.has_proximity_role()) {
    return geometry_engine_->GetDeformableAabbInWorld(geometry_id);
  }
  // For deformable geometries without proximity role, we need to manually
  // deform the geometry and compute the AABB of the deformed mesh.
  const BoundingBoxInput input = GetBoundingBoxInputFromDeformableGeometry(
      geometry, convert_to_double(kinematics_data_.q_WGs.at(geometry_id)));
  return AabbMaker<VolumeMesh<double>>(input.mesh, input.vertices).Compute();
}

template <typename T>
std::optional<Obb> GeometryState<T>::ComputeObbInWorld(
    GeometryId geometry_id) const {
  FindOrThrow(geometry_id, geometries_, [geometry_id]() {
    return fmt::format("No OBB available for invalid geometry id: {}.",
                       geometry_id);
  });
  const auto& geometry = GetValueOrThrow(geometry_id, geometries_);
  // For deformable geometries, we need to recompute the deformed mesh and
  // compute the OBB of the deformed mesh.
  if (geometry.is_deformable()) {
    const BoundingBoxInput input = GetBoundingBoxInputFromDeformableGeometry(
        geometry, convert_to_double(kinematics_data_.q_WGs.at(geometry_id)));
    return ObbMaker<VolumeMesh<double>>(input.mesh, input.vertices).Compute();
  }
  // For rigid geometries, we use the cached the geometry frame OBB and simply
  // transform it to the world frame.
  const std::optional<Obb>& obb_G = GetObbInGeometryFrame(geometry_id);
  if (!obb_G.has_value()) {
    return std::nullopt;
  }
  const math::RigidTransform<double>& X_WG =
      convert_to_double(get_pose_in_world(geometry_id));
  const math::RigidTransform<double>& X_GB = obb_G->pose();
  return Obb(X_WG * X_GB, obb_G->half_width());
}

template <typename T>
const math::RigidTransform<T>& GeometryState<T>::get_pose_in_parent(
    FrameId frame_id) const {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "No pose available for invalid frame id: " + to_string(frame_id);
  });
  return kinematics_data_.X_PFs[frames_.at(frame_id).index()];
}

template <typename T>
const VectorX<T>& GeometryState<T>::get_configurations_in_world(
    GeometryId geometry_id) const {
  FindOrThrow(geometry_id, geometries_, [geometry_id]() {
    return "No world configurations available for invalid geometry id: " +
           to_string(geometry_id);
  });
  const auto& geometry = GetValueOrThrow(geometry_id, geometries_);
  if (!geometry.is_deformable()) {
    throw std::logic_error(
        "Non-deformable geometries are characterized by poses. Use "
        "get_pose_in_world() instead.");
  }
  return kinematics_data_.q_WGs.at(geometry_id);
}

template <typename T>
std::vector<VectorX<T>> GeometryState<T>::GetDrivenMeshConfigurationsInWorld(
    GeometryId geometry_id, Role role) const {
  FindOrThrow(geometry_id, geometries_, [geometry_id]() {
    return "No mesh configurations available for invalid geometry id: " +
           to_string(geometry_id);
  });
  const auto& geometry = GetValueOrThrow(geometry_id, geometries_);
  DRAKE_THROW_UNLESS(geometry.is_deformable());
  DRAKE_THROW_UNLESS(geometry.has_role(role));
  DRAKE_THROW_UNLESS(role != Role::kUnassigned);

  auto calc_configuration = [&](const internal::DrivenMeshData& data) {
    std::vector<VectorX<T>> result;
    DRAKE_THROW_UNLESS(data.driven_meshes().contains(geometry_id));
    for (const auto& mesh : data.driven_meshes().at(geometry_id)) {
      result.emplace_back(mesh.GetDrivenVertexPositions());
    }
    return result;
  };

  return calc_configuration(kinematics_data_.driven_mesh_data.at(role));
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
  source_deformable_geometry_id_map_[source_id];
  source_frame_name_map_[source_id];
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

  if (frames_.contains(frame_id)) {
    throw std::logic_error(
        "Registering frame with an id that has already been registered: " +
        to_string(frame_id));
  }

  FrameIdSet& f_set = GetMutableValueOrThrow(source_id, &source_frame_id_map_);
  if (parent_id != InternalFrame::world_frame_id()) {
    FindOrThrow(parent_id, f_set, [parent_id, source_id]() {
      return "Indicated parent id " + to_string(parent_id) +
             " does not belong "
             "to the indicated source id " +
             to_string(source_id) + ".";
    });
    frames_[parent_id].add_child(frame_id);
  } else {
    // The parent is the world frame; register it as a root frame.
    source_root_frame_map_[source_id].insert(frame_id);
  }
  FrameNameSet& f_name_set = source_frame_name_map_[source_id];
  const auto& [iterator, was_inserted] = f_name_set.insert(frame.name());
  if (!was_inserted) {
    throw std::logic_error(
        fmt::format("Registering frame for source '{}'"
                    " with a duplicate name '{}'",
                    source_names_[source_id], frame.name()));
  }

  DRAKE_ASSERT(kinematics_data_.X_PFs.size() == frame_index_to_id_map_.size());
  int index(static_cast<int>(kinematics_data_.X_PFs.size()));
  kinematics_data_.X_PFs.emplace_back(RigidTransform<T>::Identity());
  kinematics_data_.X_WFs.emplace_back(RigidTransform<T>::Identity());
  frame_index_to_id_map_.push_back(frame_id);
  f_set.insert(frame_id);
  frames_.emplace(frame_id,
                  InternalFrame(source_id, frame_id, frame.name(),
                                frame.frame_group(), index, parent_id));
  return frame_id;
}

template <typename T>
void GeometryState<T>::RenameFrame(FrameId frame_id, const std::string& name) {
  FindOrThrow(frame_id, frames_, [frame_id]() {
    return "Cannot rename frame with invalid frame id: " + to_string(frame_id);
  });
  InternalFrame& frame = frames_.at(frame_id);
  const std::string old_name(frame.name());
  if (old_name == name) {
    return;
  }

  SourceId source_id = frame.source_id();

  // Edit source_frame_name_map_.
  FrameNameSet& f_name_set = source_frame_name_map_.at(source_id);
  f_name_set.erase(old_name);
  const auto& [iterator, was_inserted] = f_name_set.insert(std::string(name));
  if (!was_inserted) {
    throw std::logic_error(
        fmt::format("Renaming frame from '{}'"
                    " to an already existing name '{}'",
                    old_name, name));
  }

  // Edit internal frame object.
  frame.set_name(name);
}

template <typename T>
GeometryId GeometryState<T>::RegisterGeometry(
    SourceId source_id, FrameId frame_id,
    std::unique_ptr<GeometryInstance> geometry) {
  if (geometry == nullptr) {
    throw std::logic_error("Registering null geometry to frame " +
                           to_string(frame_id) + ", on source " +
                           to_string(source_id) + ".");
  }
  const GeometryId geometry_id = geometry->id();
  ValidateRegistrationAndSetTopology(source_id, frame_id, geometry_id);

  // pose() is always RigidTransform<double>. To account for
  // GeometryState<AutoDiff>, we need to cast it to the common type T.
  const InternalFrame& frame = frames_[frame_id];
  kinematics_data_.X_WGs[geometry_id] =
      kinematics_data_.X_WFs[frame.index()] * geometry->pose().cast<T>();
  geometries_.emplace(
      geometry_id,
      InternalGeometry(source_id, geometry->release_shape(), frame_id,
                       geometry_id, geometry->name(), geometry->pose()));

  AssignAllDefinedRoles(source_id, std::move(geometry));

  return geometry_id;
}

template <typename T>
GeometryId GeometryState<T>::RegisterDeformableGeometry(
    SourceId source_id, FrameId frame_id,
    std::unique_ptr<GeometryInstance> geometry, double resolution_hint) {
  if (geometry == nullptr) {
    throw std::logic_error("Registering null geometry to frame " +
                           to_string(frame_id) + ", on source " +
                           to_string(source_id) + ".");
  }

  const GeometryId geometry_id = geometry->id();
  if (frame_id != InternalFrame::world_frame_id()) {
    throw std::logic_error("Registering deformable geometry with id " +
                           to_string(geometry_id) + " to a non-world frame");
  }

  ValidateRegistrationAndSetTopology(source_id, frame_id, geometry_id);
  source_deformable_geometry_id_map_[source_id].insert(geometry_id);

  InternalGeometry internal_geometry(source_id, geometry->release_shape(),
                                     frame_id, geometry_id, geometry->name(),
                                     geometry->pose(), resolution_hint);
  // The reference mesh is defined in the frame F.
  const VolumeMesh<double>* reference_mesh = internal_geometry.reference_mesh();
  DRAKE_DEMAND(reference_mesh != nullptr);
  const InternalFrame& frame = frames_[frame_id];
  const RigidTransform<T> X_WG =
      kinematics_data_.X_WFs[frame.index()] * geometry->pose().cast<T>();
  VectorX<T> q_WG(reference_mesh->num_vertices() * 3);
  for (int v = 0; v < reference_mesh->num_vertices(); ++v) {
    q_WG.template segment<3>(3 * v) =
        X_WG * Vector3<T>(reference_mesh->vertex(v));
  }
  kinematics_data_.q_WGs[geometry_id] = std::move(q_WG);
  geometries_.emplace(geometry_id, std::move(internal_geometry));

  AssignAllDefinedRoles(source_id, std::move(geometry));

  return geometry_id;
}

template <typename T>
void GeometryState<T>::RenameGeometry(GeometryId geometry_id,
                                      const std::string& name) {
  InternalGeometry* geometry = GetMutableGeometry(geometry_id);
  if (geometry == nullptr) {
    throw std::logic_error("Cannot rename geometry with invalid geometry id: " +
                           to_string(geometry_id));
  }
  if (geometry->name() == name) {
    return;
  }

  // Check for name uniqueness in all assigned roles. Note: if the universe of
  // roles grows, this iteration will need to grow as well.
  for (Role role : {Role::kProximity, Role::kIllustration, Role::kPerception}) {
    if (geometry->has_role(role)) {
      ThrowIfNameExistsInRole(geometry->frame_id(), role, name);
    }
  }

  // Edit internal geometry object.
  geometry->set_name(name);
}

template <typename T>
void GeometryState<T>::ChangeShape(SourceId source_id, GeometryId geometry_id,
                                   const Shape& shape,
                                   std::optional<RigidTransformd> X_FG) {
  if (!BelongsToSource(geometry_id, source_id)) {
    throw std::logic_error("Given geometry id " + to_string(geometry_id) +
                           " does not belong to the given source id " +
                           to_string(source_id));
  }
  InternalGeometry* geometry = GetMutableGeometry(geometry_id);
  // Must be non-null, otherwise, we never would've gotten past the
  // `BelongsToSource()` call.
  DRAKE_DEMAND(geometry != nullptr);

  // TODO(SeanCurtis-TRI) Allow changing deformable geometries after the fact;
  // this would require coordination with MbP because the state of the
  // deformable object  must be consistent between the two systems; the size of
  // the data in the corresponding port would have to change on both sides.
  if (geometry->is_deformable()) {
    throw std::logic_error(
        "Cannot use ChangeShape() to change the shape of deformable "
        "geometries.");
  }

  geometry->SetShape(shape);
  if (X_FG.has_value()) {
    // As documented on SceneGraph::SetShape(); use the old pose unless
    // explicitly changed.
    geometry->set_pose(*X_FG);
  }
  // We've changed pose and shape; now we just need to notify the various
  // engines to update themselves.
  if (geometry->has_proximity_role()) {
    // Proximity engine is best handled by removal and re-addition; we use the
    // unchecked version because we just need the engine mechanism; no
    // further GeometryState checking.
    RemoveFromProximityEngineUnchecked(*geometry);
    AddToProximityEngineUnchecked(*geometry);
  }
  if (geometry->has_illustration_role()) {
    // Illustration has no "engine"; it's just the InternalGeometry. All
    // reifications of illustration geometry happen outside of SceneGraph. We
    // just need to let them know that the work is necessary.
    geometry_version_.modify_illustration();
  }
  if (geometry->has_perception_role()) {
    // Render engines are best handled by removal and re-addition; we use the
    // unchecked version because we just need the engine mechanism; no
    // further GeometryState checking.
    RemoveFromAllRenderersUnchecked(geometry_id);
    AddToCompatibleRenderersUnchecked(*geometry);
  }
}

template <typename T>
GeometryId GeometryState<T>::RegisterAnchoredGeometry(
    SourceId source_id, std::unique_ptr<GeometryInstance> geometry) {
  return RegisterGeometry(source_id, InternalFrame::world_frame_id(),
                          std::move(geometry));
}

template <typename T>
void GeometryState<T>::RemoveGeometry(SourceId source_id,
                                      GeometryId geometry_id) {
  if (!BelongsToSource(geometry_id, source_id)) {
    throw std::logic_error("Trying to remove geometry " +
                           to_string(geometry_id) +
                           " from "
                           "source " +
                           to_string(source_id) +
                           ", but the geometry doesn't "
                           "belong to that source.");
  }

  const InternalGeometry& geometry = GetValueOrThrow(geometry_id, geometries_);
  auto& frame = GetMutableValueOrThrow(geometry.frame_id(), &frames_);
  frame.remove_child(geometry_id);

  RemoveProximityRole(geometry_id);
  RemovePerceptionRole(geometry_id);
  RemoveIllustrationRole(geometry_id);

  // Clean up state collections.
  kinematics_data_.X_WGs.erase(geometry_id);
  kinematics_data_.q_WGs.erase(geometry_id);

  // Remove from the geometries.
  geometries_.erase(geometry_id);
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

template <typename T>
void GeometryState<T>::AssignRole(SourceId source_id, GeometryId geometry_id,
                                  ProximityProperties properties,
                                  RoleAssign assign) {
  InternalGeometry& geometry =
      ValidateRoleAssign(source_id, geometry_id, Role::kProximity, assign);

  geometry_version_.modify_proximity();
  switch (assign) {
    case RoleAssign::kNew: {
      geometry.SetRole(std::move(properties));
      if (geometry.is_deformable()) {
        DRAKE_DEMAND(geometry.reference_mesh() != nullptr);
        const VolumeMesh<double>& reference_mesh = *geometry.reference_mesh();
        std::vector<int> surface_vertices;
        std::vector<int> surface_tri_to_volume_tet;
        TriangleSurfaceMesh<double> surface_mesh =
            ConvertVolumeToSurfaceMeshWithBoundaryVertices(
                reference_mesh, &surface_vertices, &surface_tri_to_volume_tet);

        geometry_engine_->AddDeformableGeometry(
            reference_mesh, surface_mesh, surface_vertices,
            surface_tri_to_volume_tet, geometry_id);
        VertexSampler vertex_sampler(std::move(surface_vertices),
                                     reference_mesh);
        std::vector<DrivenTriangleMesh> driven_meshes;
        driven_meshes.emplace_back(vertex_sampler, surface_mesh);
        kinematics_data_.driven_mesh_data[Role::kProximity].SetMeshes(
            geometry_id, std::move(driven_meshes));
      } else if (geometry.is_dynamic()) {
        // Pass the geometry to the engine.
        const RigidTransformd& X_WG =
            convert_to_double(kinematics_data_.X_WGs.at(geometry_id));
        geometry_engine_->AddDynamicGeometry(geometry.shape(), X_WG,
                                             geometry_id,
                                             *geometry.proximity_properties());
      } else {
        geometry_engine_->AddAnchoredGeometry(geometry.shape(), geometry.X_FG(),
                                              geometry_id,
                                              *geometry.proximity_properties());
      }
      // The set of geometries G such that I need to introduce filtered pairs
      // (geometry_id, gᵢ) ∀ gᵢ ∈ G. Generally, it consists of those proximity
      // geometries affixed to the same frame as geometry_id (that frame would
      // be the world frame for anchored geometry). To that end, we'll blindly
      // add the id for the geometry's frame to the set. Worst case, there are
      // no other geometries affixed to that frame -- attempting to apply
      // filters in that case would be a harmless act.
      GeometrySet ids_for_filtering;
      ids_for_filtering.Add(geometry.frame_id());
      // Apply collision filter between geometry id and any geometries that have
      // been identified. If none have been identified, this makes no changes.
      // Per public documentation of SceneGraph, we exclude deformable
      // geometries and only filter among rigid geometries.
      geometry_engine_->collision_filter().Apply(
          CollisionFilterDeclaration(CollisionFilterScope::kOmitDeformable)
              .ExcludeBetween(GeometrySet(geometry_id), ids_for_filtering),
          [this](const GeometrySet& set, CollisionFilterScope scope) {
            return this->CollectIds(set, Role::kProximity, scope);
          },
          true /* is_invariant */);
    } break;
    case RoleAssign::kReplace:
      // Give the engine a chance to compare properties before and after.
      geometry_engine_->UpdateRepresentationForNewProperties(geometry,
                                                             properties);
      geometry.SetRole(std::move(properties));
      break;
    default:
      DRAKE_UNREACHABLE();
  }

  // If the geometry is deformable, we need to register its driven meshes. We
  // always blindly throw out the old driven mesh data and replace it with a new
  // driven mesh data.
  if (geometry.is_deformable()) {
    RegisterDrivenMesh(geometry_id, Role::kProximity);
  }
}

template <typename T>
void GeometryState<T>::AssignRole(SourceId source_id, GeometryId geometry_id,
                                  PerceptionProperties properties,
                                  RoleAssign assign) {
  if (assign == RoleAssign::kReplace) {
    throw std::logic_error(
        "AssignRole() with RoleAssign::kReplace does not work for perception "
        "properties");
  }

  InternalGeometry& geometry =
      ValidateRoleAssign(source_id, geometry_id, Role::kPerception, assign);

  // TODO(SeanCurtis-TRI): To support RoleAssign::kReplace, the render engines
  //  need to handle these changes.

  geometry.SetRole(std::move(properties));

  if (geometry.is_deformable()) {
    RegisterDrivenMesh(geometry_id, Role::kPerception);
  }

  const bool added_to_renderer = AddToCompatibleRenderersUnchecked(geometry);

  if (!added_to_renderer && render_engines_.size() > 0) {
    // TODO(SeanCurtis-TRI): This message would be better with a geometry name.
    drake::log()->warn(
        "Perception role assigned to geometry {}, but no renderer accepted it",
        geometry_id);
  }
}

template <typename T>
void GeometryState<T>::AssignRole(SourceId source_id, GeometryId geometry_id,
                                  IllustrationProperties properties,
                                  RoleAssign assign) {
  InternalGeometry& geometry =
      ValidateRoleAssign(source_id, geometry_id, Role::kIllustration, assign);

  // We don't warn until after all the error checking has already happened.
  if (properties.HasProperty("phong", "diffuse_map")) {
    static logging::Warn log_once(
        "Explicitly defined values for the ('phong', 'diffuse_map') property "
        "are not currently used in illustration roles -- only perception "
        "roles. This warning is only shown during SceneGraph's first encounter "
        "with an ignored 'diffuse_map', which occurred with the geometry named "
        "'{}' on a geometry frame named '{}'; "
        "further encounters will be silently ignored.",
        GetName(geometry_id), GetName(GetFrameId(geometry_id)));
  }
  // TODO(SeanCurtis-TRI): We want to remove this warning. For that to happen,
  // we need systems dependent on illustration properties to recognize if there
  // has been a change since last they processed the state. The simplest way to
  // do that is to have a monotonically increasing serial number on
  // GeometryState. It would increment for *every* change to GeometryState. A
  // visualizer could query to determine if the serial number matches the value
  // it had when it last initialized. If not, it can re-intialize (whatever that
  // entails). It might also be advisable to have a collection of serial numbers
  // with reduced scope (i.e., so a change to proximity properties doesn't
  // cause illustration systems to reinitialize).
  if (assign == RoleAssign::kReplace) {
    static logging::Warn log_once(
        "Updating illustration role properties must be done before visualizer "
        "initialization to have an effect. When in doubt, after making "
        "property changes, force the visualizer to re-initialize via its API.");
  }

  geometry_version_.modify_illustration();

  geometry.SetRole(std::move(properties));

  if (geometry.is_deformable()) {
    RegisterDrivenMesh(geometry_id, Role::kIllustration);
  }
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
    throw std::logic_error("Trying to remove the role " + to_string(role) +
                           " from the geometry " + to_string(geometry_id) +
                           " from source " + to_string(source_id) +
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
    throw std::logic_error("Trying to remove geometry " +
                           to_string(geometry_id) +
                           " from the "
                           "renderer '" +
                           renderer_name +
                           "', but the geometry doesn't belong to "
                           "given source " +
                           to_string(source_id) + ".");
  }

  return RemoveFromRendererUnchecked(renderer_name, geometry_id) ? 1 : 0;
}

namespace {

void ThrowForNonProximity(const internal::InternalGeometry& g,
                          const char* purpose) {
  if (!g.has_proximity_role()) {
    const char* role_description =
        g.has_illustration_role()
            ? "the illustration role"
            : (g.has_perception_role() ? "the perception role" : "no role");
    throw std::logic_error(
        fmt::format("The geometry {} cannot be used in {}; it does not have a "
                    "proximity role. It has {}.",
                    g.id(), purpose, role_description));
  }
}

}  // namespace

template <typename T>
SignedDistancePair<T> GeometryState<T>::ComputeSignedDistancePairClosestPoints(
    GeometryId id_A, GeometryId id_B) const {
  ThrowForNonProximity(GetValueOrThrow(id_A, geometries_), __func__);
  ThrowForNonProximity(GetValueOrThrow(id_B, geometries_), __func__);
  return geometry_engine_->ComputeSignedDistancePairClosestPoints(
      id_A, id_B, kinematics_data_.X_WGs);
}

template <typename T>
std::vector<SignedDistanceToPoint<T>>
GeometryState<T>::ComputeSignedDistanceGeometryToPoint(
    const Vector3<T>& p_WQ, const GeometrySet& geometries) const {
  // We're supposed to throw for bad geometry ids and deformable geometry ids.
  // CollectIds will throw for bad geometry ids, but not deformable ids; it will
  // only ignore them. So, we include them in the `ids` (kAll) and rely on
  // ProximityEngine to throw if `ids` includes deformable ids.
  std::unordered_set<GeometryId> ids =
      CollectIds(geometries, std::nullopt, CollisionFilterScope::kAll);
  return geometry_engine_->ComputeSignedDistanceGeometryToPoint(
      p_WQ, kinematics_data_.X_WGs, ids);
}

template <typename T>
void GeometryState<T>::AddRenderer(
    std::string name, std::shared_ptr<render::RenderEngine> renderer) {
  if (render_engines_.contains(name)) {
    throw std::logic_error(fmt::format(
        "AddRenderer(): A renderer with the name '{}' already exists", name));
  }
  render::RenderEngine* render_engine = renderer.get();
  render_engines_.emplace(name, std::move(renderer));
  bool accepted = false;
  for (auto& id_geo_pair : geometries_) {
    InternalGeometry& geometry = id_geo_pair.second;
    // To add this geometry to the renderer, it must:
    //   1. Have perception role and
    //   2. Be an acceptable renderer -- it is acceptable if the geometry hasn't
    //      declared *any* acceptable renderers or if this renderer's name has
    //      been explicitly included in its acceptable set.
    if (geometry.has_perception_role()) {
      const PerceptionProperties* properties = geometry.perception_properties();
      DRAKE_DEMAND(properties != nullptr);
      auto accepting_renderers = properties->GetPropertyOrDefault(
          "renderer", "accepting", set<string>{});
      if (accepting_renderers.empty() || accepting_renderers.contains(name)) {
        const GeometryId id = id_geo_pair.first;
        if (geometry.is_deformable()) {
          accepted |= render_engine->RegisterDeformableVisual(
              id, deformable_render_meshes_.at(Role::kPerception).at(id),
              *properties);
        } else {
          accepted |= render_engine->RegisterVisual(
              id, geometry.shape(), *properties,
              RigidTransformd(geometry.X_FG()), geometry.is_dynamic());
        }
      }
    }
  }
  // Increment version number if any geometry is registered to the new
  // renderer.
  if (accepted) {
    geometry_version_.modify_perception();
  }
}

template <typename T>
void GeometryState<T>::RemoveRenderer(const std::string& name) {
  if (!render_engines_.contains(name)) {
    throw std::logic_error(fmt::format(
        "RemoveRenderer(): A renderer with the name '{}' does not exist",
        name));
  }
  render_engines_.erase(name);
  geometry_version_.modify_perception();
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
void GeometryState<T>::RenderColorImage(const ColorRenderCamera& camera,
                                        FrameId parent_frame,
                                        const RigidTransformd& X_PC,
                                        ImageRgba8U* color_image_out) const {
  const RigidTransformd X_WC =
      CalcCameraWorldPose(camera.core(), parent_frame, X_PC);
  const render::RenderEngine& engine =
      GetRenderEngineOrThrow(camera.core().renderer_name());
  // TODO(SeanCurtis-TRI): Invoke UpdateViewpoint() as part of a calc cache
  //  entry. Challenge: how to do that with a parameter passed here?
  const_cast<render::RenderEngine&>(engine).UpdateViewpoint(X_WC);
  engine.RenderColorImage(camera, color_image_out);
}

template <typename T>
void GeometryState<T>::RenderDepthImage(const DepthRenderCamera& camera,
                                        FrameId parent_frame,
                                        const RigidTransformd& X_PC,
                                        ImageDepth32F* depth_image_out) const {
  const RigidTransformd X_WC =
      CalcCameraWorldPose(camera.core(), parent_frame, X_PC);
  const render::RenderEngine& engine =
      GetRenderEngineOrThrow(camera.core().renderer_name());
  // See note in RenderColorImage() about this const cast.
  const_cast<render::RenderEngine&>(engine).UpdateViewpoint(X_WC);
  engine.RenderDepthImage(camera, depth_image_out);
}

template <typename T>
void GeometryState<T>::RenderLabelImage(const ColorRenderCamera& camera,
                                        FrameId parent_frame,
                                        const RigidTransformd& X_PC,
                                        ImageLabel16I* label_image_out) const {
  const RigidTransformd X_WC =
      CalcCameraWorldPose(camera.core(), parent_frame, X_PC);
  const render::RenderEngine& engine =
      GetRenderEngineOrThrow(camera.core().renderer_name());
  // See note in RenderColorImage() about this const cast.
  const_cast<render::RenderEngine&>(engine).UpdateViewpoint(X_WC);
  engine.RenderLabelImage(camera, label_image_out);
}

template <typename T>
std::unique_ptr<GeometryState<AutoDiffXd>> GeometryState<T>::ToAutoDiffXd()
    const {
  return std::unique_ptr<GeometryState<AutoDiffXd>>(
      new GeometryState<AutoDiffXd>(*this));
}

template <typename T>
void GeometryState<T>::ApplyProximityDefaults(
    const DefaultProximityProperties& defaults) {
  for (const auto& geometry_id : GetAllGeometryIds(Role::kProximity)) {
    ApplyProximityDefaults(defaults, geometry_id);
  }
}

template <typename T>
void GeometryState<T>::ApplyProximityDefaults(
    const DefaultProximityProperties& defaults, GeometryId geometry_id) {
  // TODO(#20820) Maybe this can be removed later.
  // Leave deformables untouched.
  if (IsDeformableGeometry(geometry_id)) {
    return;
  }

  // Get current proximity properties, required by documented precondition.
  const auto* found_props = GetProximityProperties(geometry_id);
  DRAKE_DEMAND(found_props != nullptr);
  ProximityProperties props(*found_props);

  // Update properties with defaults. Return early if nothing changed.
  bool changed = BackfillDefaults(&props, defaults);
  if (!changed) {
    return;
  }

  // Make the final changes to proximity properties.
  AssignRole(get_source_id(geometry_id), geometry_id, props,
             RoleAssign::kReplace);
}

template <typename T>
unordered_set<GeometryId> GeometryState<T>::CollectIds(
    const GeometrySet& geometry_set, std::optional<Role> role,
    CollisionFilterScope scope) const {
  auto must_include = [scope](const InternalGeometry& g,
                              const std::optional<Role>& r) {
    // Must have compatible role and be part of the scope.
    return (!r.has_value() || g.has_role(*r)) &&
           (scope == CollisionFilterScope::kAll || !g.is_deformable());
  };
  unordered_set<GeometryId> resultant_ids;
  for (auto frame_id : geometry_set.frames()) {
    const auto& frame = GetValueOrThrow(frame_id, frames_);
    for (auto geometry_id : frame.child_geometries()) {
      const InternalGeometry& geometry = geometries_.at(geometry_id);
      if (must_include(geometry, role)) {
        resultant_ids.insert(geometry_id);
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
    if (must_include(*geometry, role)) {
      resultant_ids.insert(geometry_id);
    }
  }

  return resultant_ids;
}

template <typename T>
void GeometryState<T>::SetFramePoses(
    const SourceId source_id, const FramePoseVector<T>& poses,
    internal::KinematicsData<T>* kinematics_data) const {
  // TODO(SeanCurtis-TRI): Down the road, make this validation depend on
  // ASSERT_ARMED.
  ValidateFrameIds(source_id, poses);
  const RigidTransform<T> world_pose = RigidTransform<T>::Identity();
  for (auto frame_id : source_root_frame_map_.at(source_id)) {
    UpdatePosesRecursively(frames_.at(frame_id), world_pose, poses,
                           kinematics_data);
  }
}

template <typename T>
void GeometryState<T>::SetGeometryConfiguration(
    SourceId source_id, const GeometryConfigurationVector<T>& configurations,
    internal::KinematicsData<T>* kinematics_data) const {
  const GeometryIdSet& g_ids =
      GetValueOrThrow(source_id, source_deformable_geometry_id_map_);
  for (const auto g_id : g_ids) {
    kinematics_data->q_WGs[g_id] = configurations.value(g_id);
  }
}

template <typename T>
template <typename ValueType>
void GeometryState<T>::ValidateFrameIds(
    const SourceId source_id,
    const KinematicsVector<FrameId, ValueType>& kinematics_data) const {
  auto& frames = FramesForSource(source_id);
  const int ref_frame_count = static_cast<int>(frames.size());
  if (ref_frame_count != kinematics_data.size()) {
    // TODO(SeanCurtis-TRI): Determine if more specific information is required.
    // e.g., which frames are missing/added.
    throw std::runtime_error("Disagreement in expected number of frames (" +
                             to_string(frames.size()) +
                             ") and the given number of frames (" +
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
void GeometryState<T>::ValidateRegistrationAndSetTopology(
    SourceId source_id, FrameId frame_id, GeometryId geometry_id) {
  if (geometries_.contains(geometry_id)) {
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

  FrameIdSet& set =
      GetMutableValueOrThrow(frame_source_id, &source_frame_id_map_);

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
}

template <typename T>
void GeometryState<T>::FinalizePoseUpdate(
    const internal::KinematicsData<T>& kinematics_data,
    internal::ProximityEngine<T>* proximity_engine,
    std::vector<render::RenderEngine*> render_engines) const {
  proximity_engine->UpdateWorldPoses(kinematics_data.X_WGs);
  for (auto* render_engine : render_engines) {
    render_engine->UpdatePoses(kinematics_data.X_WGs);
  }
}

template <typename T>
void GeometryState<T>::FinalizeConfigurationUpdate(
    const internal::KinematicsData<T>& kinematics_data,
    internal::ProximityEngine<T>* proximity_engine,
    std::vector<render::RenderEngine*> render_engines) const {
  const internal::DrivenMeshData& proximity_driven_mesh_data =
      kinematics_data.driven_mesh_data.at(Role::kProximity);
  proximity_engine->UpdateDeformableVertexPositions(
      kinematics_data.q_WGs, proximity_driven_mesh_data.driven_meshes());
  const internal::DrivenMeshData& perception_driven_mesh_data =
      kinematics_data.driven_mesh_data.at(Role::kPerception);
  for (const auto& [id, meshes] : perception_driven_mesh_data.driven_meshes()) {
    // Vertex positions of driven meshes.
    std::vector<VectorX<double>> q_WDs(meshes.size());
    // Vertex normals of driven meshes.
    std::vector<VectorX<double>> nhats_W(meshes.size());
    for (int i = 0; i < ssize(meshes); ++i) {
      // TODO(xuchenhan-tri): Consider eliminating the copy here if performance
      // is an issue.
      q_WDs[i] = meshes[i].GetDrivenVertexPositions();
      nhats_W[i] = meshes[i].GetDrivenVertexNormals();
    }
    for (auto* render_engine : render_engines) {
      render_engine->UpdateDeformableConfigurations(id, q_WDs, nhats_W);
    }
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
void GeometryState<T>::UpdatePosesRecursively(
    const internal::InternalFrame& frame, const RigidTransform<T>& X_WP,
    const FramePoseVector<T>& poses,
    internal::KinematicsData<T>* kinematics_data) const {
  const auto frame_id = frame.id();
  const auto& X_PF = poses.value(frame_id);
  // Cache this transform for later use.
  kinematics_data->X_PFs[frame.index()] = X_PF;
  RigidTransform<T> X_WF = X_WP * X_PF;
  kinematics_data->X_WFs[frame.index()] = X_WF;
  // Update the geometry which belong to *this* frame.
  for (auto child_id : frame.child_geometries()) {
    const auto& child_geometry = geometries_.at(child_id);
    // X_FG() is always RigidTransform<double>, to account for
    // GeometryState<AutoDiff>, we need to cast it to the common type T.
    RigidTransform<double> X_FG(child_geometry.X_FG());
    kinematics_data->X_WGs[child_id] = X_WF * X_FG.cast<T>();
  }

  // Update each child frame.
  for (auto child_id : frame.child_frames()) {
    const auto& child_frame = frames_.at(child_id);
    UpdatePosesRecursively(child_frame, X_WF, poses, kinematics_data);
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
    throw std::logic_error("The name '" + name +
                           "' has already been used by "
                           "a geometry with the '" +
                           to_string(role) + "' role.");
  }
}

template <typename T>
void GeometryState<T>::AssignAllDefinedRoles(
    SourceId source_id, std::unique_ptr<GeometryInstance> geometry) {
  DRAKE_DEMAND(geometry != nullptr);

  const GeometryId geometry_id = geometry->id();
  // Any roles defined on the geometry instance propagate through
  // automatically.
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
}

template <typename T>
InternalGeometry& GeometryState<T>::ValidateRoleAssign(SourceId source_id,
                                                       GeometryId geometry_id,
                                                       Role role,
                                                       RoleAssign assign) {
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
        "Trying to assign the '" + to_string(role) + "' role to geometry id " +
        to_string(geometry_id) +
        " for the first time; it already has the role assigned");
  } else if (!has_role && assign == RoleAssign::kReplace) {
    throw std::logic_error(
        "Trying to replace the properties on geometry id " +
        to_string(geometry_id) + " for the '" + to_string(role) +
        "' role; it has not had the role initially assigned");
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
void GeometryState<T>::AddToProximityEngineUnchecked(
    const InternalGeometry& geometry) {
  const GeometryId geometry_id = geometry.id();
  if (geometry.is_deformable()) {
    DRAKE_DEMAND(geometry.reference_mesh() != nullptr);
    const VolumeMesh<double>& reference_mesh = *geometry.reference_mesh();
    std::vector<int> surface_vertices;
    std::vector<int> surface_tri_to_volume_tet;
    TriangleSurfaceMesh<double> surface_mesh =
        ConvertVolumeToSurfaceMeshWithBoundaryVertices(
            reference_mesh, &surface_vertices, &surface_tri_to_volume_tet);
    geometry_engine_->AddDeformableGeometry(
        reference_mesh, surface_mesh, surface_vertices,
        surface_tri_to_volume_tet, geometry_id);
    VertexSampler vertex_sampler(std::move(surface_vertices), reference_mesh);
    std::vector<DrivenTriangleMesh> driven_meshes;
    driven_meshes.emplace_back(vertex_sampler, surface_mesh);
    kinematics_data_.driven_mesh_data[Role::kProximity].SetMeshes(
        geometry_id, driven_meshes);
  } else if (geometry.is_dynamic()) {
    // Pass the geometry to the engine.
    const RigidTransformd& X_WG =
        convert_to_double(kinematics_data_.X_WGs.at(geometry_id));
    geometry_engine_->AddDynamicGeometry(geometry.shape(), X_WG, geometry_id,
                                         *geometry.proximity_properties());
  } else {
    geometry_engine_->AddAnchoredGeometry(geometry.shape(), geometry.X_FG(),
                                          geometry_id,
                                          *geometry.proximity_properties());
  }
  geometry_version_.modify_proximity();
}

template <typename T>
void GeometryState<T>::RemoveFromProximityEngineUnchecked(
    const InternalGeometry& geometry) {
  geometry_engine_->RemoveGeometry(geometry.id(), geometry.is_dynamic());
  geometry_version_.modify_proximity();
}

template <typename T>
bool GeometryState<T>::RemoveFromRendererUnchecked(
    const std::string& renderer_name, GeometryId id) {
  render::RenderEngine* engine = render_engines_[renderer_name].get_mutable();
  if (engine->has_geometry(id)) {
    // The engine has reported the belief that it has geometry `id`. Therefore,
    // removal should report true.
    DRAKE_DEMAND(engine->RemoveGeometry(id) == true);
    geometry_version_.modify_perception();
    return true;
  }
  return false;
}

template <typename T>
bool GeometryState<T>::AddToCompatibleRenderersUnchecked(
    const internal::InternalGeometry& geometry) {
  bool added_to_renderer = false;
  auto accepting_renderers =
      geometry.perception_properties()->GetPropertyOrDefault(
          "renderer", "accepting", set<string>{});
  std::vector<render::RenderEngine*> candidate_renderers;
  for (auto& [name, engine] : render_engines_) {
    // If no "accepting_renderer" has been specified, every renderer will be
    // given the chance to register the geometry.
    if (accepting_renderers.empty() || accepting_renderers.contains(name)) {
      candidate_renderers.emplace_back(engine.get_mutable());
    }
  }
  if (candidate_renderers.empty()) return false;
  if (geometry.is_deformable()) {
    added_to_renderer = AddDeformableToCompatibleRenderersUnchecked(
        geometry, &candidate_renderers);
  } else {
    added_to_renderer =
        AddRigidToCompatibleRenderersUnchecked(geometry, &candidate_renderers);
  }
  if (added_to_renderer) {
    // Increment version number only if some renderer picks up the role
    // assignment.
    geometry_version_.modify_perception();
  }
  return added_to_renderer;
}

template <typename T>
bool GeometryState<T>::AddRigidToCompatibleRenderersUnchecked(
    const internal::InternalGeometry& geometry,
    std::vector<render::RenderEngine*>* candidate_renderers) {
  const PerceptionProperties& properties = *geometry.perception_properties();

  const RigidTransformd& X_WG =
      convert_to_double(kinematics_data_.X_WGs.at(geometry.id()));

  bool added_to_renderer{false};
  for (auto& engine : *candidate_renderers) {
    added_to_renderer =
        engine->RegisterVisual(geometry.id(), geometry.shape(), properties,
                               X_WG, geometry.is_dynamic()) ||
        added_to_renderer;
  }
  return added_to_renderer;
}

template <typename T>
bool GeometryState<T>::AddDeformableToCompatibleRenderersUnchecked(
    const internal::InternalGeometry& geometry,
    std::vector<render::RenderEngine*>* candidate_renderers) {
  const GeometryId id = geometry.id();
  const PerceptionProperties& properties = *geometry.perception_properties();
  bool added_to_renderer{false};
  for (auto& engine : *candidate_renderers) {
    added_to_renderer =
        engine->RegisterDeformableVisual(
            id, deformable_render_meshes_.at(Role::kPerception).at(id),
            properties) ||
        added_to_renderer;
  }
  return added_to_renderer;
}

template <typename T>
void GeometryState<T>::RegisterDrivenMesh(GeometryId geometry_id, Role role) {
  InternalGeometry& geometry = geometries_[geometry_id];
  DRAKE_DEMAND(geometry.is_deformable());
  DRAKE_DEMAND(role != Role::kUnassigned);
  DRAKE_DEMAND(geometry.has_role(role));
  const GeometryProperties& properties = *geometry.properties(role);

  const VolumeMesh<double>* control_mesh_ptr = geometry.reference_mesh();
  DRAKE_DEMAND(control_mesh_ptr != nullptr);
  const VolumeMesh<double>& control_mesh = *control_mesh_ptr;

  std::vector<RenderMesh> render_meshes;
  std::vector<DrivenTriangleMesh> driven_meshes;

  if (role == Role::kPerception) {
    // TODO(xuchenhan-tri): consider allowing embedded mesh for illustration
    // similar to the driven perception mesh.
    const std::filesystem::path render_meshes_file =
        properties.GetPropertyOrDefault("deformable", "embedded_mesh",
                                        string{});
    if (!render_meshes_file.empty()) {
      render_meshes =
          internal::LoadRenderMeshesFromObj(render_meshes_file, properties, {});
      for (const internal::RenderMesh& render_mesh : render_meshes) {
        driven_meshes.emplace_back(MakeTriangleSurfaceMesh(render_mesh),
                                   control_mesh);
      }
      kinematics_data_.driven_mesh_data[Role::kPerception].SetMeshes(
          geometry_id, std::move(driven_meshes));
      deformable_render_meshes_[Role::kPerception][geometry_id] =
          std::move(render_meshes);
      return;
    }
  }

  // Simply go with the surface mesh of the control mesh.
  driven_meshes.emplace_back(internal::MakeDrivenSurfaceMesh(control_mesh));
  kinematics_data_.driven_mesh_data[role].SetMeshes(geometry_id, driven_meshes);
  render_meshes.emplace_back(MakeRenderMeshFromTriangleSurfaceMesh(
      driven_meshes.back().triangle_surface_mesh(), properties));
  deformable_render_meshes_[role][geometry_id] = std::move(render_meshes);
}

template <typename T>
void GeometryState<T>::RemoveFromAllRenderersUnchecked(GeometryId id) {
  for (auto& name_engine_pair : render_engines_) {
    const std::string& engine_name = name_engine_pair.first;
    RemoveFromRendererUnchecked(engine_name, id);
  }
}

template <typename T>
bool GeometryState<T>::RemoveProximityRole(GeometryId geometry_id) {
  internal::InternalGeometry* geometry = GetMutableGeometry(geometry_id);
  DRAKE_DEMAND(geometry != nullptr);

  // Geometry is not registered with the proximity engine.
  if (!geometry->has_proximity_role()) return false;

  // Geometry *is* registered; do the work to remove it.
  RemoveFromProximityEngineUnchecked(*geometry);
  geometry->RemoveProximityRole();

  // TODO(SeanCurtis-TRI): This doesn't remove the geometry from collision
  // filters; it should.
  return true;
}

template <typename T>
bool GeometryState<T>::RemoveIllustrationRole(GeometryId geometry_id) {
  internal::InternalGeometry* geometry = GetMutableGeometry(geometry_id);
  DRAKE_DEMAND(geometry != nullptr);

  // Geometry has no illustration role.
  if (!geometry->has_illustration_role()) return false;

  geometry->RemoveIllustrationRole();
  geometry_version_.modify_illustration();
  return true;
}

template <typename T>
bool GeometryState<T>::RemovePerceptionRole(GeometryId geometry_id) {
  internal::InternalGeometry* geometry = GetMutableGeometry(geometry_id);
  DRAKE_DEMAND(geometry != nullptr);

  // Geometry has no perception role.
  if (!geometry->has_perception_role()) return false;

  // Geometry has a perception role; do the work to remove it from whichever
  // render engines it happens to be present in and also remove its driven
  // perception meshes.
  RemoveFromAllRenderersUnchecked(geometry_id);
  if (IsDeformableGeometry(geometry_id)) {
    kinematics_data_.driven_mesh_data[Role::kPerception].Remove(geometry_id);
    deformable_render_meshes_[Role::kPerception].erase(geometry_id);
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
    const FrameIdSet& set = GetValueOrThrow(source_id, source_frame_id_map_);
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
    return *iter->second.get();
  }

  throw std::logic_error(
      fmt::format("No renderer exists with name: '{}'", renderer_name));
}

template <typename T>
RigidTransformd GeometryState<T>::CalcCameraWorldPose(
    const render::RenderCameraCore& core, FrameId parent_frame,
    const RigidTransformd& X_PC) const {
  return GetDoubleWorldPose(parent_frame) * X_PC *
         core.sensor_pose_in_camera_body();
}

template <typename T>
RigidTransformd GeometryState<T>::GetDoubleWorldPose(FrameId frame_id) const {
  if (frame_id == InternalFrame::world_frame_id()) {
    return RigidTransformd::Identity();
  }
  const internal::InternalFrame& frame = GetValueOrThrow(frame_id, frames_);
  return internal::convert_to_double(kinematics_data_.X_WFs[frame.index()]);
}

// Explicitly instantiate all variations of the scalar-converting constructor.
using symbolic::Expression;
template GeometryState<double>::GeometryState(const GeometryState<AutoDiffXd>&);
template GeometryState<double>::GeometryState(const GeometryState<Expression>&);
template GeometryState<AutoDiffXd>::GeometryState(const GeometryState<double>&);
template GeometryState<AutoDiffXd>::GeometryState(
    const GeometryState<Expression>&);
template GeometryState<Expression>::GeometryState(const GeometryState<double>&);
template GeometryState<Expression>::GeometryState(
    const GeometryState<AutoDiffXd>&);

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::geometry::GeometryState);
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (&drake::geometry::internal::DrivenMeshData::
         template SetControlMeshPositions<T>));
