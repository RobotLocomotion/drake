#include "drake/geometry/scene_graph_inspector.h"

#include <memory>

#include "drake/geometry/geometry_state.h"

namespace drake {
namespace geometry {

template <typename T>
int SceneGraphInspector<T>::num_sources() const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->get_num_sources();
}

template <typename T>
int SceneGraphInspector<T>::num_frames() const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->get_num_frames();
}

template <typename T>
std::vector<FrameId> SceneGraphInspector<T>::GetAllFrameIds() const {
  DRAKE_DEMAND(state_ != nullptr);
  typename GeometryState<T>::FrameIdRange range = state_->get_frame_ids();
  return std::vector<FrameId>(range.begin(), range.end());
}

template <typename T>
int SceneGraphInspector<T>::num_geometries() const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->get_num_geometries();
}

template <typename T>
const std::vector<GeometryId> SceneGraphInspector<T>::GetAllGeometryIds()
    const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->GetAllGeometryIds();
}

template <typename T>
std::unordered_set<GeometryId> SceneGraphInspector<T>::GetGeometryIds(
    const GeometrySet& geometry_set, const std::optional<Role>& role) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->GetGeometryIds(geometry_set, role);
}

template <typename T>
int SceneGraphInspector<T>::NumGeometriesWithRole(Role role) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->NumGeometriesWithRole(role);
}

template <typename T>
int SceneGraphInspector<T>::NumDynamicGeometries() const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->NumDynamicGeometries();
}

template <typename T>
int SceneGraphInspector<T>::NumAnchoredGeometries() const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->NumAnchoredGeometries();
}

template <typename T>
std::set<std::pair<GeometryId, GeometryId>>
SceneGraphInspector<T>::GetCollisionCandidates() const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->GetCollisionCandidates();
}

template <typename T>
const GeometryVersion& SceneGraphInspector<T>::geometry_version() const {
  return state_->geometry_version();
}

template <typename T>
bool SceneGraphInspector<T>::SourceIsRegistered(SourceId source_id) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->SourceIsRegistered(source_id);
}

template <typename T>
const std::string& SceneGraphInspector<T>::GetName(SourceId source_id) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->GetName(source_id);
}

template <typename T>
int SceneGraphInspector<T>::NumFramesForSource(SourceId source_id) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->NumFramesForSource(source_id);
}

template <typename T>
const std::unordered_set<FrameId>& SceneGraphInspector<T>::FramesForSource(
    SourceId source_id) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->FramesForSource(source_id);
}

template <typename T>
bool SceneGraphInspector<T>::BelongsToSource(FrameId frame_id,
                                             SourceId source_id) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->BelongsToSource(frame_id, source_id);
}

template <typename T>
const std::string& SceneGraphInspector<T>::GetOwningSourceName(
    FrameId frame_id) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->GetOwningSourceName(frame_id);
}

template <typename T>
const std::string& SceneGraphInspector<T>::GetName(FrameId frame_id) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->GetName(frame_id);
}

template <typename T>
FrameId SceneGraphInspector<T>::GetParentFrame(FrameId frame_id) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->GetParentFrame(frame_id);
}

template <typename T>
int SceneGraphInspector<T>::GetFrameGroup(FrameId frame_id) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->GetFrameGroup(frame_id);
}

template <typename T>
int SceneGraphInspector<T>::NumGeometriesForFrame(FrameId frame_id) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->NumGeometriesForFrame(frame_id);
}

template <typename T>
int SceneGraphInspector<T>::NumGeometriesForFrameWithRole(FrameId frame_id,
                                                          Role role) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->NumGeometriesForFrameWithRole(frame_id, role);
}

template <typename T>
std::vector<GeometryId> SceneGraphInspector<T>::GetGeometries(
    FrameId frame_id, const std::optional<Role>& role) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->GetGeometries(frame_id, role);
}

template <typename T>
GeometryId SceneGraphInspector<T>::GetGeometryIdByName(
    FrameId frame_id, Role role, const std::string& name) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->GetGeometryIdByName(frame_id, role, name);
}

template <typename T>
bool SceneGraphInspector<T>::BelongsToSource(GeometryId geometry_id,
                                             SourceId source_id) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->BelongsToSource(geometry_id, source_id);
}

template <typename T>
const std::string& SceneGraphInspector<T>::GetOwningSourceName(
    GeometryId geometry_id) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->GetOwningSourceName(geometry_id);
}

template <typename T>
FrameId SceneGraphInspector<T>::GetFrameId(GeometryId geometry_id) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->GetFrameId(geometry_id);
}

template <typename T>
const std::string& SceneGraphInspector<T>::GetName(
    GeometryId geometry_id) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->GetName(geometry_id);
}

template <typename T>
const Shape& SceneGraphInspector<T>::GetShape(GeometryId geometry_id) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->GetShape(geometry_id);
}

template <typename T>
const math::RigidTransform<double>& SceneGraphInspector<T>::GetPoseInParent(
    GeometryId geometry_id) const {
  DRAKE_DEMAND(state_ != nullptr);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  return state_->GetPoseInParent(geometry_id);
#pragma GCC diagnostic pop
}

template <typename T>
const math::RigidTransform<double>& SceneGraphInspector<T>::GetPoseInFrame(
    GeometryId geometry_id) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->GetPoseInFrame(geometry_id);
}

template <typename T>
std::variant<std::monostate, const TriangleSurfaceMesh<double>*,
             const VolumeMesh<double>*>
SceneGraphInspector<T>::maybe_get_hydroelastic_mesh(
    GeometryId geometry_id) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->maybe_get_hydroelastic_mesh(geometry_id);
}

template <typename T>
const GeometryProperties* SceneGraphInspector<T>::GetProperties(
    GeometryId geometry_id, Role role) const {
  DRAKE_DEMAND(state_ != nullptr);
  switch (role) {
    case Role::kProximity:
      return state_->GetProximityProperties(geometry_id);
    case Role::kIllustration:
      return state_->GetIllustrationProperties(geometry_id);
    case Role::kPerception:
      return state_->GetPerceptionProperties(geometry_id);
    case Role::kUnassigned:
      return nullptr;
  }
  return nullptr;
}

template <typename T>
const ProximityProperties* SceneGraphInspector<T>::GetProximityProperties(
    GeometryId geometry_id) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->GetProximityProperties(geometry_id);
}

template <typename T>
const IllustrationProperties* SceneGraphInspector<T>::GetIllustrationProperties(
    GeometryId geometry_id) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->GetIllustrationProperties(geometry_id);
}

template <typename T>
const PerceptionProperties* SceneGraphInspector<T>::GetPerceptionProperties(
    GeometryId geometry_id) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->GetPerceptionProperties(geometry_id);
}

template <typename T>
const VolumeMesh<double>* SceneGraphInspector<T>::GetReferenceMesh(
    GeometryId geometry_id) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->GetReferenceMesh(geometry_id);
}

template <typename T>
bool SceneGraphInspector<T>::IsDeformableGeometry(GeometryId id) const {
  return state_->IsDeformableGeometry(id);
}

template <typename T>
std::vector<GeometryId> SceneGraphInspector<T>::GetAllDeformableGeometryIds()
    const {
  return state_->GetAllDeformableGeometryIds();
}

template <typename T>
bool SceneGraphInspector<T>::CollisionFiltered(GeometryId geometry_id1,
                                               GeometryId geometry_id2) const {
  DRAKE_DEMAND(state_ != nullptr);
  return state_->CollisionFiltered(geometry_id1, geometry_id2);
}

template <typename T>
void SceneGraphInspector<T>::Reify(GeometryId geometry_id,
                                   ShapeReifier* reifier) const {
  DRAKE_DEMAND(state_ != nullptr);
  state_->GetShape(geometry_id).Reify(reifier);
}

template <typename T>
std::unique_ptr<GeometryInstance> SceneGraphInspector<T>::CloneGeometryInstance(
    GeometryId id) const {
  const std::string name = GetName(id);
  const math::RigidTransformd X_PG = GetPoseInFrame(id);
  std::unique_ptr<Shape> shape = GetShape(id).Clone();
  auto geometry_instance =
      std::make_unique<GeometryInstance>(X_PG, std::move(shape), name);
  if (const auto* props = GetProximityProperties(id)) {
    geometry_instance->set_proximity_properties(*props);
  }
  if (const auto* props = GetIllustrationProperties(id)) {
    geometry_instance->set_illustration_properties(*props);
  }
  if (const auto* props = GetPerceptionProperties(id)) {
    geometry_instance->set_perception_properties(*props);
  }
  return geometry_instance;
}

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::geometry::SceneGraphInspector)
