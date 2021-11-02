#include "drake/geometry/meshcat_visualizer.h"

#include <memory>
#include <string>
#include <utility>

#include <fmt/format.h>

#include "drake/geometry/utilities.h"

namespace drake {
namespace geometry {

template <typename T>
MeshcatVisualizer<T>::MeshcatVisualizer(std::shared_ptr<Meshcat> meshcat,
                                        MeshcatVisualizerParams params)
    : systems::LeafSystem<T>(systems::SystemTypeTag<MeshcatVisualizer>{}),
      meshcat_(std::move(meshcat)),
      params_(std::move(params)),
      animation_(
          std::make_unique<MeshcatAnimation>(1.0 / params_.publish_period)) {
  DRAKE_DEMAND(meshcat_ != nullptr);
  DRAKE_DEMAND(params_.publish_period >= 0.0);
  if (params_.role == Role::kUnassigned) {
    throw std::runtime_error(
        "MeshcatVisualizer cannot be used for geometries with the "
        "Role::kUnassigned value. Please choose kProximity, kPerception, or "
        "kIllustration");
  }

  this->DeclarePeriodicPublishEvent(params_.publish_period, 0.0,
                                    &MeshcatVisualizer<T>::UpdateMeshcat);
  this->DeclareForcedPublishEvent(&MeshcatVisualizer<T>::UpdateMeshcat);

  if (params_.delete_on_initialization_event) {
    this->DeclareInitializationPublishEvent(
        &MeshcatVisualizer<T>::OnInitialization);
  }

  query_object_input_port_ =
      this->DeclareAbstractInputPort("query_object", Value<QueryObject<T>>())
          .get_index();
}

template <typename T>
template <typename U>
MeshcatVisualizer<T>::MeshcatVisualizer(const MeshcatVisualizer<U>& other)
    : MeshcatVisualizer(other.meshcat_, other.params_) {}

template <typename T>
void MeshcatVisualizer<T>::Delete() const {
  meshcat_->Delete(params_.prefix);
  version_ = GeometryVersion();
}

template <typename T>
void MeshcatVisualizer<T>::PublishRecording() const {
  meshcat_->SetAnimation(*animation_);
}

template <typename T>
void MeshcatVisualizer<T>::DeleteRecording() {
  animation_ = std::make_unique<MeshcatAnimation>(1.0 / params_.publish_period);
}

template <typename T>
MeshcatVisualizer<T>& MeshcatVisualizer<T>::AddToBuilder(
    systems::DiagramBuilder<T>* builder, const SceneGraph<T>& scene_graph,
    std::shared_ptr<Meshcat> meshcat, MeshcatVisualizerParams params) {
  return AddToBuilder(builder, scene_graph.get_query_output_port(),
                      std::move(meshcat), std::move(params));
}

template <typename T>
MeshcatVisualizer<T>& MeshcatVisualizer<T>::AddToBuilder(
    systems::DiagramBuilder<T>* builder,
    const systems::OutputPort<T>& query_object_port,
    std::shared_ptr<Meshcat> meshcat, MeshcatVisualizerParams params) {
  auto& visualizer = *builder->template AddSystem<MeshcatVisualizer<T>>(
      std::move(meshcat), std::move(params));
  builder->Connect(query_object_port, visualizer.query_object_input_port());
  return visualizer;
}

template <typename T>
systems::EventStatus MeshcatVisualizer<T>::UpdateMeshcat(
    const systems::Context<T>& context) const {
  const auto& query_object =
      query_object_input_port().template Eval<QueryObject<T>>(context);
  const GeometryVersion& current_version =
      query_object.inspector().geometry_version();

  if (!version_.IsSameAs(current_version, params_.role)) {
    SetObjects(query_object.inspector());
    version_ = current_version;
  }
  SetTransforms(context, query_object);

  return systems::EventStatus::Succeeded();
}

template <typename T>
void MeshcatVisualizer<T>::SetObjects(
    const SceneGraphInspector<T>& inspector) const {
  // Frames registered previously that are not set again here should be deleted.
  std::map <FrameId, std::string> frames_to_delete{};
  dynamic_frames_.swap(frames_to_delete);

  // Geometries registered previously that are not set again here should be
  // deleted.
  std::map <GeometryId, std::string> geometries_to_delete{};
  geometries_.swap(geometries_to_delete);

  // TODO(SeanCurtis-TRI): Mimic the full tree structure in SceneGraph.
  // SceneGraph supports arbitrary hierarchies of frames just like Meshcat.
  // This code is arbitrarily flattening it because the current SceneGraph API
  // is insufficient to support walking the tree.
  for (FrameId frame_id : inspector.GetAllFrameIds()) {
    std::string frame_path =
        frame_id == inspector.world_frame_id()
            ? params_.prefix
            : fmt::format("{}/{}", params_.prefix, inspector.GetName(frame_id));
    // MultibodyPlant declares frames with SceneGraph using "::". We replace
    // those with `/` here to expose the full tree to Meshcat.
    size_t pos = 0;
    while ((pos = frame_path.find("::", pos)) != std::string::npos) {
      frame_path.replace(pos++, 2, "/");
    }
    if (frame_id != inspector.world_frame_id() &&
        inspector.NumGeometriesForFrameWithRole(frame_id, params_.role) > 0) {
      dynamic_frames_[frame_id] = frame_path;
      frames_to_delete.erase(frame_id);  // Don't delete this one.
    }

    for (GeometryId geom_id : inspector.GetGeometries(frame_id, params_.role)) {
      // Note: We use the frame_path/id instead of instance.GetName(geom_id),
      // which is a garbled mess of :: and _ and a memory address by default
      // when coming from MultibodyPlant.
      // TODO(russt): Use the geometry names if/when they are cleaned up.
      const std::string path =
          fmt::format("{}/{}", frame_path, geom_id.get_value());
      const Rgba rgba = inspector.GetProperties(geom_id, params_.role)
          ->GetPropertyOrDefault("phong", "diffuse", params_.default_color);

      meshcat_->SetObject(path, inspector.GetShape(geom_id), rgba);
      meshcat_->SetTransform(path, inspector.GetPoseInFrame(geom_id));
      geometries_[geom_id] = path;
      geometries_to_delete.erase(geom_id);  // Don't delete this one.
    }
  }

  for (const auto& [geom_id, path] : geometries_to_delete) {
    unused(geom_id);
    meshcat_->Delete(path);
  }
  for (const auto& [frame_id, path] : frames_to_delete) {
    unused(frame_id);
    meshcat_->Delete(path);
  }
}

template <typename T>
void MeshcatVisualizer<T>::SetTransforms(
    const systems::Context<T>& context,
    const QueryObject<T>& query_object) const {
  for (const auto& [frame_id, path] : dynamic_frames_) {
    const math::RigidTransformd X_WF =
        internal::convert_to_double(query_object.GetPoseInWorld(frame_id));
    meshcat_->SetTransform(path, X_WF);
    if (recording_) {
      animation_->SetTransform(
          animation_->frame(ExtractDoubleOrThrow(context.get_time())), path,
          X_WF);
    }
  }
}

template <typename T>
systems::EventStatus MeshcatVisualizer<T>::OnInitialization(
    const systems::Context<T>&) const {
  Delete();
  return systems::EventStatus::Succeeded();
}

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::MeshcatVisualizer)
