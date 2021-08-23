#include "drake/geometry/meshcat_visualizer.h"

#include <memory>
#include <string>
#include <utility>

#include <fmt/format.h>

#include "drake/geometry/utilities.h"

namespace drake {
namespace geometry {

template <typename T>
MeshcatVisualizer<T>::MeshcatVisualizer(
    Meshcat* meshcat, const std::shared_ptr<Meshcat>& owned_meshcat,
    bool scalar_conversion, MeshcatVisualizerParams params)
    : systems::LeafSystem<T>(systems::SystemTypeTag<MeshcatVisualizer>{}),
      meshcat_(owned_meshcat ? owned_meshcat.get() : meshcat),
      owned_meshcat_(owned_meshcat),
      params_(std::move(params)) {
  if (scalar_conversion && meshcat_ == nullptr) {
    throw std::runtime_error(
        "MeshcatVisualizer can only be scalar converted if it owns its "
        "Meshcat instance.  Construct MeshcatVisualizer using the shared_ptr "
        "constructor instead.");
  }
  DRAKE_DEMAND(meshcat_ != nullptr);
  DRAKE_DEMAND(params_.publish_period >= 0.0);
  if (params_.role == Role::kUnassigned) {
    throw std::runtime_error(
        "MeshcatVisualizer cannot be used for geometries with the "
        "Role::kUnassigned value. Please choose proximity, perception, or "
        "illustration");
  }

  std::cout << "period = " << params_.publish_period << std::endl;
  this->DeclarePeriodicPublishEvent(params_.publish_period, 0.0,
                                    &MeshcatVisualizer<T>::UpdateMeshcat);
  this->DeclareForcedPublishEvent(&MeshcatVisualizer<T>::UpdateMeshcat);

  if (params_.delete_prefix_on_initialization_event) {
    this->DeclareInitializationPublishEvent(
        &MeshcatVisualizer<T>::DeletePrefix);
  }

  query_object_input_port_ =
      this->DeclareAbstractInputPort("query_object", Value<QueryObject<T>>())
          .get_index();
}

template <typename T>
MeshcatVisualizer<T>::MeshcatVisualizer(Meshcat* meshcat,
                                        MeshcatVisualizerParams params)
    : MeshcatVisualizer(meshcat, nullptr, false, std::move(params)) {}

template <typename T>
MeshcatVisualizer<T>::MeshcatVisualizer(const std::shared_ptr<Meshcat>& meshcat,
                                        MeshcatVisualizerParams params)
    : MeshcatVisualizer(nullptr, meshcat, false, std::move(params)) {}

template <typename T>
template <typename U>
MeshcatVisualizer<T>::MeshcatVisualizer(const MeshcatVisualizer<U>& other)
    : MeshcatVisualizer(nullptr, other.owned_meshcat_, true, other.params_) {}

template <typename T>
void MeshcatVisualizer<T>::DeletePrefix() const {
  meshcat_->Delete(params_.prefix);
  version_ = GeometryVersion();
}

template <typename T>
const MeshcatVisualizer<T>& MeshcatVisualizer<T>::AddToBuilder(
    systems::DiagramBuilder<T>* builder, const SceneGraph<T>& scene_graph,
    Meshcat* meshcat, MeshcatVisualizerParams params) {
  return AddToBuilder(builder, scene_graph.get_query_output_port(), meshcat,
                      std::move(params));
}

template <typename T>
const MeshcatVisualizer<T>& MeshcatVisualizer<T>::AddToBuilder(
    systems::DiagramBuilder<T>* builder,
    const systems::OutputPort<T>& query_object_port, Meshcat* meshcat,
    MeshcatVisualizerParams params) {
  auto& visualizer = *builder->template AddSystem<MeshcatVisualizer<T>>(
      meshcat, std::move(params));
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

  bool set_objects = false;
  if (!version_.IsSameAs(current_version, params_.role)) {
    set_objects = true;
    version_ = current_version;
  }

  if (set_objects) {
    SetObjects(query_object.inspector());
  }
  SetTransforms(query_object);

  return systems::EventStatus::Succeeded();
}

template <typename T>
void MeshcatVisualizer<T>::SetObjects(
    const SceneGraphInspector<T>& inspector) const {
  // Recreate the dynamic frames from scratch in case existing frames have been
  // removed.
  dynamic_frames_.clear();

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
    }

    for (GeometryId geom_id : inspector.GetGeometries(frame_id, params_.role)) {
      // Note: We use the frame_path/id instead of instance.GetName(geom_id),
      // which is a garbled mess of :: and _ and a memory address by default
      // when coming from MultibodyPlant.
      const std::string path =
          fmt::format("{}/{}", frame_path, geom_id.get_value());
      Rgba rgba = params_.default_color;
      if (params_.role == Role::kIllustration) {
        const IllustrationProperties* props =
            inspector.GetIllustrationProperties(geom_id);
        if (props && props->HasProperty("phong", "diffuse")) {
          rgba = props->GetProperty<Rgba>("phong", "diffuse");
        }
      }

      meshcat_->SetObject(path, inspector.GetShape(geom_id), rgba);
      meshcat_->SetTransform(path, inspector.GetPoseInFrame(geom_id));
    }
  }
}

template <typename T>
void MeshcatVisualizer<T>::SetTransforms(
    const QueryObject<T>& query_object) const {
  for (const auto& [frame_id, path] : dynamic_frames_) {
    meshcat_->SetTransform(path, internal::convert_to_double(
                                     query_object.GetPoseInWorld(frame_id)));
  }
}

template <typename T>
systems::EventStatus MeshcatVisualizer<T>::DeletePrefix(
    const systems::Context<T>&) const {
  DeletePrefix();
  return systems::EventStatus::Succeeded();
}

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::MeshcatVisualizer)
