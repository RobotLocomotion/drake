#include "drake/geometry/meshcat_visualizer.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <fmt/format.h>

#include "drake/common/extract_double.h"
#include "drake/common/overloaded.h"
#include "drake/geometry/meshcat_graphviz.h"
#include "drake/geometry/meshcat_internal.h"
#include "drake/geometry/proximity/polygon_to_triangle_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/geometry/utilities.h"

namespace drake {
namespace geometry {

template <typename T>
MeshcatVisualizer<T>::MeshcatVisualizer(std::shared_ptr<Meshcat> meshcat,
                                        MeshcatVisualizerParams params)
    : systems::LeafSystem<T>(systems::SystemTypeTag<MeshcatVisualizer>{}),
      meshcat_(std::move(meshcat)),
      params_(std::move(params)),
      alpha_slider_name_(std::string(params_.prefix + " α")) {
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

  if (params_.enable_alpha_slider) {
    alpha_value_ = params_.initial_alpha_slider_value;
    meshcat_->AddSlider(alpha_slider_name_, 0.02, 1.0, 0.02, alpha_value_);
  }
}

template <typename T>
template <typename U>
MeshcatVisualizer<T>::MeshcatVisualizer(const MeshcatVisualizer<U>& other)
    : MeshcatVisualizer(other.meshcat_, other.params_) {}

template <typename T>
MeshcatVisualizer<T>::~MeshcatVisualizer() = default;

template <typename T>
void MeshcatVisualizer<T>::Delete() const {
  meshcat_->Delete(params_.prefix);
  version_ = std::nullopt;
}

template <typename T>
MeshcatAnimation* MeshcatVisualizer<T>::StartRecording(
    bool set_transforms_while_recording) {
  meshcat_->StartRecording(1.0 / params_.publish_period,
                           set_transforms_while_recording);
  return &meshcat_->get_mutable_recording();
}

template <typename T>
void MeshcatVisualizer<T>::StopRecording() {
  meshcat_->StopRecording();
}

template <typename T>
void MeshcatVisualizer<T>::PublishRecording() const {
  meshcat_->PublishRecording();
}

template <typename T>
void MeshcatVisualizer<T>::DeleteRecording() {
  meshcat_->DeleteRecording();
}

template <typename T>
MeshcatAnimation* MeshcatVisualizer<T>::get_mutable_recording() {
  return &meshcat_->get_mutable_recording();
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
  const std::string aspirational_name =
      fmt::format("meshcat_visualizer({})", params.prefix);
  auto& visualizer = *builder->template AddSystem<MeshcatVisualizer<T>>(
      std::move(meshcat), std::move(params));
  if (!builder->HasSubsystemNamed(aspirational_name)) {
    visualizer.set_name(aspirational_name);
  }
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
  if (!version_.has_value()) {
    // When our current version is null, that means we haven't added any
    // geometry to Meshcat yet, which means we also need to establish our
    // default visibility just prior to sending the geometry.
    meshcat_->SetProperty(params_.prefix, "visible",
                          params_.visible_by_default);
  }
  if (!version_.has_value() ||
      !version_->IsSameAs(current_version, params_.role)) {
    SetObjects(query_object);
    SetAlphas(/* initializing = */ true);
    version_ = current_version;
  }
  SetTransforms(context, query_object);
  BroadcastDeformables(query_object);
  if (params_.enable_alpha_slider) {
    double new_alpha_value = meshcat_->GetSliderValue(alpha_slider_name_);
    if (new_alpha_value != alpha_value_) {
      alpha_value_ = new_alpha_value;
      SetAlphas(/* initializing = */ false);
    }
  }
  meshcat_->SetSimulationTime(ExtractDoubleOrThrow(context.get_time()));

  return systems::EventStatus::Succeeded();
}

template <typename T>
void MeshcatVisualizer<T>::RegisterDeformable(
    const QueryObject<T>& query_object, GeometryId deformable_id) const {
  if constexpr (std::is_same_v<T, double>) {
    if (dynamic_deformable_geometries_.contains(deformable_id)) {
      throw std::logic_error(fmt::format(
          "Deformable geometry {} has already been registered. It cannot be "
          "registered again.",
          deformable_id));
    }

    const auto& inspector = query_object.inspector();
    const std::vector<internal::RenderMesh>& render_meshes =
        inspector.GetDrivenRenderMeshes(deformable_id, params_.role);
    const int num_render_meshes = ssize(render_meshes);
    dynamic_deformable_geometries_[deformable_id].reserve(num_render_meshes);

    for (int i = 0; i < num_render_meshes; ++i) {
      const Rgba& diffuse_color = render_meshes[i].material.has_value()
                                      ? render_meshes[i].material->diffuse
                                      : params_.default_color;

      dynamic_deformable_geometries_[deformable_id].push_back(
          {internal::MakeTriangleSurfaceMesh(render_meshes[i]), diffuse_color});
    }
  } else {
    throw std::runtime_error(
        "Only MeshcatVisualizer<double> supports deformable geometry.");
  }
}

template <typename T>
void MeshcatVisualizer<T>::BroadcastDeformables(
    const QueryObject<T>& query_object) const {
  if constexpr (std::is_same_v<T, double>) {
    for (auto& [deformable_id, colored_meshes] :
         dynamic_deformable_geometries_) {
      const int num_render_meshes = std::size(colored_meshes);
      const std::string& path = geometries_.at(deformable_id);

      // We already checked the T is double
      const std::vector<VectorX<double>> vertex_positions =
          query_object.GetDrivenMeshConfigurationsInWorld(deformable_id,
                                                          params_.role);
      DRAKE_DEMAND(std::ssize(vertex_positions) == num_render_meshes);

      for (int i = 0; i < num_render_meshes; ++i) {
        colored_meshes[i].mesh.SetAllPositions(vertex_positions[i]);

        const std::string final_sub_path =
            num_render_meshes == 1 ? path : fmt::format("{}/{}", path, i);
        meshcat_->SetObject(final_sub_path, colored_meshes[i].mesh,
                            colored_meshes[i].diffuse);
      }
      meshcat_->SetTransform(path, math::RigidTransformd::Identity());
    }
  }
  // Else do nothing -- we should've already thrown in RegisterDeformable()
  // for T != double.
}

template <typename T>
void MeshcatVisualizer<T>::SetObjects(
    const QueryObject<T>& query_object) const {
  const auto& inspector = query_object.inspector();
  // Frames registered previously that are not set again here should be deleted.
  std::map<FrameId, std::string> frames_to_delete{};
  dynamic_frames_.swap(frames_to_delete);
  dynamic_deformable_geometries_.clear();  // no need to swap

  // Geometries registered previously that are not set again here should be
  // deleted.
  std::map<GeometryId, std::string> geometries_to_delete{};
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

    bool frame_has_any_geometry = false;
    for (GeometryId geom_id : inspector.GetGeometries(frame_id, params_.role)) {
      const GeometryProperties& properties =
          *inspector.GetProperties(geom_id, params_.role);
      if (properties.HasProperty("meshcat", "accepting")) {
        if (properties.GetProperty<std::string>("meshcat", "accepting") !=
            params_.prefix) {
          continue;
        }
      } else if (!params_.include_unspecified_accepting) {
        continue;
      }

      // We'll turn scoped names into meshcat paths.
      const std::string geometry_name =
          internal::TransformGeometryName(geom_id, inspector);
      std::string path = fmt::format("{}/{}", frame_path, geometry_name);
      const Rgba rgba = properties.GetPropertyOrDefault("phong", "diffuse",
                                                        params_.default_color);
      const std::string diffuse_map = properties.GetPropertyOrDefault(
          "phong", "diffuse_map", std::string{});

      // The "object" will typically be the geometry's shape. But, for the
      // proximity role, we prefer, first, the hydroelastic surface if
      // available, or, second, the convex hull. Record if we've used one of
      // those proxies.
      bool geometry_already_set = false;
      bool is_a_deformable_geometry = inspector.IsDeformableGeometry(geom_id);

      if constexpr (std::is_same_v<T, double>) {
        if (params_.show_hydroelastic) {
          auto maybe_mesh = inspector.maybe_get_hydroelastic_mesh(geom_id);
          std::visit<void>(
              overloaded{[](std::monostate) {},
                         [&](const TriangleSurfaceMesh<double>* mesh) {
                           DRAKE_DEMAND(mesh != nullptr);
                           meshcat_->SetObject(path, *mesh, rgba);
                           geometry_already_set = true;
                         },
                         [&](const VolumeMesh<double>* mesh) {
                           DRAKE_DEMAND(!is_a_deformable_geometry);
                           DRAKE_DEMAND(mesh != nullptr);
                           meshcat_->SetObject(
                               path, ConvertVolumeToSurfaceMesh(*mesh), rgba);
                           geometry_already_set = true;
                         }},
              maybe_mesh);
        }
      }

      // For non-deformables, proximity role favors convex hulls if available.
      if (const PolygonSurfaceMesh<double>* hull = nullptr;
          (!geometry_already_set) && (params_.role == Role::kProximity) &&
          (!is_a_deformable_geometry) &&  // Use convex hull for non-deformables
          (hull = inspector.GetConvexHull(geom_id))) {
        // Convert polygonal surface mesh to triangle surface mesh.
        const TriangleSurfaceMesh<double> tri_hull =
            internal::MakeTriangleFromPolygonMesh(*hull);
        meshcat_->SetObject(path, tri_hull, rgba);
        geometry_already_set = true;
      }

      if (!geometry_already_set) {
        // Test if this geometry is a deformable geometry that we want to
        // visualize. For now, we only accept deformable geometry imported from
        // a .vtk extension.
        if (is_a_deformable_geometry) {
          path = fmt::format("{}/{}/{}", frame_path, "deformable_geometries",
                             geometry_name);

          RegisterDeformable(query_object, geom_id);
        } else {
          // Fall back to the geometry's shape.
          meshcat_->SetObject(path, inspector.GetShape(geom_id), rgba,
                              diffuse_map);
        }
      }

      if (!is_a_deformable_geometry) {
        meshcat_->SetTransform(path, inspector.GetPoseInFrame(geom_id));
      }

      geometries_[geom_id] = path;
      geometries_to_delete.erase(geom_id);  // Don't delete this one.
      frame_has_any_geometry = true;
    }

    if (frame_has_any_geometry && (frame_id != inspector.world_frame_id())) {
      dynamic_frames_[frame_id] = frame_path;
      frames_to_delete.erase(frame_id);  // Don't delete this one.
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
    meshcat_->SetTransform(path, X_WF,
                           ExtractDoubleOrThrow(context.get_time()));
  }
}

template <typename T>
void MeshcatVisualizer<T>::SetAlphas(bool initializing) const {
  if (initializing) {
    for (const auto& [_, geo_path] : geometries_) {
      meshcat_->SetProperty(geo_path, "modulated_opacity", alpha_value_);
    }
  } else {
    // The geometries visualized by this visualizer (stored in geometries_) all
    // have a common prefix and for a well-configured visualizer, it is a
    // *unique* prefix. So, we can rely on meshcat's behavior to update all
    // materials in a tree with a single invocation on the root path. This
    // requires that all object instantiations are complete in the visualizer
    // instance.
    meshcat_->SetProperty(params_.prefix, "modulated_opacity", alpha_value_);
  }
}

template <typename T>
systems::EventStatus MeshcatVisualizer<T>::OnInitialization(
    const systems::Context<T>&) const {
  Delete();
  return systems::EventStatus::Succeeded();
}

template <typename T>
typename systems::LeafSystem<T>::GraphvizFragment
MeshcatVisualizer<T>::DoGetGraphvizFragment(
    const typename systems::LeafSystem<T>::GraphvizFragmentParams& params)
    const {
  internal::MeshcatGraphviz meshcat_graphviz(params_.prefix,
                                             /* subscribe = */ false);
  return meshcat_graphviz.DecorateResult(
      systems::LeafSystem<T>::DoGetGraphvizFragment(
          meshcat_graphviz.DecorateParams(params)));
}

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::MeshcatVisualizer);
