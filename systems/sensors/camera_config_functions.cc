#include "drake/systems/sensors/camera_config_functions.h"

#include <initializer_list>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include <fmt/format.h>

#include "drake/common/eigen_types.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/overloaded.h"
#include "drake/geometry/render/render_camera.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/geometry/render_gltf_client/factory.h"
#include "drake/geometry/render_vtk/factory.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/scoped_names.h"
#include "drake/systems/lcm/lcm_config_functions.h"
#include "drake/systems/sensors/rgbd_sensor.h"
#include "drake/systems/sensors/rgbd_sensor_async.h"
#include "drake/systems/sensors/sim_rgbd_sensor.h"

namespace drake {
namespace systems {
namespace sensors {

using drake::lcm::DrakeLcmInterface;
using drake::systems::lcm::LcmBuses;
using Eigen::Vector3d;
using geometry::MakeRenderEngineGl;
using geometry::MakeRenderEngineGltfClient;
using geometry::MakeRenderEngineVtk;
using geometry::RenderEngineGlParams;
using geometry::RenderEngineGltfClientParams;
using geometry::RenderEngineVtkParams;
using geometry::SceneGraph;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRenderCamera;
using internal::AddSimRgbdSensor;
using internal::AddSimRgbdSensorLcmPublisher;
using internal::SimRgbdSensor;
using math::RigidTransformd;
using multibody::Frame;
using multibody::MultibodyPlant;
using multibody::parsing::GetScopedFrameByName;

namespace {

// Given a valid string containing a supported RenderEngine class name, returns
// the full name as would be returned by SceneGraph::GetRendererTypeName().
// @pre We already know that `class_name` is a "supported" value.
const std::string& LookupEngineType(const std::string& class_name) {
  using Dict = std::map<std::string, std::string>;
  static const never_destroyed<Dict> type_lookup(
      std::initializer_list<Dict::value_type>{
          {"RenderEngineVtk",
           "drake::geometry::render_vtk::internal::RenderEngineVtk"},
          {"RenderEngineGl",
           "drake::geometry::render_gl::internal::RenderEngineGl"},
          {"RenderEngineGltfClient",
           "drake::geometry::render_gltf_client::internal::"
           "RenderEngineGltfClient"}});
  return type_lookup.access().at(class_name);
}

template <typename ParamsType>
const char* GetEngineName(const ParamsType&) {
  if constexpr (std::is_same_v<ParamsType, RenderEngineVtkParams>) {
    return "RenderEngineVtk";
  } else if constexpr (std::is_same_v<ParamsType, RenderEngineGlParams>) {
    return "RenderEngineGl";
    // NOLINTNEXTLINE(readability/braces) The line break confuses the linter.
  } else if constexpr (std::is_same_v<ParamsType,
                                      RenderEngineGltfClientParams>) {
    return "RenderEngineGltfClient";
  }
}

void MakeEngineByClassName(const std::string& class_name,
                           const CameraConfig& config,
                           SceneGraph<double>* scene_graph) {
  if (class_name == "RenderEngineGl") {
    if (!geometry::kHasRenderEngineGl) {
      throw std::logic_error(
          "Invalid camera configuration; renderer_class = 'RenderEngineGl' "
          "is not supported in current build.");
    }
    RenderEngineGlParams params{.default_clear_color = config.background};
    scene_graph->AddRenderer(config.renderer_name, MakeRenderEngineGl(params));
    return;
  } else if (class_name == "RenderEngineGltfClient") {
    scene_graph->AddRenderer(config.renderer_name,
                             MakeRenderEngineGltfClient({}));
    return;
  }
  // Note: if we add *other* supported render engine implementations, add the
  // logic for detecting and instantiating those types here.

  // Fall through to the default render engine type (name is either empty or
  // the only remaining possible value: "RenderEngineVtk").
  DRAKE_DEMAND(class_name.empty() || class_name == "RenderEngineVtk");
  RenderEngineVtkParams params;
  const geometry::Rgba& rgba = config.background;
  params.default_clear_color = Vector3d{rgba.r(), rgba.g(), rgba.b()};
  scene_graph->AddRenderer(config.renderer_name, MakeRenderEngineVtk(params));
}

// Validates the render engine specification in `config`. If the specification
// is valid, upon return, the specified RenderEngine is defined in `scene_graph`
// (this may require creating the RenderEngine instance). Throws if the
// specification is invalid.
// @pre If config.renderer_class *names* a class, it is a "supported" value.
void ValidateEngineAndMaybeAdd(const CameraConfig& config,
                               SceneGraph<double>* scene_graph) {
  DRAKE_DEMAND(scene_graph != nullptr);

  // Querying for the type_name of the named renderer will simultaneously tell
  // if a render engine already exists (non-empty value) *and* give us a string
  // to match against config.renderer_class.
  const std::string type_name =
      scene_graph->GetRendererTypeName(config.renderer_name);

  // Non-empty type name says that it already exists.
  bool already_exists = !type_name.empty();

  if (already_exists) {
    visit_overloaded<void>(
        overloaded{
            [&type_name, &config](const std::string& class_name) {
              if (!class_name.empty() &&
                  LookupEngineType(class_name) != type_name) {
                throw std::logic_error(fmt::format(
                    "Invalid camera configuration; requested renderer_name "
                    "= '{}' and renderer_class = '{}'. The name is already "
                    "used with a different type: {}.",
                    config.renderer_name, class_name, type_name));
              }
            },
            [&config](auto&&) {
              throw std::logic_error(fmt::format(
                  "Invalid camera configuration; requested renderer_name "
                  " = '{}' with renderer parameters, but the named renderer "
                  "already exists. Only the first instance of the named "
                  "renderer can use parameters.",
                  config.renderer_name));
            }},
        config.renderer_class);
  }

  if (already_exists) return;

  visit_overloaded<void>(
      overloaded{
          [&config, scene_graph](const std::string& class_name) {
            MakeEngineByClassName(class_name, config, scene_graph);
          },
          [&config, scene_graph](const RenderEngineVtkParams& params) {
            scene_graph->AddRenderer(config.renderer_name,
                                     MakeRenderEngineVtk(params));
          },
          [&config, scene_graph](const RenderEngineGlParams& params) {
            if (!geometry::kHasRenderEngineGl) {
              throw std::logic_error(
                  "Invalid camera configuration; renderer_class = "
                  "'RenderEngineGl' is not supported in current build.");
            }
            scene_graph->AddRenderer(config.renderer_name,
                                     MakeRenderEngineGl(params));
          },
          [&config, scene_graph](const RenderEngineGltfClientParams& params) {
            scene_graph->AddRenderer(config.renderer_name,
                                     MakeRenderEngineGltfClient(params));
          }},
      config.renderer_class);
}

}  // namespace

void ApplyCameraConfig(const CameraConfig& config,
                       DiagramBuilder<double>* builder,
                       const LcmBuses* lcm_buses,
                       const MultibodyPlant<double>* plant,
                       SceneGraph<double>* scene_graph,
                       DrakeLcmInterface* lcm) {
  if (!(config.rgb || config.depth || config.label)) {
    return;
  }

  // Find the plant and scene graph.
  if (plant == nullptr) {
    plant = &builder->GetDowncastSubsystemByName<MultibodyPlant>("plant");
  }
  if (scene_graph == nullptr) {
    scene_graph =
        &builder->GetMutableDowncastSubsystemByName<SceneGraph>("scene_graph");
  }

  config.ValidateOrThrow();

  ValidateEngineAndMaybeAdd(config, scene_graph);

  // Extract the camera extrinsics from the config struct.
  const Frame<double>& base_frame =
      config.X_PB.base_frame
          ? GetScopedFrameByName(*plant, *config.X_PB.base_frame)
          : plant->world_frame();
  const RigidTransformd X_PB = config.X_PB.GetDeterministicValue();

  // Extract camera intrinsics from the config struct.
  const auto [color_camera, depth_camera] = config.MakeCameras();

  // Add the sensor system. When there is no output delay, we use an RgbdSensor;
  // when there is an output delay, we need to use an RgbdSensorAsync.
  System<double>* camera_sys{};
  if (config.output_delay == 0.0) {
    const SimRgbdSensor sim_camera(config.name, base_frame, config.fps, X_PB,
                                   color_camera, depth_camera);
    camera_sys = AddSimRgbdSensor(*scene_graph, *plant, sim_camera, builder);
  } else {
    const auto [frame_A, X_AB] = internal::GetGeometryFrame(base_frame, X_PB);
    camera_sys = builder->AddSystem<RgbdSensorAsync>(
        scene_graph, frame_A, X_AB, config.fps, config.capture_offset,
        config.output_delay,
        (config.rgb || config.label)
            ? std::optional<ColorRenderCamera>{color_camera}
            : std::nullopt,
        config.depth ? std::optional<DepthRenderCamera>{depth_camera}
                     : std::nullopt,
        config.label);
    camera_sys->set_name(fmt::format("rgbd_sensor_{}", config.name));
    builder->Connect(scene_graph->get_query_output_port(),
                     camera_sys->get_input_port());
  }

  // Find the LCM bus.
  lcm = FindOrCreateLcmBus(lcm, lcm_buses, builder, "ApplyCameraConfig",
                           config.lcm_bus);
  DRAKE_DEMAND(lcm != nullptr);
  if (lcm->get_lcm_url() == LcmBuses::kLcmUrlMemqNull) {
    // The user has opted-out of LCM.
    return;
  }

  // TODO(jwnimmer-tri) When the Simulator has concurrent update + publish
  // events, it effectively runs the publish event before the update event.
  // Therefore, we need to fudge the publish time here so that the published
  // image is correct. Once the framework allows us to schedule publish events
  // with the ordering we want, we should remove this 0.1% fudge factor.
  const double lcm_publisher_offset =
      config.capture_offset + (1.001 * config.output_delay);

  // Connect the sensor to the lcm system.
  AddSimRgbdSensorLcmPublisher(
      config.name, config.fps, lcm_publisher_offset,
      config.rgb ? &camera_sys->GetOutputPort("color_image") : nullptr,
      config.depth ? &camera_sys->GetOutputPort("depth_image_16u") : nullptr,
      config.label ? &camera_sys->GetOutputPort("label_image") : nullptr,
      config.do_compress, builder, lcm);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
