#include "drake/systems/sensors/camera_config_functions.h"

#include <initializer_list>
#include <map>
#include <memory>
#include <string>

#include <fmt/format.h>

#include "drake/common/eigen_types.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/nice_type_name.h"
#include "drake/geometry/render/render_camera.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/geometry/render_vtk/factory.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/scoped_names.h"
#include "drake/systems/lcm/lcm_config_functions.h"
#include "drake/systems/sensors/rgbd_sensor.h"
#include "drake/systems/sensors/sim_rgbd_sensor.h"

namespace drake {
namespace systems {
namespace sensors {

using drake::lcm::DrakeLcmInterface;
using drake::systems::lcm::LcmBuses;
using Eigen::Vector3d;
using geometry::MakeRenderEngineGl;
using geometry::MakeRenderEngineVtk;
using geometry::RenderEngineGlParams;
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

// Validates the render engine specification in `config`. If the specification
// is valid, upon return, the specified RenderEngine is defined in `scene_graph`
// (this may require creating the RenderEngine instance). Throws if the
// specification is invalid.
// @pre we already know that config.renderer_class has a "supported" value.
void ValidateEngineAndMaybeAdd(const CameraConfig& config,
                               SceneGraph<double>* scene_graph) {
  using Dict = std::map<std::string, std::string>;
  static const never_destroyed<Dict> type_lookup(
      std::initializer_list<Dict::value_type>{
          {"RenderEngineVtk",
           "drake::geometry::render_vtk::internal::RenderEngineVtk"},
          {"RenderEngineGl",
           "drake::geometry::render_gl::internal::RenderEngineGl"}});

  DRAKE_DEMAND(scene_graph != nullptr);

  // Querying for the type_name of the named renderer will simultaneously tell
  // if a render engine already exists (non-empty value) *and* give us a string
  // to match against config.renderer_class.
  const std::string type_name =
      scene_graph->GetRendererTypeName(config.renderer_name);

  // Non-empty type name says that it already exists.
  bool already_exists = !type_name.empty();
  if (already_exists && !config.renderer_class.empty()) {
    if (type_lookup.access().at(config.renderer_class) != type_name) {
      throw std::logic_error(
          fmt::format("Invalid camera configuration; requested "
                      "renderer_name = '{}' and renderer_class = '{}'. The "
                      "name is already used with a different type: {}.",
                      config.renderer_name, config.renderer_class, type_name));
    }
  }

  if (already_exists) return;

  // Now we know we need to add one. Confirm we can add the specified class.
  if (config.renderer_class == "RenderEngineGl") {
    if (!geometry::kHasRenderEngineGl) {
      throw std::logic_error(
          "Invalid camera configuration; renderer_class = 'RenderEngineGl' "
          "is not supported in current build.");
    }
    RenderEngineGlParams params{.default_clear_color = config.background};
    scene_graph->AddRenderer(config.renderer_name,
                              MakeRenderEngineGl(params));
    return;
  }
  // Note: if we add *other* supported render engine implementations, add the
  // logic for detecting and instantiating those types here.

  // Fall through to the default render engine type (name is either empty or
  // the only remaining possible value: "RenderEngineVtk").
  DRAKE_DEMAND(config.renderer_class.empty() ||
               config.renderer_class == "RenderEngineVtk");
  RenderEngineVtkParams params;
  const geometry::Rgba& rgba = config.background;
  params.default_clear_color = Vector3d{rgba.r(), rgba.g(), rgba.b()};
  scene_graph->AddRenderer(config.renderer_name, MakeRenderEngineVtk(params));
}

}  // namespace

void ApplyCameraConfig(const CameraConfig& config,
                       DiagramBuilder<double>* builder,
                       const LcmBuses* lcm_buses,
                       const MultibodyPlant<double>* plant,
                       SceneGraph<double>* scene_graph,
                       DrakeLcmInterface* lcm) {
  if (!(config.rgb || config.depth)) {
    return;
  }

  // Find the plant and scene graph.
  if (plant == nullptr) {
    plant = &builder->GetDowncastSubsystemByName<MultibodyPlant>("plant");
  }
  if (scene_graph == nullptr) {
    scene_graph = &builder->GetMutableDowncastSubsystemByName<SceneGraph>(
        "scene_graph");
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

  // Construct the sensor itself.
  const SimRgbdSensor sim_camera(config.name, base_frame, config.fps, X_PB,
                                 color_camera, depth_camera);
  const RgbdSensor* camera_sys =
      AddSimRgbdSensor(*scene_graph, *plant, sim_camera, builder);

  // Find the LCM bus.
  lcm = FindOrCreateLcmBus(
      lcm, lcm_buses, builder, "ApplyCameraConfig", config.lcm_bus);
  DRAKE_DEMAND(lcm != nullptr);

  // Connect the sensor the the lcm system.
  const auto* rgb_port =
      config.rgb ? &camera_sys->color_image_output_port() : nullptr;
  const auto* depth_16u_port =
      config.depth ? &camera_sys->depth_image_16U_output_port() : nullptr;
  AddSimRgbdSensorLcmPublisher(sim_camera, rgb_port, depth_16u_port,
                               config.do_compress, builder, lcm);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
