#include "drake/systems/sensors/camera_config_functions.h"

#include <string>

#include <fmt/format.h>

#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"
#include "drake/geometry/render/render_camera.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/geometry/render_vtk/factory.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/scoped_names.h"
#include "drake/systems/sensors/rgbd_sensor.h"
#include "drake/systems/sensors/sim_rgbd_sensor.h"

namespace drake {
namespace systems {
namespace sensors {

using drake::lcm::DrakeLcmInterface;
using Eigen::Vector3d;
using geometry::render::MakeRenderEngineGl;
using geometry::render::RenderEngineGlParams;
using geometry::MakeRenderEngineVtk;
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
  DRAKE_DEMAND(scene_graph != nullptr);

  // Querying for the type_name of the named renderer will simultaneously tell
  // if a render engine already exists (non-empty value) *and* give us a string
  // to match against config.renderer_class.
  const std::string type_name =
      scene_graph->GetRendererTypeName(config.renderer_name);

  // Non-empty type name says that it already exists.
  bool already_exists = !type_name.empty();
  if (already_exists && !config.renderer_class.empty()) {
    // We have to test a *declared* type against the previously existing type.
    // Note: This isn't bulletproof. If a user were to add a render engine
    // instance by hand called `foo::RenderEngineGl` and then declare
    // `RenderEngineGl` for the `renderer_class`, this would incorrectly
    // infer they are the same. However, this would constitute an aggressive
    // application of a foot gun on the user's part.
    if (config.renderer_class != NiceTypeName::RemoveNamespaces(type_name)) {
      throw std::logic_error(
          fmt::format("Invalid camera configuration; renderer_name = '{}' "
                      "already exists with a different type than the requested "
                      "renderer_class = '{}'.",
                      config.renderer_name, config.renderer_class));
    }
  }

  if (already_exists) return;

  // Now we know we need to add one. Confirm we can add the specified class.
  if (config.renderer_class == "RenderEngineGl") {
    if (geometry::render::kHasRenderEngineGl) {
      RenderEngineGlParams params{.default_clear_color = config.background};
      scene_graph->AddRenderer(config.renderer_name,
                               MakeRenderEngineGl(params));
      return;
    } else {
      throw std::logic_error(
          "Invalid camera configuration; renderer_class = 'RenderEngineGl' "
          "is not supported in current build.");
    }
  }
  // Note: if we add *other* supported render engine implementations, add the
  // logic for detecting and instantiating those types here.

  // Fall through to the default render engine type (name is either empty or
  // the only remaining possible value: "RenderEngineVtk").
  RenderEngineVtkParams params;
  const geometry::Rgba& rgba = config.background;
  params.default_clear_color = Vector3d{rgba.r(), rgba.g(), rgba.b()};
  scene_graph->AddRenderer(config.renderer_name, MakeRenderEngineVtk(params));
}

}  // namespace

void ApplyCameraConfig(const CameraConfig& config,
                       MultibodyPlant<double>* plant,
                       DiagramBuilder<double>* builder,
                       SceneGraph<double>* scene_graph,
                       DrakeLcmInterface* lcm) {
  DRAKE_DEMAND(plant != nullptr);
  DRAKE_DEMAND(builder != nullptr);
  DRAKE_DEMAND(scene_graph != nullptr);
  if (!(config.rgb || config.depth)) {
    return;
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
