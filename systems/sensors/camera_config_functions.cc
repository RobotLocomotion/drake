#include "drake/systems/sensors/camera_config_functions.h"

#include <string>

#include "drake/common/eigen_types.h"
#include "drake/geometry/render/render_camera.h"
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

void ApplyCameraConfig(const CameraConfig& config,
                       MultibodyPlant<double>* plant,
                       DiagramBuilder<double>* builder,
                       SceneGraph<double>* scene_graph,
                       DrakeLcmInterface* lcm) {
  if (!(config.rgb || config.depth)) {
    return;
  }

  config.ValidateOrThrow();

  // Extract the camera extrinsics from the config struct.
  const Frame<double>& base_frame =
      config.X_PB.base_frame
          ? GetScopedFrameByName(*plant, *config.X_PB.base_frame)
          : plant->world_frame();
  const RigidTransformd X_PB = config.X_PB.GetDeterministicValue();

  // Confirm presence of RenderEngine for this camera, adding one as necessary.
  if (!scene_graph->HasRenderer(config.renderer_name)) {
    // TODO(SeanCurtis-TRI): Vtk is the always supportable render engine
    // implementation. Provide a mechanism to allow for other render engine
    // types to be instantiated. This might be a bit tricky if multiple cameras
    // declare a renderer of the same *name* but different types. This will
    // have to be reconciled.
    RenderEngineVtkParams vtk_params;
    const geometry::Rgba& rgba = config.background;
    vtk_params.default_clear_color = Vector3d{rgba.r(), rgba.g(), rgba.b()};
    scene_graph->AddRenderer(config.renderer_name,
                             MakeRenderEngineVtk(vtk_params));
  }

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
