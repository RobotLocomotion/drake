#include "sim/common/camera_config_functions.h"

#include <string>

#include "drake/geometry/render/render_camera.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/scoped_names.h"
#include "drake/systems/sensors/rgbd_sensor.h"
#include "sim/common/sim_rgbd_sensor.h"

using drake::geometry::render::ColorRenderCamera;
using drake::geometry::render::DepthRenderCamera;
using drake::geometry::render::MakeRenderEngineGl;
using drake::geometry::render::RenderEngineGlParams;
using drake::geometry::SceneGraph;
using drake::lcm::DrakeLcmInterface;
using drake::math::RigidTransformd;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::multibody::parsing::GetScopedFrameByName;
using drake::systems::DiagramBuilder;
using drake::systems::sensors::CameraInfo;
using drake::systems::sensors::RgbdSensor;

namespace anzu {
namespace sim {

void DrakeApplyCameraConfig(const CameraConfig& camera_config,
                            MultibodyPlant<double>* sim_plant,
                            DiagramBuilder<double>* builder,
                            SceneGraph<double>* scene_graph,
                            DrakeLcmInterface* lcm) {
  if (!(camera_config.rgb || camera_config.depth)) {
    return;
  }

  camera_config.ValidateOrThrow();

  // Extract the camera extrinsics from the camera_config struct.
  const Frame<double>& base_frame =
      camera_config.X_PB.base_frame
          ? GetScopedFrameByName(*sim_plant, *camera_config.X_PB.base_frame)
          : sim_plant->world_frame();
  const RigidTransformd X_PB = camera_config.X_PB.GetDeterministicValue();

  // Confirm presence of RenderEngine for this camera, adding one as necessary.
  if (!scene_graph->HasRenderer(camera_config.renderer_name)) {
    // TODO(SeanCurtis-TRI): This cannot always be the gl renderer (mac). This
    //  either needs an availability test that falls back to vtk, or it needs
    //  to be specified somewhere and fail when unsupported.
    RenderEngineGlParams gl_params;
    gl_params.default_clear_color = camera_config.background;
    scene_graph->AddRenderer(camera_config.renderer_name,
                             MakeRenderEngineGl(gl_params));
  }

  // Extract camera intrinsics from the camera_config struct.
  auto [color_camera, depth_camera] = camera_config.MakeCameras();

  // Construct the sensor itself.
  const SimRgbdSensor sim_camera(camera_config.name, base_frame,
                                 camera_config.fps, X_PB, color_camera,
                                 depth_camera);
  const RgbdSensor* camera_sys =
      AddSimRgbdSensor(*scene_graph, *sim_plant, sim_camera, builder);

  // Connect the sensor the the lcm system.
  const auto* rgb_port =
      camera_config.rgb ? &camera_sys->color_image_output_port() : nullptr;
  const auto* depth_16u_port = camera_config.depth
                                   ? &camera_sys->depth_image_16U_output_port()
                                   : nullptr;
  DrakeAddSimRgbdSensorLcmPublisher(sim_camera, rgb_port, depth_16u_port,
                                    camera_config.do_compress, builder, lcm);
}

}  // namespace sim
}  // namespace anzu
