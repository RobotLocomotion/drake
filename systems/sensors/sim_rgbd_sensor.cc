#include "drake/systems/sensors/sim_rgbd_sensor.h"

#include <memory>
#include <optional>
#include <regex>

#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"

namespace drake {
namespace systems {
namespace sensors {
namespace internal {

using geometry::SceneGraph;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRenderCamera;
using math::RigidTransformd;
using multibody::MultibodyPlant;

RgbdSensor* AddSimRgbdSensor(const SceneGraph<double>& scene_graph,
                             const MultibodyPlant<double>& plant,
                             const SimRgbdSensor& sim_rgbd_sensor,
                             DiagramBuilder<double>* builder) {
  DRAKE_DEMAND(builder != nullptr);
  // TODO(eric.cousineau): Simplify this if drake#10247 is resolved.

  // 'A' is the body for the parent sensor frame.
  // `P` is the parent of the sensor frame.
  // `B` is the sensor frame.

  const auto& frame_P = sim_rgbd_sensor.frame();
  const auto& body_A = frame_P.body();
  const std::optional<geometry::FrameId> body_A_id =
      plant.GetBodyFrameIdIfExists(body_A.index());
  DRAKE_THROW_UNLESS(body_A_id.has_value());

  // We must include the offset of the body to ensure that we posture the camera
  // appropriately.
  const RigidTransformd X_AP = frame_P.GetFixedPoseInBodyFrame();
  const RigidTransformd X_AB = X_AP * sim_rgbd_sensor.X_PB();

  auto* rgbd_sensor_sys = builder->AddSystem<RgbdSensor>(
      *body_A_id, X_AB, sim_rgbd_sensor.color_properties(),
      sim_rgbd_sensor.depth_properties());
  // `set_name` is only used for debugging.
  rgbd_sensor_sys->set_name("rgbd_sensor_" + sim_rgbd_sensor.serial());
  builder->Connect(scene_graph.get_query_output_port(),
                   rgbd_sensor_sys->query_object_input_port());
  return rgbd_sensor_sys;
}

void AddSimRgbdSensorLcmPublisher(const SimRgbdSensor& sim_rgbd_sensor,
                                  const OutputPort<double>* rgb_port,
                                  const OutputPort<double>* depth_16u_port,
                                  bool do_compress,
                                  DiagramBuilder<double>* builder,
                                  drake::lcm::DrakeLcmInterface* lcm) {
  DRAKE_DEMAND(builder != nullptr);
  DRAKE_DEMAND(lcm != nullptr);
  if (!rgb_port && !depth_16u_port) return;

  auto image_to_lcm_image_array =
      builder->AddSystem<ImageToLcmImageArrayT>(do_compress);
  image_to_lcm_image_array->set_name("image_to_lcm_" +
                                     sim_rgbd_sensor.serial());
  if (depth_16u_port) {
    const auto& lcm_depth_port =
        image_to_lcm_image_array->DeclareImageInputPort<PixelType::kDepth16U>(
            "depth");
    builder->Connect(*depth_16u_port, lcm_depth_port);
  }
  if (rgb_port) {
    const auto& lcm_rgb_port =
        image_to_lcm_image_array->DeclareImageInputPort<PixelType::kRgba8U>(
            "rgb");
    builder->Connect(*rgb_port, lcm_rgb_port);
  }
  auto image_array_lcm_publisher =
      builder->AddSystem(lcm::LcmPublisherSystem::Make<lcmt_image_array>(
          "DRAKE_RGBD_CAMERA_IMAGES_" + sim_rgbd_sensor.serial(), lcm,
          1. / sim_rgbd_sensor.rate_hz()));
  builder->Connect(image_to_lcm_image_array->image_array_t_msg_output_port(),
                   image_array_lcm_publisher->get_input_port());
}

}  // namespace internal
}  // namespace sensors
}  // namespace systems
}  // namespace drake
