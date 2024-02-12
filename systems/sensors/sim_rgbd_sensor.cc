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

// TODO(#10247) Remove this function once all MbP frames are geometry frames.
std::pair<geometry::FrameId, RigidTransformd> GetGeometryFrame(
    const multibody::Frame<double>& sensor_frame, const RigidTransformd& X_PB) {
  // 'A' is the body for the parent sensor frame.
  // `P` is the parent of the sensor frame.
  // `B` is the sensor frame.

  const auto& frame_P = sensor_frame;
  const auto& body_A = frame_P.body();
  const auto& plant = sensor_frame.GetParentPlant();
  const std::optional<geometry::FrameId> body_A_id =
      plant.GetBodyFrameIdIfExists(body_A.index());
  DRAKE_THROW_UNLESS(body_A_id.has_value());

  // We must include the offset of the body to ensure that we posture the camera
  // appropriately.
  const RigidTransformd X_AP = frame_P.GetFixedPoseInBodyFrame();
  const RigidTransformd X_AB = X_AP * X_PB;

  return std::make_pair(*body_A_id, X_AB);
}

RgbdSensor* AddSimRgbdSensor(const SceneGraph<double>& scene_graph,
                             const MultibodyPlant<double>& /* plant */,
                             const SimRgbdSensor& sim_rgbd_sensor,
                             DiagramBuilder<double>* builder) {
  DRAKE_DEMAND(builder != nullptr);

  const auto [frame_A, X_AB] =
      GetGeometryFrame(sim_rgbd_sensor.frame(), sim_rgbd_sensor.X_PB());
  auto* rgbd_sensor_sys = builder->AddSystem<RgbdSensor>(
      frame_A, X_AB, sim_rgbd_sensor.color_properties(),
      sim_rgbd_sensor.depth_properties());
  // `set_name` is only used for debugging.
  rgbd_sensor_sys->set_name("rgbd_sensor_" + sim_rgbd_sensor.serial());
  builder->Connect(scene_graph.get_query_output_port(),
                   rgbd_sensor_sys->query_object_input_port());
  return rgbd_sensor_sys;
}

void AddSimRgbdSensorLcmPublisher(std::string_view serial, double fps,
                                  double publish_offset,
                                  const OutputPort<double>* rgb_port,
                                  const OutputPort<double>* depth_16u_port,
                                  const OutputPort<double>* label_port,
                                  bool do_compress,
                                  DiagramBuilder<double>* builder,
                                  drake::lcm::DrakeLcmInterface* lcm) {
  DRAKE_DEMAND(builder != nullptr);
  DRAKE_DEMAND(lcm != nullptr);
  if (!rgb_port && !depth_16u_port && !label_port) return;

  auto image_to_lcm_image_array =
      builder->AddSystem<ImageToLcmImageArrayT>(do_compress);
  image_to_lcm_image_array->set_name(fmt::format("image_to_lcm_{}", serial));
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
  if (label_port) {
    const auto& lcm_label_port =
        image_to_lcm_image_array->DeclareImageInputPort<PixelType::kLabel16I>(
            "label");
    builder->Connect(*label_port, lcm_label_port);
  }
  auto image_array_lcm_publisher =
      builder->AddSystem(lcm::LcmPublisherSystem::Make<lcmt_image_array>(
          fmt::format("DRAKE_RGBD_CAMERA_IMAGES_{}", serial), lcm, 1.0 / fps,
          publish_offset));
  builder->Connect(image_to_lcm_image_array->image_array_t_msg_output_port(),
                   image_array_lcm_publisher->get_input_port());
}

void AddSimRgbdSensorLcmPublisher(const SimRgbdSensor& sim_camera,
                                  const OutputPort<double>* rgb_port,
                                  const OutputPort<double>* depth_16u_port,
                                  const OutputPort<double>* label_port,
                                  bool do_compress,
                                  DiagramBuilder<double>* builder,
                                  drake::lcm::DrakeLcmInterface* lcm) {
  const double publish_offset = 0.0;
  AddSimRgbdSensorLcmPublisher(sim_camera.serial(), sim_camera.rate_hz(),
                               publish_offset, rgb_port, depth_16u_port,
                               label_port, do_compress, builder, lcm);
}

}  // namespace internal
}  // namespace sensors
}  // namespace systems
}  // namespace drake
