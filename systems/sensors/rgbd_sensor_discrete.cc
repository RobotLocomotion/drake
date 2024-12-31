#include "drake/systems/sensors/rgbd_sensor_discrete.h"

#include <utility>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/zero_order_hold.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {

RgbdSensorDiscrete::RgbdSensorDiscrete(std::shared_ptr<RgbdSensor> camera,
                                       double period, bool render_label_image)
    : camera_(camera.get()), period_(period) {
  const auto& color_camera_info =
      camera->default_color_render_camera().core().intrinsics();
  const auto& depth_camera_info =
      camera->default_depth_render_camera().core().intrinsics();

  DiagramBuilder<double> builder;
  builder.AddSystem(std::move(camera));
  query_object_input_port_ =
      builder.ExportInput(camera_->query_object_input_port(), "geometry_query");

  // Color image.
  const Value<ImageRgba8U> image_color(color_camera_info.width(),
                                       color_camera_info.height());
  const auto* const zoh_color =
      builder.AddSystem<ZeroOrderHold>(period_, image_color);
  builder.Connect(camera_->color_image_output_port(),
                  zoh_color->get_input_port());
  color_image_output_port_ =
      builder.ExportOutput(zoh_color->get_output_port(), "color_image");

  // Depth images.
  const Value<ImageDepth32F> image_depth_32F(depth_camera_info.width(),
                                             depth_camera_info.height());
  const auto* const zoh_depth_32F =
      builder.AddSystem<ZeroOrderHold>(period_, image_depth_32F);
  builder.Connect(camera_->depth_image_32F_output_port(),
                  zoh_depth_32F->get_input_port());
  depth_image_32F_output_port_ =
      builder.ExportOutput(zoh_depth_32F->get_output_port(), "depth_image_32f");

  // Depth images.
  const Value<ImageDepth16U> image_depth_16U(depth_camera_info.width(),
                                             depth_camera_info.height());
  const auto* const zoh_depth_16U =
      builder.AddSystem<ZeroOrderHold>(period_, image_depth_16U);
  builder.Connect(camera_->depth_image_16U_output_port(),
                  zoh_depth_16U->get_input_port());
  depth_image_16U_output_port_ =
      builder.ExportOutput(zoh_depth_16U->get_output_port(), "depth_image_16u");

  // Label image.
  if (render_label_image) {
    const Value<ImageLabel16I> image_label(color_camera_info.width(),
                                           color_camera_info.height());
    const auto* const zoh_label =
        builder.AddSystem<ZeroOrderHold>(period_, image_label);
    builder.Connect(camera_->label_image_output_port(),
                    zoh_label->get_input_port());
    label_image_output_port_ =
        builder.ExportOutput(zoh_label->get_output_port(), "label_image");
  }

  const auto* const zoh_body_pose =
      builder.AddSystem<ZeroOrderHold>(period_, Value<math::RigidTransformd>{});
  builder.Connect(camera_->body_pose_in_world_output_port(),
                  zoh_body_pose->get_input_port());
  body_pose_in_world_output_port_ = builder.ExportOutput(
      zoh_body_pose->get_output_port(), "body_pose_in_world");

  const auto* const zoh_image_time =
      builder.AddSystem<ZeroOrderHold>(period_, 1);
  builder.Connect(camera_->image_time_output_port(),
                  zoh_image_time->get_input_port());
  image_time_output_port_ =
      builder.ExportOutput(zoh_body_pose->get_output_port(), "image_time");

  builder.BuildInto(this);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
