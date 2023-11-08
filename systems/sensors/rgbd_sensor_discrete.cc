#include "drake/systems/sensors/rgbd_sensor_discrete.h"

#include <string>
#include <utility>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/zero_order_hold.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {
namespace {

using geometry::FrameId;
using geometry::render::ClippingRange;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRange;
using geometry::render::DepthRenderCamera;
using geometry::render::RenderCameraCore;
using math::RigidTransformd;

std::unique_ptr<RgbdSensor> MakeSensor(
    FrameId parent_id, const RigidTransformd& X_PB,
    const std::optional<ColorRenderCamera>& color_camera_in,
    const std::optional<DepthRenderCamera>& depth_camera_in) {
  DRAKE_THROW_UNLESS(color_camera_in.has_value() ||
                     depth_camera_in.has_value());
  std::optional<ColorRenderCamera> color_camera;
  std::optional<DepthRenderCamera> depth_camera;
  if (color_camera_in.has_value()) {
    color_camera.emplace(*color_camera_in);
  } else {
    DRAKE_DEMAND(depth_camera_in.has_value());
    const RenderCameraCore& core = depth_camera_in->core();
    color_camera.emplace(core);
  }
  if (depth_camera_in.has_value()) {
    depth_camera.emplace(*depth_camera_in);
  } else {
    DRAKE_DEMAND(color_camera_in.has_value());
    const RenderCameraCore& core = color_camera_in->core();
    const ClippingRange& clip = core.clipping();
    // N.B. Avoid using clip.far() here; it can trip the "16 bit mm depth"
    // logger spam from RgbdSensor.
    depth_camera.emplace(core, DepthRange(clip.near(), clip.near() * 1.001));
  }
  return std::make_unique<RgbdSensor>(parent_id, X_PB, std::move(*color_camera),
                                      std::move(*depth_camera));
}

}  // namespace

RgbdSensorDiscrete::RgbdSensorDiscrete(
    FrameId parent_id, const RigidTransformd& X_PB, double fps,
    double capture_offset, const std::optional<ColorRenderCamera>& color_camera,
    const std::optional<DepthRenderCamera>& depth_camera,
    bool render_label_image)
    : RgbdSensorDiscrete(
          MakeSensor(parent_id, X_PB, color_camera, depth_camera),
          false /* full_sized_empty */, 1.0 / fps, fps, capture_offset,
          color_camera.has_value(), depth_camera.has_value(),
          render_label_image) {}

RgbdSensorDiscrete::RgbdSensorDiscrete(std::unique_ptr<RgbdSensor> camera,
                                       double period, bool render_label_image)
    : RgbdSensorDiscrete(std::move(camera), true /* full_sized_empty */, period,
                         1.0 / period, 0 /* capture offset */,
                         true /* render_color_image */,
                         true /* render_depth_image */, render_label_image) {}

RgbdSensorDiscrete::RgbdSensorDiscrete(std::unique_ptr<RgbdSensor> camera,
                                       bool full_sized_empty, double period,
                                       double fps, double capture_offset,
                                       bool render_color_image,
                                       bool render_depth_image,
                                       bool render_label_image)
    : camera_(camera.get()),
      period_(period),
      fps_(fps),
      capture_offset_(capture_offset) {
  DiagramBuilder<double> builder;
  builder.AddSystem(std::move(camera));
  query_object_input_port_ =
      builder.ExportInput(camera_->query_object_input_port(), "geometry_query");

  // We'll use a helper functor to de-duplicate the ZOH boilerplate.
  auto connect_and_export_zoh = [&](const std::string& port_name,
                                    const auto& camera_info,
                                    auto temporary_image) {
    if (full_sized_empty) {
      temporary_image.resize(camera_info.width(), camera_info.height());
    }
    const auto* const zoh = builder.AddSystem<ZeroOrderHold>(
        period, Value(std::move(temporary_image)), capture_offset);
    builder.Connect(camera_->GetOutputPort(port_name), zoh->get_input_port());
    return builder.ExportOutput(zoh->get_output_port(), port_name);
  };

  // Color image.
  if (render_color_image) {
    output_port_color_image_ = connect_and_export_zoh(
        "color_image", camera_->color_camera_info(), ImageRgba8U());
  }

  // Depth images.
  if (render_depth_image) {
    output_port_depth_image_32F_ = connect_and_export_zoh(
        "depth_image_32f", camera_->depth_camera_info(), ImageDepth32F());
    output_port_depth_image_16U_ = connect_and_export_zoh(
        "depth_image_16u", camera_->depth_camera_info(), ImageDepth16U());
  }

  // Label image.
  if (render_label_image) {
    output_port_label_image_ = connect_and_export_zoh(
        "label_image", camera_->color_camera_info(), ImageLabel16I());
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

const OutputPort<double>& RgbdSensorDiscrete::color_image_output_port() const {
  if (!output_port_color_image_.has_value()) {
    throw std::logic_error(
        "RgbdSensorDiscrete has no color_image output port because no "
        "ColorRenderCamera was supplied to the constructor");
  }
  return get_output_port(*output_port_color_image_);
}

const OutputPort<double>& RgbdSensorDiscrete::depth_image_32F_output_port()
    const {
  if (!output_port_depth_image_32F_.has_value()) {
    throw std::logic_error(
        "RgbdSensorDiscrete has no depth_image_32F output port because no "
        "DepthRenderCamera was supplied to the constructor");
  }
  return get_output_port(*output_port_depth_image_32F_);
}

const OutputPort<double>& RgbdSensorDiscrete::depth_image_16U_output_port()
    const {
  if (!output_port_depth_image_16U_.has_value()) {
    throw std::logic_error(
        "RgbdSensorDiscrete has no depth_image_16U output port because no "
        "DepthRenderCamera was supplied to the constructor");
  }
  return get_output_port(*output_port_depth_image_16U_);
}

const OutputPort<double>& RgbdSensorDiscrete::label_image_output_port() const {
  if (!output_port_label_image_.has_value()) {
    throw std::logic_error(
        "RgbdSensorDiscrete has no label_image output port because "
        "render_label_image was set to false in the constructor");
  }
  return get_output_port(*output_port_label_image_);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
