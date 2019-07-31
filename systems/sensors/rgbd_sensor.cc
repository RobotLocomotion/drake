#include "drake/systems/sensors/rgbd_sensor.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include <Eigen/Dense>

#include "drake/geometry/render/camera_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/zero_order_hold.h"
#include "drake/systems/rendering/pose_vector.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {

using Eigen::Translation3d;
using geometry::FrameId;
using geometry::QueryObject;
using geometry::render::CameraProperties;
using geometry::render::DepthCameraProperties;
using geometry::SceneGraph;
using math::RigidTransformd;
using std::move;

RgbdSensor::RgbdSensor(FrameId parent_id,
                       const RigidTransformd& X_PB,
                       const CameraProperties& color_properties,
                       const DepthCameraProperties& depth_properties,
                       const CameraPoses& camera_poses,
                       bool show_window)
    : parent_frame_id_(parent_id),
      show_window_(show_window),
      color_camera_info_(color_properties.width, color_properties.height,
                         color_properties.fov_y),
      depth_camera_info_(depth_properties.width, depth_properties.height,
                         depth_properties.fov_y),
      color_properties_(color_properties),
      depth_properties_(depth_properties),
      X_PB_(X_PB),
      X_BC_(camera_poses.X_BC),
      X_BD_(camera_poses.X_BD) {
  query_object_input_port_ = &this->DeclareAbstractInputPort(
      "geometry_query", Value<geometry::QueryObject<double>>{});

  ImageRgba8U color_image(color_camera_info_.width(),
                          color_camera_info_.height());
  color_image_port_ = &this->DeclareAbstractOutputPort(
      "color_image", color_image, &RgbdSensor::CalcColorImage);

  ImageDepth32F depth32(depth_camera_info_.width(),
                        depth_camera_info_.height());
  depth_image_32F_port_ = &this->DeclareAbstractOutputPort(
      "depth_image_32f", depth32, &RgbdSensor::CalcDepthImage32F);

  ImageDepth16U depth16(depth_camera_info_.width(),
                        depth_camera_info_.height());
  depth_image_16U_port_ = &this->DeclareAbstractOutputPort(
      "depth_image_16u", depth16, &RgbdSensor::CalcDepthImage16U);

  ImageLabel16I label_image(color_camera_info_.width(),
                            color_camera_info_.height());
  label_image_port_ = &this->DeclareAbstractOutputPort(
      "label_image", label_image, &RgbdSensor::CalcLabelImage);

  X_WB_pose_port_ = &this->DeclareVectorOutputPort(
      "X_WB", rendering::PoseVector<double>(), &RgbdSensor::CalcX_WB);

  const float kMaxValidDepth16UInMM =
      (std::numeric_limits<uint16_t>::max() - 1) / 1000.;
  if (depth_properties.z_far > kMaxValidDepth16UInMM) {
    drake::log()->warn(
        "Specified max depth is {} > max valid depth for 16 bits {}. "
        "depth_image_16u might not be able to capture the full depth range.",
        depth_properties.z_far, kMaxValidDepth16UInMM);
  }
}

const InputPort<double>& RgbdSensor::query_object_input_port() const {
  return *query_object_input_port_;
}

const OutputPort<double>& RgbdSensor::color_image_output_port() const {
  return *color_image_port_;
}

const OutputPort<double>& RgbdSensor::depth_image_32F_output_port() const {
  return *depth_image_32F_port_;
}

const OutputPort<double>& RgbdSensor::depth_image_16U_output_port() const {
  return *depth_image_16U_port_;
}

const OutputPort<double>& RgbdSensor::label_image_output_port() const {
  return *label_image_port_;
}

const OutputPort<double>& RgbdSensor::X_WB_output_port() const {
  return *X_WB_pose_port_;
}

void RgbdSensor::CalcColorImage(const Context<double>& context,
                                ImageRgba8U* color_image) const {
  const QueryObject<double>& query_object = get_query_object(context);
  query_object.RenderColorImage(color_properties_, parent_frame_id_,
                                X_PB_ * X_BC_, show_window_, color_image);
}

void RgbdSensor::CalcDepthImage32F(const Context<double>& context,
                                   ImageDepth32F* depth_image) const {
  const QueryObject<double>& query_object = get_query_object(context);
  query_object.RenderDepthImage(depth_properties_, parent_frame_id_,
                                X_PB_ * X_BD_, depth_image);
}

void RgbdSensor::CalcDepthImage16U(const Context<double>& context,
                                   ImageDepth16U* depth_image) const {
  ImageDepth32F depth32(depth_image->width(), depth_image->height());
  CalcDepthImage32F(context, &depth32);
  ConvertDepth32FTo16U(depth32, depth_image);
}

void RgbdSensor::CalcLabelImage(const Context<double>& context,
                                ImageLabel16I* label_image) const {
  const QueryObject<double>& query_object = get_query_object(context);
  query_object.RenderLabelImage(color_properties_, parent_frame_id_,
                                X_PB_ * X_BC_, show_window_, label_image);
}

void RgbdSensor::CalcX_WB(
    const Context<double>& context,
    rendering::PoseVector<double>* pose_vector) const {
  // Calculates X_WB.
  RigidTransformd X_WB;
  if (parent_frame_id_ == SceneGraph<double>::world_frame_id()) {
    X_WB = X_PB_;
  } else {
    const QueryObject<double>& query_object = get_query_object(context);
    X_WB = query_object.X_WF(parent_frame_id_) * X_PB_;
  }

  Translation3d trans{X_WB.translation()};
  pose_vector->set_translation(trans);

  pose_vector->set_rotation(X_WB.rotation().ToQuaternion());
}

void RgbdSensor::ConvertDepth32FTo16U(const ImageDepth32F& d32,
                                      ImageDepth16U* d16) {
  // Convert to mm and 16bits.
  const float kDepth16UOverflowDistance =
      std::numeric_limits<uint16_t>::max() / 1000.;
  for (int w = 0; w < d16->width(); w++) {
    for (int h = 0; h < d16->height(); h++) {
      const double dist = std::min(d32.at(w, h)[0], kDepth16UOverflowDistance);
      d16->at(w, h)[0] = static_cast<uint16_t>(dist * 1000);
    }
  }
}

// Note: Ideally, this would be inlined. However, inlining this caused GCC
// to do something weird with GeometryState such that the rgbd_sensor_test.cc
// was unable to instantiate an GeometryStateTester that GCC recognized as a
// friend to GeometryState.
const geometry::QueryObject<double>& RgbdSensor::get_query_object(
    const Context<double>& context) const {
  return query_object_input_port().
      Eval<geometry::QueryObject<double>>(context);
}

RgbdSensorDiscrete::RgbdSensorDiscrete(std::unique_ptr<RgbdSensor> camera,
                                       double period, bool render_label_image)
    : camera_(camera.get()), period_(period) {
  const auto& color_camera_info = camera->color_camera_info();
  const auto& depth_camera_info = camera->depth_camera_info();

  DiagramBuilder<double> builder;
  builder.AddSystem(std::move(camera));
  query_object_port_ =
      builder.ExportInput(camera_->query_object_input_port(), "geometry_query");

  // Color image.
  const Value<ImageRgba8U> image_color(color_camera_info.width(),
                                       color_camera_info.height());
  const auto* const zoh_color =
      builder.AddSystem<ZeroOrderHold>(period_, image_color);
  builder.Connect(camera_->color_image_output_port(),
                  zoh_color->get_input_port());
  output_port_color_image_ =
      builder.ExportOutput(zoh_color->get_output_port(), "color_image");

  // Depth images.
  const Value<ImageDepth32F> image_depth_32F(depth_camera_info.width(),
                                             depth_camera_info.height());
  const auto* const zoh_depth_32F =
      builder.AddSystem<ZeroOrderHold>(period_, image_depth_32F);
  builder.Connect(camera_->depth_image_32F_output_port(),
                  zoh_depth_32F->get_input_port());
  output_port_depth_image_32F_ =
      builder.ExportOutput(zoh_depth_32F->get_output_port(), "depth_image_32f");

  // Depth images.
  const Value<ImageDepth16U> image_depth_16U(depth_camera_info.width(),
                                             depth_camera_info.height());
  const auto* const zoh_depth_16U =
      builder.AddSystem<ZeroOrderHold>(period_, image_depth_16U);
  builder.Connect(camera_->depth_image_16U_output_port(),
                  zoh_depth_16U->get_input_port());
  output_port_depth_image_16U_ =
      builder.ExportOutput(zoh_depth_16U->get_output_port(), "depth_image_16u");

  // Label image.
  if (render_label_image) {
    const Value<ImageLabel16I> image_label(color_camera_info.width(),
                                           color_camera_info.height());
    const auto* const zoh_label =
        builder.AddSystem<ZeroOrderHold>(period_, image_label);
    builder.Connect(camera_->label_image_output_port(),
                    zoh_label->get_input_port());
    output_port_label_image_ =
        builder.ExportOutput(zoh_label->get_output_port(), "label_image");
  }

  // No need to place a ZOH on pose output.
  X_WB_output_port_ =
      builder.ExportOutput(camera_->X_WB_output_port(), "X_WB");

  builder.BuildInto(this);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
