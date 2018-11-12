#include "drake/systems/sensors/dev/rgbd_camera.h"

#include <string>
#include <utility>

#include <Eigen/Dense>

#include "drake/geometry/dev/render/camera_properties.h"
#include "drake/geometry/dev/scene_graph.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/zero_order_hold.h"
#include "drake/systems/rendering/pose_vector.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {
namespace dev {

using geometry::FrameId;
using geometry::dev::QueryObject;
using geometry::dev::render::DepthCameraProperties;
using geometry::dev::SceneGraph;

RgbdCamera::RgbdCamera(const std::string& name, const Eigen::Vector3d& p_WB_W,
                       const Eigen::Vector3d& rpy_WB_W,
                       const DepthCameraProperties& properties,
                       bool show_window)
    : parent_frame_(SceneGraph<double>::world_frame_id()),
      show_window_(show_window),
      color_camera_info_(properties.width, properties.height, properties.fov_y),
      depth_camera_info_(properties.width, properties.height, properties.fov_y),
      properties_(properties),
      X_PB_(Eigen::Translation3d(p_WB_W[0], p_WB_W[1], p_WB_W[2]) *
            Eigen::Isometry3d(math::RollPitchYaw<double>(rpy_WB_W)
                                  .ToMatrix3ViaRotationMatrix())) {
  InitPorts(name);
}

RgbdCamera::RgbdCamera(const std::string& name, FrameId parent_frame,
                       const Isometry3<double>& X_PB,
                       const DepthCameraProperties& properties,
                       bool show_window)
    : parent_frame_(parent_frame),
      show_window_(show_window),
      color_camera_info_(properties.width, properties.height, properties.fov_y),
      depth_camera_info_(properties.width, properties.height, properties.fov_y),
      properties_(properties),
      X_PB_(X_PB) {
  InitPorts(name);
}

void RgbdCamera::InitPorts(const std::string& name) {
  this->set_name(name);

  query_object_input_port_ = &this->DeclareAbstractInputPort(
      "geometry_query", systems::Value<geometry::dev::QueryObject<double>>{});

  ImageRgba8U color_image(color_camera_info_.width(),
                          color_camera_info_.height());
  color_image_port_ = &this->DeclareAbstractOutputPort(
      "color_image", color_image, &RgbdCamera::CalcColorImage);

  ImageDepth32F depth_image(depth_camera_info_.width(),
                            depth_camera_info_.height());
  depth_image_port_ = &this->DeclareAbstractOutputPort(
      "depth_image", depth_image, &RgbdCamera::CalcDepthImage);

  ImageLabel16I label_image(color_camera_info_.width(),
                            color_camera_info_.height());
  label_image_port_ = &this->DeclareAbstractOutputPort(
      "label_image", label_image, &RgbdCamera::CalcLabelImage);

  camera_base_pose_port_ = &this->DeclareVectorOutputPort(
      "X_WB", rendering::PoseVector<double>(), &RgbdCamera::CalcPoseVector);
}

const InputPort<double>& RgbdCamera::query_object_input_port() const {
  return *query_object_input_port_;
}

const OutputPort<double>& RgbdCamera::camera_base_pose_output_port() const {
  return *camera_base_pose_port_;
}

const OutputPort<double>& RgbdCamera::color_image_output_port() const {
  return *color_image_port_;
}

const OutputPort<double>& RgbdCamera::depth_image_output_port() const {
  return *depth_image_port_;
}

const OutputPort<double>& RgbdCamera::label_image_output_port() const {
  return *label_image_port_;
}

void RgbdCamera::CalcPoseVector(
    const Context<double>& context,
    rendering::PoseVector<double>* pose_vector) const {
  // Calculates X_WB.
  Eigen::Isometry3d X_WB;
  if (parent_frame_ == SceneGraph<double>::world_frame_id()) {
    X_WB = X_PB_;
  } else {
    const QueryObject<double>& query_object = get_query_object(context);
    X_WB = query_object.GetPoseInWorld(parent_frame_) * X_PB_;
  }

  Eigen::Translation<double, 3> trans{X_WB.translation()};
  pose_vector->set_translation(trans);

  Eigen::Quaterniond quat{X_WB.linear()};
  pose_vector->set_rotation(quat);
}

void RgbdCamera::CalcColorImage(const Context<double>& context,
                                ImageRgba8U* color_image) const {
  const QueryObject<double>& query_object = get_query_object(context);
  query_object.RenderColorImage(properties_, parent_frame_, X_PB_ * X_BC_,
                                color_image, show_window_);
}

void RgbdCamera::CalcDepthImage(const Context<double>& context,
                                ImageDepth32F* depth_image) const {
  const QueryObject<double>& query_object = get_query_object(context);
  query_object.RenderDepthImage(properties_, parent_frame_, X_PB_ * X_BD_,
                                depth_image);
}

void RgbdCamera::CalcLabelImage(const Context<double>& context,
                                ImageLabel16I* label_image) const {
  const QueryObject<double>& query_object = get_query_object(context);
  query_object.RenderLabelImage(properties_, parent_frame_, X_PB_ * X_BC_,
                                label_image, show_window_);
}

RgbdCameraDiscrete::RgbdCameraDiscrete(std::unique_ptr<RgbdCamera> camera,
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

  // Depth image.
  const Value<ImageDepth32F> image_depth(depth_camera_info.width(),
                                         depth_camera_info.height());
  const auto* const zoh_depth =
      builder.AddSystem<ZeroOrderHold>(period_, image_depth);
  builder.Connect(camera_->depth_image_output_port(),
                  zoh_depth->get_input_port());
  output_port_depth_image_ =
      builder.ExportOutput(zoh_depth->get_output_port(), "depth_image");

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
  output_port_pose_ =
      builder.ExportOutput(camera_->camera_base_pose_output_port(), "X_WB");

  builder.BuildInto(this);
}

}  // namespace dev
}  // namespace sensors
}  // namespace systems
}  // namespace drake
