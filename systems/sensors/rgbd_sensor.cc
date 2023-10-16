#include "drake/systems/sensors/rgbd_sensor.h"

#include <limits>
#include <utility>

#include "drake/common/text_logging.h"
#include "drake/geometry/scene_graph.h"

namespace drake {
namespace systems {
namespace sensors {

using geometry::FrameId;
using geometry::QueryObject;
using geometry::SceneGraph;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRenderCamera;
using math::RigidTransformd;

RgbdSensor::RgbdSensor(FrameId parent_id, const RigidTransformd& X_PB,
                       const DepthRenderCamera& depth_camera,
                       bool show_color_window)
    : RgbdSensor(parent_id, X_PB,
                 ColorRenderCamera(depth_camera.core(), show_color_window),
                 depth_camera) {}

RgbdSensor::RgbdSensor(FrameId parent_id, const RigidTransformd& X_PB,
                       ColorRenderCamera color_camera,
                       DepthRenderCamera depth_camera)
    : parent_frame_id_(parent_id),
      color_camera_(std::move(color_camera)),
      depth_camera_(std::move(depth_camera)),
      X_PB_(X_PB) {
  const CameraInfo& color_intrinsics = color_camera_.core().intrinsics();
  const CameraInfo& depth_intrinsics = depth_camera_.core().intrinsics();

  query_object_input_port_ = &this->DeclareAbstractInputPort(
      "geometry_query", Value<geometry::QueryObject<double>>{});

  ImageRgba8U color_image(color_intrinsics.width(), color_intrinsics.height());
  color_image_port_ = &this->DeclareAbstractOutputPort(
      "color_image", color_image, &RgbdSensor::CalcColorImage);

  ImageDepth32F depth32(depth_intrinsics.width(), depth_intrinsics.height());
  depth_image_32F_port_ = &this->DeclareAbstractOutputPort(
      "depth_image_32f", depth32, &RgbdSensor::CalcDepthImage32F);

  ImageDepth16U depth16(depth_intrinsics.width(), depth_intrinsics.height());
  depth_image_16U_port_ = &this->DeclareAbstractOutputPort(
      "depth_image_16u", depth16, &RgbdSensor::CalcDepthImage16U);

  ImageLabel16I label_image(color_intrinsics.width(),
                            color_intrinsics.height());
  label_image_port_ = &this->DeclareAbstractOutputPort(
      "label_image", label_image, &RgbdSensor::CalcLabelImage);

  body_pose_in_world_output_port_ = &this->DeclareAbstractOutputPort(
      "body_pose_in_world", &RgbdSensor::CalcX_WB);

  image_time_output_port_ = &this->DeclareVectorOutputPort(
      "image_time", 1, &RgbdSensor::CalcImageTime, {this->time_ticket()});

  // The depth_16U represents depth in *millimeters*. With 16 bits there is
  // an absolute limit on the farthest distance it can register. This tests to
  // see if the user has specified a maximum depth value that exceeds that
  // value.
  const float kMaxValidDepth16UInM =
      (std::numeric_limits<uint16_t>::max() - 1) / 1000.;
  const double max_depth = depth_camera_.depth_range().max_depth();
  if (max_depth > kMaxValidDepth16UInM) {
    drake::log()->warn(
        "Specified max depth is {} m > max valid depth for 16 bits {} m. "
        "depth_image_16u might not be able to capture the full depth range.",
        max_depth, kMaxValidDepth16UInM);
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

const OutputPort<double>& RgbdSensor::body_pose_in_world_output_port() const {
  return *body_pose_in_world_output_port_;
}

const OutputPort<double>& RgbdSensor::image_time_output_port() const {
  return *image_time_output_port_;
}

void RgbdSensor::CalcColorImage(const Context<double>& context,
                                ImageRgba8U* color_image) const {
  const QueryObject<double>& query_object = get_query_object(context);
  query_object.RenderColorImage(color_camera_, parent_frame_id_, X_PB_,
                                color_image);
}

void RgbdSensor::CalcDepthImage32F(const Context<double>& context,
                                   ImageDepth32F* depth_image) const {
  const QueryObject<double>& query_object = get_query_object(context);
  query_object.RenderDepthImage(depth_camera_, parent_frame_id_, X_PB_,
                                depth_image);
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
  query_object.RenderLabelImage(color_camera_, parent_frame_id_, X_PB_,
                                label_image);
}

void RgbdSensor::CalcX_WB(const Context<double>& context,
                          RigidTransformd* X_WB) const {
  DRAKE_DEMAND(X_WB != nullptr);
  if (parent_frame_id_ == SceneGraph<double>::world_frame_id()) {
    *X_WB = X_PB_;
  } else {
    const QueryObject<double>& query_object = get_query_object(context);
    *X_WB = query_object.GetPoseInWorld(parent_frame_id_) * X_PB_;
  }
}

void RgbdSensor::CalcImageTime(const Context<double>& context,
                               BasicVector<double>* output) const {
  output->SetFromVector(Vector1d{context.get_time()});
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
