#include "drake/systems/sensors/rgbd_sensor.h"

#include <limits>
#include <utility>

#include "drake/common/text_logging.h"
#include "drake/geometry/scene_graph.h"

namespace drake {
namespace systems {
namespace sensors {
namespace {

using geometry::FrameId;
using geometry::QueryObject;
using geometry::SceneGraph;
using geometry::render::ClippingRange;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRenderCamera;
using math::RigidTransformd;

/* If `camera` has a value, then returns a copy of it. Otherwise, creates a
 suitable value based on `other` and return that instead. If neither one
 contained a value, then returns a dummy value not intended for use. */
template <typename ColorOrDepthRenderCamera, typename OtherRenderCamera>
ColorOrDepthRenderCamera GetOrMakeCamera(
    const std::optional<ColorOrDepthRenderCamera>& camera,
    const std::optional<OtherRenderCamera>& other) {
  if (camera.has_value()) {
    return *camera;
  }
  if (other.has_value()) {
    return ColorOrDepthRenderCamera(other->core());
  }
  return ColorOrDepthRenderCamera({{}, {6, 4, M_PI}, {1, 2}, {}});
}

}  // namespace

RgbdSensor::RgbdSensor(FrameId parent_id, const RigidTransformd& X_PB,
                       const std::optional<ColorRenderCamera>& color_camera,
                       const std::optional<DepthRenderCamera>& depth_camera)
    : RgbdSensor(-1 /* ignored */, parent_id, X_PB,
                 GetOrMakeCamera(color_camera, depth_camera),
                 GetOrMakeCamera(depth_camera, color_camera)) {
  DRAKE_THROW_UNLESS(color_camera.has_value() || depth_camera.has_value());
}

RgbdSensor::RgbdSensor(FrameId parent_id, const RigidTransformd& X_PB,
                       const DepthRenderCamera& depth_camera,
                       bool show_color_window)
    : RgbdSensor(-1 /* ignored */, parent_id, X_PB,
                 ColorRenderCamera(depth_camera.core(), show_color_window),
                 depth_camera) {}

RgbdSensor::RgbdSensor(int /* dummy value to disambiguate our overloads */,
                       FrameId parent_id, const RigidTransformd& X_PB,
                       const ColorRenderCamera& color_camera,
                       const DepthRenderCamera& depth_camera)
    : parent_frame_id_(parent_id),
      color_camera_(color_camera),
      depth_camera_(depth_camera),
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

  // The depth_image_16u represents depth in *millimeters*. With 16 bits there
  // is an absolute limit on the farthest distance it can register.
  constexpr float kMaxValidDepth16UInMeters =
      (std::numeric_limits<uint16_t>::max() - 1) / 1000.0;
  // Check if the user has specified a maximum depth value that exceeds what we
  // can represent on the depth_image_16u output port.
  const double max_depth = depth_camera_.depth_range().max_depth();
  if (max_depth > kMaxValidDepth16UInMeters) {
    drake::log()->warn(
        "Specified max depth is {} m > max valid depth for 16 bits {} m. "
        "depth_image_16u might not be able to capture the full depth range.",
        max_depth, kMaxValidDepth16UInMeters);
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

void RgbdSensor::CalcColorImage(const Context<double>& context,
                                ImageRgba8U* color_image) const {
  const QueryObject<double>& query_object = get_query_object(context);
  query_object.RenderColorImage(
      color_camera_, parent_frame_id_,
      X_PB_ * color_camera_.core().sensor_pose_in_camera_body(), color_image);
}

void RgbdSensor::CalcDepthImage32F(const Context<double>& context,
                                   ImageDepth32F* depth_image) const {
  const QueryObject<double>& query_object = get_query_object(context);
  query_object.RenderDepthImage(
      depth_camera_, parent_frame_id_,
      X_PB_ * depth_camera_.core().sensor_pose_in_camera_body(), depth_image);
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
  query_object.RenderLabelImage(
      color_camera_, parent_frame_id_,
      X_PB_ * color_camera_.core().sensor_pose_in_camera_body(), label_image);
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

}  // namespace sensors
}  // namespace systems
}  // namespace drake
