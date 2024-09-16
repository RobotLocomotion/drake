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
    : defaults_{parent_id, std::move(color_camera), std::move(depth_camera),
                X_PB} {
  const CameraInfo& color_intrinsics =
      defaults_.color_camera.core().intrinsics();
  const CameraInfo& depth_intrinsics =
      defaults_.depth_camera.core().intrinsics();

  query_object_input_port_ = &this->DeclareAbstractInputPort(
      "geometry_query", Value<QueryObject<double>>{});

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

  // Declare parameters.
  parameter_indices_.parent_frame_id =
      AbstractParameterIndex{this->DeclareAbstractParameter(Value<FrameId>())};
  parameter_indices_.color_camera =
      AbstractParameterIndex{this->DeclareAbstractParameter(
          Value<ColorRenderCamera>(defaults_.color_camera))};
  parameter_indices_.depth_camera =
      AbstractParameterIndex{this->DeclareAbstractParameter(
          Value<DepthRenderCamera>(defaults_.depth_camera))};
  parameter_indices_.X_PB = AbstractParameterIndex{
      this->DeclareAbstractParameter(Value<RigidTransformd>())};

  // The depth_16U represents depth in *millimeters*. With 16 bits there is
  // an absolute limit on the farthest distance it can register. This tests to
  // see if the user has specified a maximum depth value that exceeds that
  // value.
  const float kMaxValidDepth16UInM =
      (std::numeric_limits<uint16_t>::max() - 1) / 1000.;
  const double max_depth = defaults_.depth_camera.depth_range().max_depth();
  if (max_depth > kMaxValidDepth16UInM) {
    drake::log()->warn(
        "Specified max depth is {} m > max valid depth for 16 bits {} m. "
        "depth_image_16u might not be able to capture the full depth range.",
        max_depth, kMaxValidDepth16UInM);
  }
}

RgbdSensor::~RgbdSensor() = default;

const ColorRenderCamera& RgbdSensor::default_color_render_camera() const {
  return defaults_.color_camera;
}

void RgbdSensor::set_default_color_render_camera(
    ColorRenderCamera color_camera) {
  defaults_.color_camera = color_camera;
}

ColorRenderCamera RgbdSensor::GetColorRenderCamera(
    const Context<double>& context) const {
  return context.get_abstract_parameter(parameter_indices_.color_camera)
      .get_value<ColorRenderCamera>();
}

void RgbdSensor::SetColorRenderCamera(Context<double>* context,
                                      ColorRenderCamera color_camera) const {
  context->get_mutable_abstract_parameter(parameter_indices_.color_camera)
      .set_value(color_camera);
}

const DepthRenderCamera& RgbdSensor::default_depth_render_camera() const {
  return defaults_.depth_camera;
}

void RgbdSensor::set_default_depth_render_camera(
    DepthRenderCamera depth_camera) {
  defaults_.depth_camera = depth_camera;
}

DepthRenderCamera RgbdSensor::GetDepthRenderCamera(
    const Context<double>& context) const {
  return context.get_abstract_parameter(parameter_indices_.depth_camera)
      .get_value<DepthRenderCamera>();
}

void RgbdSensor::SetDepthRenderCamera(Context<double>* context,
                                      DepthRenderCamera depth_camera) const {
  context->get_mutable_abstract_parameter(parameter_indices_.depth_camera)
      .set_value(depth_camera);
}

const RigidTransformd& RgbdSensor::default_X_PB() const {
  return defaults_.X_PB;
}

void RgbdSensor::set_default_X_PB(RigidTransformd sensor_pose) {
  defaults_.X_PB = sensor_pose;
}

RigidTransformd RgbdSensor::GetX_PB(const Context<double>& context) const {
  return context.get_abstract_parameter(parameter_indices_.X_PB)
      .get_value<RigidTransformd>();
}

void RgbdSensor::SetX_PB(Context<double>* context,
                         RigidTransformd sensor_pose) const {
  context->get_mutable_abstract_parameter(parameter_indices_.X_PB)
      .set_value(sensor_pose);
}

const RigidTransformd& RgbdSensor::default_X_BC() const {
  return defaults_.color_camera.core().sensor_pose_in_camera_body();
}

const RigidTransformd& RgbdSensor::default_X_BD() const {
  return defaults_.depth_camera.core().sensor_pose_in_camera_body();
}

FrameId RgbdSensor::default_parent_frame_id() const {
  return defaults_.parent_frame_id;
}

void RgbdSensor::set_default_parent_frame_id(FrameId id) {
  defaults_.parent_frame_id = id;
}

FrameId RgbdSensor::GetParentFrameId(const Context<double>& context) const {
  return context.get_abstract_parameter(parameter_indices_.parent_frame_id)
      .get_value<FrameId>();
}

void RgbdSensor::SetParentFrameId(Context<double>* context, FrameId id) const {
  context->get_mutable_abstract_parameter(parameter_indices_.parent_frame_id)
      .set_value(id);
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

const CameraInfo& RgbdSensor::color_camera_info() const {
  return defaults_.color_camera.core().intrinsics();
}

const CameraInfo& RgbdSensor::depth_camera_info() const {
  return defaults_.depth_camera.core().intrinsics();
}

const ColorRenderCamera& RgbdSensor::color_render_camera() const {
  return default_color_render_camera();
}

const DepthRenderCamera& RgbdSensor::depth_render_camera() const {
  return default_depth_render_camera();
}

const RigidTransformd& RgbdSensor::X_PB() const {
  return default_X_PB();
}

const RigidTransformd& RgbdSensor::X_BC() const {
  return default_X_BC();
}

const RigidTransformd& RgbdSensor::X_BD() const {
  return default_X_BD();
}

FrameId RgbdSensor::parent_frame_id() const {
  return default_parent_frame_id();
}

void RgbdSensor::CalcColorImage(const Context<double>& context,
                                ImageRgba8U* color_image) const {
  const QueryObject<double>& query_object = get_query_object(context);
  query_object.RenderColorImage(GetColorRenderCamera(context),
                                GetParentFrameId(context), GetX_PB(context),
                                color_image);
}

void RgbdSensor::CalcDepthImage32F(const Context<double>& context,
                                   ImageDepth32F* depth_image) const {
  const QueryObject<double>& query_object = get_query_object(context);
  query_object.RenderDepthImage(GetDepthRenderCamera(context),
                                GetParentFrameId(context), GetX_PB(context),
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
  query_object.RenderLabelImage(GetColorRenderCamera(context),
                                GetParentFrameId(context), GetX_PB(context),
                                label_image);
}

void RgbdSensor::CalcX_WB(const Context<double>& context,
                          RigidTransformd* X_WB) const {
  DRAKE_DEMAND(X_WB != nullptr);
  FrameId parent_frame_id = GetParentFrameId(context);
  RigidTransformd X_PB = GetX_PB(context);
  if (parent_frame_id == SceneGraph<double>::world_frame_id()) {
    *X_WB = X_PB;
  } else {
    const QueryObject<double>& query_object = get_query_object(context);
    *X_WB = query_object.GetPoseInWorld(parent_frame_id) * X_PB;
  }
}

void RgbdSensor::CalcImageTime(const Context<double>& context,
                               BasicVector<double>* output) const {
  output->SetFromVector(Vector1d{context.get_time()});
}

const QueryObject<double>& RgbdSensor::get_query_object(
    const Context<double>& context) const {
  return query_object_input_port().Eval<QueryObject<double>>(context);
}

void RgbdSensor::SetDefaultParameters(const Context<double>& context,
                                      Parameters<double>* parameters) const {
  LeafSystem<double>::SetDefaultParameters(context, parameters);
  parameters->get_mutable_abstract_parameter(parameter_indices_.parent_frame_id)
      .set_value(default_parent_frame_id());
  parameters->get_mutable_abstract_parameter(parameter_indices_.color_camera)
      .set_value(default_color_render_camera());
  parameters->get_mutable_abstract_parameter(parameter_indices_.depth_camera)
      .set_value(default_depth_render_camera());
  parameters->get_mutable_abstract_parameter(parameter_indices_.X_PB)
      .set_value(default_X_PB());
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
