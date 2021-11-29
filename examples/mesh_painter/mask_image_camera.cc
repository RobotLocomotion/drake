#include "drake/examples/mesh_painter/mask_image_camera.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include <Eigen/Dense>

#include "drake/common/nice_type_name.h"
#include "drake/examples/mesh_painter/render_engine_vtk_with_mask_images.h"
#include "drake/geometry/render/render_camera.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/zero_order_hold.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace examples {
namespace mesh_painter {

using geometry::FrameId;
using geometry::GeometryId;
using geometry::PerceptionProperties;
using geometry::QueryObject;
using geometry::SceneGraph;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRenderCamera;
using geometry::render::RenderEngine;
using math::RigidTransformd;
using systems::Context;
using systems::EventStatus;
using systems::InputPort;
using systems::LeafContext;
using systems::OutputPort;
using systems::State;
using systems::sensors::ImageDepth16U;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;
using systems::sensors::RgbdSensor;
using Eigen::Translation3d;
using std::move;

MaskImageCamera::MaskImageCamera(FrameId parent_id, const RigidTransformd& X_PB,
                                 std::string render_engine_name,
                                 const GeometryId masked_id,
                                 const ColorRenderCamera& color_properties,
                                 const DepthRenderCamera& depth_properties)
    : parent_frame_id_(parent_id),
      color_properties_(color_properties),
      depth_properties_(depth_properties),
      X_PB_(X_PB),
      render_engine_name_(move(render_engine_name)),
      masked_geometry_(masked_id) {
  query_object_input_port_ = &this->DeclareAbstractInputPort(
      "geometry_query", Value<QueryObject<double>>{});

  mask_texture_input_port_ =
      &this->DeclareAbstractInputPort("mask_image", Value<ImageRgba8U>{});

  ImageRgba8U color_image(color_camera_info().width(),
                          color_camera_info().height());
  color_image_port_ = &this->DeclareAbstractOutputPort(
      "color_image", color_image, &MaskImageCamera::CalcColorImage);

  ImageDepth32F depth32(depth_camera_info().width(),
                        depth_camera_info().height());
  depth_image_32F_port_ = &this->DeclareAbstractOutputPort(
      "depth_image_32f", depth32, &MaskImageCamera::CalcDepthImage32F);

  ImageDepth16U depth16(depth_camera_info().width(),
                        depth_camera_info().height());
  depth_image_16U_port_ = &this->DeclareAbstractOutputPort(
      "depth_image_16u", depth16, &MaskImageCamera::CalcDepthImage16U);

  ImageLabel16I label_image(color_camera_info().width(),
                            color_camera_info().height());
  label_image_port_ = &this->DeclareAbstractOutputPort(
      "label_image", label_image, &MaskImageCamera::CalcLabelImage);

  camera_body_pose_in_world_output_port_ = &this->DeclareAbstractOutputPort(
      "camera_body_pose_in_world", RigidTransformd(),
      &MaskImageCamera::CalcX_WB);

  const float kMaxValidDepth16UInMM =
      (std::numeric_limits<uint16_t>::max() - 1) / 1000.;
  if (depth_properties.depth_range().max_depth() > kMaxValidDepth16UInMM) {
    log()->warn(
        "Specified max depth is {} > max valid depth for 16 bits {}. "
        "depth_image_16u might not be able to capture the full depth range.",
        depth_properties.depth_range().max_depth(), kMaxValidDepth16UInMM);
  }

  this->DeclareInitializationUnrestrictedUpdateEvent(
      &MaskImageCamera::InitMaskedGeometry);
}

const InputPort<double>& MaskImageCamera::query_object_input_port() const {
  return *query_object_input_port_;
}

const InputPort<double>& MaskImageCamera::mask_texture_input_port() const {
  return *mask_texture_input_port_;
}

const OutputPort<double>& MaskImageCamera::color_image_output_port() const {
  return *color_image_port_;
}

const OutputPort<double>& MaskImageCamera::depth_image_32F_output_port() const {
  return *depth_image_32F_port_;
}

const OutputPort<double>& MaskImageCamera::depth_image_16U_output_port() const {
  return *depth_image_16U_port_;
}

const OutputPort<double>& MaskImageCamera::label_image_output_port() const {
  return *label_image_port_;
}

const OutputPort<double>&
MaskImageCamera::camera_body_pose_in_world_output_port() const {
  return *camera_body_pose_in_world_output_port_;
}

void MaskImageCamera::UpdateMaskImage(const Context<double>& context,
                                      MaskType mask_type) const {
  const QueryObject<double>& query_object = get_query_object(context);
  const RenderEngine* render_engine =
      query_object.GetRenderEngineByName(render_engine_name_);
  if (render_engine) {
    const RenderEngineVtkWithMaskImages* render_engine_w_mask =
        dynamic_cast<const RenderEngineVtkWithMaskImages*>(render_engine);

    const auto& mask = mask_texture_input_port().Eval<ImageRgba8U>(context);
    switch (mask_type) {
      case kColor:
        render_engine_w_mask->UpdateRgbaMaskForGeometry(masked_geometry_, mask);
        break;
      case kLabel:
        render_engine_w_mask->UpdateLabelMaskForGeometry(masked_geometry_,
                                                         mask);
        break;
    }
  }
}

EventStatus MaskImageCamera::InitMaskedGeometry(const Context<double>& context,
                                                State<double>*) const {
  const QueryObject<double>& query_object = get_query_object(context);
  const RenderEngine* render_engine =
      query_object.GetRenderEngineByName(render_engine_name_);

  if (render_engine == nullptr) return EventStatus::DidNothing();

  const RenderEngineVtkWithMaskImages* render_engine_w_mask =
      dynamic_cast<const RenderEngineVtkWithMaskImages*>(render_engine);

  const PerceptionProperties* props =
      query_object.inspector().GetPerceptionProperties(masked_geometry_);
  DRAKE_DEMAND(props != nullptr);
  render_engine_w_mask->InitializeMasksForGeometry(masked_geometry_, *props);
  return EventStatus::Succeeded();
}

void MaskImageCamera::CalcColorImage(const Context<double>& context,
                                     ImageRgba8U* color_image) const {
  UpdateMaskImage(context, kColor);
  const QueryObject<double>& query_object = get_query_object(context);

  query_object.RenderColorImage(color_properties_, parent_frame_id_,
                                X_PB_ * X_BC(), color_image);
}

void MaskImageCamera::CalcDepthImage32F(const Context<double>& context,
                                        ImageDepth32F* depth_image) const {
  const QueryObject<double>& query_object = get_query_object(context);
  query_object.RenderDepthImage(depth_properties_, parent_frame_id_,
                                X_PB_ * X_BD(), depth_image);
}

void MaskImageCamera::CalcDepthImage16U(const Context<double>& context,
                                        ImageDepth16U* depth_image) const {
  ImageDepth32F depth32(depth_image->width(), depth_image->height());
  CalcDepthImage32F(context, &depth32);
  ConvertDepth32FTo16U(depth32, depth_image);
}

void MaskImageCamera::CalcLabelImage(const Context<double>& context,
                                     ImageLabel16I* label_image) const {
  UpdateMaskImage(context, kLabel);
  const QueryObject<double>& query_object = get_query_object(context);
  query_object.RenderLabelImage(color_properties_, parent_frame_id_,
                                X_PB_ * X_BC(), label_image);
}

void MaskImageCamera::CalcX_WB(const Context<double>& context,
                               RigidTransformd* X_WB) const {
  if (parent_frame_id_ == SceneGraph<double>::world_frame_id()) {
    *X_WB = X_PB_;
  } else {
    const QueryObject<double>& query_object = get_query_object(context);
    *X_WB = query_object.GetPoseInWorld(parent_frame_id_) * X_PB_;
  }
}

void MaskImageCamera::ConvertDepth32FTo16U(const ImageDepth32F& d32,
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
const QueryObject<double>& MaskImageCamera::get_query_object(
    const Context<double>& context) const {
  return query_object_input_port().Eval<QueryObject<double>>(context);
}

}  // namespace mesh_painter
}  // namespace examples
}  // namespace drake
