#include "drake/geometry/perception_query_object.h"

#include "drake/common/default_scalars.h"
#include "drake/geometry/geometry_state.h"

namespace drake {
namespace geometry {

using math::RigidTransformd;
using render::CameraProperties;
using render::DepthCameraProperties;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

template <typename T>
void PerceptionQueryObject<T>::RenderColorImage(const CameraProperties& camera,
                                      FrameId parent_frame,
                                      const RigidTransformd& X_PC,
                                      bool show_window,
                                      ImageRgba8U* color_image_out) const {
  this->ValidateAndUpdate();
  const GeometryState<T>& state = this->geometry_state();
  return state.RenderColorImage(camera, parent_frame, X_PC, show_window,
                                color_image_out);
}

template <typename T>
void PerceptionQueryObject<T>::RenderDepthImage(
    const DepthCameraProperties& camera, FrameId parent_frame,
    const RigidTransformd& X_PC, ImageDepth32F* depth_image_out) const {
  this->ValidateAndUpdate();
  const GeometryState<T>& state = this->geometry_state();
  return state.RenderDepthImage(camera, parent_frame, X_PC, depth_image_out);
}

template <typename T>
void PerceptionQueryObject<T>::RenderLabelImage(const CameraProperties& camera,
                                      FrameId parent_frame,
                                      const RigidTransformd& X_PC,
                                      bool show_window,
                                      ImageLabel16I* label_image_out) const {
  this->ValidateAndUpdate();
  const GeometryState<T>& state = this->geometry_state();
  return state.RenderLabelImage(camera, parent_frame, X_PC, show_window,
                                label_image_out);
}

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::PerceptionQueryObject)
