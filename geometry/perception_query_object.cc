#include "drake/geometry/query_object.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace geometry {

using math::RigidTransform;
using math::RigidTransformd;
using render::CameraProperties;
using render::DepthCameraProperties;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

template <typename T>
void QueryObject<T>::RenderColorImage(const CameraProperties& camera,
                                      FrameId parent_frame,
                                      const RigidTransformd& X_PC,
                                      bool show_window,
                                      ImageRgba8U* color_image_out) const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.RenderColorImage(camera, parent_frame, X_PC, show_window,
                                color_image_out);
}

template <typename T>
void QueryObject<T>::RenderDepthImage(const DepthCameraProperties& camera,
                                      FrameId parent_frame,
                                      const RigidTransformd& X_PC,
                                      ImageDepth32F* depth_image_out) const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.RenderDepthImage(camera, parent_frame, X_PC, depth_image_out);
}

template <typename T>
void QueryObject<T>::RenderLabelImage(const CameraProperties& camera,
                                      FrameId parent_frame,
                                      const RigidTransformd& X_PC,
                                      bool show_window,
                                      ImageLabel16I* label_image_out) const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.RenderLabelImage(camera, parent_frame, X_PC, show_window,
                                label_image_out);
}

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::QueryObject)
