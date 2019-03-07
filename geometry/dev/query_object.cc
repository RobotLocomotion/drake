#include "drake/geometry/dev/query_object.h"

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/geometry/dev/scene_graph.h"

namespace drake {
namespace geometry {
namespace dev {

using render::CameraProperties;
using render::DepthCameraProperties;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

template <typename T>
QueryObject<T>::QueryObject(const QueryObject&)
    : context_{nullptr}, scene_graph_{nullptr} {}

template <typename T>
QueryObject<T>& QueryObject<T>::operator=(const QueryObject<T>&) {
  context_ = nullptr;
  scene_graph_ = nullptr;
  return *this;
}

template <typename T>
const Isometry3<T>& QueryObject<T>::GetPoseInWorld(FrameId frame_id) const {
  ThrowIfDefault();
  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  scene_graph_->FullPoseUpdate(*context_);
  const GeometryState<T>& state = context_->get_geometry_state();
  return state.get_pose_in_world(frame_id);
}

template <typename T>
const Isometry3<T>& QueryObject<T>::GetPoseInWorld(
    GeometryId geometry_id) const {
  ThrowIfDefault();
  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  scene_graph_->FullPoseUpdate(*context_);
  const GeometryState<T>& state = context_->get_geometry_state();
  return state.get_pose_in_world(geometry_id);
}

template <typename T>
std::vector<PenetrationAsPointPair<double>>
QueryObject<T>::ComputePointPairPenetration() const {
  throw std::logic_error(
      "The development SceneGraph only supports render queries");
}

template <typename T>
std::vector<SignedDistancePair<double>>
QueryObject<T>::ComputeSignedDistancePairwiseClosestPoints() const {
  throw std::logic_error(
      "The development SceneGraph only supports render queries");
}

template <typename T>
void QueryObject<T>::RenderColorImage(const CameraProperties& camera,
                                      const Isometry3<double>& X_WC,
                                      ImageRgba8U* color_image_out,
                                      bool show_window) const {
  ThrowIfDefault();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  scene_graph_->FullPoseUpdate(*context_);
  const GeometryState<T>& state = context_->get_geometry_state();
  return state.RenderColorImage(camera, X_WC, color_image_out, show_window);
}

template <typename T>
void QueryObject<T>::RenderColorImage(const CameraProperties& camera,
                                      FrameId parent_frame,
                                      const Isometry3<double>& X_PC,
                                      ImageRgba8U* color_image_out,
                                      bool show_window) const {
  ThrowIfDefault();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  scene_graph_->FullPoseUpdate(*context_);
  const GeometryState<T>& state = context_->get_geometry_state();
  return state.RenderColorImage(camera, parent_frame, X_PC, color_image_out,
                                show_window);
}

template <typename T>
void QueryObject<T>::RenderDepthImage(
    const DepthCameraProperties& camera, const Isometry3<double>& X_WC,
    ImageDepth32F* depth_image_out) const {
  ThrowIfDefault();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  scene_graph_->FullPoseUpdate(*context_);
  const GeometryState<T>& state = context_->get_geometry_state();
  return state.RenderDepthImage(camera, X_WC, depth_image_out);
}

template <typename T>
void QueryObject<T>::RenderDepthImage(const DepthCameraProperties& camera,
                                      FrameId parent_frame,
                                      const Isometry3<double>& X_PC,
                                      ImageDepth32F* depth_image_out) const {
  ThrowIfDefault();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  scene_graph_->FullPoseUpdate(*context_);
  const GeometryState<T>& state = context_->get_geometry_state();
  return state.RenderDepthImage(camera, parent_frame, X_PC, depth_image_out);
}

template <typename T>
void QueryObject<T>::RenderLabelImage(const CameraProperties& camera,
                                      const Isometry3<double>& X_WC,
                                      ImageLabel16I* label_image_out,
                                      bool show_window) const {
  ThrowIfDefault();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  scene_graph_->FullPoseUpdate(*context_);
  const GeometryState<T>& state = context_->get_geometry_state();
  return state.RenderLabelImage(camera, X_WC, label_image_out, show_window);
}

template <typename T>
void QueryObject<T>::RenderLabelImage(const CameraProperties& camera,
                                      FrameId parent_frame,
                                      const Isometry3<double>& X_PC,
                                      ImageLabel16I* label_image_out,
                                      bool show_window) const {
  ThrowIfDefault();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  scene_graph_->FullPoseUpdate(*context_);
  const GeometryState<T>& state = context_->get_geometry_state();
  return state.RenderLabelImage(camera, parent_frame, X_PC, label_image_out,
                                show_window);
}

template <typename T>
const GeometryState<T>& QueryObject<T>::geometry_state() const {
  // TODO(SeanCurtis-TRI): Handle the "baked" query object case.
  DRAKE_DEMAND(scene_graph_ != nullptr);
  DRAKE_DEMAND(context_ != nullptr);
  return context_->get_geometry_state();
}

}  // namespace dev
}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::dev::QueryObject)
