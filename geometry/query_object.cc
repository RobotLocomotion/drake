#include "drake/geometry/query_object.h"

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/geometry/scene_graph.h"

namespace drake {
namespace geometry {

using math::RigidTransform;
using math::RigidTransformd;
using render::ColorRenderCamera;
using render::DepthRenderCamera;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

template <typename T>
QueryObject<T>::QueryObject(const QueryObject& query_object) {
  *this = query_object;
}

template <typename T>
QueryObject<T>& QueryObject<T>::operator=(const QueryObject<T>& query_object) {
  if (this == &query_object) return *this;

  DRAKE_DEMAND(query_object.is_copyable());

  context_ = nullptr;
  scene_graph_ = nullptr;
  state_.reset();

  if (query_object.state_) {
    // Share the underlying baked state.
    state_ = query_object.state_;
  } else if (query_object.context_ && query_object.scene_graph_) {
    // Create a new baked state; make sure the source is fully updated.
    query_object.FullPoseUpdate();
    state_ = std::make_shared<GeometryState<T>>(query_object.geometry_state());
  }
  inspector_.set(state_.get());
  // If `query_object` is default, then this will likewise be default.

  return *this;
}

template <typename T>
const RigidTransform<T>& QueryObject<T>::GetPoseInWorld(
    FrameId frame_id) const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.get_pose_in_world(frame_id);
}

template <typename T>
const RigidTransform<T>& QueryObject<T>::GetPoseInParent(
    FrameId frame_id) const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.get_pose_in_parent(frame_id);
}

template <typename T>
const RigidTransform<T>& QueryObject<T>::GetPoseInWorld(
    GeometryId geometry_id) const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.get_pose_in_world(geometry_id);
}

template <typename T>
std::vector<PenetrationAsPointPair<T>>
QueryObject<T>::ComputePointPairPenetration() const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.ComputePointPairPenetration();
}

template <typename T>
std::vector<SortedPair<GeometryId>> QueryObject<T>::FindCollisionCandidates()
    const {
  ThrowIfNotCallable();
  // TODO(amcastro-tri): Modify this when the cache system is in place.
  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.FindCollisionCandidates();
}

template <typename T>
bool QueryObject<T>::HasCollisions() const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.HasCollisions();
}

template <typename T>
std::vector<ContactSurface<T>> QueryObject<T>::ComputeContactSurfaces(
    HydroelasticContactRepresentation representation) const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.ComputeContactSurfaces(representation);
}

template <typename T>
void QueryObject<T>::ComputeContactSurfacesWithFallback(
    HydroelasticContactRepresentation representation,
    std::vector<ContactSurface<T>>* surfaces,
    std::vector<PenetrationAsPointPair<T>>* point_pairs) const {
  DRAKE_DEMAND(surfaces != nullptr);
  DRAKE_DEMAND(point_pairs != nullptr);

  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  state.ComputeContactSurfacesWithFallback(representation, surfaces,
                                           point_pairs);
}

template <typename T>
std::vector<SignedDistancePair<T>>
QueryObject<T>::ComputeSignedDistancePairwiseClosestPoints(
    const double max_distance) const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.ComputeSignedDistancePairwiseClosestPoints(max_distance);
}

template <typename T>
SignedDistancePair<T> QueryObject<T>::ComputeSignedDistancePairClosestPoints(
    GeometryId geometry_id_A, GeometryId geometry_id_B) const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.ComputeSignedDistancePairClosestPoints(geometry_id_A,
                                                      geometry_id_B);
}

template <typename T>
std::vector<SignedDistanceToPoint<T>>
QueryObject<T>::ComputeSignedDistanceToPoint(
    const Vector3<T>& p_WQ,
    const double threshold) const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.ComputeSignedDistanceToPoint(p_WQ, threshold);
}

template <typename T>
void QueryObject<T>::RenderColorImage(const ColorRenderCamera& camera,
                                      FrameId parent_frame,
                                      const RigidTransformd& X_PC,
                                      ImageRgba8U* color_image_out) const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.RenderColorImage(camera, parent_frame, X_PC, color_image_out);
}

template <typename T>
void QueryObject<T>::RenderDepthImage(const DepthRenderCamera& camera,
                                      FrameId parent_frame,
                                      const RigidTransformd& X_PC,
                                      ImageDepth32F* depth_image_out) const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.RenderDepthImage(camera, parent_frame, X_PC, depth_image_out);
}

template <typename T>
void QueryObject<T>::RenderLabelImage(const ColorRenderCamera& camera,
                                      FrameId parent_frame,
                                      const RigidTransformd& X_PC,
                                      ImageLabel16I* label_image_out) const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.RenderLabelImage(camera, parent_frame, X_PC, label_image_out);
}

template <typename T>
const render::RenderEngine* QueryObject<T>::GetRenderEngineByName(
    const std::string& name) const {
  ThrowIfNotCallable();
  FullPoseUpdate();

  const GeometryState<T>& state = geometry_state();
  return state.GetRenderEngineByName(name);
}

template <typename T>
const GeometryState<T>& QueryObject<T>::geometry_state() const {
  // Some extra insurance in case some query *hadn't* called this.
  DRAKE_ASSERT_VOID(ThrowIfNotCallable());
  if (context_) {
    return scene_graph_->geometry_state(*context_);
  } else {
    return *state_;
  }
}

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::QueryObject)
