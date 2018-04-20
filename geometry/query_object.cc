#include "drake/geometry/query_object.h"

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/geometry/scene_graph.h"

namespace drake {
namespace geometry {

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
const std::string& QueryObject<T>::GetSourceName(SourceId id) const {
  ThrowIfDefault();
  return context_->get_geometry_state().get_source_name(id);
}

template <typename T>
FrameId QueryObject<T>::GetFrameId(GeometryId geometry_id) const {
  ThrowIfDefault();
  return context_->get_geometry_state().GetFrameId(geometry_id);
}

template <typename T>
std::vector<PenetrationAsPointPair<double>>
QueryObject<T>::ComputePointPairPenetration() const {
  ThrowIfDefault();

  // TODO(SeanCurtis-TRI): Modify this when the cache system is in place.
  scene_graph_->FullPoseUpdate(*context_);
  const GeometryState<T>& state = context_->get_geometry_state();
  return state.ComputePointPairPenetration();
}

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::QueryObject)
