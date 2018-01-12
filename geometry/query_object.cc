#include "drake/geometry/query_object.h"

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/geometry/geometry_system.h"

namespace drake {
namespace geometry {

template <typename T>
QueryObject<T>::QueryObject(const QueryObject&)
    : context_{nullptr},
      system_{nullptr} {}

template <typename T>
QueryObject<T>& QueryObject<T>::operator=(const QueryObject<T>&) {
  context_ = nullptr;
  system_ = nullptr;
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

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::QueryObject)
