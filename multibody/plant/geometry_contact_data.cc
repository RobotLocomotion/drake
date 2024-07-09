#include "drake/multibody/plant/geometry_contact_data.h"

#include "drake/common/never_destroyed.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
void GeometryContactData<T>::Clear() {
  data_.reset();
}

template <typename T>
NestedGeometryContactData<T>& GeometryContactData<T>::Allocate() {
  data_ = std::make_shared<NestedGeometryContactData<T>>();
  return const_cast<NestedGeometryContactData<T>&>(*data_);
}

template <typename T>
std::shared_ptr<const NestedGeometryContactData<T>>
GeometryContactData<T>::Share() const {
  return (data_ != nullptr)
             ? data_
             : std::shared_ptr<const NestedGeometryContactData<T>>(
                   /* managed object = */ std::shared_ptr<const void>{},
                   /* stored pointer = */ &get_empty_ref());
}

template <typename T>
const NestedGeometryContactData<T>& GeometryContactData<T>::get_empty_ref() {
  static const never_destroyed<NestedGeometryContactData<T>> result;
  return result.access();
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct ::drake::multibody::internal::NestedGeometryContactData);

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::GeometryContactData);
