#include "drake/multibody/rigid_body_plant/point_contact_detail.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

using std::unique_ptr;
using std::make_unique;

template <typename T>
PointContactDetail<T>::PointContactDetail(const ContactForce<T>& force)
    : force_(force) {}

template <typename T>
unique_ptr<ContactDetail<T>> PointContactDetail<T>::Clone() const {
  return unique_ptr<ContactDetail<T>>(new PointContactDetail(force_));
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::PointContactDetail)
