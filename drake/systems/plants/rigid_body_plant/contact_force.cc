#include "drake/systems/plants/rigid_body_plant/contact_force.h"
#include <drake/common/drake_assert.h>

namespace drake {
namespace systems {

template <typename T>
ContactForce<T>::ContactForce(const Vector3<T>& application_point,
                              const Vector3<T>& force, const Vector3<T>& normal,
                              const Vector3<T>& pure_torque)
    : application_point_(application_point),
      force_(force),
      normal_(normal),
      pure_torque_(pure_torque) {
  DRAKE_ASSERT(abs(normal.norm() - 1.0) <
               Eigen::NumTraits<T>::dummy_precision());
}

template <typename T>
ContactForce<T>::ContactForce(const Vector3<T>& application_point,
                              const Vector3<T>& force, const Vector3<T>& normal)
    : application_point_(application_point), force_(force), normal_(normal) {
  DRAKE_ASSERT(abs(normal.norm() - 1.0) <
               Eigen::NumTraits<T>::dummy_precision());
}

template class ContactForce<double>;

}  // namespace systems
}  // namespace drake
