#include "drake/multibody/rigid_body_plant/contact_force.h"

#include <cmath>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

using std::abs;

template <typename T>
    ContactForce<T>::ContactForce() {
  T kNan((T)NAN);
  application_point_ << kNan, kNan, kNan;
  force_ << kNan, kNan, kNan;
  normal_ << kNan, kNan, kNan;
  pure_torque_ << kNan, kNan, kNan;
}

template <typename T>
ContactForce<T>::ContactForce(const Vector3<T>& application_point,
                              const Vector3<T>& normal, const Vector3<T>& force,
                              const Vector3<T>& torque)
    : application_point_(application_point),
      normal_(normal),
      force_(force),
      torque_(torque) {
  DRAKE_ASSERT(abs(normal.norm() - 1.0) <
               Eigen::NumTraits<T>::dummy_precision());
}

template <typename T>
ContactForce<T>::ContactForce(const Vector3<T>& application_point,
                              const Vector3<T>& normal, const Vector3<T>& force)
    : application_point_(application_point),
      normal_(normal),
      force_(force),
      torque_(Vector3<T>::Zero()) {
  DRAKE_ASSERT(abs(normal.norm() - 1.0) <
               Eigen::NumTraits<T>::dummy_precision());
}

template class ContactForce<double>;

}  // namespace systems
}  // namespace drake
