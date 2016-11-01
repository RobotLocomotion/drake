#include "drake/systems/plants/rigid_body_plant/contact_force.h"

namespace drake {
namespace systems {

template <typename T>
ContactForce<T>::ContactForce(const Vector3<T>& application_point,
                              const Vector3<T>& force,
                              const Vector3<T>& normal,
                              const Vector3<T>& pure_torque)
    : application_point_(application_point),
      force_(force),
      normal_(normal),
      pure_torque_(pure_torque) {}

template <typename T>
ContactForce<T>::ContactForce(const Vector3<T>& application_point,
                              const Vector3<T>& force,
                              const Vector3<T>& normal)
    : application_point_(application_point),
      force_(force),
      normal_(normal) {}

template class ContactForce<double>;

}  // namespace systems
}  // namespace drake
