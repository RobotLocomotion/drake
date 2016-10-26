#include "drake/systems/plants/rigid_body_plant/contact_force.h"

namespace drake {
namespace systems {

template <typename T>
ContactForce<T>::ContactForce(const Vector3<T>& application_point,
                              const Vector3<T>& normal_force,
                              const Vector3<T>& tangent_force,
                              const Vector3<T>& pure_torque)
    : application_point_(application_point),
      normal_force_(normal_force),
      tangent_force_(tangent_force),
      pure_torque_(pure_torque) {}

template <typename T>
ContactForce<T>::ContactForce(const Vector3<T>& application_point,
                              const Vector3<T>& normal_force,
                              const Vector3<T>& tangent_force)
    : application_point_(application_point),
      normal_force_(normal_force),
      tangent_force_(tangent_force) {}

template class ContactForce<double>;

}  // namespace systems
}  // namespace drake