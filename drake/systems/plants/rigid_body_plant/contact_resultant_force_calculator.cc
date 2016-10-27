#include "drake/systems/plants/rigid_body_plant/contact_resultant_force_calculator.h"

namespace drake {
namespace systems {

template <typename T>
void ContactResultantForceCalculator<T>::AddForce(
    const Vector3<T>& normal_force, const Vector3<T>& tangent_force,
    const Vector3<T>& application_point) {
  // TODO(SeanCurtis-TRI): Add this to the accumulation.
  dirty_state_ = kDirtyAll;
}

template <typename T>
void ContactResultantForceCalculator<T>::AddForce(
    const Vector3<T>& normal_force, const Vector3<T>& tangent_force,
    const Vector3<T>& application_point, const Vector3<T>& pure_torque) {
  // TODO(SeanCurtis-TRI): Add this to the accumulation.
  dirty_state_ = kDirtyAll;
}

template <typename T>
Vector3<T> ContactResultantForceCalculator<T>::ComputeMinimumMomentPoint()
    const {
  if (dirty_state_ & kDirtyPoint) {
    // TODO(SeanCurtis-TRI): Implement this.
  }
  return minimum_moment_point_;
}

template <typename T>
WrenchVector<T> ContactResultantForceCalculator<T>::ComputeResultantForce()
    const {
  if (dirty_state_ & kDirtyForce) {
    // TODO(SeanCurtis-TRI): Implement this.
  }
  return resultant_force_;
}

template class ContactResultantForceCalculator<double>;

}  // namespace systems
}  // namespace drake
