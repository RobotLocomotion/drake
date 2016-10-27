#include "drake/systems/plants/rigid_body_plant/contact_resultant_force_calculator.h"

namespace drake {
namespace systems {

template <typename T>
void ContactResultantForceCalculator<T>::AddForce(
    const ContactForce<T>& force) {
  forces_.push_back(force);
  is_dirty_ = true;
}

template <typename T>
void ContactResultantForceCalculator<T>::AddForce(
    const Vector3<T>& normal_force, const Vector3<T>& tangent_force,
    const Vector3<T>& application_point) {
  forces_.emplace_back(application_point, normal_force, tangent_force);
  is_dirty_ = true;
}

template <typename T>
void ContactResultantForceCalculator<T>::AddForce(
    const Vector3<T>& normal_force, const Vector3<T>& tangent_force,
    const Vector3<T>& application_point, const Vector3<T>& pure_torque) {
  forces_.emplace_back(application_point, normal_force, tangent_force,
                       pure_torque);
  is_dirty_ = true;
}

template <typename T>
Vector3<T> ContactResultantForceCalculator<T>::ComputeMinimumMomentPoint()
    const {
  if (is_dirty_) {
    // TODO(SeanCurtis-TRI): Implement this.
  }
  return minimum_moment_point_;
}

template <typename T>
WrenchVector<T> ContactResultantForceCalculator<T>::ComputeResultantWrench()
    const {
  if (is_dirty_) {
    // TODO(SeanCurtis-TRI): Implement this.
  }
  return resultant_wrench_;
}

template <typename T>
void ContactResultantForceCalculator<T>::ComputeCachedData() const {}

template class ContactResultantForceCalculator<double>;

}  // namespace systems
}  // namespace drake
