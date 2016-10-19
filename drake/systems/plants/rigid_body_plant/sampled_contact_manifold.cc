#include "drake/systems/plants/rigid_body_plant/sampled_contact_manifold.h"

namespace drake {
namespace systems {

template <typename T>
SampledContactManifold<T>::SampledContactManifold() {}

template <typename T>
SampledContactManifold<T>::SampledContactManifold(
    const SampledContactManifold<T>& other) {
  for (const auto & detail : other.contact_details_) {
    std::unique_ptr<ContactDetail<T>> copy(detail->clone());
    contact_details_.push_back(std::move(copy));
  }
}

template <typename T>
ContactDetail<T> SampledContactManifold<T>::ComputeNetResponse() const {
  WrenchVector<T> wrench;
  wrench.setZero();

  Vector3<T> point = Vector3<T>::Zero();
  T scale = 0;

  for (const auto & detail : contact_details_) {
    const Vector3<T>& contact_point = detail->get_application_point();
    const WrenchVector<T>& contact_wrench = detail->get_wrench();

    wrench += contact_wrench;

    T weight = contact_wrench.template head<3>().norm();
    scale += weight;
    point += contact_point * weight;
  }

  // TODO(SeanCurtis-TRI): Figure out where this epislon definition belongs.
  const T kEpsilon = 1e-10;
  if (scale > kEpsilon) point /= scale;

  auto accum_torque = wrench.template tail<3>();
  for (const auto & detail : contact_details_) {
    const Vector3<T>& contact_point = detail->get_application_point();
    // cross product doesn't work on "head"
    const WrenchVector<T>& contact_wrench = detail->get_wrench();
    accum_torque += (point - contact_point).cross(
        contact_wrench.template head<3>());
  }

  return ContactDetail<T>(point, wrench);
}

template <typename T>
const ContactDetail<T>* SampledContactManifold<T>::get_ith_contact(
    size_t i) const {
  if (i < contact_details_.size()) {
    return contact_details_[i].get();
  }
  throw std::logic_error(
      "Attempted to acquire a contact detail with an invalid index: " +
      std::to_string(i) + " from a valid range of [0, " +
  std::to_string(contact_details_.size()) + "].");
}

template <typename T>
void SampledContactManifold<T>::AddContactDetail(
    std::unique_ptr<ContactDetail<T>> detail) {
  contact_details_.push_back(move(detail));
}

template <typename T>
ContactManifold<T>* SampledContactManifold<T>::clone() const {
  return new SampledContactManifold(*this);
}

// Explicitly instantiates on the most common scalar types.
template class SampledContactManifold<double>;

}  // namespace systems
}  // namespace drake
