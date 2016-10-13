#include "drake/systems/plants/rigid_body_plant/sampled_contact_manifold.h"

namespace drake {
namespace systems {

template class SampledContactManifold<double>;


template <typename T>
SampledContactManifold<T>::SampledContactManifold() {}

template <typename T>
SampledContactManifold<T>::SampledContactManifold(
    const SampledContactManifold<T>& other)
{
  for (const auto & detail : other.contact_details_) {
    std::unique_ptr<ContactDetail<T>> copy(detail->clone());
    contact_details_.push_back(std::move(copy));
  }
}

template <typename T>
ContactDetail<T> SampledContactManifold<T>::ComputeNetResponse() const {
  // The "net" contact is defined as follows:
  //  point = sum_i [p_i * |f_i|] / sum_i |f_i|
  //  Force = sum_i F_i + sum_i [(point - p_i) x f_i , 0, 0, 0]
  //
  //  where p_i is the ith application point.
  //  F_i is the ith spatial force (aka wrench).
  //  f_i, |f_i| are the force component (and its magnitude) of the ith spatial
  //  force, respectively.
  //  [ f, 0, 0, 0] is a zero-torque wrench built off the given force.
  //
  //  The net application point is an approximation of the center of pressure.
  WrenchVector<T> wrench;
  wrench.setZero();

  Vector3<T> point = Vector3<T>::Zero(3,1);
  T scale = 0;

  for ( const auto & detail : contact_details_ ) {
    const Vector3<T>& contactPoint = detail->get_application_point();
    const WrenchVector<T>& contactWrench = detail->get_force();

    wrench += contactWrench;

    T weight = contactWrench.head(3).norm();
    scale += weight;
    point += contactPoint * weight;
  }

  // TODO(SeanCurtis-TRI): Figure out where this epislon definition belongs.
  const double kEpsilon = 1e-10;
  if (scale > kEpsilon) point /= scale;

  auto accumTorque = wrench.tail(3);
  for (const auto & detail : contact_details_) {
    const Vector3<T>& contactPoint = detail->get_application_point();
    // cross product doesn't work on "head"
    const WrenchVector<T>& contactWrench = detail->get_force();
    accumTorque += (point - contactPoint).cross(contactWrench.template head<3>());
  }

  return ContactDetail<T>(point, wrench);
}

template <typename T>
const ContactDetail<T>* SampledContactManifold<T>::get_ith_contact(
    size_t i) const {
  if ( i < contact_details_.size()) {
    return contact_details_[i].get();
  }
  return nullptr;
}

template <typename T>
void SampledContactManifold<T>::AddContactDetail(
    std::unique_ptr<ContactDetail<T>>& detail) {
  contact_details_.push_back(move(detail));
}

template <typename T>
ContactManifold<T>* SampledContactManifold<T>::clone() const {
  return new SampledContactManifold(*this);
}

}
}
