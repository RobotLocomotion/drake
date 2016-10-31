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
    const Vector3<T>& application_point, const Vector3<T>& normal_force,
    const Vector3<T>& tangent_force) {
  forces_.emplace_back(application_point, normal_force, tangent_force);
  is_dirty_ = true;
}

template <typename T>
void ContactResultantForceCalculator<T>::AddForce(
    const Vector3<T>& application_point, const Vector3<T>& normal_force,
    const Vector3<T>& tangent_force, const Vector3<T>& pure_torque) {
  forces_.emplace_back(application_point, normal_force, tangent_force,
                       pure_torque);
  is_dirty_ = true;
}

template <typename T>
Vector3<T> ContactResultantForceCalculator<T>::ComputeMinimumMomentPoint()
    const {
  if (is_dirty_) {
    ComputeResultantValues();
  }
  return minimum_moment_point_;
}

template <typename T>
WrenchVector<T> ContactResultantForceCalculator<T>::ComputeResultantWrench()
    const {
  if (is_dirty_) {
    ComputeResultantValues();
  }
  return resultant_wrench_;
}

template <typename T>
void ContactResultantForceCalculator<T>::ComputeResultantValues() const {
  // Set the cache as clean.
  is_dirty_ = false;

  // Treat a single force specially.
  if (forces_.size() == 1) {
    minimum_moment_point_ = forces_[0].get_application_point();
    resultant_wrench_ = forces_[0].get_wrench();
    return;
  }

  // compute resultant wrench and resultant normal force -- the component of the
  // resultant force due to the normal components of the contact forces.
  resultant_wrench_ = WrenchVector<T>::Zero();
  Vector3<T> normal_resultant = Vector3<T>::Zero();
  for (const auto& force : forces_) {
    normal_resultant += force.get_normal_force();
    resultant_wrench_ += force.get_wrench();
  }

  T denom = normal_resultant.dot(normal_resultant);
  if (denom > 1e-14) {

    // pick the first force application point as a temporary origin.  This assumes
    // contacts are all local and will keep the math near the origin, even if the
    // points in the world frame are off in some distant region.
    Vector3<T> O_temp = forces_[0].get_application_point();

    // For the sake of efficiency, this loop is doing two activities:
    //  1) Compute the moment around the temporary origin induced by the normal
    //     forces.
    //  2) Compute a more "optimal" origin.  See ContactResultantForceCalculator
    //     for the characterization of this origin point.
    Vector3<T> normal_moment = Vector3<T>::Zero();
    T best(0.);
    size_t candidate_index = 0;
    for (size_t i = 0; i < forces_.size(); ++i) {
      const auto &force = forces_[i];
      // Compute normal moment.
      Vector3<T> local_r = force.get_application_point() - O_temp;
      normal_moment += local_r.cross(force.get_normal_force());

      // Update optimal origin candidate.
      T projection = local_r.dot(normal_resultant);
      if (projection < best) {
        best = projection;
        candidate_index = i;
      }
    }

    // Use the mean-shift theorem to change the resultant moment from the
    // temporary origin to the "optimal" origin.  Only necessary if the "optimal"
    // origin is not the first point we arbitrarily selected.
    Vector3<T> O = O_temp;
    if (candidate_index != 0) {
      O = forces_[candidate_index].get_application_point();
      normal_moment -= (O_temp - O).cross(normal_resultant);
    }

    // Compute the minimum moment point.
    minimum_moment_point_ = normal_resultant.cross(normal_moment) / denom + O;
  } else {
    // There is no normal force component which means the minimum moment point
    // can be *anywhere*.  We pick the first point just so it is "local" to the
    // contact data.
    minimum_moment_point_ = forces_[0].get_application_point();
  }

  // Account for moments introduced by moving forces from defined point
  // locations to minimum moment location.
  // This must be done for two reasons:
  //    1. The previous calculations did *not* account for the tangential
  //       components of the forces.
  //    2. If the minimum moment point is not the "center of pressure", there
  //       must be some residual moment that gets induced.
  for (const auto& force : forces_) {
    auto offset = force.get_application_point() - minimum_moment_point_;
    resultant_wrench_.template head<3>() += offset.cross(force.get_force());
  }
}

template class ContactResultantForceCalculator<double>;

}  // namespace systems
}  // namespace drake
