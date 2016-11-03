#include "drake/systems/plants/rigid_body_plant/contact_resultant_force_calculator.h"

namespace drake {
namespace systems {

template <typename T>
void ContactResultantForceCalculator<T>::AddForce(
    const ContactForce<T>& force) {
  forces_.push_back(force);
}

template <typename T>
void ContactResultantForceCalculator<T>::AddForce(
    const Vector3<T>& application_point, const Vector3<T>& force,
    const Vector3<T>& normal) {
  forces_.emplace_back(application_point, force, normal);
}

template <typename T>
void ContactResultantForceCalculator<T>::AddForce(
    const Vector3<T>& application_point, const Vector3<T>& force,
    const Vector3<T>& normal, const Vector3<T>& pure_torque) {
  forces_.emplace_back(application_point, force, normal, pure_torque);
}

template <typename T>
ContactForce<T> ContactResultantForceCalculator<T>::ComputeResultant() const {
  // Treat a single force specially.
  if (forces_.size() == 1) {
    return forces_[0];
  }

  // compute resultant wrench and resultant normal force -- the component of the
  // resultant force due to the normal components of the contact forces.
  Vector3<T> result_torque = Vector3<T>::Zero();
  Vector3<T> result_tan = Vector3<T>::Zero();
  Vector3<T> result_norm = Vector3<T>::Zero();
  for (const auto& force : forces_) {
    result_norm += force.get_normal_force();
    result_torque += force.get_pure_torque();
    result_tan += force.get_tangent_force();
  }

  Vector3<T> min_point;
  Vector3<T> norm;
  T denom = result_norm.dot(result_norm);
  if (denom > 1e-14) {
    norm = result_norm.normalized();
    // pick the first force application point as a temporary origin.  This
    // assumes contacts are all local and will keep the math near the origin,
    // even if the points in the world frame are off in some distant region.
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
      const auto& force = forces_[i];
      // Compute normal moment.
      Vector3<T> local_r = force.get_application_point() - O_temp;
      normal_moment += local_r.cross(force.get_normal_force());

      // Update optimal origin candidate.
      T projection = local_r.dot(result_norm);
      if (projection < best) {
        best = projection;
        candidate_index = i;
      }
    }

    // Use the mean-shift theorem to change the resultant moment from the
    // temporary origin to the "optimal" origin.  Only necessary if the
    // "optimal" origin is not the first point we arbitrarily selected.
    Vector3<T> O = O_temp;
    if (candidate_index != 0) {
      O = forces_[candidate_index].get_application_point();
      normal_moment -= (O - O_temp).cross(result_norm);
    }

    // Compute the minimum moment point.
    min_point = result_norm.cross(normal_moment) / denom + O;
  } else {
    // There is no translational force.  Pick an arbitrary unit normal.
    norm << 1, 0, 0;
    // No normal force component implies the minimum moment point can be
    // *anywhere*.  We pick the centroid.
    min_point = Vector3<T>::Zero();
    for (const auto& force : forces_) {
      min_point += force.get_application_point();
    }
    min_point /= forces_.size();
  }

  // Account for moments introduced by moving forces from defined point
  // locations to minimum moment location.
  // This must be done for two reasons:
  //    1. The previous calculations did *not* account for the tangential
  //       components of the forces.
  //    2. If the minimum moment point is not the "center of pressure", there
  //       must be some residual moment that gets induced.
  for (const auto& force : forces_) {
    auto offset = force.get_application_point() - min_point;
    result_torque += offset.cross(force.get_force());
  }
  return ContactForce<T>(min_point, result_norm + result_tan, norm,
                         result_torque);
}

template class ContactResultantForceCalculator<double>;

}  // namespace systems
}  // namespace drake
