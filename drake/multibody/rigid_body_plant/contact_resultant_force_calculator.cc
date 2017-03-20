#include "drake/multibody/rigid_body_plant/contact_resultant_force_calculator.h"

#include <utility>

#include "drake/multibody/rigid_body_plant/point_contact_detail.h"

namespace drake {
namespace systems {

using std::unique_ptr;
using std::vector;
using std::move;

template <typename T>
ContactResultantForceCalculator<T>::ContactResultantForceCalculator()
    : detail_accumulator_(nullptr) {}

template <typename T>
ContactResultantForceCalculator<T>::ContactResultantForceCalculator(
    vector<unique_ptr<ContactDetail<T>>>* detail_accumulator)
    : detail_accumulator_(detail_accumulator) {}

template <typename T>
void ContactResultantForceCalculator<T>::AddForce(
    const ContactForce<T>& force) {
  forces_.push_back(force);
  AccumulateForce(force);
}

template <typename T>
void ContactResultantForceCalculator<T>::AddForce(
    std::unique_ptr<ContactDetail<T>> contact_detail) {
  forces_.push_back(contact_detail->ComputeContactForce());
  if (detail_accumulator_ != nullptr) {
    detail_accumulator_->emplace_back(move(contact_detail));
  }
  // No accumulator means the contact detail can be destroyed; it has served its
  // purpose.
}

template <typename T>
void ContactResultantForceCalculator<T>::AddForce(
    const Vector3<T>& application_point, const Vector3<T>& normal,
    const Vector3<T>& force) {
  forces_.emplace_back(application_point, normal, force);
  AccumulateForce(forces_.back());
}

template <typename T>
void ContactResultantForceCalculator<T>::AddForce(
    const Vector3<T>& application_point, const Vector3<T>& normal,
    const Vector3<T>& force, const Vector3<T>& pure_torque) {
  forces_.emplace_back(application_point, normal, force, pure_torque);
  AccumulateForce(forces_.back());
}

template <typename T>
ContactForce<T> ContactResultantForceCalculator<T>::ComputeResultant() const {
  // Treat a single force specially; it is its own resultant.
  if (forces_.size() == 1) {
    return forces_[0];
  }

  // For a set of forces applied at specific points (whose sum is non-zero)
  // there is a line of points (called the central axis) about which the forces
  // have a moment with minimum magnitude. A point, Q, on the line can be
  // found as follows:
  //
  //   Q = ( F x M ) / ||F||^2
  //
  // Where,
  //    Q: a point on the central axis (defined relative to origin O). In fact,
  //       it is the projection of O on that line.  Thus, a different O can lead
  //       to a different Q, but they all lie on the central axis and all have
  //       the "minimum moment" property.
  //    F: The sum of the set of forces.
  //    M: The sum of the moments induced by each force (around origin O).
  //    All points and vectors are defined in a common frame whose origin is O.
  // See Paul Mitiguy's book for more details.
  //
  // This method implements this equation with the caveat that only the
  // *normal* components of the set of contact forces are used in the
  // calculation.

  // Compute resultant spatial force -- as the sum of the torques, and the
  // sums of the normal and tangential components of the contact forces,
  // respectively.
  Vector3<T> result_torque = Vector3<T>::Zero();
  Vector3<T> tangent_component_sum = Vector3<T>::Zero();
  Vector3<T> normal_component_sum = Vector3<T>::Zero();
  for (const auto& force : forces_) {
    normal_component_sum += force.get_normal_force();
    result_torque += force.get_torque();
    tangent_component_sum += force.get_tangent_force();
  }

  Vector3<T> min_point;
  Vector3<T> normal;
  T denom = normal_component_sum.dot(normal_component_sum);
  if (denom > Eigen::NumTraits<T>::dummy_precision()) {
    normal = normal_component_sum.normalized();
    // Pick the first force application point as a temporary origin.  This
    // assumes contacts are all local and will keep the math near the origin,
    // even if the points in the world frame are off in some distant region.
    Vector3<T> temp_origin = forces_[0].get_application_point();

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
      Vector3<T> local_r = force.get_application_point() - temp_origin;
      normal_moment += local_r.cross(force.get_normal_force());

      // Update optimal origin candidate.
      T projection = local_r.dot(normal_component_sum);
      if (projection < best) {
        best = projection;
        candidate_index = i;
      }
    }

    // Use the mean-shift theorem to change the resultant moment from the
    // temporary origin to the "optimal" origin.  Only necessary if the
    // "optimal" origin is not the first point we arbitrarily selected.
    Vector3<T> origin = temp_origin;
    if (candidate_index != 0) {
      origin = forces_[candidate_index].get_application_point();
      normal_moment -= (origin - temp_origin).cross(normal_component_sum);
    }

    // Compute the minimum moment point.
    min_point = normal_component_sum.cross(normal_moment) / denom + origin;
  } else {
    // There is no translational force.  Pick an arbitrary unit normal.
    normal << 1, 0, 0;
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
    auto arm = force.get_application_point() - min_point;
    result_torque += arm.cross(force.get_force());
  }
  return ContactForce<T>(min_point, normal,
                         normal_component_sum + tangent_component_sum,
                         result_torque);
}

template <typename T>
ContactForce<T> ContactResultantForceCalculator<T>::ComputeResultant(
    const Vector3<T>& reference_point) const {
  Vector3<T> result_torque = Vector3<T>::Zero();
  Vector3<T> tangent_component_sum = Vector3<T>::Zero();
  Vector3<T> normal_component_sum = Vector3<T>::Zero();
  Vector3<T> arm;

  for (const auto& force : forces_) {
    normal_component_sum += force.get_normal_force();
    result_torque += force.get_torque();
    tangent_component_sum += force.get_tangent_force();

    arm = force.get_application_point() - reference_point;
    result_torque += arm.cross(force.get_force());
  }

  Vector3<T> normal;
  T denom = normal_component_sum.squaredNorm();
  if (denom > Eigen::NumTraits<T>::dummy_precision()) {
    normal = normal_component_sum.normalized();
  } else {
    normal << 0, 0, 1;
  }

  return ContactForce<T>(reference_point, normal,
                         normal_component_sum + tangent_component_sum,
                         result_torque);
}

template <typename T>
void ContactResultantForceCalculator<T>::AccumulateForce(
    const ContactForce<T>& force) {
  if (detail_accumulator_ != nullptr) {
    detail_accumulator_->emplace_back(new PointContactDetail<T>(force));
  }
}

template class ContactResultantForceCalculator<double>;

}  // namespace systems
}  // namespace drake
