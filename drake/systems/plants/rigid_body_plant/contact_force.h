#pragma once

#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

/**
 The data for a single contact force applied to one body of a contacting pair.

 Ultimately, a contact force consists of an application point, a spatial
 force, and a unit vector. The spatial force includes:
  - a translational force, that is a pure force applied at a point,
  - a "pure torque", the rotational force. This is not the same as the moment
    induced by the translational force.  This is a pure, abstract torque (e.g.,
    modeling torsional friction.)

  The unit normal indicates the normal direction of the translational force.
  Used to decompose the force into normal and tangential components. The
  normal is typically defined by the contact normal.

  The application point, normal vector, and force vectors should all be measured
  and expressed in a common frame.

  ContactForce is frame agnostic. However, the vectors representing application
  point location, contact normal, force, and torque must all be expressed in a
  common frame, with the application point measured from the origin of that
  frame. Every use of ContactForce must make clear which frame is being used.

  @tparam T The scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
class DRAKE_EXPORT ContactForce {
 public:
  /**
   Fully-specified constructor.

   @param application_point         The point at which the wrench is applied.
   @param force                     The translational force.
   @param normal                    The translational force's unit-length normal
                                    direction.
   @param pure_torque               The pure torque component
   */
  ContactForce(const Vector3<T>& application_point, const Vector3<T>& force,
               const Vector3<T>& normal, const Vector3<T>& pure_torque);
  /**
   Zero-pure-torque constructor.  This constructor sets the pure torque
   component to be zero.

   @param application_point         The point at which the wrench is applied.
   @param force                     The translational force.
   @param normal                    The translational force's normal direction.
   */
  ContactForce(const Vector3<T>& application_point, const Vector3<T>& force,
               const Vector3<T>& normal);

  // Contact force is copyable and movable
  ContactForce(const ContactForce& other) = default;
  ContactForce& operator=(const ContactForce& other) = default;
  ContactForce(ContactForce&& other) = default;
  ContactForce& operator=(ContactForce&& other) = default;

  Vector3<T> get_application_point() { return application_point_; }
  const Vector3<T>& get_application_point() const { return application_point_; }

  Vector3<T> get_normal_force() { return force_.dot(normal_) * normal_; }
  const Vector3<T> get_normal_force() const {
    return force_.dot(normal_) * normal_;
  }

  Vector3<T> get_tangent_force() { return force_ - get_normal_force(); }
  const Vector3<T> get_tangent_force() const {
    return force_ - get_normal_force();
  }

  Vector3<T> get_pure_torque() { return pure_torque_; }
  const Vector3<T> get_pure_torque() const { return pure_torque_; }

  Vector3<T> get_normal() { return normal_; }
  const Vector3<T> get_normal() const { return normal_; }

  Vector3<T> get_force() const { return force_; }

  /**
   This is a utility function for returning a compact representation of the
   spatial force: a vector in R6 representing a concatentation of a rotational
   and translational force.

   The rotational force does *not* include an `r X f` moment term.  It is simply
   the pure torque that was provided at initialization time. Ultimately, the
   responsibility for computing this moment term belongs to the portion of the
   code which knows the origin around which the moment is generated.

   @returns the resultant spatial force.
   */
  SpatialForce <T> get_spatial_force() const {
    SpatialForce<T> spatial_force;
    spatial_force.template head<3>() = pure_torque_;
    spatial_force.template tail<3>() = force_;
    return spatial_force;
  }

 private:
  Vector3<T> application_point_{Vector3<T>::Zero()};
  Vector3<T> force_{Vector3<T>::Zero()};
  Vector3<T> normal_{Vector3<T>::Zero()};
  Vector3<T> pure_torque_{Vector3<T>::Zero()};
};
}  // namespace systems
}  // namespace drake
