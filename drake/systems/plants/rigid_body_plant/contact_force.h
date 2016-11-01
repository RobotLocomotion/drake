#pragma once

#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

/**
 The data for a single contact force.

 Ultimately, a contact force consists of an application point and forces.
 The forces are defined with three pieces of data:
  - a translational force,
  - the normal direction of the translational force.  Assumed to be unit length
    and used to decompose the force into normal and tangential components. The
    normal is typically defined by the contact normal.
  - a "pure torque", the rotational force. This is not the same as the moment
    induced by the translational force.  This is a pure, abstract force (e.g.,
    modeling torsional friction.)

  The ContactForce class provides an interface to accessing the components as
  well as the combined components as a wrench.
 */
template <typename T>
class DRAKE_EXPORT ContactForce {
 public:
  /** Default constructor.  All values initialized to zero. */
  ContactForce() {}

  /**
   Fully-specified constructor.

   @param application_point         The point at which the wrench is applied.
   @param force                     The translational force.
   @param normal                    The translational force normal direction.
   @param pure_torque               The pure torque component
   */
  ContactForce(const Vector3<T>& application_point,
               const Vector3<T>& force, const Vector3<T>& normal,
               const Vector3<T>& pure_torque);
  /**
   Zero-pure-torque constructor.  This constructor sets the pure torque
   component to be zero.

   @param application_point         The point at which the wrench is applied.
   @param force                     The translational force.
   @param normal                    The translational force normal direction.
   @param pure_torque               The pure torque component
   */
  ContactForce(const Vector3<T>& application_point,
               const Vector3<T>& force, const Vector3<T>& normal);

  // Contact force is copyable and movable
  ContactForce(const ContactForce& other) = default;
  ContactForce& operator=(const ContactForce& other) = default;
  ContactForce(ContactForce&& other) = default;
  ContactForce& operator=(ContactForce&& other) = default;

  Vector3<T> get_application_point() { return application_point_; }
  const Vector3<T>& get_application_point() const { return application_point_; }

  // TODO(SeanCurtis-TRI): Update tehse
  Vector3<T> get_normal_force() { return force_.dot(normal_) * normal_; }
  const Vector3<T> get_normal_force() const { return force_.dot(normal_) * normal_; }

  Vector3<T> get_tangent_force() { return force_ - get_normal_force(); }
  const Vector3<T> get_tangent_force() const { return force_ - get_normal_force(); }

  Vector3<T> get_pure_torque() { return pure_torque_; }
  const Vector3<T> get_pure_torque() const { return pure_torque_; }

  Vector3<T> get_normal() { return normal_; }
  const Vector3<T> get_normal() const { return normal_; }

  Vector3<T> get_force() const { return force_; }

  WrenchVector<T> get_wrench() const {
    WrenchVector<T> wrench;
    wrench.template head<3>() = pure_torque_;
    wrench.template tail<3>() = force_;
    return wrench;
  }

 private:
  Vector3<T> application_point_{Vector3<T>::Zero()};
  Vector3<T> force_{Vector3<T>::Zero()};
  Vector3<T> normal_{Vector3<T>::Zero()};
  Vector3<T> pure_torque_{Vector3<T>::Zero()};
};
}  // namespace systems
}  // namespace drake
