#pragma once

#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

/**
 The data for a single contact force.

 Ultimately, a contact force consists of an application point and a wrench.
 The wrench is composed of three components:
  - a linear normal force: the component of the linear force that lies in the
  direction of the contact normal.
  - a linear tangential force: the component of the linear force that lies in
  on the contact plane.  It arises from frictional components in the contact
  model.
  - a "pure torque": some contact models can introduce a pure torque (e.g.,
  modeling torsional friction.)

  The ContactForce class provides an interface to accessing the components as
  well as the combined components as a wrench.
 */
template <typename T>
class DRAKE_EXPORT ContactForce {
  /**
   Fully-specified constructor.

   @param application_point         The point at which the wrench is applied.
   @param normal_force              The normal component of the linear force.
   @param tangent_force             The tangential component of the linear force.
   @param pure_torque               The pure torque component
   */
  ContactForce(const Vector3<T>& application_point,
               const Vector3<T>& normal_force, const Vector3<T>& tangent_force,
               const Vector3<T>& pure_torque);
  /**
   Zero-pure-torque constructor.  This constructor sets the pure torque
   component to be zero.

   @param application_point         The point at which the wrench is applied.
   @param normal_force              The normal component of the linear force.
   @param tangent_force             The tangential component of the linear
                                    force.
   @param pure_torque               The pure torque component
   */
  ContactForce(const Vector3<T>& application_point,
               const Vector3<T>& normal_force, const Vector3<T>& tangent_force);

  Vector3<T> get_application_point() const { return application_point_; }
  const Vector3<T>& get_application_point() const { return application_point_; }

  Vector3<T> get_normal_force() const { return normal_force_; }
  const Vector3<T> get_normal_force() const { return normal_force_; }

  Vector3<T> get_tangent_force() const { return tangent_force_; }
  const Vector3<T> get_tangent_force() const { return tangent_force_; }

  Vector3<T> get_pure_torque() const { return pure_torque_; }
  const Vector3<T> get_pure_torque() const { return pure_torque_; }

  WrenchVector<T> get_wrench() const {
    WrenchVector<T> wrench;
    wrench.template head<3>() = pure_torque_;
    wrench.template tail<3>() = normal_force_ + tangent_force_;
    return wrench;
  }

 private:
  Vector3<T> application_point_;
  Vector3<T> normal_force_;
  Vector3<T> tangent_force_;
  Vector3<T> pure_torque_;
};
}  // namespace systems
}  // namespace drake
