#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

/**
 The data for a single contact force applied to one body of a contacting pair.

 Ultimately, a contact force consists of an application point, a spatial
 force, and a unit vector. The spatial force includes:

    - a translational force, that is a pure force applied at a point,
    - a torque, the rotational force. This is not the same as the moment
      induced by the translational force.  This torque can arise from a
      combination of different sources (e.g., torsional friction, the sum of
      various moments, etc.)

 The unit normal indicates the normal direction of the translational force.
 Used to decompose the force into normal and tangential components. The
 normal is typically defined by the contact normal.

 A ContactForce makes no assumptions about the frame in which it is defined
 except that the vectors representing application point location, contact
 normal, force, and torque are all expressed in a common frame, with the
 application point measured from the origin of that frame. Every external
 instantiation of ContactForce must make it clear which frame is being used.

 @tparam T The scalar type. Must be a valid Eigen scalar.

 Instantiated templates for the following ScalarTypes are provided:

    - double
    - AutoDiffXd
 */
template <typename T>
class ContactForce {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactForce)

  /** Default constructor.  All values are initialized to NaN. */
  ContactForce();

  /**
   Fully-specified constructor.

   @param application_point         The point at which the wrench is applied.
   @param normal                    The translational force's unit-length normal
                                    direction.
   @param force                     The translational force.
   @param torque                    The torque component.
   */
  ContactForce(const Vector3<T>& application_point, const Vector3<T>& normal,
               const Vector3<T>& force, const Vector3<T>& torque);
  /**
   Zero-torque constructor.  This constructor sets the torque component to be
   zero.

   @param application_point         The point at which the wrench is applied.
   @param normal                    The translational force's unit-length normal
                                    direction.
   @param force                     The translational force.
   */
  ContactForce(const Vector3<T>& application_point, const Vector3<T>& normal,
               const Vector3<T>& force);

  /**
   Returns a spatial force applied at the same application point with negative
   force and torque.
   */
  ContactForce get_reaction_force() const {
    return ContactForce(application_point_, -normal_, -force_, -torque_);
  }

  const Vector3<T>& get_application_point() const { return application_point_; }

  const Vector3<T>& get_force() const { return force_; }

  /** Computes the normal component of the translational force. */
  Vector3<T> get_normal_force() const { return force_.dot(normal_) * normal_; }

  /** Computes the tangential component of the translational force. */
  Vector3<T> get_tangent_force() const { return force_ - get_normal_force(); }

  const Vector3<T>& get_torque() const { return torque_; }

  const Vector3<T>& get_normal() const { return normal_; }

  /**
   This is a utility function for returning a compact representation of the
   contact force's overall spatial force: a pair of vectors in R3 representing a
   rotational and translational force.

   The rotational force does *not* necessarily include an `r X f` moment term.
   It is simply the pure torque that was provided at initialization time.
   Ultimately, the responsibility for computing this moment term belongs to the
   portion of the code which knows the origin around which the moment is
   generated.

   @returns the resultant spatial force.
   */
  Vector6<T> get_spatial_force() const {
    Vector6<T> spatial_force;
    spatial_force.template head<3>() = torque_;
    spatial_force.template tail<3>() = force_;
    return spatial_force;
  }

 private:
  Vector3<T> application_point_{};
  Vector3<T> normal_{};
  Vector3<T> force_{};
  Vector3<T> torque_{};
};
}  // namespace systems
}  // namespace drake
