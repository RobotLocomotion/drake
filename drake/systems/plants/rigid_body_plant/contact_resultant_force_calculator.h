#pragma once

#include <vector>

#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/plants/rigid_body_plant/contact_force.h"

namespace drake {
namespace systems {

/**
 This is a utility class for taking a set of contact forces and defining
 the resultant force and its application point.

 The application point for the resultant force will be a minimum moment
 magnitude point.  That is, if the resultant force is applied at this point,
 it will induce the minimum moment. This is a general solution that is well
 defined and can be applied to arbitrary sets of contact forces. It must be
 emphsized, that this calculation considers only the *normal* components of the
 contact forces in computing the minimum moment point.  The other, tangential
 components will be included in the final resultant, but will not affect the
 calculations.

 Center of Pressure
 ==================
 For a set of contact forces with particular properties, this minimum moment
 point can be interpreted as the center of pressure. For this to be the case,
 the following conditions must be met:
    - The normal components of all forces must lie in the same direction.
    - The force application points must all lie on a plane.
    - The plane the application points lie on must be perpendicular to the
    common normal direction of the contact forces.
 If these conditions are met, the application point will also lie on the plane
 and will be the center of pressure.  And the minimum moment due to the normal
 forces will be zero.

 Usage
 =====
 The class is designed to be instantiated by the contact model.  As each pair
 of collision elements are evaluated, the contact model should instantiate a
 ContactResultantForceCalculator.  As each contact point between the elements
 is processed and a contact force is computed, the details of the contact force
 are provided to the calculator (via calls to AddForce).

 Currently, the contact force is defined by four values:
    - the application point,
    - the component of the contact force in the *normal* direction,
    - the component of the contact force in the *tangential* direction (e.g.,
        friction force), and
    - an optional pure torque term (e.g., torsional friction).
 The application points and force directions are assumed to be measured and
 expressed in a common frame (the resultant force and application point will
 likewise be measured and expressed in that same frame.)

 After all of the forces have been added to the calculator, a minimum moment
 point and resultant wrench can be requested using the appropriate method.

 Generally, the order in which the forces are added should have no bearing on
 the final result. The exception to this is if the normal components are coupled
 and cancel each other out. See below for details.

 A Single Force
 ==============
 If the set consists of a single force, the minimum moment point and resultant
 force will be the details of that force: i.e., its application point and
 response force and pure torque.

 A Non-zero Minimum Moment
 =========================
 For an arbitrary set of forces, there may not be a well-defined center of
 pressure as with the planar case outline above.  Generally, there is an
 infinite set of minimum moment points for a set of contact forces; it is a line
 called the "central axis".  Any point on this line will lead to the same
 minimum moment.  The ContactResultantForceCalculator needs to select one of
 those points.

 We assume that the "ideal" point would be where the line intersects the contact
 surface. Generally, this can't be solved because it depends on a geometric
 query that is outside the scope of this calculator class.  Furthermore, in many
 cases, it is unnecessary. A point on the surface is good for visualization, but
 in contexts where a mathematically meaningful point is needed, then one point
 is as good as another. That said, the calculator employs a method to cheaply
 approximate the intersection of the line with the contact surface by doing the
 following.

 The central axis can be thought of as a line defined by a point and direction.
 The point can be any point on the line.  The direction is defined by the
 direction of the resultant normal force (i.e., the sum of the normal components
 of all forces.) The direction vector defines "positive" and "negative"
 directions on the line. The force originated from the negative direction and
 accelerates the body in the positive direction.  If we had access to the
 geometry, the point we would be interested in, would be the intersection of the
 line and geometry that is farthest in the "negative" direction (i.e., closest
 to the originating source of the contact).

 We will approximate this by finding the contact force application point that
 similarly lies farthest in the negative direction (simply by projecting the
 application points on the line.)  This most-negative projection point will
 serve as the reported minimum moment point.

 A Zero-Resultant Force
 ======================
 It is possible for all of the contact forces to sum up to a zero resultant. But
 there may still be a resultant moment, i.e., the forces are "coupled".  In this
 case, the minimum moment point can be literally any point in space.  In this
 case, the ContactResultantForceCalculator selects the application point of the
 first added contact force as the minimum moment point.
 */
template <typename T>
class DRAKE_EXPORT ContactResultantForceCalculator {
 public:
  /** Default constructor. */
  ContactResultantForceCalculator() {}

  /**
   Adds a new contact force to the calcualtor.

   @param force     The contact force.
   */
  void AddForce(const ContactForce<T>& force);

  /**
   Adds a new force to the calculator.
   @param application_point     The application point of the force.
   @param normal_force          The normal component of the force.
   @param tangent_force         The tangent component of the force.
   */
  void AddForce(const Vector3<T>& application_point,
                const Vector3<T>& normal_force,
                const Vector3<T>& tangent_force);

  /**
   Adds a new force with an arbitrary pure torque to the calculator.
   @param application_point     The application point of the force.
   @param normal_force          The normal component of the force.
   @param tangent_force         The tangent component of the force.
   @param pure_torque           The pure torque for the wrench.
   */
  void AddForce(const Vector3<T>& application_point,
                const Vector3<T>& normal_force, const Vector3<T>& tangent_force,
                const Vector3<T>& pure_torque);

  /**
   Compute the minimum moment point -- the point at which the resultant force
   induces the minimum moment.
   */
  Vector3<T> ComputeMinimumMomentPoint() const;

  /**
   Compute the resultant wrench to be applied at the minimum moment point.
   */
  WrenchVector<T> ComputeResultantWrench() const;

  // Neither movable or copyable.
  ContactResultantForceCalculator(
      const ContactResultantForceCalculator& other) = delete;
  ContactResultantForceCalculator& operator=(
      const ContactResultantForceCalculator& other) = delete;
  ContactResultantForceCalculator(ContactResultantForceCalculator&& other) =
      delete;
  ContactResultantForceCalculator& operator=(
      ContactResultantForceCalculator&& other) = delete;

 private:
  // Recomputes the cached resultant values: minimimum moment point and force.
  void ComputeResultantValues() const;

  // Aggregator for the force data that has been added.
  // TODO(SeanCurtis-TRI): Get the class right.
  std::vector<ContactForce<T>> forces_{};

  // To facilitate computation, this class uses a light-weight caching system
  // to prevent redundant computations.  It works with a dirty/clean bit
  // to indicate if the values need to be computed.  Because this caching
  // system is supposed to be invisible to the user, they are marked mutable
  // so the methods which perform the computation can be declared const -- i.e.,
  // the forces provided as input are guaranteed to remain unchanged.

  // The dirty bit for the caching system.
  mutable bool is_dirty_{true};
  // The cached minimum moment point.
  mutable Vector3<T> minimum_moment_point_{};
  // The cached resultant wrench.
  mutable WrenchVector<T> resultant_wrench_{};
};
}  // namespace systems
}  // namespace drake
