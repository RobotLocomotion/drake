#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_plant/contact_detail.h"
#include "drake/multibody/rigid_body_plant/contact_force.h"

namespace drake {
namespace systems {

/**
 This is a utility class for replacing a set of force/torques by an equivalent
 force/torque (defined by the ContactForce class). @sa ContactForce

 The equivalent ContactForce consists of the set's resultant force applied at a
 point `P`, together with a torque equal to the moment of the set about `P`
 summed with any torques defined in the set members.  Point `P` is chosen to
 minimize the magnitude of the moment of the *normal* forces (i.e., `P` lies
 along the "central axis" of the normal forces).  Note: only the *normal*
 components of the input contact forces affect the location of point `P`.
 Tangential components of the input contact forces affect the set's resultant,
 but not the calculation of point `P`.

 Center of Pressure (more precisely center of normal forces for planar contact)
 ==============================================================================
 For point `P` to be a center of pressure (as returned by this class):
    - The normal components of all forces must lie in the same direction, `n`.
    - The points, where each force is applied, must lie on one plane, `F`.
    - Plane `F` must be perpendicular to `n`.
 If these conditions are met, `P` will be the center of pressure and lie on
 plane `F`, and the minimum moment due to the normal forces will be zero.
 Note: this class does not rely on a center of pressure *existing*.

 Usage
 =====
 The class is designed to be exercised by a contact response model.  As each
 pair of collision elements is evaluated, the contact model should instantiate
 a ContactResultantForceCalculator.  As each contact point between the elements
 is processed and a contact force is computed, the details of the contact force
 are provided to the calculator (via calls to AddForce).
 Currently, the contact force is defined by four values (see ContactForce):
    - position of the force's point of application from a common origin O,
    - normal component of the contact force (i.e., in the *normal* direction),
    - tangential component of the contact force (e.g., friction force), and
    - Optional torque term (e.g., torsional friction).
 All input vectors must be expressed in a common frame and the position vector
 must be measured from that frame's origin.

 After all of the forces have been added to the calculator, an equivalent
 ContactForce (force/torque) can be requested using the appropriate method.

 The *order* in which the forces are added has no impact on the final result.

 By default, the contact forces that are added to the calculator are destroyed
 when the calculator is destroyed.  There is an alternative constructor which
 allows the caller to provide an STL vector which will accumulate the contact
 information and allow the caller to persist the data beyond the life span of
 the calculator.

 A Single Force
 ==============
 If the set consists of a single contact force, the minimum moment point and
 resultant force will be the details of that force: i.e., its application point,
 response force, and pure torque.

 A Non-zero Minimum Moment
 =========================
 For an arbitrary set of forces, there may not be a well-defined center of
 pressure as with the planar case outlined above.  Generally, there is an
 infinite set of minimum moment points for a set of contact forces; it is a line
 called the "central axis".  Any point on this line will lead to the same
 minimum moment.  The ContactResultantForceCalculator selects one of those
 points.

 We assume that the "ideal" point would be where the line intersects the
 (deformed) contact surface. Generally, this can't be solved because it depends
 on a geometric query that is outside the scope of this calculator class.
 Furthermore, in many cases, it is unnecessary. A point on the surface is good
 for visualization, but in contexts where only a mathematically meaningful
 point is all that is needed, then one point is as good as another. That said,
 the calculator employs a method to cheaply approximate the intersection of the
 line with the contact surface by doing the following.

 The central axis can be thought of as a line defined by a point and direction.
 The point can be any point on the line.  The direction is defined by the
 direction of the resultant normal force (i.e., the vector sum of the normal
 components of all forces.) The direction vector defines "positive" and
 "negative" directions on the line. The force originated from the negative
 direction and accelerates the body in the positive direction.  If we had
 access to the geometry, the point we would be interested in, would be the
 intersection of the line and (deformed) geometry that is farthest in the
 "negative" direction (i.e., closest to the originating source of the contact).

 We will approximate this by finding the contact force application point that
 similarly lies farthest in the negative direction (simply by projecting the
 application points on the line.)  This most-negative projection point will
 serve as the reported minimum moment point.

 This reported minimum moment point can be moved along the central axis by the
 caller if additional information is available. Movement along the axis
 preserves its "minimal-moment" property. For example, if the caller had access
 to the (deformed) geometry, the ray defined by the reported minimum moment
 point and the resultant ContactForce normal direction can be intersected with
 the geometry to create an alternate, but equally valid, minimum moment point.

 A Zero-Resultant Force
 ======================
 It is possible for all of the contact forces to sum up to a zero resultant. But
 there may still be a resultant moment, i.e., the forces are "coupled".  In this
 case, the minimum moment point can be literally any point in space.  In this
 case, the ContactResultantForceCalculator defines the minimum moment point to
 be the centroid of all application points (the "average" application point.)
 Similarly, the normal direction of this resultant force is likewise meaningless
 but will be set to the +x direction (i.e., <1, 0, 0>).

 Computation considerations
 ==========================
 Even though the resultant is reported as a ContactForce instance, it should not
 be construed to mean that the results of the calculator can be meaningfully
 composed.  For example, given a set of contact forces: `S = {f_0, ..., f_n-1}`,
 the result of computing the resultant for the set `S` (i.e.,
 `F = ComputeResultant(S)` will not necessarily provide the same answer as would
 be produced by creating two disjoint subsets, `S_a` and `S_b` and then
 performing:
 ```
  F_a = ComputeResultant(S_a);
  F_b = ComputeResultant(S_b);
  F_ab = ComputeResultant({F_a, F_b});
 ```
 Do not expect `F` to be equal to `F_ab`.

 @tparam T The scalar type. Must be a valid Eigen scalar.

 Instantiated templates for the following ScalarTypes are provided:
   - double
 */
template <typename T>
class ContactResultantForceCalculator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactResultantForceCalculator)

  /** Default constructor -- no accumulation.  As contact forces are added to
   the calculator, the force will be added to the set of forces for calculation,
   but they will be destroyed when the calculator is destroyed.
   */
  ContactResultantForceCalculator();

  /** Accumulator constructor.

   This allows the caller to provide a vector into which the contact
   forces/details can be persisted beyond the life span of the
   ContactResultantForceCalculator instance.  See the various AddForce methods
   for details on what is placed in the @p detail_accumulator.

   @param detail_accumulator If non-null, ContactDetail instances will be
   appended to the vector as they are assigned to the calculator.
   */
  ContactResultantForceCalculator(
      std::vector<std::unique_ptr<ContactDetail<T>>>* detail_accumulator);

  /**
   Adds a new contact force to the calculator.

   If the calculator was initialized with a detail accumulator, an instance of
   PointContactDetail, with this contact force, will be appended to that
   accumulator.

   @param force     The contact force.
   */
  void AddForce(const ContactForce<T>& force);

  /**
   Adds a new force to the calculator from a contact detail.  The result of
   ContactDetail::ComputeContactForce will be used in the calculation.

   If the calculator was initialized with a detail accumulator, the detail will
   be appended to that accumulator. Otherwise, the detail will be destroyed
   at the conclusion of this method invocation.
   @param contact_detail        The contact detail which will provide a
                                ContactForce for computation.
   */
  void AddForce(std::unique_ptr<ContactDetail<T>> contact_detail);

  /**
   Adds a new force to the calculator.

   If the calculator was initialized with a detail accumulator, an instance of
   PointContactDetail, with this contact information, will be appended to that
   accumulator.
   @param application_point     The application point of the force.
   @param normal                The translational force's  unit-length normal
                                direction.
   @param force                 The translational force.
   */
  void AddForce(const Vector3<T>& application_point, const Vector3<T>& normal,
                const Vector3<T>& force);

  /**
   Adds a new force with an arbitrary pure torque to the calculator.

   If the calculator was initialized with a detail accumulator, an instance of
   PointContactDetail, with this contact information, will be appended to that
   accumulator.
   @param application_point     The application point of the force.
   @param normal                The translational force's  unit-length normal
                                direction.
   @param force                 The translational force.
   @param pure_torque           The pure torque for the wrench.
   */
  void AddForce(const Vector3<T>& application_point, const Vector3<T>& normal,
                const Vector3<T>& force, const Vector3<T>& pure_torque);

  /**
   Compute the resultant contact force -- it's translational force, pure torque,
   and application point.

   The rotational component of this ContactForce is pure torque only.  It does
   not include an `r X f` moment term.  It will be non-zero due to
   contributions from sources such as:

        - the minimum moment (which may be non-zero in the general case),
        - the moments induced by the tangential components of the forces
          shifted to the minimum moment point, and
        - the sum of the pure torques of the individual input contact forces.

   The responsibility of computing the moment belongs to the code that knows
   what frame the input contact forces are defined and what the origin around
   which the moment would be induced.
   */
  ContactForce<T> ComputeResultant() const;

  /**
   Computes the resultant contact spatial force with respect to a given
   reference point.

   The force part is the summation of all f_i, where f_i is the individual
   force.
   The torque part is the summation of all tau_i + (p_i - r) X f_i, where
   tau_i is the ith pure torque, f_i is applied at p_i, and r is the given
   reference point.
   @param reference_point is in the same frame as all the individual forces.
   */
  ContactForce<T> ComputeResultant(const Vector3<T>& reference_point) const;

 private:
  // Aggregator for the force data that has been added.
  std::vector<ContactForce<T>> forces_{};

  // The optional accumulator into which contact details will be added.
  std::vector<std::unique_ptr<ContactDetail<T>>>* detail_accumulator_{};

  // Given a ContactForce, adds a PointContactDetail to the accumulator if
  // one is provided.
  void AccumulateForce(const ContactForce<T>& force);
};
}  // namespace systems
}  // namespace drake
