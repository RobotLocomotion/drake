#pragma once

#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

/**
 This is a utility class for taking a sequence of contact forces and defining
 the resultant force and its application point.

 The application point for the resultant force will be a minimum moment
 magnitude point.  That is, if the resultant force is applied at this point,
 it will induce the minimum moment.

 For contact forces with particular properties, this point is also the center
 of pressure.  However, the set of contact forces must have the following
 properties:
    - All of the normal components of the forces must be parallel.
    - All of the force application points must lie on the same plane.
    - The plane the application points lie on must be perpendicular to the
    common direction of the contact normal force.
 If these conditions are met, the application point will also lie on the plane
 and will be the center of pressure.  And the minimum moment will be zero.

 If any of these conditions are not met, the application point will still lie
 on the same line of action of the resultant force, but will not necessarily be
 the center of pressure and the minimum moment may be non-zero.

 The utility class serves as an aggregator.  For a given set of related contacts
 (i.e., contact forces between a common pair of collision elements oriented in
 a unified manner), each force can be added to a unique instance of the
 calculator.  Once all of the forces have been added, the calculator can be
 queried for its resultant force and the application point.  The forces being
 added must be measured and expressed in a common frame.  The resultant force
 and application point will be measured and expressed in that same frame.

 The queries for resulatant force and application point can be made at any time.
 The actual values will change as more forces are added to the calculator.
 */
template <typename T>
class DRAKE_EXPORT ContactResultantForceCalculator {
 public:
  /**
   Adds a new force to the calculator.
   @param normal_force          The normal component of the force.
   @param tangent_force         The tangent component of the force.
   @param application_point     The application point of the force.
   */
  void AddForce(const Vector3<T>& normal_force, const Vector3<T>& tangent_force,
                const Vector3<T>& application_point);

  /**
   Adds a new force with an arbitrary pure torque to the calculator.
   @param normal_force          The normal component of the force.
   @param tangent_force         The tangent component of the force.
   @param application_point     The application point of the force.
   @param pure_torque           The pure torque for the wrench.
   */
  void AddForce(const Vector3<T>& normal_force, const Vector3<T>& tangent_force,
                const Vector3<T>& application_point,
                const Vector3<T>& pure_torque);

  /**
   Compute the minimum moment point -- the point at which the resultant force
   induces the minimum moment.
   */
  Vector3<T> ComputeMinimumMomentPoint() const;

  /**
   Compute the resultant wrench to be applied at the minimum moment point.
   */
  WrenchVector<T> ComputeResultantWrench();

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
  // Recomputes the cached minimimum moment point and resultant force.
  void ComputeCachedData();

  // Aggregator for the force data that has been added.
  // TODO(SeanCurtis-TRI): Get the class right.
  std::vector<int> forces_{};

  // To facilitate computation, this class uses a light-weight caching system
  // to prevent redundant computations.  It works with a dirty/clean bit
  // to indicate if the values need to be computed.

  // The dirty bit for the caching system.
  bool is_dirty_{true};
  // The cached minimum moment point.
  Vector3<T> minimum_moment_point_{};
  // The cached resultant wrench.
  WrenchVector<T> resultant_wrench_{};
};
}  // namespace systems
}  // namespace drake
