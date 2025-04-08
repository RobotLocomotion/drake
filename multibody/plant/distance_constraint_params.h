#pragma once

#include <limits>

#include "drake/common/default_scalars.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

/** Parameters for a distance constraint. A distance constraint is modeled as a
 holonomic constraint. Distance constraints can be "soft", and are implemented
 as a spring force, f: f = -k⋅(d(q) - d₀) - c⋅ḋ(q), where d₀ is a fixed length,
 k a stiffness parameter in N/m and c a damping parameter in N⋅s/m. We use d(q)
 to denote the Euclidean distance between two points P and Q, rigidly affixed to
 bodies A and B respectively, as a function of the configuration of the model q.
 This constraint reduces to d(q) = d₀ in the limit to infinite stiffness and it
 behaves as a linear spring damper for finite values of stiffness and damping.

 @warning A distance constraint is the wrong modeling choice if the distance
 needs to go through zero. To constrain two points to be coincident we need a
 3-dof ball constraint, the 1-dof distance constraint is singular in this case.
 Therefore we require the distance parameter to be strictly positive. */
class DistanceConstraintParams {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DistanceConstraintParams);

  /** Construction to an invalid set of parameters only meant to facilitate use
   with STL containers. **/
  DistanceConstraintParams() = default;

  /** Constructor for a set of valid parameters.

   @param[in] bodyA Index of body A in the MultibodyPlant.
   @param[in] bodyB Index of body B in the MultibodyPlant.
   @param[in] p_AP  Position of a point P in A's body frame.
   @param[in] p_BQ  Position of a point Q in Q's body frame.
   @param[in] distance The fixed length of this constraint.
   @param[in] stiffness If finite, this constraint models a massless elastic rod
                        of length `distance` with the given `stiffness` and
                        `damping`. "Infinite" is a valid value if a hard
                        constraint is intended. See the class documentation for
                        details.
   @param[in] damping When `stiffness` is finite, this value parametrizes the
                      damping model of the elastic massless rod. See the class
                      documentation for details.

   @warning This class does not have a way to check whether body indices do
   correspond to valid indices in a MultibodyPlant. Use MultibodyPlant APIs to
   safely register distance constraints between valid bodies and to later
   retrieve their valid set of parameters.

   @throws std::exception if BodyIndex::is_valid() is false for either body.
   @throws std::exception if bodies A and B are the same body.
   @throws std::exception if `distance` is not strictly positive.
   @throws std::exception if `stiffness` is not strictly positive.
   @throws std::exception if `damping` is not positive or zero. */
  DistanceConstraintParams(BodyIndex bodyA, const Vector3<double>& p_AP,
                           BodyIndex bodyB, const Vector3<double>& p_BQ,
                           double distance, double stiffness, double damping);

  /** Index of body A. */
  BodyIndex bodyA() const { return bodyA_; }

  /** Index of body B. */
  BodyIndex bodyB() const { return bodyB_; }

  /** Position of point P in body A's frame. */
  const Vector3<double>& p_AP() const { return p_AP_; }

  /** Position of point Q in body B's frame. */
  const Vector3<double>& p_BQ() const { return p_BQ_; }

  /** The fixed length of the constraint. */
  double distance() const { return distance_; }

  /** Constraint stiffness. See class documentation. */
  double stiffness() const { return stiffness_; }

  /** Constraint damping. See class documentation. */
  double damping() const { return damping_; }

 private:
  BodyIndex bodyA_;       // Index of body A.
  Vector3<double> p_AP_;  // Position of point P in body frame A.
  BodyIndex bodyB_;       // Index of body B.
  Vector3<double> p_BQ_;  // Position of point Q in body frame B.
  double distance_{};     // Free length d₀.
  double stiffness_{};    // Constraint stiffness k in N/m.
  double damping_{};      // Constraint damping c in N⋅s/m.
};

}  // namespace multibody
}  // namespace drake
