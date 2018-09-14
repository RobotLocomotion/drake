#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/transform.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace manipulation {
namespace planner {

/** Interface class for kinematic robot models. By writing planning code against
 * this interface we avoid dependence on particular robot modelling
 * implementations.*/
class KinematicTree {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(KinematicTree);

  KinematicTree() = default;

  virtual ~KinematicTree() = default;

  /** The number of generalized position variables for this model. */
  virtual int num_positions() const = 0;

  /** The number of generalized velocity variables for this model. */
  virtual int num_velocities() const = 0;

  /** The first index of the portion of the generalized position vector that
   * corresponds to the joint connecting the body named `body_name` to the
   * tree.*/
  virtual int PositionStartIndexForBody(const std::string& body_name) const = 0;

  /** The lower bounds on the generalized positions of the model. */
  virtual const drake::VectorX<double>& joint_position_lower_limit() const = 0;

  /** The upper bounds on the generalized positions of the model. */
  virtual const drake::VectorX<double>& joint_position_upper_limit() const = 0;

  /** The lower bounds on the generalized velocities of the model. */
  virtual const drake::VectorX<double>& joint_velocity_lower_limit() const = 0;

  /** The upper bounds on the generalized velocities of the model. */
  virtual const drake::VectorX<double>& joint_velocity_upper_limit() const = 0;

  /** A generalized position vector in which all joints are set to their "zero"
   * configuration. */
  virtual drake::VectorX<double> GetZeroConfiguration() const = 0;

  /** A generalized position vector in which all joints are set to uniformly
   * distributed random values within the limits defined by
   * joint_position_lower_limit() and joint_position_upper_limit(). */
  virtual drake::VectorX<double> GetRandomConfiguration(
      std::default_random_engine* generator) const = 0;

  /* Computes the relative transform `X_AB(q)` from a frame B to a frame A, as
   * a function of the generalized positions `q` of the model.
   * That is, the position `p_AQ` of a point Q measured and expressed in
   * frame A can be computed from the position `p_BQ` of this point measured
   * and expressed in frame B using the transformation `p_AQ = X_AB⋅p_BQ`.
   *
   * @param[in] q
   *   The vector of generalized positions of the model.
   * @param[in] frame_A_name
   *   Name of the target frame A in the computed relative transform `X_AB`.
   * @param[in] frame_B_name
   *   Name of the source frame B in the computed relative transform `X_AB`.
   * @retval X_AB
   *   The relative transform from frame B to frame A, such that
   *   `p_AQ = X_AB⋅p_BQ`.
   */
  virtual drake::math::Transform<double> CalcRelativeTransform(
      const drake::VectorX<double>& q, const std::string& frame_A_name,
      const std::string& frame_B_name) const = 0;

  /** Constructs a drake::solvers::Constraint object that is only satisfied for
   * configurations that are collision-free.
   *
   * @param[in] collision_avoidance_threshold
   *   Lower bound on the distance between any two bodies in configurations that
   *   satisfy this constraint.
   */
  virtual std::shared_ptr<drake::solvers::Constraint>
  MakeCollisionAvoidanceConstraint(
      double collision_avoidance_threshold) const = 0;

  /** Constructs a drake::solvers::Constraint object that is satisfied when the
   * relative transform `X_AB(q)` from a frame B to a frame A matches
   * `X_AB_desired` to within `orientation_tolerance` (in radians) and
   * `position_tolerance` (in m).
   *
   * @param[in] frame_A_name
   *   Name of the target frame A in the computed relative transform `X_AB`.
   * @param[in] frame_B_name
   *   Name of the source frame B in the computed relative transform `X_AB`.
   * @param[in] X_AB_desired
   *   Desired relative transform from frame B to frame A.
   * @param[in] orientation_tolerance
   *   Maximum allowed rotation angle (in radians) between `X_AB(q)` and
   *   `X_AB_desired`.
   * @param[in] position_tolerance
   *   Maximum allowed norm of `p_AB - p_AB_desired`. Implementing classes may
   *   choose what norm to use.
   */
  virtual std::shared_ptr<drake::solvers::Constraint>
  MakeRelativePoseConstraint(const std::string& frame_A_name,
                             const std::string& frame_B_name,
                             const drake::math::Transform<double>& X_AB_desired,
                             double orientation_tolerance = 0,
                             double position_tolerance = 0) const = 0;

  /** Returns true if `q` lies within the joint position limits of the model. */
  bool SatisfiesJointPositionLimits(drake::VectorX<double> q) const;

  /** Sets the joint position limits of the model.
   * @pre lower_limit.size() == upper_limit.size()
   * @pre lower_limit(i) <= upper_limit(i),
   *      ∀ i ∈ {0, ..., this->num_positions()}*/
  void SetJointPositionLimits(const drake::VectorX<double>& lower_limit,
                              const drake::VectorX<double>& upper_limit);

  /** Sets the joint position limits for a particular position index.
   * @pre 0 ≤ position_index < this->num_positions()
   * @pre lower_limit <= upper_limit */
  void SetJointPositionLimits(int position_index, double lower_limit,
                              double upper_limit);

  /** Sets the joint velocity limits of the model.
   * @pre lower_limit.size() == upper_limit.size()
   * @pre lower_limit(i) <= upper_limit(i),
   *      ∀ i ∈ {0, ..., this->num_velocities()}*/
  void SetJointVelocityLimits(const drake::VectorX<double>& lower_limit,
                              const drake::VectorX<double>& upper_limit);

  /** Sets the joint velocity limits for a particular velocity index.
   * @pre 0 ≤ velocity_index < this->num_velocities()
   * @pre lower_limit <= upper_limit */
  void SetJointVelocityLimits(int velocity_index, double lower_limit,
                              double upper_limit);

 protected:
  /** Called by `SetJointPositionLimits()` once the preconditions of that method
   * have been checked. Classes implementing this interface need not re-check
   * those pre-conditions in their implementation of this method. */
  virtual void DoSetJointPositionLimits(int position_index, double lower_limit,
                                        double upper_limit) = 0;

  /** Called by `SetJointVelocityLimits()` once the preconditions of that method
   * have been checked. Classes implementing this interface need not re-check
   * those pre-conditions in their implementation of this method. */
  virtual void DoSetJointVelocityLimits(int velocity_index, double lower_limit,
                                        double upper_limit) = 0;
};
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
