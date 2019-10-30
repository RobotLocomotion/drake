#pragma once

#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/math/spatial_velocity.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace manipulation {
namespace planner {

enum class DifferentialInverseKinematicsStatus {
  kSolutionFound,    ///< Found the optimal solution.
  kNoSolutionFound,  ///< Solver unable to find a solution.
  kStuck             ///< Unable to follow the desired velocity direction
                     /// likely due to constraints.
};

std::ostream& operator<<(std::ostream& os,
                         const DifferentialInverseKinematicsStatus value);

struct DifferentialInverseKinematicsResult {
  std::optional<VectorX<double>> joint_velocities{};
  DifferentialInverseKinematicsStatus status{
      DifferentialInverseKinematicsStatus::kNoSolutionFound};
};

/**
 * Computes the pose "difference" between @p pose1 and @p pose0 s.t.
 * the linear part equals p_C1 - p_C0, and the angular part equals
 * R_C1 * R_C0.inv(), where p and R stand for the position and rotation parts,
 * and C is the common frame.
 */
Vector6<double> ComputePoseDiffInCommonFrame(const Isometry3<double>& X_C0,
                                             const Isometry3<double>& X_C1);

/**
 * Contains parameters for differential inverse kinematics.
 */
class DifferentialInverseKinematicsParameters {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(
      DifferentialInverseKinematicsParameters);

  /**
   * Constructor. Initializes the nominal joint position to zeros of size
   * @p num_positions. Timestep is initialized to 1. The end effector gains are
   * initialized to ones. All constraints are initialized to nullopt.
   * @param num_positions Number of generalized positions.
   * @param num_velocities Number of generalized velocities.
   */
  DifferentialInverseKinematicsParameters(int num_positions = 0,
                                          int num_velocities = 0);
  /// @name Getters.
  /// @{
  double get_timestep() const { return dt_; }

  int get_num_positions() const { return num_positions_; }

  int get_num_velocities() const { return num_velocities_; }

  const VectorX<double>& get_nominal_joint_position() const {
    return nominal_joint_position_;
  }

  const Vector6<double>& get_end_effector_velocity_gain() const {
    return gain_E_;
  }

  const std::optional<double>&
      get_unconstrained_degrees_of_freedom_velocity_limit() const {
    return unconstrained_degrees_of_freedom_velocity_limit_;
  }

  const std::optional<std::pair<VectorX<double>, VectorX<double>>>&
  get_joint_position_limits() const {
    return q_bounds_;
  }

  const std::optional<std::pair<VectorX<double>, VectorX<double>>>&
  get_joint_velocity_limits() const {
    return v_bounds_;
  }

  const std::optional<std::pair<VectorX<double>, VectorX<double>>>&
  get_joint_acceleration_limits() const {
    return vd_bounds_;
  }

  const std::vector<std::shared_ptr<solvers::LinearConstraint>>&
  get_linear_velocity_constraints() const;
  /// @}

  /// @name Setters.
  /// @{
  /**
   * Sets timestep to @p dt.
   * @throws std::exception if dt <= 0.
   */
  void set_timestep(double dt) {
    DRAKE_THROW_UNLESS(dt > 0);
    dt_ = dt;
  }

  /**
   * Sets the max magnitude of the velocity in the unconstrained degree of
   * freedom to @p limit.
   * @throws std::exception if limit < 0.
   */
  void set_unconstrained_degrees_of_freedom_velocity_limit(double limit) {
    DRAKE_THROW_UNLESS(limit >= 0);
    unconstrained_degrees_of_freedom_velocity_limit_ = limit;
  }

  /**
   * Sets the nominal joint position.
   * @throws std::exception if @p nominal_joint_position's dimension differs.
   */
  void set_nominal_joint_position(
      const Eigen::Ref<const VectorX<double>>& nominal_joint_position) {
    DRAKE_THROW_UNLESS(nominal_joint_position.size() == get_num_positions());
    nominal_joint_position_ = nominal_joint_position;
  }

  /**
   * Sets the end effector gains in the body frame. Gains can be used to
   * specify relative importance among different dimensions.
   * @throws std::exception if any element of @p gain_E is larger than 1 or
   * smaller than 0.
   */
  void set_end_effector_velocity_gain(const Vector6<double>& gain_E) {
    DRAKE_THROW_UNLESS((gain_E.array() >= 0).all() &&
                       (gain_E.array() <= 1).all());
    gain_E_ = gain_E;
  }

  /**
   * Sets the joint position limits.
   * @param q_bounds The first element is the lower bound, and the second is
   * the upper bound.
   * @throws std::exception if the first or second element of @p q_bounds has
   * the wrong dimension or any element of the second element is smaller than
   * its corresponding part in the first element.
   */
  void set_joint_position_limits(
      const std::pair<VectorX<double>, VectorX<double>>& q_bounds) {
    DRAKE_THROW_UNLESS(q_bounds.first.size() == get_num_positions());
    DRAKE_THROW_UNLESS(q_bounds.second.size() == get_num_positions());
    DRAKE_THROW_UNLESS(
        (q_bounds.second.array() >= q_bounds.first.array()).all());
    q_bounds_ = q_bounds;
  }

  /**
   * Sets the joint velocity limits.
   * @param q_bounds The first element is the lower bound, and the second is
   * the upper bound.
   * @throws std::exception if the first or second element of @p q_bounds has
   * the wrong dimension or any element of the second element is smaller than
   * its corresponding part in the first element.
   */
  void set_joint_velocity_limits(
      const std::pair<VectorX<double>, VectorX<double>>& v_bounds) {
    DRAKE_THROW_UNLESS(v_bounds.first.size() == get_num_velocities());
    DRAKE_THROW_UNLESS(v_bounds.second.size() == get_num_velocities());
    DRAKE_THROW_UNLESS(
        (v_bounds.second.array() >= v_bounds.first.array()).all());
    v_bounds_ = v_bounds;
  }

  /**
   * Sets the joint acceleration limits.
   * @param q_bounds The first element is the lower bound, and the second is
   * the upper bound.
   * @throws std::exception if the first or second element of @p q_bounds has
   * the wrong dimension or any element of the second element is smaller than
   * its corresponding part in the first element.
   */
  void set_joint_acceleration_limits(
      const std::pair<VectorX<double>, VectorX<double>>& vd_bounds) {
    DRAKE_THROW_UNLESS(vd_bounds.first.size() == get_num_velocities());
    DRAKE_THROW_UNLESS(vd_bounds.second.size() == get_num_velocities());
    DRAKE_THROW_UNLESS(
        (vd_bounds.second.array() >= vd_bounds.first.array()).all());
    vd_bounds_ = vd_bounds;
  }
  /// @}

  /**
   * Adds a linear velocity constraint.
   * @param linear_velocity_constraint A linear constraint on joint velocities.
   * @throws std::invalid_argument if `constraint->num_vars !=
   * this->get_num_velocities()`.
   */
  void AddLinearVelocityConstraint(
      const std::shared_ptr<solvers::LinearConstraint> constraint);

  /**
   * Clears all linear velocity constraints.
   */
  void ClearLinearVelocityConstraints();

 private:
  int num_positions_{0};
  int num_velocities_{0};
  VectorX<double> nominal_joint_position_;
  std::optional<std::pair<VectorX<double>, VectorX<double>>> q_bounds_{};
  std::optional<std::pair<VectorX<double>, VectorX<double>>> v_bounds_{};
  std::optional<std::pair<VectorX<double>, VectorX<double>>> vd_bounds_{};
  std::optional<double> unconstrained_degrees_of_freedom_velocity_limit_{};
  Vector6<double> gain_E_{Vector6<double>::Ones()};
  double dt_{1};
  std::vector<std::shared_ptr<solvers::LinearConstraint>>
      linear_velocity_constraints_;
};

/**
 * Computes a generalized velocity v_next, via the following
 * MathematicalProgram:
 *
 *   min_{v_next,alpha}   100 * | alpha - |V| |^2
 *                        // iff J.rows() < J.cols(), then
 *                          + | q_current + v_next*dt - q_nominal |^2
 *
 *   s.t. J*v_next = alpha * V / |V|  // J*v_next has the same direction as V
 *        joint_lim_min <= q_current + v_next*dt <= joint_lim_max
 *        joint_vel_lim_min <= v_next <= joint_vel_lim_max
 *        joint_accel_lim_min <= (v_next - v_current)/dt <=
 *          joint_accel_lim_max
 *        for all i > J.rows(),
 *          -unconstrained_vel_lim <= S.col(i)' v_next <= unconstrained_vel_lim
 *          where J = UΣS' is the SVD, with the singular values in decreasing
 *          order.  Note that the constraint is imposed on each column
 *          independently.
 *
 *        and any additional linear constraints added via
 *          AddLinearVelocityConstraint() in the
 *          DifferentialInverseKinematicsParameters.
 *   where J.rows() == V.size() and
 *   J.cols() == v_current.size() == q_current.size() == v_next.size().  V
 *   can have any size, with each element representing a constraint on the
 *   solution (6 constraints specifying an end-effector pose is typical, but
 *   not required).
 *
 * Intuitively, this finds a v_next such that J*v_next is in the same direction
 * as V, and the difference between |V| and |J * v_next| is minimized while all
 * constraints in @p parameters are satisfied as well. If the problem is
 * redundant, a secondary objective to minimize
 *   |q_current + v_next * dt - q_nominal|
 * is added to the problem.
 *
 * It is possible that the solver is unable to find
 * such a generalized velocity while not violating the constraints, in which
 * case, status will be set to kStuck in the returned
 * DifferentialInverseKinematicsResult.
 * @param q_current The current generalized position.
 * @param v_current The current generalized position.
 * @param V Desired spatial velocity. It must have the same number of rows as
 * @p J.
 * @param J Jacobian with respect to generalized velocities v.
 * It must have the same number of rows as @p V.
 * J * v need to represent the same spatial velocity as @p V.
 * @param parameters Collection of various problem specific constraints and
 * constants.
 * @return If the solver successfully finds a solution, joint_velocities will
 * be set to v, otherwise it will be nullopt.
 */
DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const Eigen::Ref<const VectorX<double>>& q_current,
    const Eigen::Ref<const VectorX<double>>& v_current,
    const Eigen::Ref<const VectorX<double>>& V,
    const Eigen::Ref<const MatrixX<double>>& J,
    const DifferentialInverseKinematicsParameters& parameters);

/**
 * A wrapper over
 * DoDifferentialInverseKinematics(q_current, v_current, V, J, params)
 * that tracks frame E's spatial velocity.
 * q_current and v_current are taken from @p context. V is computed by first
 * transforming @p V_WE to V_WE_E, then taking the element-wise product between
 * V_WE_E and the gains (specified in frame E) in @p parameters, and only
 * selecting the non zero elements. J is computed similarly.
 * @param robot A MultibodyPlant model.
 * @param context Must be the Context of the MultibodyPlant. Contains the
 * current generalized position and velocity.
 * @param V_WE_desired Desired world frame spatial velocity of @p frame_E.
 * @param frame_E End effector frame.
 * @param parameters Collection of various problem specific constraints and
 * constants.
 * @return If the solver successfully finds a solution, joint_velocities will
 * be set to v, otherwise it will be nullopt.
 */
DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const multibody::MultibodyPlant<double>& robot,
    const systems::Context<double>& context,
    const Vector6<double>& V_WE_desired,
    const multibody::Frame<double>& frame_E,
    const DifferentialInverseKinematicsParameters& parameters);

/**
 * A wrapper over
 * DoDifferentialInverseKinematics(robot, context, V_WE_desired, frame_E,
 * params) that tracks frame E's pose in the world frame.
 * q_current and v_current are taken from @p cache. V_WE is computed by
 * ComputePoseDiffInCommonFrame(X_WE, X_WE_desired) / dt, where X_WE is computed
 * from @p context, and dt is taken from @p parameters.
 * @param robot A MultibodyPlant model.
 * @param context Must be the Context of the MultibodyPlant. Contains the
 * current generalized position and velocity.
 * @param X_WE_desired Desired pose of @p frame_E in the world frame.
 * @param frame_E End effector frame.
 * @param parameters Collection of various problem specific constraints and
 * constants.
 * @return If the solver successfully finds a solution, joint_velocities will
 * be set to v, otherwise it will be nullopt.
 */
DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const multibody::MultibodyPlant<double>& robot,
    const systems::Context<double>& context,
    const Isometry3<double>& X_WE_desired,
    const multibody::Frame<double>& frame_E,
    const DifferentialInverseKinematicsParameters& parameters);

#ifndef DRAKE_DOXYGEN_CXX
namespace internal {
DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const Eigen::Ref<const VectorX<double>>&,
    const Eigen::Ref<const VectorX<double>>&,
    const math::RigidTransform<double>&,
    const Eigen::Ref<const Matrix6X<double>>&,
    const multibody::SpatialVelocity<double>&,
    const DifferentialInverseKinematicsParameters&);
}  // namespace internal
#endif

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
