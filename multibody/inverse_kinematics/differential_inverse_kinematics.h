#pragma once

#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/fmt_ostream.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace multibody {

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
 * Computes the pose "difference" between @p X_C0 and @p X_C1 such that
 * the linear part equals p_C1 - p_C0, and the angular part equals
 * R_C1 * R_C0.inv(), where p and R stand for the position and rotation parts,
 * and C is the common frame.
 */
Vector6<double> ComputePoseDiffInCommonFrame(
    const math::RigidTransform<double>& X_C0,
    const math::RigidTransform<double>& X_C1);

/**
 * Contains parameters for the family of differential inverse kinematics
 * function overloads below, each named DoDifferentialInverseKinematics().
 */
class DifferentialInverseKinematicsParameters {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(
      DifferentialInverseKinematicsParameters);

  /**
   * Constructor. Initializes the nominal joint position to zeros of size
   * @p num_positions. The time step is initialized to 1. The end effector
   * flags are initialized to True. The joint centering gains are initialized
   * to zero. All constraints are initialized to nullopt.
   * @param num_positions Number of generalized positions.
   * @param num_velocities Number of generalized velocities (by default it will
   * be set to num_positions).
   */
  DifferentialInverseKinematicsParameters(
      int num_positions, std::optional<int> num_velocities = std::nullopt);

  /// @name Getters.
  /// @{
  double get_time_step() const { return dt_; }

  int get_num_positions() const { return num_positions_; }

  int get_num_velocities() const { return num_velocities_; }

  const VectorX<double>& get_nominal_joint_position() const {
    return nominal_joint_position_;
  }

  const MatrixX<double>& get_joint_centering_gain() const {
    return joint_centering_gain_;
  }

  const Vector6<bool>& get_end_effector_velocity_flag() const {
    return flag_E_;
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

  double get_maximum_scaling_to_report_stuck() const {
    return max_scaling_to_report_stuck_;
  }

  double get_end_effector_angular_speed_limit() const {
    return angular_speed_limit_;
  }

  const std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>>&
  get_end_effector_translational_velocity_limits() const {
    return translational_velocity_bounds_;
  }

  const std::vector<std::shared_ptr<solvers::LinearConstraint>>&
  get_linear_velocity_constraints() const;
  /// @}

  /// @name Setters.
  /// @{
  /**
   * Sets time step to @p dt.
   * @throws std::exception if dt <= 0.
   */
  void set_time_step(double dt) {
    DRAKE_THROW_UNLESS(dt > 0);
    dt_ = dt;
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

  // TODO(russt): It's not clear that it is reasonable to enable/disable
  // independent components of the angular velocity vector.
  /**
   * Sets the end effector flags in the body frame. If a spatial velocity flag
   * is set to false, it will not be included in the differential IK
   * formulation.
   */
  void set_end_effector_velocity_flag(const Vector6<bool>& flag_E) {
    flag_E_ = flag_E;
  }

  /**
   * Sets the joint centering gain, K, so that the joint centering command is
   * attempting to achieve v_next = N⁺(q) * K * (q_nominal - q_current).
   * @pre K must be num_positions x num_positions.
   */
  void set_joint_centering_gain(const MatrixX<double>& K) {
    DRAKE_DEMAND(K.rows() == num_positions_);
    DRAKE_DEMAND(K.cols() == num_positions_);
    joint_centering_gain_ = K;
  }

  // TODO(russt): These setters do not provide a way to *unset* the optional
  // methods to nullopt.

  // TODO(russt): Change to two arguments: (lower, upper) to match
  // MathematicalProgram and other pieces of Drake.
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
   * @param v_bounds The first element is the lower bound, and the second is
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
   * @param vd_bounds The first element is the lower bound, and the second is
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

  /** Sets the threshold for α below which the status returned is
  DifferentialInverseKinematicsStatus::kStuck. α is the scaling of the
  commanded spatial velocity, so when α is small, it means that the actual
  spatial velocity magnitude will be small proportional to the commanded.
  @default 0.01. */
  void set_maximum_scaling_to_report_stuck(double scaling) {
    max_scaling_to_report_stuck_ = scaling;
  }

  /** When calling DoDifferentialInverseKinematics with a desired end-effector
  pose, this limits the magnitude of the angular velocity vector. */
  void set_end_effector_angular_speed_limit(double speed) {
    DRAKE_THROW_UNLESS(speed >= 0);
    angular_speed_limit_ = speed;
  }

  /** When calling DoDifferentialInverseKinematics with a desired end-effector
  pose, this sets limits on the translational velocity. */
  void set_end_effector_translational_velocity_limits(
      const Eigen::Ref<const Eigen::Vector3d>& lower,
      const Eigen::Ref<const Eigen::Vector3d>& upper) {
    DRAKE_THROW_UNLESS((upper.array() >= lower.array()).all());
    translational_velocity_bounds_ = std::pair(lower, upper);
  }

  /// @}

  /**
   * Adds a linear velocity constraint.
   * @param constraint A linear constraint on joint velocities.
   * @throws std::exception if `constraint->num_vars !=
   * this->get_num_velocities()`.
   */
  void AddLinearVelocityConstraint(
      const std::shared_ptr<solvers::LinearConstraint> constraint);

  /**
   * Clears all linear velocity constraints.
   */
  void ClearLinearVelocityConstraints();

  /**
   * Provides const access to read the solver options.
   */
  const solvers::SolverOptions& get_solver_options() const {
    return solver_options_;
  }

  /**
   * Provides mutable access to change the solver options, e.g., to tune for
   * speed vs accuracy.
   */
  solvers::SolverOptions& get_mutable_solver_options() {
    return solver_options_;
  }

 private:
  int num_positions_{0};
  int num_velocities_{0};
  VectorX<double> nominal_joint_position_;
  std::optional<std::pair<VectorX<double>, VectorX<double>>> q_bounds_{};
  std::optional<std::pair<VectorX<double>, VectorX<double>>> v_bounds_{};
  std::optional<std::pair<VectorX<double>, VectorX<double>>> vd_bounds_{};
  std::optional<double> unconstrained_degrees_of_freedom_velocity_limit_{};
  MatrixX<double> joint_centering_gain_;
  Vector6<bool> flag_E_{Vector6<bool>::Ones()};
  double dt_{1};
  double max_scaling_to_report_stuck_{1e-2};
  double angular_speed_limit_{std::numeric_limits<double>::infinity()};
  std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
      translational_velocity_bounds_{};
  std::vector<std::shared_ptr<solvers::LinearConstraint>>
      linear_velocity_constraints_;
  solvers::SolverOptions solver_options_;
};

/**
 * Computes a generalized velocity v_next, via the following
 * MathematicalProgram:
 *
 * ```
 *   min_{v_next,alpha}
 *     -100 * alpha + |P⋅(v_next - N⁺(q)⋅K⋅(q_nominal - q_current))|²
 *
 *   s.t.
 *     J⋅v_next = alpha⋅V, // J⋅v_next has the same direction as V
 *     0 <= alpha <= 1,        // Never go faster than V
 *     joint_lim_min <= q_current + N⋅v_next⋅dt <= joint_lim_max,
 *     joint_vel_lim_min <= v_next <= joint_vel_lim_max,
 *     joint_accel_lim_min <= (v_next - v_current)/dt <= joint_accel_lim_max,
 *     and additional linear velocity constraints,
 * ```
 * where:
 *   - The rows of P form an orthonormal basis for the nullspace of J,
 *   - J.rows() == V.size(),
 *   - J.cols() == v_current.size() == v_next.size(),
 *   - V can have any size, with each element representing a constraint on the
 *     solution (6 constraints specifying an end-effector spatial velocity is
 *     typical, but not required),
 *   - K is the joint_centering_gain,
 *   - the "additional linear velocity constraints" are added via
 *     DifferentialInverseKinematicsParameters::AddLinearVelocityConstraint().
 *
 * Intuitively, this finds a v_next such that J*v_next is in the same direction
 * as V, and the difference between |V| and |J * v_next| is minimized while all
 * constraints in @p parameters are satisfied as well. In the nullspace of this
 * objective, we have a secondary objective to minimize |v_next -
 * N⁺(q)⋅K⋅(q_nominal - q_current)|².
 *
 * For more details, see
 * https://manipulation.csail.mit.edu/pick.html#diff_ik_w_constraints .
 *
 * If q_current is a feasible point, then v_next = 0 should always be a
 * feasible solution. If the problem data is bad (q_current is infeasible, and
 * no feasible velocities can restore feasibility in one step), then it is
 * possible that the solver cannot find a solution, in which case, status will
 * be set to kNoSolution in the returned DifferentialInverseKinematicsResult.
 * If the velocity scaling, alpha, is very small, then the status will be set
 * to kStuck.
 * @param q_current The current generalized position.
 * @param v_current The current generalized position.
 * @param V Desired spatial velocity. It must have the same number of rows as
 * @p J.
 * @param J Jacobian with respect to generalized velocities v. It must have the
 * same number of rows as @p V. J * v needs to represent the same spatial
 * velocity as @p V.
 * @param parameters Collection of various problem specific constraints and
 * constants.
 * @param N (optional) matrix which maps q̇ = N(q)⋅v. See
 * MultibodyPlant::MakeVelocityToQDotMap(). By default, it is taken to be the
 * identity matrix. If dim(q) != dim(v) and any joint position limits are set in
 * `parameters`, then you *must* provide N.
 * @param Nplus (optional) matrix which maps q̇ = N⁺(q)⋅v. See
 * MultibodyPlant::MakeQDotToVelocityMap(). By default, it is taken to be the
 * identity matrix. If dim(q) != dim(v) and J is not full column rank, then you
 * *must* provide Nplus.
 * @return If the solver successfully finds a solution, joint_velocities will
 * be set to v, otherwise it will be nullopt.
 *
 * @note There is a newer framework-based formulation for differential inverse
 * kinematics: DifferentialInverseKinematicsSystem. This implementation has been
 * shown to be more effective for real-world robots. Furthermore, its
 * architecture is more flexible, allowing for more customization of the cost
 * and constraint functions.
 *
 * @ingroup planning_kinematics
 */
DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const Eigen::Ref<const VectorX<double>>& q_current,
    const Eigen::Ref<const VectorX<double>>& v_current,
    const Eigen::Ref<const VectorX<double>>& V,
    const Eigen::Ref<const MatrixX<double>>& J,
    const DifferentialInverseKinematicsParameters& parameters,
    const std::optional<Eigen::Ref<const Eigen::SparseMatrix<double>>>& N =
        std::nullopt,
    const std::optional<Eigen::Ref<const Eigen::SparseMatrix<double>>>& Nplus =
        std::nullopt);

// TODO(russt): V_WE_desired should be of type SpatialVelocity.
/**
 * A wrapper over DoDifferentialInverseKinematics(q_current, v_current, V, J,
 * params) that tracks frame E's spatial velocity. q_current and v_current are
 * taken from @p context. V and J are expressed in E, and only the elements
 * with non-zero gains in @p parameters get_end_effector_velocity_gains() are
 * used in the program.
 * @param robot A MultibodyPlant model.
 * @param context Must be the Context of the MultibodyPlant. Contains the
 * current generalized position and velocity.
 * @param V_WE_desired Desired world frame spatial velocity of @p frame_E.
 * @param frame_E End effector frame.
 * @param parameters Collection of various problem specific constraints and
 * constants.
 * @return If the solver successfully finds a solution, joint_velocities will
 * be set to v, otherwise it will be nullopt.
 *
 * @note There is a newer framework-based formulation for differential inverse
 * kinematics: DifferentialInverseKinematicsSystem. This implementation has been
 * shown to be more effective for real-world robots. Furthermore, its
 * architecture is more flexible, allowing for more customization of the cost
 * and constraint functions.
 *
 * @ingroup planning_kinematics
 */
DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const MultibodyPlant<double>& robot,
    const systems::Context<double>& context,
    const Vector6<double>& V_WE_desired, const Frame<double>& frame_E,
    const DifferentialInverseKinematicsParameters& parameters);

/**
 * A wrapper over DoDifferentialInverseKinematics(q_current, v_current, V, J,
 * params) that tracks frame E's spatial velocity in frame A.
 * q_current and v_current are taken from @p context. V and J are expressed
 * in E, and only the elements with non-zero gains in @p parameters
 * get_end_effector_velocity_gains() are used in the program.
 * @param robot A MultibodyPlant model.
 * @param context Must be the Context of the MultibodyPlant. Contains the
 * current generalized position and velocity.
 * @param V_AE_desired Desired spatial velocity of @p frame_E in @p frame A.
 * @param frame_A Reference frame (inertial or non-inertial).
 * @param frame_E End effector frame.
 * @param parameters Collection of various problem specific constraints and
 * constants.
 * @return If the solver successfully finds a solution, joint_velocities will
 * be set to v, otherwise it will be nullopt.
 *
 * @note There is a newer framework-based formulation for differential inverse
 * kinematics: DifferentialInverseKinematicsSystem. This implementation has been
 * shown to be more effective for real-world robots. Furthermore, its
 * architecture is more flexible, allowing for more customization of the cost
 * and constraint functions.
 *
 * @ingroup planning_kinematics
 */
DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const MultibodyPlant<double>& robot,
    const systems::Context<double>& context,
    const Vector6<double>& V_AE_desired, const Frame<double>& frame_A,
    const Frame<double>& frame_E,
    const DifferentialInverseKinematicsParameters& parameters);

/**
 * A wrapper over DoDifferentialInverseKinematics(robot, context, V_WE_desired,
 * frame_E, params) that tracks frame E's pose in the world frame. q_current
 * and v_current are taken from @p context. V_WE_desired is computed by
 * ComputePoseDiffInCommonFrame(X_WE, X_WE_desired) / dt, where X_WE is
 * computed from @p context, and dt is taken from @p parameters.
 * @param robot A MultibodyPlant model.
 * @param context Must be the Context of the MultibodyPlant. Contains the
 * current generalized position and velocity.
 * @param X_WE_desired Desired pose of @p frame_E in the world frame.
 * @param frame_E End effector frame.
 * @param parameters Collection of various problem specific constraints and
 * constants.
 * @return If the solver successfully finds a solution, joint_velocities will
 * be set to v, otherwise it will be nullopt.
 *
 * @note There is a newer framework-based formulation for differential inverse
 * kinematics: DifferentialInverseKinematicsSystem. This implementation has been
 * shown to be more effective for real-world robots. Furthermore, its
 * architecture is more flexible, allowing for more customization of the cost
 * and constraint functions.
 *
 * @ingroup planning_kinematics
 */
DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const MultibodyPlant<double>& robot,
    const systems::Context<double>& context,
    const math::RigidTransform<double>& X_WE_desired,
    const Frame<double>& frame_E,
    const DifferentialInverseKinematicsParameters& parameters);

/**
 * A wrapper over DoDifferentialInverseKinematics(robot, context, V_AE_desired,
 * frame_A, frame_E, params) that tracks frame E's pose in frame A. q_current
 * and v_current are taken from @p context. V_AE_desired is computed by
 * ComputePoseDiffInCommonFrame(X_AE, X_AE_desired) / dt, where X_WE is
 * computed from @p context, and dt is taken from @p parameters.
 * @param robot A MultibodyPlant model.
 * @param context Must be the Context of the MultibodyPlant. Contains the
 * current generalized position and velocity.
 * @param X_AE_desired Desired pose of @p frame_E in @p frame_A.
 * @param frame_A Reference frame (inertial or non-inertial).
 * @param frame_E End effector frame.
 * @param parameters Collection of various problem specific constraints and
 * constants.
 * @return If the solver successfully finds a solution, joint_velocities will
 * be set to v, otherwise it will be nullopt.
 *
 * @note There is a newer framework-based formulation for differential inverse
 * kinematics: DifferentialInverseKinematicsSystem. This implementation has been
 * shown to be more effective for real-world robots. Furthermore, its
 * architecture is more flexible, allowing for more customization of the cost
 * and constraint functions.
 *
 * @ingroup planning_kinematics
 */
DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const MultibodyPlant<double>& robot,
    const systems::Context<double>& context,
    const math::RigidTransform<double>& X_AE_desired,
    const Frame<double>& frame_A, const Frame<double>& frame_E,
    const DifferentialInverseKinematicsParameters& parameters);

#ifndef DRAKE_DOXYGEN_CXX
namespace internal {
DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const Eigen::Ref<const VectorX<double>>& q_current,
    const Eigen::Ref<const VectorX<double>>& v_current,
    const math::RigidTransform<double>& X_WE,
    const Eigen::Ref<const Matrix6X<double>>& J_WE_W,
    const SpatialVelocity<double>& V_WE_desired,
    const DifferentialInverseKinematicsParameters& parameters,
    const std::optional<Eigen::Ref<const MatrixX<double>>>& N = std::nullopt,
    const std::optional<Eigen::Ref<const MatrixX<double>>>& Nplus =
        std::nullopt);
}  // namespace internal
#endif

}  // namespace multibody
}  // namespace drake

namespace fmt {
template <>
struct formatter<drake::multibody::DifferentialInverseKinematicsStatus>
    : drake::ostream_formatter {};
}  // namespace fmt
