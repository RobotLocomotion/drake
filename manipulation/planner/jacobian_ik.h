#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/scs_solver.h"

namespace drake {
namespace manipulation {
namespace planner {

/**
 * This class provides a greedy instantaneous inverse kinematics solver: given
 * current configuration (q), and desired end effector velocity (V_WE_desired),
 * outputs a generalized velocity (v), s.t. the resulting V_WE tracks
 * V_WE_desired as closely as possible subject to position and velocity
 * constraints. Note that the V_WE also has the same direction as V_WE_desired,
 * although the magnitude can be different. When the problem is
 * under-constrained, either due to redundant mechanism or untracked dimensions
 * in V_WE_desired, an extra cost term to minimize squared difference to a
 * nominal configuration (q_nominal) is used to resolved the redundancy.
 */
class JacobianIk {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JacobianIk)

  /**
   * The pose difference X_WErr is defined as:
   * <pre>
   * X_WErr = X_W1 * X_W0.inv() or X_W1 = X_WErr * X_W0
   * </pre>
   * where X_W1 is @p pose1, X_W0 is @p pose0, and W is the common frame.
   * @return A 6 dimensional vector representing the error. The position part
   * is X_WErr's translation part, and the rotation part is computed by
   * converting X_WErr's rotation part to an angle axis representation and
   * multiply the axis by the angle.
   */
  static Vector6<double> CalcPoseDifference(const Isometry3<double>& pose0,
                                            const Isometry3<double>& pose1);

  /**
   * Constructor. The joint position limits are copied from @p robot, and the
   * joint velocity limits default to M_PI rad per second. The
   * unconstrained dof velocity limit is default to 0.6.
   * throws std::runtime_error if @p robot's position and velocity have
   * different dimensions.
   */
  explicit JacobianIk(const RigidBodyTree<double>* robot);

  /**
   * Computes a joint velocity that "best" achieves the desired end effector
   * velocity @p V_WE subject to constraints. Note that, the solved
   * instantaneous end effector motion is always constrained to be in the
   * same direction as @p V_WE.
   * The problem is setup as a QP. The cost function penalizes the squared
   * magnitude difference of the end effector motion. Joint position and
   * velocity constraints are always applied.
   * If the problem is redundant (i.e. the robot has more degrees of freedom
   * than number of end effector velocity constraints), @p q_nominal is used to
   * resolved redundancy as an extra term in the cost function, and velocity
   * constraints for each unconstrained dof are also applied.
   *
   * @param cache0 The current state of the robot.
   * @param frame_E The E frame
   * @param V_WE Desired spatial velocity of E in the world.
   * @param gain_E Gain on V_WE_E specified in the E frame. Can have zero
   * element, in which case tracking in that dimension is disabled.
   * @param dt Delta time.
   * @param q_nominal Nominal posture of the robot, used to resolve redundancy.
   * @param[out] v The solved joint velocity if return value is true,
   * unchanged otherwise.
   * @param[out] is_stuck If set to true, the QP is stuck in a local minima and
   * unable to achieve @p V_WE given other constraints.
   * @return True if solver is able to find a solution, false otherwise.
   *
   * @throws std::runtime_error if @p dt < 0, @p q_nominal has a different
   * dimension or @p gain_E has any negative element.
   */
  bool CalcJointVelocity(const KinematicsCache<double>& cache0,
                         const RigidBodyFrame<double>& frame_E,
                         const Vector6<double>& V_WE,
                         const Vector6<double>& gain_E, double dt,
                         const VectorX<double>& q_nominal, VectorX<double>* v,
                         bool* is_stuck) const;

  /**
   * Returns constant reference to the robot model.
   */
  const RigidBodyTree<double>& get_robot() const { return *robot_; }

  /**
   * Sets the upper joint velocity limit to @p u.
   * @throws std::runtime_error if u is of different size, or has any element
   * smaller than the corresponding current lower joint speed limit.
   */
  void set_joint_velocity_upper_limit(const VectorX<double>& u);

  /**
   * Sets the lower joint velocity limit to @p l.
   * @throws std::runtime_error if l is of different size, or has any element
   * larger than the corresponding current upper joint speed limit.
   */
  void set_joint_velocity_lower_limit(const VectorX<double>& l);

  /**
   * Sets the upper velocity limit of the unconstrained dof to @p u.
   * @throws std::runtime_error if @p u is smaller than the current lower limit.
   */
  void set_unconstrained_dof_velocity_upper_limit(double u);

  /**
   * Sets the lower velocity limit of the unconstrained dof to @p l.
   * @throws std::runtime_error if @p l is larger than the current upper limit.
   */
  void set_unconstrained_dof_velocity_lower_limit(double l);

  /**
   * Returns the velocity upper limit.
   */
  const VectorX<double>& get_joint_velocity_upper_limit() const {
    return v_upper_;
  }

  /**
   * Returns the velocity lower limit.
   */
  const VectorX<double>& get_joint_velocity_lower_limit() const {
    return v_lower_;
  }

  /**
   * Returns the unconstrained dof velocity upper limit.
   */
  double get_unconstrained_dof_velocity_upper_limit() const {
    return unconstrained_dof_v_upper_(0);
  }

  /**
   * Returns the unconstrained dof velocity lower limit.
   */
  double get_unconstrained_dof_velocity_lower_limit() const {
    return unconstrained_dof_v_lower_(0);
  }

 private:
  const RigidBodyTree<double>* robot_{nullptr};
  const int num_joints_;
  const Eigen::MatrixXd identity_;
  const VectorX<double> zero_;

  VectorX<double> q_lower_;
  VectorX<double> q_upper_;
  VectorX<double> v_lower_;
  VectorX<double> v_upper_;
  VectorX<double> unconstrained_dof_v_upper_;
  VectorX<double> unconstrained_dof_v_lower_;

  mutable drake::solvers::ScsSolver solver_;
};

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
