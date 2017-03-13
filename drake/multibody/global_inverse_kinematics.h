#pragma once
#include <vector>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace multibody {
/** Solves the inverse kinematics problem as a mixed integer convex optimization
 * problem.
 * We use a convex relaxation of the rotation matrix. So if this global inverse
 * kinematics problem says the solution is infeasible, then it is guaranteed
 * that the kinematics constraints are not satisfiable.
 * If the global inverse kinematics returns a solution, the posture should
 * satisfy the kinematics constraints, with some error.
 */
class GlobalInverseKinematics : public solvers::MathematicalProgram {
// TODO(hongkai.dai): create a function globalIK, with interface similar to
// inverseKin(), that accepts RigidBodyConstraint objects and cost function.
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GlobalInverseKinematics)

  /**
   * Parses the robot kinematics tree. The decision variables include the
   * pose for each body (position/orientation). This constructor loops through
   * each body inside the robot kinematics tree, adds the constraint on each
   * body pose, so that the adjacent bodies are connected correctly by the joint
   * in between the bodies.
   * @param robot The robot on which the inverse kinematics problem is solved.
   * @param num_binary_vars_per_half_axis The number of binary variables for
   * each half axis, to segment the unit circle.
   * @see AddRotationMatrixMcCormickEnvelopeMilpConstraints() for more details
   * on num_binary_vars_per_half_axis.
   */
  GlobalInverseKinematics(const RigidBodyTreed& robot,
                          int num_binary_vars_per_half_axis = 2);

  ~GlobalInverseKinematics() override {}

  /** Getter for the decision variables on the rotation matrix `R_WB` for a body
   * with the specified index. This is the orientation of body i's frame
   * measured and expressed in the world frame.
   * @param body_index  The index of the queried body. Notice that body 0 is
   * the world, and thus not a decision variable. Throws a runtime_error if
   * the index is smaller than 1, or no smaller than the total number of bodies
   * in the robot.
   */
  const solvers::MatrixDecisionVariable<3, 3>& body_rotation_matrix(
      int body_index) const;

  /** Getter for the decision variables on the position p_WBo of the body B's
   * origin measured and expressed in the world frame.
   * @param body_index The index of the queried body. Notice that body 0 is
   * the world, and thus not a decision variable. Throws a runtime_error if
   * the index is smaller than 1, or greater than or equal to the total number
   * of bodies in the robot.
   */
  const solvers::VectorDecisionVariable<3>& body_position(int body_index) const;

  /**
   * After solving the inverse kinematics problem and finding out the pose of
   * each body, reconstruct the robot generalized position (joint angles, etc)
   * that matches with the body poses. Notice that since the rotation matrix is
   * approximated, that the solution of body_rotmat() might not be on SO(3)
   * exactly, the reconstructed body posture might not match with the body poses
   * exactly, and the kinematics constraint might not be satisfied exactly with
   * this reconstructed posture.
   * @warning Do not call this method if the problem is not solved successfully!
   * The returned value can be NaN or meaningless number if the problem is
   * not solved.
   * @retval q The reconstructed posture of the robot of the generalized
   * coordinates, corresponding to the RigidBodyTree on which the inverse
   * kinematics problem is solved.
   */
  Eigen::VectorXd ReconstructGeneralizedPositionSolution() const;

  /**
   * Adds the constraint that the position of a point `Q` on a body `B`
   * (whose index is `body_idx`), is within a box in a specified frame `F`.
   * The constraint is that the point position, computed as
   * <pre>
   *   p_WQ = p_WBo + R_WB * p_BQ
   * </pre>
   * where
   *   - p_WQ is the position of the body point Q measured and expressed in the
   *     world frame `W`.
   *   - p_WBo is the position of the body origin Bo measured and expressed in
   *     the world frame `W`.
   *   - R_WB is the rotation matrix of the body measured and expressed in the
   *     world frame `W`.
   *   - p_BQ is the position of the body point Q measured and expressed in the
   *     body frame `B`.
   * p_WQ should lie within a bounding box in the frame `F`. Namely
   * <pre>
   *   box_lb_F <= p_FQ <= box_ub_F
   * </pre>
   * where p_FQ is the position of the point Q measured and expressed in the
   * `F`.
   * The inequality is imposed elementwisely.
   *
   * Notice that since the rotation matrix `R_WB` does not lie exactly on the
   * SO(3), due to the McCormick envelope relaxation, this constraint is subject
   * to the accumulated error from the root of the kinematics tree.
   * @param body_idx The index of the body on which the position of a point is
   * constrained.
   * @param p_BQ The position of the point Q measured and expressed in the
   * body frame B.
   * @param box_lb_F The lower bound of the box in frame `F`.
   * @param box_ub_F The upper bound of the box in frame `F`.
   * @param X_WF. The frame in which the box is specified. This
   * frame is represented by an isometry transform X_WF, the transform from
   * the constraint frame F to the world frame W. Namely if the position of
   * the point `Q` in the world frame is `p_WQ`, then the constraint is
   * <pre>
   *    box_lb_F <= R_FW * (p_WQ-p_WFo) <= box_ub_F
   * </pre>
   * where
   *   - R_FW is the rotation matrix of frame `W` expressed and measured in
   *     frame `F`. `R_FW = X_WF.linear().transpose()`.
   *   - p_WFo is the position of frame `F`'s origin, expressed and measured in
   *     frame `W`. `p_WFo = X_WF.translation()`.
   * @default is the identity transform.
   */
  void AddWorldPositionConstraint(int body_idx, const Eigen::Vector3d& p_BQ,
                                  const Eigen::Vector3d& box_lb_F,
                                  const Eigen::Vector3d& box_ub_F,
                                  const Eigen::Isometry3d& X_WF =
                                      Eigen::Isometry3d::Identity());

  /**
   * Add a constraint that the angle between the body orientation and the
   * desired orientation should not be larger than `angle_tol`. If we denote the
   * angle between two rotation matrices `R1` and `R2` as `θ`, namely θ is the
   * angle of the angle-axis representation of the rotation matrix `R1ᵀ * R2`,
   * we then know
   * <pre>
   *    trace(R1ᵀ * R2) = 2 * cos(θ) + 1
   * </pre>
   * as in http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/
   * To constraint `θ < angle_tol`, we can impose the following constraint
   * <pre>
   *    2 * cos(angle_tol) + 1 <= trace(R1ᵀ * R2) <= 3
   * </pre>
   *
   * @param body_idx The index of the body whose orientation will be
   * constrained.
   * @param desired_orientation The desired orientation of the body.
   * @param angle_tol The tolerance on the angle between the body orientation
   * and the desired orientation. Unit is radians.
   */
  void AddWorldOrientationConstraint(
      int body_idx, const Eigen::Quaterniond& desired_orientation,
      double angle_tol);

  /** Penalizes the deviation to the desired posture.
   * For each body (except the world) in the kinematic tree, we add the cost
   *  `∑ᵢ body_position_cost(i) * body_position_error(i) +
   *  body_orientation_cost(i) * body_orientation_error(i)`
   * where `body_position_error(i)` is computed as the Euclidean distance error
   * |p_WBo(i) - p_WBo_desired(i)|
   * where
   * - p_WBo(i)        : position of body i'th origin `Bo` in the world frame
   *                     `W`.
   * - p_WBo_desired(i): position of body i'th origin `Bo` in the world frame
   *                     `W`, computed from the desired posture `q_desired`.
   *
   * body_orientation_error(i) is computed as (1 - cos(θ)), where θ is the
   * angle between the orientation of body i'th frame and body i'th frame using
   * the desired posture. Notice that 1 - cos(θ) = θ²/2 + O(θ⁴), so this cost
   * is on the square of θ, when θ is small.
   * Notice that since body 0 is the world, the cost on that body is always 0,
   * no matter what value `body_position_cost(0)` and `body_orientation_cost(0)`
   * take.
   * @param q_desired  The desired posture.
   * @param body_position_cost  The cost for each body's position error. Unit is
   * [1/m] (one over meters).
   * @pre
   * 1. body_position_cost.rows() == robot->get_num_bodies(), where `robot`
   *    is the input argument in the constructor of the class.
   * 2. body_position_cost(i) is non-negative.
   * @throw a runtime error if the precondition is not satisfied.
   * @param body_orientation_cost The cost for each body's orientation error.
   * @pre
   * 1. body_orientation_cost.rows() == robot->get_num_bodies() , where
   *    `robot` is the input argument in the constructor of the class.
   * 2. body_position_cost(i) is non-negative.
   * @throw a runtime_error if the precondition is not satisfied.
   */
  void AddPostureCost(
      const Eigen::Ref<const Eigen::VectorXd>& q_desired,
      const Eigen::Ref<const Eigen::VectorXd>& body_position_cost,
      const Eigen::Ref<const Eigen::VectorXd>& body_orientation_cost);

 private:
  const RigidBodyTree<double> *robot_;

  // R_WB_[i] is the orientation of body i in the world reference frame,
  // it is expressed in the world frame.
  std::vector<solvers::MatrixDecisionVariable<3, 3>> R_WB_;

  // p_WBo_[i] is the position of the origin Bo of body frame B for the i'th
  // body, measured and expressed in the world frame.
  std::vector<solvers::VectorDecisionVariable<3>> p_WBo_;
};
}  // namespace multibody
}  // namespace drake
