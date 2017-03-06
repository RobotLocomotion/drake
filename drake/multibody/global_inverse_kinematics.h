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
   * body pose, so that the adjacent bodies are welded correctly by the joint.
   * @param robot The robot on which the inverse kinematics problem is solved.
   * @param num_binary_vars_per_half_axis The number of binary variables for
   * each half axis, to segment the unit circle.
   * @see AddRotationMatrixMcCormickEnvelopeMilpConstraints() for more details
   * on num_binary_vars_per_half_axis.
   */
  GlobalInverseKinematics(const RigidBodyTreed& robot,
                          int num_binary_vars_per_half_axis = 2);

  ~GlobalInverseKinematics() override {}

  /** Getter for the decision variables on the rotation matrix R_WB for body
   * with the specified index. This is the orientation of body i's frame in the
   * world frame.
   * @param body_index  The index of the queried body. Notice that body 0 is
   * the world, and thus not a decision variable. Throws a runtime error if
   * the index is smaller than 1, or no smaller than the total number of bodies
   * in the robot.
   */
  const solvers::MatrixDecisionVariable<3, 3>& body_rotmat(int body_index)
      const;

  /** Getter for the decision variables on the position p_WBo of the body B's
   * origin in the world frame.
   * @param body_index  The index of the queried body. Notice that body 0 is
   * the world, and thus not a decision variable. Throws a runtime error if
   * the index is smaller than 1, or no smaller than the total number of bodies
   * in the robot.
   */
  const solvers::VectorDecisionVariable<3>& body_pos(int body_index) const;

  /**
   * After solving the inverse kinematics problem and finding out the pose of
   * each body, reconstruct the robot posture (joint angles, etc) that matches
   * with the body poses. Notice that since the rotation matrix is approximated,
   * that the solution of body_rotmat() might not be on SO(3) exactly, the
   * reconstructed body posture might not match with the body poses exactly, and
   * the kinematics constraint might not be satisfied exactly with this
   * reconstructed posture.
   * @warning Do not call this method if the problem is not solved successfully!
   * The returned value can be NaN or meaningless number if the problem is
   * not solved.
   * @retval q The reconstructed posture of the robot. q.cols() is the same as
   * robot_->get_num_positions().
   */
  Eigen::VectorXd ReconstructPostureSolution() const;

  /**
   * Adds the constraint that position of a point Q on a body
   * (whose index is `body_idx`), is within a box in a specified frame.
   * The constrain is that the point position, computed as
   *   p_WQ = p_WBo + R_WB * p_BQ
   * where
   *   p_WQ is the position of the body point Q in the world frame.
   *   p_WBo is the position of the body origin O in the world frame.
   *   R_WB is the rotation matrix of the body in the world frame.
   *   p_BQ is the position of the body point Q in the body frame.
   * p_WQ should lie within a bounding box in the specified `constraint_frame`.
   * Notice that since the body_rotmat does not lie exactly on the SO(3), due
   * to the McCormick envelope relaxation, this constraint is subject to the
   * accumulated error from the root of the kinematics tree.
   * @param body_idx The index of the body on which the position of a point is
   * constrained.
   * @param body_pt The position of the point measured and expressed in the body
   * frame.
   * @param box_lb The lower bound of the box in frame `constraint_frame`.
   * @param box_ub The upper bound of the box in frame `constraint_frame`.
   * @param constraint_frame. The frame in which the box is specified. This
   * frame is represented by an isometry transform T_WF, the transform from
   * the constraint frame F to the world frame W. Namely if the position of
   * `body_pt` in the world frame is x, then the constraint is
   * box_lb <= T_WF.linear().transpose() * (x - T_WF.translation()) <= box_ub.
   * @default is the identity transform.
   */
  void AddWorldPositionConstraint(int body_idx, const Eigen::Vector3d& body_pt,
                                  const Eigen::Vector3d& box_lb,
                                  const Eigen::Vector3d& box_ub,
                                  const Eigen::Isometry3d& constraint_frame =
                                      Eigen::Isometry3d::Identity());

  /**
   * Add a constraint that the angle between the body orientation and the
   * desired orientation should not be larger than `angle_tol`. The angle
   * between two rotation matrix R1 and R2 is computed as
   * AngleAxis(R1'*R2).angle(). Namely the angle of the rotation matrix
   * R1'*R2.
   * The actual constraint imposed is on cos(angle_tol). The math is
   * <pre>
   * 2 * cos(angle_tol) + 1 <= trace(R_error) <= 3
   * </pre>
   * where R_error is the error rotation matrix, between the body orientation
   * and the desired orientation.
   * Suppose in the angle axis representation for R_error, the angle is θ, it is
   * easy to prove that trace(R_error) = 2 * cos(θ) + 1.
   * @param body_idx The index of the body whose orientation will be
   * constrained.
   * @param desired_orientation The desired orientation of the body.
   * @param angle_tol The tolerance on the angle between the body orientation
   * and the desired orientation. Unit is radians.
   */
  void AddWorldOrientationConstraint(
      int body_idx, const Eigen::Quaterniond& desired_orientation,
      double angle_tol);

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
