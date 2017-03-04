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
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GlobalInverseKinematics)

  /**
   * Parses the robot kinematics tree. The decision variables include the
   * pose for each body (position/orientation).
   * @param robot The robot on which the inverse kinematics problem is solved.
   * @param num_binary_vars_per_half_axis The number of binary variable for
   * each half axis, to segment the unit circle.
   * @see AddRotationMatrixMcCormickEnvelopeMilpConstraints for more details.
   */
  GlobalInverseKinematics(const RigidBodyTreed& robot,
                          int num_binary_vars_per_half_axis = 2);

  ~GlobalInverseKinematics() override {}

  /** Getter for the decision variables on the rotation matrix for body with
   * the specified index.
   * @param body_index  The index of the queried body. Notice that body 0 is
   * the world, and thus not a decision variable. Throws a runtime error if
   * the index is smaller than 1, or no smaller than the total number of bodies
   * in the robot.
   */
  const solvers::MatrixDecisionVariable<3, 3>& body_rotmat(int body_index)
      const;

  /** Getter for the decision variables on the position of the body with the
   * specified index.
   * @param body_index  The index of the queried body. Notice that body 0 is
   * the world, and thus not a decision variable. Throws a runtime error if
   * the index is smaller than 1, or no smaller than the total number of bodies
   * in the robot.
   */
  const solvers::VectorDecisionVariable<3>& body_pos(int body_index) const;

  /**
   * After solving the inverse kinematics problem and find out the pose of each
   * body, reconstruct the robot posture (joint angles, etc) that matches with
   * the body poses. Notice that since the rotation matrix is approximated, that
   * the solution of body_rotmat might not be on so(3) exactly, the
   * reconstructed body posture might not match with the body poses exactly, and
   * the kinematics constraint might not be satisfied exactly with this
   * reconstructed posture.
   * Do not call this method if the problem is not solved successfully!
   * @return The reconstructed posture.
   */
  Eigen::VectorXd ReconstructPostureSolution() const;

  /**
   * Adds the constraint that position of a point `body_pt` on a body
   * (whose index is `body_idx`), is within a box in a specified frame.
   * where the inequality is elementwise. The constrain is that the point
   * position, computed as
   *   x = body_pos + body_rotmat * body_pt
   * should lie within a bounding box in the specified `measured_frame`.
   * Notice that since the body_rotmat does not lie exactly on the so(3), due
   * to the McCormick envelope relaxation, this constraint is subject to the
   * accumulated error from the root of the kinematics tree.
   * @param body_idx The index of the body on which the position of a point is
   * constrained.
   * @param body_pt The position of the point measured in the body `body_idx`.
   * @param box_lb The lower bound of the box.
   * @param box_ub The upper bound of the box.
   * @param measured_frame. The frame in which the box is specified. Namely if
   * the position of `body_pt` in the world frame is x, then the constraint
   * is box_lb <= measured_transform.linear().transpose() * (x -
   * measured_transform.translation()) <= box_ub. @default is the identity
   * transform.
   */
  void AddWorldPositionConstraint(
      int body_idx, const Eigen::Vector3d& body_pt,
      const Eigen::Vector3d& box_lb, const Eigen::Vector3d& box_ub,
      const Eigen::Isometry3d& measured_frame = Eigen::Isometry3d::Identity());

  /**
   * Add a constraint that the angle between the body orientation and the
   * desired orientation should not be larger than `angle_tol`.
   * The actual constraint imposed is on cos(angle_tol). The math is
   * 2 * cos(angle_tol) + 1 <= trace(R_error) <= 3
   * where R_error is the error rotation matrix, between the body orientation
   * and the desired orientation.
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

  // body_rotmat_[i] is the rotation matrix for body i.
  std::vector<solvers::MatrixDecisionVariable<3, 3>> body_rotmat_;

  // body_pos_[i] is the position for body i.
  std::vector<solvers::VectorDecisionVariable<3>> body_pos_;
};
}  // namespace multibody
}  // namespace drake
