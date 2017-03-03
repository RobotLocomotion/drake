#pragma once
#include <vector>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace multibody {
/**
 * Solves the inverse kinematics problem as a mixed integer convex optimization
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

  /** Getter for the decision variables on the rotation matrices of each body.
   */
  const std::vector<solvers::MatrixDecisionVariable<3, 3>>& body_rotmat()
      const {
    return body_rotmat_;
  }

  /** Getter for the decision variables on the position of each body. */
  const std::vector<solvers::VectorDecisionVariable<3>>& body_pos() const {
    return body_pos_;
  }

  /**
   * After solving the inverse kinematics problem and find out the pose of each
   * body, reconstruct the robot posture (joint angles, etc) that matches with
   * the body poses.
   * Notice that since the rotation matrix is approximated, that
   * the solution of body_rotmat might not be on so(3) exactly, the
   * reconstructed body posture might not match with the body poses exactly, and
   * the kinematics constraint might not be satisfied exactly with this
   * reconstructed posture.
   * @return The reconstructed posture.
   */
  Eigen::VectorXd ReconstructPostureSolution() const;

  /**
   * Adds the constraint that position of a point `body_pt` on a body
   * (whose index is `body_idx`), is within a box in a specified frame.
   * where the inequality is elementwise.
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
