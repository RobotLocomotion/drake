#pragma once

#include <memory>

#include "drake/multibody/inverse_kinematics/kinematic_constraint.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace multibody {
// Solves an inverse kinematics (IK) problem on a MultibodyTree, to find the
// postures of the robot satisfying certain constraint.
// The decision variables include the generalized position of the robot. The
// bounds on the generalized positions (i.e., joint limits) are imposed
// automatially.
// Internally this class creates and stores a MultibodyTreeContext, which will
// cache the kinematic result for a posture. So when the user adds kinematic
// constraint, he/she should construct the kinematic constraint using the
// MultibodyTreeContext stored inside this class, accessed by
// `get_mutable_context()` function.
class InverseKinematics : public solvers::MathematicalProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InverseKinematics)

  ~InverseKinematics() override {}

  /**
   * Constructs an inverse kinematics problem for a MultibodyTree.
   * @param MultibodyTree The robot on which the inverse kinematics problem is
   * solved.
   */
  explicit InverseKinematics(const MultibodyTree<AutoDiffXd>& tree);

  MultibodyTreeContext<AutoDiffXd>* get_mutable_context() {
    return dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(context_.get());
  }

  /** Adds the kinematic constraint that a point Q, fixed in frame B, should lie
   * within a bounding box expressed in frame A as p_AQ_lower <= p_AQ <=
   * p_AQ_upper, where p_AQ is the position of point Q measured and expressed
   * in frame A.
   * @param frameB_idx The index of frame B.
   * @param p_BQ The position of the point Q, rigidly attached to frame B,
   * measured and expressed in frame A.
   * @param frameA_idx The index of frame A.
   * @param p_AQ_lower The lower bound on the position of point Q, measured and
   * expressed in frame A.
   * @param p_AQ_upper The upper bound on the position of point Q, measured and
   * expressed in frame A.
   */
  solvers::Binding<PositionConstraint> AddPositionConstraint(
      const FrameIndex& frameB_idx,
      const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
      const FrameIndex& frameA_idx,
      const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
      const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper);

  /**
   * Constrains that the angle difference θ between the orientation of frame A
   * and the orientation of frame B is satisfied 0 ≤ θ ≤ θ_bound. The angle
   * difference between frame A's orientation R_WA and B's orientation R_WB is θ
   * if there exists a rotation axis a, such that by rotating frame A about axis
   * a by angle θ, the frame A after rotation aligns with frame B. Namely θ =
   * AngleAxisd(R_AB).angle(). where R_AB is the orientation of frame A measured
   * and expressed in frame B. If the users wants frame A and frame B to align
   * perfectly, they can set θ_bound = 0.
   * @param frameA_idx The index of frame A.
   * @param frameB_idx The index of frame B.
   * @param angle_bound The bound on the angle difference between frame A's
   * orientation and frame B's orientation. It is denoted as θ_bound in the
   * class documentation.
   */
  solvers::Binding<OrientationConstraint> AddOrientationConstraint(
      const FrameIndex& frameA_idx, const FrameIndex& frameB_idx,
      double angle_bound);

  /**
   * Constraints that a point T is contained within a cone K. The point T is
   * fixed in a frame B. The cone originates from a point S fixed in frame A,
   * with the unit length directional vector of the cone being n, also fixed in
   * frame A. The half angle of the cone is θ. A common usage of this constraint
   * is that a camera should gaze at some target; namely the target falls within
   * a gaze cone, originating from the camera eye.
   * @param frameA_idx The frame where the gaze cone is fixed to.
   * @param p_AS The position of the cone source point S, measured and
   * expressed in frame A.
   * @param n_A The directional vector representing the center ray of the
   * cone.
   * @pre @p n_A cannot be a zero vector. @throw a logic error is n_A is close
   * to a zero vector.
   * @param frameB_idx The frame where the target point T is fixed to.
   * @param p_BT The position of the target point T, measured and expressed in
   * frame B.
   * @param cone_half_angle The half angle of the cone. We denote it as θ in the
   * class documentation. @pre @p 0 <= cone_half_angle <= pi. @throw a logic
   * error if cone_half_angle is outside of the bound.
   */
  solvers::Binding<GazeTargetConstraint> AddGazeTargetConstraint(
      const FrameIndex& frameA_idx,
      const Eigen::Ref<const Eigen::Vector3d>& p_AS,
      const Eigen::Ref<const Eigen::Vector3d>& n_A,
      const FrameIndex& frameB_idx,
      const Eigen::Ref<const Eigen::Vector3d>& p_BT, double cone_half_angle);

  /** Getter for q. q is the decision variable for the generalized positions of
   * the robot. */
  const solvers::VectorXDecisionVariable& q() const { return q_; }

 private:
  const MultibodyTree<AutoDiffXd>& tree_;
  std::unique_ptr<systems::LeafContext<AutoDiffXd>> const context_;
  solvers::VectorXDecisionVariable q_;
};
}  // namespace multibody
}  // namespace drake
