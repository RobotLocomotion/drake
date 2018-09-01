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
