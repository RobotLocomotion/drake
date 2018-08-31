#pragma once

#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace multibody {
// Constrains the position of a point Q, rigidly attached to a frame B, to be
// within a bounding box measured and expressed in frame A. Namely
// p_AQ_lower <= p_AQ <= p_AQ_upper.
class PositionConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PositionConstraint)

  /**
   * @param tree The multibody tree on which the constraint is imposed. @p tree
   * should be alive during the whole lifetime of this constraint.
   * @param frameB_idx The index of frame B.
   * @param p_BQ The position of the point Q, rigidly attached to frame B,
   * measured and expressed in frame A.
   * @param frameA_idx The index of frame A.
   * @param p_AQ_lower The lower bound on the position of point Q, measured and
   * expressed in frame A.
   * @param p_AQ_upper The upper bound on the position of point Q, measured and
   * expressed in frame A.
   * @param context The Context that has been allocated for this @p tree. We
   * will update the context when evaluating the constraint. @p context should
   * be alive during the lifetime of this constraint.
   */
  PositionConstraint(const multibody::MultibodyTree<AutoDiffXd>& tree,
                     const FrameIndex& frameB_idx,
                     const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
                     const FrameIndex& frameA_idx,
                     const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
                     const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
                     MultibodyTreeContext<AutoDiffXd>* context);

  ~PositionConstraint() override {}

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override {
    unused(x);
    unused(y);
    throw std::logic_error(
        "PositionConstraint::DoEval() does not work for symbolic variables.");
  }

  const MultibodyTree<AutoDiffXd>& tree_;
  const FrameIndex frameB_idx_;
  const FrameIndex frameA_idx_;
  const Eigen::Vector3d p_BQ_;
  MultibodyTreeContext<AutoDiffXd>* const context_;
};

/**
 * Constrains that the angle difference θ between the orientation of frame A and
 * the orientation of frame B is satisfied 0 ≤ θ ≤ θ_bound. The angle
 * difference between frame A's orientation R_WA and B's orientation R_WB is θ
 * if there exists a rotation axis a, such that by rotating frame A about axis
 * a by angle θ, the frame A after rotation aligns with frame B. Namely θ =
 * AngleAxisd(R_AB).angle(). where R_AB is the orientation of frame A measured
 * and expressed in frame B. If the users wants frame A and frame B to align
 * perfectly, they can set θ_bound = 0.
 * Mathematically, this constraint is imposed as
 * trace(R_AB) ≥ 2cos(θ_bound) - 1 (1)
 * To derive (1), using Rodriguez formula
 * https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
 * R_AB = I + sinθ â + (1-cosθ)â²
 * where â is the skew symmetric matrix of the rotation axis a.
 * trace(R_AB) = 2cos(θ) - 1 ≥ 2cos(θ_bound) - 1
 */
class OrientationConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OrientationConstraint)

  /**
   * @param tree The MultibodyTree on which the constraint is imposed. @p tree
   * should be alive during the lifetime of this constraint.
   * @param frameA_idx The index of frame A.
   * @param frameB_idx The index of frame B.
   * @param angle_bound The bound on the angle difference between frame A's
   * orientation and frame B's orientation. It is denoted as θ_bound in the
   * class documentation. @pre angle_bound >= 0. @throw a logic error if
   * angle_bound < 0.
   * @param context The Context that has been allocated for this @p tree. We
   * will update the context when evaluating the constraint. @p context should
   * be alive during the lifetime of this constraint.
   */
  OrientationConstraint(const MultibodyTree<AutoDiffXd>& tree,
                        const FrameIndex& frameA_idx,
                        const FrameIndex& frameB_idx, double angle_bound,
                        MultibodyTreeContext<AutoDiffXd>* context);

  ~OrientationConstraint() override {}
 
 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override {
    unused(x);
    unused(y);
    throw std::logic_error(
        "PositionConstraint::DoEval() does not work for symbolic variables.");
  }

  const MultibodyTree<AutoDiffXd>& tree_;
  const FrameIndex frameA_idx_;
  const FrameIndex frameB_idx_;
  MultibodyTreeContext<AutoDiffXd>* const context_;
};
}  // namespace multibody
}  // namespace drake
