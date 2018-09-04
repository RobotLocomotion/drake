#pragma once

#include <memory>

#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace multibody {
namespace internal {
/**
 * Constrains that the angle between a vector n_A and another vector n_B is
 * between [θ_lower, θ_upper]. n_A is fixed to a frame A, while n_B is fixed to
 * a frame B.
 * Mathematically, if we denote n_A_A as n_A measured and expressed in frame A
 * after normalization (n_A_A has unit length), and n_B_B as n_B measured and
 * expressed in frame B after normalization, the constraint is
 * cos(θ_upper) ≤ n_A_Aᵀ * R_AB * n_B_B ≤ cos(θ_lower)
 */
class AngleBetweenVectorsConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AngleBetweenVectorsConstraint)

  /**
   * @param tree The MultibodyTree on which the constraint is imposed. @p tree
   * should be alive during the lifetime of this constraint.
   * @param frameA_idx The index of frame A.
   * @param n_A The vector n_A fixed to frame A, measured and expressed in frame
   * A. @pre n_A should be a non-zero vector. @throw logic error if n_A is
   * close to zero.
   * @param frameB_idx The index of frame B.
   * @param n_B The vector n_A fixed to frame B, measured and expressed in frame
   * B. @pre n_B should be a non-zero vector. @throw logic error if n_B is
   * close to zero.
   * @param angle_lower The lower bound on the angle between n_A and n_B. It is
   * denoted as θ_lower in the class documentation. @pre angle_lower >= 0.
   * @throw a logic error if angle_lower is negative.
   * @param angle_upper The upper bound on the angle between n_A and n_B. it is
   * denoted as θ_upper in the class documentation. @pre angle_lower <=
   * angle_upper <= pi. @throw a logic error if angle_upper is outside the
   * bounds.
   * @param context The Context that has been allocated for this @p tree. We
   * will update the context when evaluating the constraint. @p context should
   * be alive during the lifetime of this constraint.
   */
  AngleBetweenVectorsConstraint(const MultibodyTree<AutoDiffXd>& tree,
                                const FrameIndex& frameA_idx,
                                const Eigen::Ref<const Eigen::Vector3d>& n_A,
                                const FrameIndex& frameB_idx,
                                const Eigen::Ref<const Eigen::Vector3d>& n_B,
                                double angle_lower, double angle_upper,
                                MultibodyTreeContext<AutoDiffXd>* context);

  ~AngleBetweenVectorsConstraint() override {}

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
        "AngleBetweenVectorsConstraint::DoEval() does not work for symbolic "
        "variables.");
  }
  const MultibodyTree<AutoDiffXd>& tree_;
  const Frame<AutoDiffXd>& frameA_;
  const Eigen::Vector3d n_A_A_;
  const Frame<AutoDiffXd>& frameB_;
  const Eigen::Vector3d n_B_B_;
  MultibodyTreeContext<AutoDiffXd>* context_;
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake

