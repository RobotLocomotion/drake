#pragma once

#include <memory>

#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace multibody {
namespace internal {
// Constrains the position of a point Q, rigidly attached to a frame B, to be
// within a bounding box measured and expressed in frame A. Namely
// p_AQ_lower <= p_AQ <= p_AQ_upper.
class PositionConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PositionConstraint)

  // @param tree The multibody tree on which the constraint is imposed. @p tree
  // should be alive during the whole lifetime of this constraint.
  // @param frameB_idx The index of frame B.
  // @param p_BQ The position of the point Q, rigidly attached to frame B,
  // measured and expressed in frame B.
  // @param frameA_idx The index of frame A.
  // @param p_AQ_lower The lower bound on the position of point Q, measured and
  // expressed in frame A.
  // @param p_AQ_upper The upper bound on the position of point Q, measured and
  // expressed in frame A.
  // @param context The Context that has been allocated for this @p tree. We
  // will update the context when evaluating the constraint. @p context should
  // be alive during the lifetime of this constraint.
  // TODO(hongkai.dai): use MultibodyTree<double> and LeafContext<double> when
  // MBT provides the API for computing analytical Jacobian.
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

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "PositionConstraint::DoEval() does not work for symbolic variables.");
  }

  const MultibodyTree<AutoDiffXd>& tree_;
  const Frame<AutoDiffXd>& frameB_;
  const Frame<AutoDiffXd>& frameA_;
  const Eigen::Vector3d p_BQ_;
  MultibodyTreeContext<AutoDiffXd>* const context_;
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
