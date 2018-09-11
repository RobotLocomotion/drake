#pragma once

#include <memory>

#include "drake/math/rotation_matrix.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace multibody {
namespace internal {
// Constrains that the angle difference θ between the orientation of frame A
// and the orientation of frame B to satisfy θ ≤ θ_bound. The angle
// difference between frame A's orientation R_WA and B's orientation R_WB is θ
// (θ ∈ [0, π]), if there exists a rotation axis a, such that rotating frame
// A by angle θ about axis a aligns it with frame B. Namely
// R_AB = I + sinθ â + (1-cosθ)â²   (1)
// where R_AB is the orientation of frame B expressed in frame A. â is the skew
// symmetric matrix of the rotation axis a. Equation (1) is the Rodrigues
// formula that computes the rotation matrix froma rotation axis a and an angle
// θ, https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
// If the users want frame A and frame B to align perfectly, they can set
// θ_bound = 0.
// Mathematically, this constraint is imposed as
// trace(R_AB) ≥ 2cos(θ_bound) + 1   (1)
// To derive (1), using Rodrigues formula
// R_AB = I + sinθ â + (1-cosθ)â²
// where
// trace(R_AB) = 2cos(θ) + 1 ≥ 2cos(θ_bound) + 1
class OrientationConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OrientationConstraint)

  // The frame A is fixed to a frame A̅, with orientatation R_AbarA measured
  // in frame A̅. The frame B is fixed to a frame B̅, with orientation R_BbarB
  // measured in frame B. We constrain the angle between frame A and B to be
  // less than theta_bound.
  // @param tree The MultibodyTree on which the constraint is imposed. @p tree
  // should be alive during the lifetime of this constraint.
  // @param frameAbar_idx The index of frame A̅.
  // @param R_AbarA The orientation of frame A measured in frame A̅.
  // @param frameBbar_idx The index of frame B̅.
  // @param R_BbarB The orientation of frame B measured in frame B̅.
  // @param theta_bound The bound on the angle difference between frame A's
  // orientation and frame B's orientation. It is denoted as θ_bound in the
  // class documentation. @p theta_bound is in radians.
  // @pre angle_bound >= 0.
  // @throw invalid_argument if angle_bound < 0.
  // @param context The Context that has been allocated for this @p tree. We
  // will update the context when evaluating the constraint. @p context should
  // be alive during the lifetime of this constraint.
  OrientationConstraint(const MultibodyTree<AutoDiffXd>& tree,
                        FrameIndex frameAbar_idx,
                        const math::RotationMatrix<double>& R_AbarA,
                        FrameIndex frameBbar_idx,
                        const math::RotationMatrix<double>& R_BbarB,
                        double theta_bound,
                        MultibodyTreeContext<AutoDiffXd>* context);

  ~OrientationConstraint() override {}

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "OrientationConstraint::DoEval() does not work for symbolic "
        "variables.");
  }

  const MultibodyTree<AutoDiffXd>& tree_;
  const Frame<AutoDiffXd>& frameAbar_;
  const Matrix3<AutoDiffXd> R_AbarA_;
  const Frame<AutoDiffXd>& frameBbar_;
  const Matrix3<AutoDiffXd> R_BbarB_;
  MultibodyTreeContext<AutoDiffXd>* const context_;
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
