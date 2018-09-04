#pragma once

#include <memory>

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
  const Frame<AutoDiffXd>& frameB_;
  const Frame<AutoDiffXd>& frameA_;
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
 * trace(R_AB) = 2cos(θ) + 1 ≥ 2cos(θ_bound) + 1
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
        "OrientationConstraint::DoEval() does not work for symbolic "
        "variables.");
  }

  const MultibodyTree<AutoDiffXd>& tree_;
  const Frame<AutoDiffXd>& frameA_;
  const Frame<AutoDiffXd>& frameB_;
  MultibodyTreeContext<AutoDiffXd>* const context_;
};

/**
 * Constraints that a point T is contained within a cone K. The point T is
 * fixed in a frame B. The cone originates from a point S fixed in frame A, with
 * the unit length directional vector of the cone being n, also fixed in frame
 * A. The half angle of the cone is θ. A common usage of this constraint is that
 * a camera should gaze at some target; namely the target falls within a gaze
 * cone, originating from the camera eye.
 *
 * Mathematically the constraint is
 * p_ST_Aᵀ * n_A ≥ 0
 * (p_ST_Aᵀ * n_A)² ≥ (cosθ)²p_ST_Aᵀ * p_ST_A
 * where p_ST_A is the vector from S to T, expressed in frame A. n_A is the unit
 * length directional vector representing the center ray of the cone.
 */
class GazeTargetConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GazeTargetConstraint)

  /**
   * @param tree The MultibodyTree on which the constraint is imposed. @p tree
   * should be alive during the lifetime of this constraint.
   * @param frameA_idx The frame where the gaze cone is fixed to.
   * @param p_AS The position of the cone source point S, measured and expressed
   * in frame A.
   * @param n_A The directional vector representing the center ray of the cone.
   * @pre @p n_A cannot be a zero vector. @throw a logic error is n_A is close
   * to a zero vector.
   * @param frameB_idx The frame where the target point T is fixed to.
   * @param p_BT The position of the target point T, measured and expressed in
   * frame B.
   * @param cone_half_angle The half angle of the cone. We denote it as θ in the
   * class documentation. @pre @p 0 <= cone_half_angle <= pi. @throw a logic
   * error if cone_half_angle is outside of the bound.
   * @param context The Context that has been allocated for this @p tree. We
   * will update the context when evaluating the constraint. @p context should
   * be alive during the lifetime of this constraint.
   */
  GazeTargetConstraint(const MultibodyTree<AutoDiffXd>& tree,
                       const FrameIndex& frameA_idx,
                       const Eigen::Ref<const Eigen::Vector3d>& p_AS,
                       const Eigen::Ref<const Eigen::Vector3d>& n_A,
                       const FrameIndex& frameB_idx,
                       const Eigen::Ref<const Eigen::Vector3d>& p_BT,
                       double cone_half_angle,
                       MultibodyTreeContext<AutoDiffXd>* context);

  ~GazeTargetConstraint() override{};

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
        "GazeTargetConstraint::DoEval() does not work for symbolic variables.");
  }
  const MultibodyTree<AutoDiffXd>& tree_;
  const Frame<AutoDiffXd>& frameA_;
  const Eigen::Vector3d p_AS_;
  const Eigen::Vector3d n_A_;
  const Frame<AutoDiffXd>& frameB_;
  const Vector3<AutoDiffXd> p_BT_;
  const double cone_half_angle_;
  const double cos_cone_half_angle_;
  MultibodyTreeContext<AutoDiffXd>* const context_;
};

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
}  // namespace multibody
}  // namespace drake
