#pragma once

#include <memory>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace multibody {
/**
 * Constrain the position of points P1, P2, ..., Pn to satisfy the constraint A
 * * [p_FP1; p_FP2; ...; p_FPn] <= b, where p_FPi is the position of point Pi
 * measured and expressed in frame F. Notice the constraint is imposed on the
 * stacked column vector [p_FP1; p_FP2; ...; p_FPn], not on each individual
 * point.
 * @ingroup solver_evaluators
 */
class PolyhedronConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PolyhedronConstraint);

  /**
   * Construct the constraint that the position of P1, ..., Pn satisfy A *
   * [p_FP1; p_FP2; ...; p_FPn] <= b.
   * @param plant The MultibodyPlant on which the constraint is imposed.
   * `plant` should be alive during the lifetime of this constraint.
   * @param frameF The frame in which the position P is measured and expressed
   * @param frameG The frame in which the point P is rigidly attached.
   * @param p_GP p_GP.col(i) is the position of the i'th point Pi measured and
   * expressed in frame G.
   * @param A We impose the constraint A * [p_FP1; p_FP2; ...; p_FPn] <= b. @pre
   * A.cols() = 3 * p_GP.cols();
   * @param b We impose the constraint A * [p_FP1; p_FP2; ...; p_FPn] <= b
   * @param plant_context The Context that has been allocated for this
   * `plant`.  We will update the context when evaluating the constraint.
   * `plant_context` should be alive during the lifetime of this constraint.
   * @pydrake_mkdoc_identifier{double}
   */
  PolyhedronConstraint(const MultibodyPlant<double>* plant,
                       const Frame<double>& frameF, const Frame<double>& frameG,
                       const Eigen::Ref<const Eigen::Matrix3Xd>& p_GP,
                       const Eigen::Ref<const Eigen::MatrixXd>& A,
                       const Eigen::Ref<const Eigen::VectorXd>& b,
                       systems::Context<double>* plant_context);

  /**
   * Overloaded constructor. Same as the constructor with the double version
   * (using MultibodyPlant<double> and Context<double>). Except the gradient of
   * the constraint is computed from autodiff.
   * @pydrake_mkdoc_identifier{autodiff}
   */
  PolyhedronConstraint(const MultibodyPlant<AutoDiffXd>* plant,
                       const Frame<AutoDiffXd>& frameF,
                       const Frame<AutoDiffXd>& frameG,
                       const Eigen::Ref<const Eigen::Matrix3Xd>& p_GP,
                       const Eigen::Ref<const Eigen::MatrixXd>& A,
                       const Eigen::Ref<const Eigen::VectorXd>& b,
                       systems::Context<AutoDiffXd>* plant_context);

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "PolyhedronConstraint::DoEval() does not work for symbolic variables.");
  }

  bool use_autodiff() const { return plant_autodiff_; }

  const MultibodyPlant<double>* const plant_double_;
  const FrameIndex frameF_index_;
  const FrameIndex frameG_index_;
  const Eigen::Matrix3Xd p_GP_;
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
  systems::Context<double>* const context_double_;

  const MultibodyPlant<AutoDiffXd>* const plant_autodiff_;
  systems::Context<AutoDiffXd>* const context_autodiff_;
};
}  // namespace multibody
}  // namespace drake
