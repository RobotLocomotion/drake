#pragma once

#include "drake/multibody/inverse_kinematics/orientation_constraint.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/cost.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
/**
 * Implements a cost of the form `c * (1 - cos(θ))`, where θ is the angle
 * between the orientation of frame A and the orientation of frame B, and `c`
 * is a cost scaling.
 *
 * @ingroup solver_evaluators
 */
class OrientationCost : public solvers::Cost {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OrientationCost);

  /**
   * Constructs OrientationCost object.
   * @param plant The MultibodyPlant on which the cost is implemented. `plant`
   *   should be alive during the lifetime of this cost.
   * @param frameAbar A frame on the MultibodyPlant.
   * @param R_AbarA The rotation matrix describing the orientation of frame A
   * relative to Abar.
   * @param frameBbar A frame on the MultibodyPlant.
   * @param R_BbarB The rotation matrix describing the orientation of frame B
   * relative to Bbar.
   * @param c A scalar cost weight.
   * @param plant_context A context for the `plant`.
   *
   * @throws std::exception if `plant` is nullptr.
   * @throws std::exception if `plant_context` is nullptr.
   * @pydrake_mkdoc_identifier{double}
   */
  OrientationCost(const MultibodyPlant<double>* plant,
                  const Frame<double>& frameAbar,
                  const math::RotationMatrix<double>& R_AbarA,
                  const Frame<double>& frameBbar,
                  const math::RotationMatrix<double>& R_BbarB, double c,
                  systems::Context<double>* plant_context);

  /**
   * Overloaded constructor. Same as the constructor with the double version
   * (using MultibodyPlant<double> and Context<double>). Except the gradient of
   * the cost is computed from autodiff.
   * @pydrake_mkdoc_identifier{autodiff}
   */
  OrientationCost(const MultibodyPlant<AutoDiffXd>* plant,
                  const Frame<AutoDiffXd>& frameAbar,
                  const math::RotationMatrix<double>& R_AbarA,
                  const Frame<AutoDiffXd>& frameBbar,
                  const math::RotationMatrix<double>& R_BbarB, double c,
                  systems::Context<AutoDiffXd>* plant_context);

  ~OrientationCost() override;

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override;

  const OrientationConstraint constraint_;
  const double c_;
};
}  // namespace multibody
}  // namespace drake
