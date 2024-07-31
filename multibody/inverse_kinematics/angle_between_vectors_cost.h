#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/inverse_kinematics/angle_between_vectors_constraint.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/cost.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
/**
 * Implements a cost of the form c*(1-cosθ), where θ is the angle between two
 * vectors `a` and `b`. `c` is a constant scalar.
 * @ingroup solver_evaluators
 */
class AngleBetweenVectorsCost : public solvers::Cost {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AngleBetweenVectorsCost);

  /**
   * Constructs an AngleBetweenVectorsCost.
   * @param plant The MultibodyPlant on which the cost is imposed. `plant`
   *   should be alive during the lifetime of this cost.
   * @param frameA The Frame object for frame A.
   * @param a_A The vector `a` fixed to frame A, expressed in frame A.
   * @param frameB The Frame object for frame B.
   * @param b_B The vector `b` fixed to frame B, expressed in frameB.
   * @param c The cost is c*(1-cosθ).
   * @param plant_context The Context that has been allocated for this
   *   `plant`.  We will update the context when evaluating the constraint.
   *   `plant_context` should be alive during the lifetime of this cost.
   * @pre `frameA` and `frameB` must belong to `plant`.
   * @throws std::exception if `plant` is nullptr.
   * @throws std::exception if `a_A` is close to zero.
   * @throws std::exception if `b_B` is close to zero.
   * @throws std::exception if `plant_context` is nullptr.
   * @pydrake_mkdoc_identifier{double}
   */
  AngleBetweenVectorsCost(const MultibodyPlant<double>* plant,
                          const Frame<double>& frameA,
                          const Eigen::Ref<const Eigen::Vector3d>& a_A,
                          const Frame<double>& frameB,
                          const Eigen::Ref<const Eigen::Vector3d>& b_B,
                          double c, systems::Context<double>* plant_context);

  /**
   * Overloaded constructor. Use MultibodyPlant<AutoDiffXd> instead of
   * MultibodyPlant<double>.
   * @pydrake_mkdoc_identifier{autodiff}
   */
  AngleBetweenVectorsCost(const MultibodyPlant<AutoDiffXd>* plant,
                          const Frame<AutoDiffXd>& frameA,
                          const Eigen::Ref<const Eigen::Vector3d>& a_A,
                          const Frame<AutoDiffXd>& frameB,
                          const Eigen::Ref<const Eigen::Vector3d>& b_B,
                          double c,
                          systems::Context<AutoDiffXd>* plant_context);

  ~AngleBetweenVectorsCost() override;

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override;

  // We don't really impose constraint, but instead we will re-use the math
  // implemented in AngleBetweenVectorsConstraint to evaluate this cost.
  const AngleBetweenVectorsConstraint constraint_;
  double c_;
};
}  // namespace multibody
}  // namespace drake
