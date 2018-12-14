#pragma once

#include <memory>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace multibody {
namespace internal {
/**
 * Constrains that the angle between a vector na and another vector nb is
 * between [θ_lower, θ_upper]. na is fixed to a frame A, while nb is fixed
 * to a frame B.
 * Mathematically, if we denote na_unit_A as na expressed in frame A after
 * normalization (na_unit_A has unit length), and nb_unit_B as nb expressed in
 * frame B after normalization, the constraint is
 * cos(θ_upper) ≤ na_unit_Aᵀ * R_AB * nb_unit_B ≤ cos(θ_lower)
 */
class AngleBetweenVectorsConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AngleBetweenVectorsConstraint)

  /**
   * Constructs an AngleBetweenVectorsConstraint.
   * @param tree The MultibodyTree on which the constraint is imposed. @p tree
   * should be alive during the lifetime of this constraint.
   * @param frameA_idx The index of frame A.
   * @param na_A The vector na_A fixed to frame A, expressed in frame A.
   * @pre na_A should be a non-zero vector.
   * @throws std::logic_error if na_A is close to zero.
   * @param frameB_idx The index of frame B.
   * @param nb_B The vector nb fixed to frame B, expressed in frameB.
   * @pre nb_B should be a non-zero vector.
   * @throws std::logic_error if nb_B is close to zero.
   * @param angle_lower The lower bound on the angle between na and nb. It is
   * denoted as θ_lower in the class documentation.
   * @pre angle_lower >= 0.
   * @throws std::invalid_argument error if angle_lower is negative.
   * @param angle_upper The upper bound on the angle between na and nb. it is
   * denoted as θ_upper in the class documentation.
   * @pre angle_lower <= angle_upper <= pi.
   * @throws std::invalid_argument if angle_upper is outside the bounds.
   * @param context The Context that has been allocated for this @p tree. We
   * will update the context when evaluating the constraint. @p context should
   * be alive during the lifetime of this constraint.
   */
  AngleBetweenVectorsConstraint(
      const multibody_plant::MultibodyPlant<double>& plant,
      const Frame<double>& frameA,
      const Eigen::Ref<const Eigen::Vector3d>& na_A,
      const Frame<double>& frameB,
      const Eigen::Ref<const Eigen::Vector3d>& nb_B, double angle_lower,
      double angle_upper, systems::Context<double>* context);

  ~AngleBetweenVectorsConstraint() override {}

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "AngleBetweenVectorsConstraint::DoEval() does not work for symbolic "
        "variables.");
  }
  const multibody_plant::MultibodyPlant<double>& plant_;
  const Frame<double>& frameA_;
  const Eigen::Vector3d na_unit_A_;
  const Frame<double>& frameB_;
  const Eigen::Vector3d nb_unit_B_;
  systems::Context<double>* const context_;
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
