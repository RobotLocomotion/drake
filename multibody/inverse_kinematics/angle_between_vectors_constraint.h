#pragma once

#include <memory>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace multibody {
/**
 * Constrains that the angle between a vector `a` and another vector `b` is
 * between [θ_lower, θ_upper]. `a` is fixed to a frame A, while `b` is fixed to
 * a frame B.
 * Mathematically, if we denote a_unit_A as `a` expressed in frame A after
 * normalization (a_unit_A has unit length), and b_unit_B as `b` expressed in
 * frame B after normalization, the constraint is
 *   cos(θ_upper) ≤ a_unit_Aᵀ * R_AB * b_unit_B ≤ cos(θ_lower)
 */
class AngleBetweenVectorsConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AngleBetweenVectorsConstraint)

  /**
   * Constructs an AngleBetweenVectorsConstraint.
   * @param plant The MultibodyPlant on which the constraint is imposed. `plant`
   *   should be alive during the lifetime of this constraint.
   * @param frameA The Frame object for frame A.
   * @param a_A The vector `a` fixed to frame A, expressed in frame A.
   * @param frameB The Frame object for frame B.
   * @param b_B The vector `b` fixed to frame B, expressed in frameB.
   * @param angle_lower The lower bound on the angle between `a` and `b`. It is
   *   denoted as θ_lower in the class documentation.
   * @param angle_upper The upper bound on the angle between `a` and `b`. it is
   *   denoted as θ_upper in the class documentation.
   * @param plant_context The Context that has been allocated for this
   *   `plant`.  We will update the context when evaluating the constraint.
   *   `plant_context` should be alive during the lifetime of this constraint.
   * @pre `frameA` and `frameB` must belong to `plant`.
   * @throws std::invalid_argument if `plant` is nullptr.
   * @throws std::invalid_argument if `a_A` is close to zero.
   * @throws std::invalid_argument if `b_B` is close to zero.
   * @throws std::invalid_argument if `angle_lower` is negative.
   * @throws std::invalid_argument if `angle_upper` ∉ [`angle_lower`, π].
   * @throws std::invalid_argument if `plant_context` is nullptr.
   */
  AngleBetweenVectorsConstraint(const MultibodyPlant<double>* plant,
                                const Frame<double>& frameA,
                                const Eigen::Ref<const Eigen::Vector3d>& a_A,
                                const Frame<double>& frameB,
                                const Eigen::Ref<const Eigen::Vector3d>& b_B,
                                double angle_lower, double angle_upper,
                                systems::Context<double>* plant_context);

  /**
   * Overloaded constructor. Use MultibodyPlant<AutoDiffXd> instead of
   * MultibodyPlant<double>.
   * @exclude_from_pydrake_mkdoc{Suppressed due to ambiguity in mkdoc.
   * Documentation string is manually recreated in Python.}
   */
  AngleBetweenVectorsConstraint(const MultibodyPlant<AutoDiffXd>* plant,
                                const Frame<AutoDiffXd>& frameA,
                                const Eigen::Ref<const Eigen::Vector3d>& a_A,
                                const Frame<AutoDiffXd>& frameB,
                                const Eigen::Ref<const Eigen::Vector3d>& b_B,
                                double angle_lower, double angle_upper,
                                systems::Context<AutoDiffXd>* plant_context);

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

  bool use_autodiff() const { return plant_autodiff_; }

  const MultibodyPlant<double>* plant_double_;
  const FrameIndex frameA_index_;
  const FrameIndex frameB_index_;
  const Eigen::Vector3d a_unit_A_;
  const Eigen::Vector3d b_unit_B_;
  systems::Context<double>* const context_double_;

  const MultibodyPlant<AutoDiffXd>* plant_autodiff_;
  systems::Context<AutoDiffXd>* const context_autodiff_;
};
}  // namespace multibody
}  // namespace drake
