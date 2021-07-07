#pragma once

#include <memory>

#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace multibody {
/**
 * Constrains that the angle difference θ between the orientation of frame A
 * and the orientation of frame B to satisfy θ ≤ θ_bound. The angle
 * difference between frame A's orientation R_WA and B's orientation R_WB is θ
 *   (θ ∈ [0, π]), if there exists a rotation axis a, such that rotating frame
 * A by angle θ about axis a aligns it with frame B. Namely
 *   R_AB = I + sinθ â + (1-cosθ)â²   (1)
 * where R_AB is the orientation of frame B expressed in frame A. â is the skew
 * symmetric matrix of the rotation axis a. Equation (1) is the Rodrigues
 * formula that computes the rotation matrix froma rotation axis a and an angle
 * θ, https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
 * If the users want frame A and frame B to align perfectly, they can set
 *   θ_bound = 0.
 * Mathematically, this constraint is imposed as
 *   trace(R_AB) ≥ 2cos(θ_bound) + 1   (1)
 * To derive (1), using Rodrigues formula
 *   R_AB = I + sinθ â + (1-cosθ)â²
 * where
 *   trace(R_AB) = 2cos(θ) + 1 ≥ 2cos(θ_bound) + 1
 *
 * @ingroup solver_evaluators
 */
class OrientationConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OrientationConstraint)

  /**
   * Constructs an OrientationConstraint object.
   * The frame A is fixed to a frame A̅, with orientatation `R_AbarA` measured
   * in frame A̅. The frame B is fixed to a frame B̅, with orientation `R_BbarB`
   * measured in frame B. We constrain the angle between frame A and B to be
   * less than `theta_bound`.
   * @param plant The MultibodyPlant on which the constraint is imposed. `plant`
   *   should be alive during the lifetime of this constraint.
   * @param frameAbar The frame A̅ in the model to which frame A is fixed.
   * @param R_AbarA The orientation of frame A measured in frame A̅.
   * @param frameBbar The frame B̅ in the model to which frame B is fixed.
   * @param R_BbarB The orientation of frame B measured in frame B̅.
   * @param theta_bound The bound on the angle difference between frame A's
   *   orientation and frame B's orientation. It is denoted as θ_bound in the
   *   class documentation. `theta_bound` is in radians.
   * @param plant_context The Context that has been allocated for this
   *   `plant`. We will update the context when evaluating the constraint.
   *   `plant_context` should be alive during the lifetime of this constraint.
   * @throws std::exception if `plant` is nullptr.
   * @throws std::exception if `frameAbar` or `frameBbar` does not belong to
   *   `plant`.
   * @throws std::exception if angle_bound < 0.
   * @throws std::exception if `plant_context` is nullptr.
   */
  OrientationConstraint(
      const MultibodyPlant<double>* const plant,
      const Frame<double>& frameAbar,
      const math::RotationMatrix<double>& R_AbarA,
      const Frame<double>& frameBbar,
      const math::RotationMatrix<double>& R_BbarB, double theta_bound,
      systems::Context<double>* plant_context);

  /**
   * Overloaded constructor.
   * Constructs the constraint using MultibodyPlant<AutoDiffXd>
   * @exclude_from_pydrake_mkdoc{Suppressed due to ambiguity in mkdoc.
   * Documentation string is manually recreated in Python.}
   */
  OrientationConstraint(const MultibodyPlant<AutoDiffXd>* const plant,
                        const Frame<AutoDiffXd>& frameAbar,
                        const math::RotationMatrix<double>& R_AbarA,
                        const Frame<AutoDiffXd>& frameBbar,
                        const math::RotationMatrix<double>& R_BbarB,
                        double theta_bound,
                        systems::Context<AutoDiffXd>* plant_context);

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

  bool use_autodiff() const { return plant_autodiff_; }

  const MultibodyPlant<double>* const plant_double_;
  const FrameIndex frameAbar_index_;
  const FrameIndex frameBbar_index_;
  const math::RotationMatrix<double> R_AbarA_;
  const math::RotationMatrix<double> R_BbarB_;
  systems::Context<double>* const context_double_;

  const MultibodyPlant<AutoDiffXd>* const plant_autodiff_;
  systems::Context<AutoDiffXd>* context_autodiff_;
};
}  // namespace multibody
}  // namespace drake
