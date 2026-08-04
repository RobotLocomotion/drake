#include "drake/multibody/inverse_kinematics/orientation_cost.h"

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_evaluator_utilities.h"

using drake::multibody::internal::PtrOrThrow;
using drake::multibody::internal::RefFromPtrOrThrow;
using drake::multibody::internal::UpdateContextConfiguration;

namespace drake {
namespace multibody {

OrientationCost::OrientationCost(const MultibodyPlant<double>* const plant,
                                 const Frame<double>& frameAbar,
                                 const math::RotationMatrix<double>& R_AbarA,
                                 const Frame<double>& frameBbar,
                                 const math::RotationMatrix<double>& R_BbarB,
                                 double c,
                                 systems::Context<double>* plant_context)
    : solvers::Cost(RefFromPtrOrThrow(plant).num_positions()),
      constraint_(plant, frameAbar, R_AbarA, frameBbar, R_BbarB, 0,
                  PtrOrThrow(plant_context,
                             "OrientationCost(): plant_context is nullptr")),
      c_{c} {}

OrientationCost::OrientationCost(const MultibodyPlant<AutoDiffXd>* const plant,
                                 const Frame<AutoDiffXd>& frameAbar,
                                 const math::RotationMatrix<double>& R_AbarA,
                                 const Frame<AutoDiffXd>& frameBbar,
                                 const math::RotationMatrix<double>& R_BbarB,
                                 double c,
                                 systems::Context<AutoDiffXd>* plant_context)
    : solvers::Cost(RefFromPtrOrThrow(plant).num_positions()),
      constraint_(plant, frameAbar, R_AbarA, frameBbar, R_BbarB, 0,
                  PtrOrThrow(plant_context,
                             "OrientationCost(): plant_context is nullptr")),
      c_{c} {}

OrientationCost::~OrientationCost() = default;

void OrientationCost::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                             Eigen::VectorXd* y) const {
  // OrientationConstraint computes tr(R_AB) = 1 + 2cosθ
  // So 1 - cosθ = (3 - tr(R_AB))/2
  y->resize(1);
  Eigen::VectorXd trace_R_AB(1);
  constraint_.Eval(x, &trace_R_AB);
  (*y)[0] = c_ * (3.0 - trace_R_AB[0]) / 2.0;
}

void OrientationCost::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                             AutoDiffVecXd* y) const {
  y->resize(1);
  VectorX<AutoDiffXd> trace_R_AB(1);
  constraint_.Eval(x, &trace_R_AB);
  (*y)[0] = c_ * (3.0 - trace_R_AB[0]) / 2.0;
}

void OrientationCost::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>&,
    VectorX<symbolic::Expression>*) const {
  throw std::logic_error(
      "OrientationCost::DoEval() does not work for symbolic variables.");
}

}  // namespace multibody
}  // namespace drake
