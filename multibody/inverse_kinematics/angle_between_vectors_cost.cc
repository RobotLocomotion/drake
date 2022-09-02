#include "drake/multibody/inverse_kinematics/angle_between_vectors_cost.h"

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_evaluator_utilities.h"

using drake::multibody::internal::PtrOrThrow;
using drake::multibody::internal::RefFromPtrOrThrow;
using drake::multibody::internal::UpdateContextConfiguration;

namespace drake {
namespace multibody {
AngleBetweenVectorsCost::AngleBetweenVectorsCost(
    const MultibodyPlant<double>* plant, const Frame<double>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& a_A, const Frame<double>& frameB,
    const Eigen::Ref<const Eigen::Vector3d>& b_B, double c,
    systems::Context<double>* plant_context)
    : solvers::Cost(RefFromPtrOrThrow(plant).num_positions()),
      constraint_(
          plant, frameA, a_A, frameB, b_B, 0, M_PI,
          PtrOrThrow(plant_context,
                     "AngleBetweenVectorsCost(): plant_context is nullptr")),
      c_{c} {}

AngleBetweenVectorsCost::AngleBetweenVectorsCost(
    const MultibodyPlant<AutoDiffXd>* plant, const Frame<AutoDiffXd>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& a_A,
    const Frame<AutoDiffXd>& frameB,
    const Eigen::Ref<const Eigen::Vector3d>& b_B, double c,
    systems::Context<AutoDiffXd>* plant_context)
    : solvers::Cost(RefFromPtrOrThrow(plant).num_positions()),
      constraint_(
          plant, frameA, a_A, frameB, b_B, 0, M_PI,
          PtrOrThrow(plant_context,
                     "AngleBetweenVectorsCost(): plant_context is null ptr")),
      c_{c} {}

AngleBetweenVectorsCost::~AngleBetweenVectorsCost() = default;

void AngleBetweenVectorsCost::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                     Eigen::VectorXd* y) const {
  y->resize(1);
  Eigen::VectorXd cos_theta;
  constraint_.Eval(x, &cos_theta);
  (*y)(0) = c_ * (1 - cos_theta(0));
}

void AngleBetweenVectorsCost::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                     AutoDiffVecXd* y) const {
  y->resize(1);
  AutoDiffVecXd cos_theta;
  constraint_.Eval(x, &cos_theta);
  (*y)(0) = c_ * (1 - cos_theta(0));
}

void AngleBetweenVectorsCost::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>&,
    VectorX<symbolic::Expression>*) const {
  throw std::logic_error(
      "AngleBetweenVectorsCost::DoEval() does not work for symbolic "
      "variables.");
}
}  // namespace multibody
}  // namespace drake
