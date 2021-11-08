#include "drake/multibody/optimization/centroidal_momentum_constraint.h"

#include <utility>

#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

using drake::multibody::internal::RefFromPtrOrThrow;
using drake::multibody::internal::UpdateContextPositionsAndVelocities;

namespace drake {
namespace multibody {
namespace {
int momentum_dim(bool angular_only) { return angular_only ? 3 : 6; }
}  // namespace

CentroidalMomentumConstraint::CentroidalMomentumConstraint(
    const MultibodyPlant<AutoDiffXd>* plant,
    std::optional<std::vector<ModelInstanceIndex>> model_instances,
    systems::Context<AutoDiffXd>* plant_context, bool angular_only)
    : solvers::Constraint(momentum_dim(angular_only),
                          RefFromPtrOrThrow(plant).num_positions() +
                              RefFromPtrOrThrow(plant).num_velocities() +
                              momentum_dim(angular_only),
                          VectorX<double>::Zero(momentum_dim(angular_only)),
                          VectorX<double>::Zero(momentum_dim(angular_only))),
      model_instances_{std::move(model_instances)},
      plant_autodiff_{plant},
      context_autodiff_{plant_context},
      angular_only_{angular_only} {
  if (plant_context == nullptr)
    throw std::invalid_argument("plant_context is nullptr.");
  this->set_description("Centroidal momentum constraint");
}

// Currently we only support (T, S) = (AutodiffXd, AutoDiffXd).
// In the future, when we can compute the gradient of the centroidal momentum
// using MultibodyPlant<double>, we will also support (T, S) = (double, double),
// (double, AutoDiffXd)
template <typename T, typename S>
void DoEvalGeneric(
    const MultibodyPlant<T>& plant, systems::Context<T>* context,
    const std::optional<std::vector<ModelInstanceIndex>>& model_instances,
    bool angular_only, const Eigen::Ref<const VectorX<S>>& x, VectorX<S>* y) {
  y->resize(momentum_dim(angular_only));
  UpdateContextPositionsAndVelocities(
      context, plant, x.head(plant.num_positions() + plant.num_velocities()));
  Vector3<T> p_WC;
  SpatialMomentum<T> h_WC_eval;
  if (model_instances.has_value()) {
    p_WC = plant.CalcCenterOfMassPositionInWorld(*context,
                                                 model_instances.value());
    h_WC_eval = plant.CalcSpatialMomentumInWorldAboutPoint(
        *context, model_instances.value(), p_WC);
  } else {
    p_WC = plant.CalcCenterOfMassPositionInWorld(*context);
    h_WC_eval = plant.CalcSpatialMomentumInWorldAboutPoint(*context, p_WC);
  }
  // TODO(hongkai.dai): the next line will be changed once we support T=double
  // and S=AutoDiffXd, namely we use the Jacobian matrix of MBP<double> to
  // compute the gradient of the centroidal momentum.
  if (angular_only) {
    *y = h_WC_eval.rotational() - x.template tail<3>();
  } else {
    *y = h_WC_eval.get_coeffs() - x.template tail<6>();
  }
}

void CentroidalMomentumConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  if (use_autodiff()) {
    AutoDiffVecXd y_t;
    Eval(x.cast<AutoDiffXd>(), &y_t);
    *y = math::ExtractValue(y_t);
  } else {
    // Implement this part once we support computing the Jacobian of centroidal
    // momentum with MBP<double>
    throw std::runtime_error("Not implemented yet");
  }
}

void CentroidalMomentumConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  if (use_autodiff()) {
    DoEvalGeneric(*plant_autodiff_, context_autodiff_, model_instances_,
                  angular_only_, x, y);
  } else {
    // Implement this part once we support computing the Jacobian of centroidal
    // momentum with MBP<double>
    throw std::runtime_error("Not implemented yet");
  }
}
}  // namespace multibody
}  // namespace drake
