#include "drake/multibody/fem/fem_solver.h"

#include <algorithm>

#include "drake/common/text_logging.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
FemSolver<T>::FemSolver(const FemModel<T>* model,
                        const DiscreteTimeIntegrator<T>* integrator)
    : model_(model), integrator_(integrator) {
  DRAKE_DEMAND(model_ != nullptr);
  DRAKE_DEMAND(integrator_ != nullptr);
}

template <typename T>
int FemSolver<T>::AdvanceOneTimeStep(const FemState<T>& prev_state,
                                     FemState<T>* next_state,
                                     FemSolverData<T>* data) const {
  DRAKE_DEMAND(next_state != nullptr);
  model_->ThrowIfModelStateIncompatible(__func__, prev_state);
  model_->ThrowIfModelStateIncompatible(__func__, *next_state);
  const VectorX<T>& unknown_variable = integrator_->GetUnknowns(prev_state);
  integrator_->AdvanceOneTimeStep(prev_state, unknown_variable, next_state);
  /* Run Newton-Raphson iterations. */
  return SolveWithInitialGuess(next_state, data);
}

template <typename T>
bool FemSolver<T>::solver_converged(const T& residual_norm,
                                    const T& initial_residual_norm) const {
  return residual_norm < std::max(relative_tolerance_ * initial_residual_norm,
                                  absolute_tolerance_);
}

template <typename T>
double FemSolver<T>::linear_solve_tolerance(
    const T& residual_norm, const T& initial_residual_norm) const {
  /* The relative tolerance when solving for A * dz = -b, where A is the tangent
   matrix. We set it to be on the order of the residual norm to achieve local
   second order convergence [Nocedal and Wright, section 7.1]. We also set it to
   be smaller than the relative tolerance to ensure that linear models converge
   in exact one Newton iteration.

   [Nocedal and Wright] Nocedal, J., & Wright, S. (2006). Numerical
   optimization. Springer Science & Business Media. */
  constexpr double kLinearToleranceFactor = 0.2;
  double linear_solve_tolerance =
      std::min(kLinearToleranceFactor * relative_tolerance_,
               ExtractDoubleOrThrow(residual_norm) /
                   std::max(ExtractDoubleOrThrow(initial_residual_norm),
                            absolute_tolerance_));
  return linear_solve_tolerance;
}

template <typename T>
int FemSolver<T>::SolveWithInitialGuess(FemState<T>* state,
                                        FemSolverData<T>* data) const {
  /* We scale the entire system by dt so that we are solving
    A * dt * dt = -b * dt.
  The reason is that we want the Schur complement of the tangent matrix
  of the momentum balance equation (A * dt) instead of the force balance
  equation (A). */
  VectorX<T>& b = data->b;
  VectorX<T>& dz = data->dz;
  contact_solvers::internal::Block3x3SparseSymmetricMatrix& tangent_matrix =
      *data->tangent_matrix;
  contact_solvers::internal::SchurComplement& schur_complement =
      data->schur_complement;
  const double dt = integrator_->dt();

  model_->ApplyBoundaryCondition(state);
  model_->CalcResidual(*state, &b);
  /* If the model is linear, we know that we only need one Newton iteration to
   converge. */
  if (model_->is_linear()) {
    model_->CalcTangentMatrix(*state, integrator_->GetWeights() * dt,
                              &tangent_matrix);
    schur_complement = contact_solvers::internal::SchurComplement(
        tangent_matrix, data->nonparticipating_vertices);
    dz = schur_complement.Solve(-b * dt);
    integrator_->UpdateStateFromChangeInUnknowns(dz, state);
    return 1;
  }
  /* For non-linear FEM models, the system of equations is non-linear and we use
   a Newton-Raphson solver. */
  int iter = 0;
  T residual_norm = b.norm() * dt;
  const T initial_residual_norm = residual_norm;
  /* Newton-Raphson iterations. We iterate until any of the following is true:
   1. The max number of allowed iterations is reached;
   2. The norm of the residual is smaller than the absolute tolerance.
   3. The relative error (the norm of the residual divided by the norm of the
      initial residual) is smaller than the unitless relative tolerance. */
  while (iter < kMaxIterations_ &&
         /* Equivalent to residual_norm < absolute_tolerance_ on first
            iteration. */
         !solver_converged(residual_norm, initial_residual_norm)) {
    model_->CalcTangentMatrix(*state, integrator_->GetWeights() * dt,
                              &tangent_matrix);
    data->linear_solver.UpdateMatrix(tangent_matrix);
    const bool factored = data->linear_solver.Factor();
    if (!factored) {
      throw std::runtime_error(
          "Tangent matrix factorization failed in FemSolver because the FEM "
          "tangent matrix is not symmetric positive definite (SPD). This may "
          "be triggered by a combination of a stiff nonlinear constitutive "
          "model and a large time step.");
    }
    dz = data->linear_solver.Solve(-b * dt);
    integrator_->UpdateStateFromChangeInUnknowns(dz, state);
    model_->CalcResidual(*state, &b);
    residual_norm = b.norm() * dt;
    ++iter;
  }
  if (!solver_converged(residual_norm, initial_residual_norm)) {
    return -1;
  }
  /* Build the Schur complement after the Newton iterations have converged. */
  schur_complement = contact_solvers::internal::SchurComplement(
      tangent_matrix, data->nonparticipating_vertices);
  return iter;
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

template class drake::multibody::fem::internal::FemSolverData<double>;
template class drake::multibody::fem::internal::FemSolver<double>;
