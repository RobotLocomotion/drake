#include "drake/multibody/fem/fem_solver.h"

#include <algorithm>

#include "drake/common/text_logging.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
FemSolverData<T>::FemSolverData(const FemModel<T>& model) {
  b_.resize(model.num_dofs());
  dz_.resize(model.num_dofs());
  tangent_matrix_ = model.MakeTangentMatrix();
  /* For non-linear models, we use a BlockSparseCholeskySolver to solve the
   linear systems from Newton-Raphson iterations. This usually allow for
   better elimination ordering. For linear models, we use `schur_complement`
   to both solve the linear system and to find the Schur complement to avoid
   factoring the matrix more times than necessary. */
  if (!model.is_linear()) {
    linear_solver_.SetMatrix(*tangent_matrix_);
  }
}

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
  DRAKE_DEMAND(data != nullptr);
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
int FemSolver<T>::SolveWithInitialGuess(FemState<T>* state,
                                        FemSolverData<T>* data) const {
  VectorX<T>& b = data->b_;
  VectorX<T>& dz = data->dz_;
  contact_solvers::internal::Block3x3SparseSymmetricMatrix& tangent_matrix =
      *data->tangent_matrix_;
  contact_solvers::internal::SchurComplement& schur_complement =
      data->schur_complement_;

  model_->ApplyBoundaryCondition(state);
  model_->CalcResidual(*state, &b);
  T residual_norm = b.norm();
  const T initial_residual_norm = residual_norm;
  /* Immediately return if at steady state. This is true iff residual_norm <
   absolute_tolerance_. */
  if (solver_converged(residual_norm, initial_residual_norm)) {
    return 0;
  }
  /* If the model is linear, we know that we only need one Newton iteration to
   converge. */
  if (model_->is_linear()) {
    model_->CalcTangentMatrix(*state, integrator_->GetWeights(),
                              &tangent_matrix);
    schur_complement = contact_solvers::internal::SchurComplement(
        tangent_matrix, data->nonparticipating_vertices());
    dz = schur_complement.Solve(-b);
    integrator_->UpdateStateFromChangeInUnknowns(dz, state);
    return 1;
  }
  int iter = 0;
  /* For non-linear FEM models, the system of equations is non-linear and we use
   a Newton-Raphson solver. We iterate until any of the following is true:
   1. The max number of allowed iterations is reached;
   2. The norm of the residual is smaller than the absolute tolerance.
   3. The relative error (the norm of the residual divided by the norm of the
      initial residual) is smaller than the dimensionless relative tolerance. */
  while (iter < kMaxIterations_ &&
         !solver_converged(residual_norm, initial_residual_norm)) {
    model_->CalcTangentMatrix(*state, integrator_->GetWeights(),
                              &tangent_matrix);
    data->linear_solver_.UpdateMatrix(tangent_matrix);
    const bool factored = data->linear_solver_.Factor();
    if (!factored) {
      throw std::runtime_error(
          "Tangent matrix factorization failed in FemSolver because the FEM "
          "tangent matrix is not symmetric positive definite (SPD). This may "
          "be triggered by a combination of a stiff nonlinear constitutive "
          "model and a large time step.");
    }
    dz = data->linear_solver_.Solve(-b);
    integrator_->UpdateStateFromChangeInUnknowns(dz, state);
    model_->CalcResidual(*state, &b);
    residual_norm = b.norm();
    ++iter;
  }
  if (!solver_converged(residual_norm, initial_residual_norm)) {
    /* Solver failed to converge with max number of Newton iterations. */
    return -1;
  }
  /* Build the Schur complement after the Newton iterations have converged. */
  model_->CalcTangentMatrix(*state, integrator_->GetWeights(), &tangent_matrix);
  schur_complement = contact_solvers::internal::SchurComplement(
      tangent_matrix, data->nonparticipating_vertices());
  return iter;
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

template class drake::multibody::fem::internal::FemSolverData<double>;
template class drake::multibody::fem::internal::FemSolver<double>;
