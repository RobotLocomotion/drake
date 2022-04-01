#include "drake/multibody/fem/fem_solver.h"

#include <algorithm>

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
FemSolver<T>::FemSolver(const FemModel<T>* model,
                        const DiscreteTimeIntegrator<T>* integrator)
    : model_(model), integrator_(integrator), scratch_data_(*model_) {
  DRAKE_DEMAND(model_ != nullptr);
  DRAKE_DEMAND(integrator_ != nullptr);
}

template <typename T>
int FemSolver<T>::AdvanceOneTimeStep(const FemState<T>& prev_state,
                                     FemState<T>* next_state) const {
  DRAKE_DEMAND(next_state != nullptr);
  model_->ThrowIfModelStateIncompatible(__func__, prev_state);
  model_->ThrowIfModelStateIncompatible(__func__, *next_state);
  /* Make initial guess of the unknown variable that it stays the same. */
  const VectorX<T>& unknown_variable = integrator_->GetUnknowns(prev_state);
  integrator_->AdvanceOneTimeStep(prev_state, unknown_variable, next_state);
  /* Run Newton-Raphson iterations. */
  return SolveWithInitialGuess(next_state);
}

template <typename T>
void FemSolver<T>::set_linear_solve_tolerance(const T& residual_norm) const {
  /* The relative tolerance when solving for A * dz = -b, where A is the tangent
   matrix. We set it to be on the order of the residual norm to achieve local
   second order convergence [Nocedal and Wright, section 7.1]. We set it to be
   smaller than or equal to the relative tolerance to ensure that linear models
   converge in exact one Newton iteration.

   [Nocedal and Wright] Nocedal, J., & Wright, S. (2006). Numerical
   optimization. Springer Science & Business Media. */
  double linear_solve_tolerance =
      std::min(ExtractDoubleOrThrow(relative_tolerance_),
               ExtractDoubleOrThrow(residual_norm));
  DRAKE_DEMAND(scratch_data_.tangent_matrix != nullptr);
  scratch_data_.tangent_matrix->set_relative_tolerance(linear_solve_tolerance);
}

template <typename T>
int FemSolver<T>::SolveWithInitialGuess(FemState<T>* state) const {
  /* Make sure the scratch quantities are of the correct sizes. */
  ResetScratchDataIfNecessary();
  model_->ApplyBoundaryCondition(state);
  model_->CalcResidual(*state, &scratch_data_.b);
  T residual_norm = scratch_data_.b.norm();
  T initial_residual_norm = residual_norm;
  int iter = 0;
  /* Newton-Raphson iterations. We iterate until any of the following is true:
   1. The max number of allowed iterations is reached;
   2. The norm of the residual is smaller than the absolute tolerance.
   3. The relative error (the norm of the residual divided by the norm of the
      initial residual) is smaller than the unitless relative tolerance. */
  do {
    set_linear_solve_tolerance(residual_norm);
    /* Use PETSc matrix when scalar type is double. Otherwise, use Eigen
     matrix. */
    model_->CalcTangentMatrix(*state, integrator_->GetWeights(),
                              scratch_data_.tangent_matrix.get());
    scratch_data_.tangent_matrix->AssembleIfNecessary();
    /* Solve for A * dz = -b, where A is the tangent matrix. */
    scratch_data_.dz = scratch_data_.tangent_matrix->Solve(
        internal::PetscSymmetricBlockSparseMatrix::SolverType::
            kConjugateGradient,
        internal::PetscSymmetricBlockSparseMatrix::PreconditionerType::
            kIncompleteCholesky,
        -scratch_data_.b);
    integrator_->UpdateStateFromChangeInUnknowns(scratch_data_.dz, state);
    model_->CalcResidual(*state, &scratch_data_.b);
    residual_norm = scratch_data_.b.norm();
    ++iter;
  } while (iter < kMaxIterations_ &&
           residual_norm > relative_tolerance_ * initial_residual_norm &&
           residual_norm > absolute_tolerance_);
  if (iter == kMaxIterations_) {
    throw std::runtime_error(fmt::format(
        "The solver did not converge in {} iterations. Please provide a "
        "better initial guess. Consider taking a smaller time step, "
        "especially if the constitutive model you are using is nonlinear "
        "(e.g. CorotatedModel).",
        kMaxIterations_));
  }
  return iter;
}

template <typename T>
void FemSolver<T>::ResetScratchDataIfNecessary() const {
  DRAKE_DEMAND(model_ != nullptr);
  if (scratch_data_.num_dofs() != model_->num_dofs())
    scratch_data_.Resize(*model_);
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

template class drake::multibody::fem::internal::FemSolver<double>;
