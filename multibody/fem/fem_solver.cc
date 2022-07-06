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
  const VectorX<T>& unknown_variable = integrator_->GetUnknowns(prev_state);
  integrator_->AdvanceOneTimeStep(prev_state, unknown_variable, next_state);
  /* Run Newton-Raphson iterations. */
  return SolveWithInitialGuess(next_state);
}

template <typename T>
bool FemSolver<T>::solver_converged(const T& residual_norm,
                                    const T& initial_residual_norm) const {
  return residual_norm < std::max(relative_tolerance_ * initial_residual_norm,
                                  absolute_tolerance_);
}

template <typename T>
void FemSolver<T>::set_linear_solve_tolerance(
    const T& residual_norm, const T& initial_residual_norm) const {
  /* The relative tolerance when solving for A * dz = -b, where A is the tangent
   matrix. We set it to be on the order of the residual norm to achieve local
   second order convergence [Nocedal and Wright, section 7.1]. We also set it to
   be smaller than the relative tolerance to ensure that linear models converge
   in exact one Newton iteration.

   [Nocedal and Wright] Nocedal, J., & Wright, S. (2006). Numerical
   optimization. Springer Science & Business Media. */
  constexpr double kLinearToleranceFactor = 0.2;
  const double linear_solve_tolerance =
      std::min(kLinearToleranceFactor * relative_tolerance_,
               ExtractDoubleOrThrow(residual_norm) /
                   std::max(ExtractDoubleOrThrow(initial_residual_norm),
                            absolute_tolerance_));
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
  const T initial_residual_norm = residual_norm;
  int iter = 0;
  /* Newton-Raphson iterations. We iterate until any of the following is true:
   1. The max number of allowed iterations is reached;
   2. The norm of the residual is smaller than the absolute tolerance.
   3. The relative error (the norm of the residual divided by the norm of the
      initial residual) is smaller than the unitless relative tolerance. */
  while (iter < kMaxIterations_ &&
         /* Equivalent to residual_norm < absolute_tolerance_ on first
            iteration. */
         !solver_converged(residual_norm, initial_residual_norm)) {
    set_linear_solve_tolerance(residual_norm, initial_residual_norm);
    model_->CalcTangentMatrix(*state, integrator_->GetWeights(),
                              scratch_data_.tangent_matrix.get());
    scratch_data_.tangent_matrix->AssembleIfNecessary();
    /* Solve for A * dz = -b, where A is the tangent matrix. */
    // TODO(xuchenhan-tri): PetscSymmetricBlockSparseMatrix should have a way of
    // communicating solver failure.
    const auto linear_solve_status = scratch_data_.tangent_matrix->Solve(
        internal::PetscSymmetricBlockSparseMatrix::SolverType::
            kConjugateGradient,
        internal::PetscSymmetricBlockSparseMatrix::PreconditionerType::
            kIncompleteCholesky,
        -scratch_data_.b, &scratch_data_.dz);
    if (linear_solve_status == PetscSolverStatus::kFailure) {
      drake::log()->warn(
          "Linear solve did not converge in Newton iterations in FemSolver.");
      return -1;
    }
    integrator_->UpdateStateFromChangeInUnknowns(scratch_data_.dz, state);
    model_->CalcResidual(*state, &scratch_data_.b);
    residual_norm = scratch_data_.b.norm();
    ++iter;
  }
  if (!solver_converged(residual_norm, initial_residual_norm)) {
    return -1;
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
