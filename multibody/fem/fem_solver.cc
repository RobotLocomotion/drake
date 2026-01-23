#include "drake/multibody/fem/fem_solver.h"

#include <algorithm>

#include "drake/common/text_logging.h"
#include "drake/multibody/contact_solvers/eigen_block_3x3_sparse_symmetric_matrix.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

using contact_solvers::internal::BlockSparseSymmetricMatrix3d;
using contact_solvers::internal::EigenBlock3x3SparseSymmetricMatrix;
using contact_solvers::internal::SchurComplement;
using LinearSolver =
    contact_solvers::internal::BlockSparseCholeskySolver<Matrix3<double>>;

template <typename T>
FemSolver<T>::FemStateAndSchurComplement::FemStateAndSchurComplement(
    const FemModel<T>& model)
    : state(model.MakeFemState()) {}

template <typename T>
FemSolver<T>::FemStateAndSchurComplement::~FemStateAndSchurComplement() =
    default;

template <typename T>
FemSolver<T>::Scratch::Scratch(const FemModel<T>& model) {
  ReinitializeIfNeeded(model);
}

template <typename T>
FemSolver<T>::Scratch::~Scratch() = default;

template <typename T>
void FemSolver<T>::Scratch::ReinitializeIfNeeded(const FemModel<T>& model) {
  if (b.size() != model.num_dofs()) {
    b.resize(model.num_dofs());
    dz.resize(model.num_dofs());
    tangent_matrix = model.MakeTangentMatrix();
  }
}

template <typename T>
FemSolver<T>::FemSolver(const FemModel<T>* model,
                        const DiscreteTimeIntegrator<T>* integrator)
    : model_(model),
      integrator_(integrator),
      next_state_and_schur_complement_(*model),
      scratch_(*model) {
  DRAKE_DEMAND(model_ != nullptr);
  DRAKE_DEMAND(integrator_ != nullptr);
}

template <typename T>
FemSolver<T>::~FemSolver() = default;

template <typename T>
int FemSolver<T>::AdvanceOneTimeStep(
    const FemState<T>& prev_state, const FemPlantData<T>& plant_data,
    const std::unordered_set<int>& nonparticipating_vertices) {
  model_->ThrowIfModelStateIncompatible(__func__, prev_state);
  next_state_and_schur_complement_.ReinitializeIfNeeded(*model_);
  scratch_.ReinitializeIfNeeded(*model_);
  const VectorX<T>& unknown_variable = integrator_->GetUnknowns(prev_state);
  FemState<T>* next_state =
      next_state_and_schur_complement_.state.get_mutable();
  integrator_->AdvanceOneTimeStep(prev_state, unknown_variable, next_state);
  if (model_->is_linear()) {
    return SolveLinearModel(plant_data, nonparticipating_vertices);
  }
  /* Run Newton-Raphson iterations. */
  const int iterations =
      SolveNonlinearModel(plant_data, nonparticipating_vertices);
  if (iterations == -1) {
    throw std::runtime_error(
        "FemSolver::AdvanceOneTimeStep() failed to converge on a nonlinear FEM "
        "model. Consider using a smaller timestep or reduce the stiffness of "
        "the material.");
  }
  return iterations;
}

template <typename T>
double FemSolver<T>::ComputeLinearSolverTolerance(
    const double current_residual, const double previous_residual,
    const double previous_tolerance) const {
  /* first iteration */
  if (previous_tolerance <= 0.0) {
    return max_linear_solver_tolerance_;
  }
  /* ratio = ‖Fₖ‖/‖Fₖ₋₁‖  */
  const double ratio = current_residual / previous_residual;
  /* candidate = γ * ratio^α  with γ=1, α=2.
   Eq.(2.6) [Eisenstat and Walker, 1996]*/
  const double candidate = ratio * ratio;
  /* Safe-guard to prevent the tolerance from being too small.
   (Choice 2 safeguard in section 2.1 [Eisenstat and Walker, 1996]) */
  const double safe_guard = previous_tolerance * previous_tolerance;
  return std::clamp(candidate, safe_guard, max_linear_solver_tolerance_);
}

template <typename T>
void FemSolver<T>::SetNextFemState(const FemState<T>& next_state) {
  next_state_and_schur_complement_.state->CopyFrom(next_state);
  next_state_and_schur_complement_.schur_complement =
      contact_solvers::internal::SchurComplement{};
}

template <typename T>
bool FemSolver<T>::solver_converged(const T& residual_norm,
                                    const T& initial_residual_norm) const {
  return residual_norm < std::max(relative_tolerance_ * initial_residual_norm,
                                  absolute_tolerance_);
}

template <typename T>
int FemSolver<T>::SolveLinearModel(
    const FemPlantData<T>& plant_data,
    const std::unordered_set<int>& nonparticipating_vertices) {
  DRAKE_DEMAND(model_->is_linear());
  FemState<T>& state = *next_state_and_schur_complement_.state;
  VectorX<T>& b = scratch_.b;
  VectorX<T>& dz = scratch_.dz;
  BlockSparseSymmetricMatrix3d& tangent_matrix = *scratch_.tangent_matrix;

  model_->ApplyBoundaryCondition(&state);
  model_->CalcResidual(state, plant_data, &b);
  T residual_norm = b.norm();
  model_->CalcTangentMatrix(state, &tangent_matrix);
  next_state_and_schur_complement_.schur_complement =
      contact_solvers::internal::SchurComplement(tangent_matrix,
                                                 nonparticipating_vertices);
  if (residual_norm < absolute_tolerance_) {
    return 0;
  }
  dz = next_state_and_schur_complement_.schur_complement.Solve(-b);
  integrator_->UpdateStateFromChangeInUnknowns(dz, &state);
  return 1;
}

template <typename T>
int FemSolver<T>::SolveNonlinearModel(
    const FemPlantData<T>& plant_data,
    const std::unordered_set<int>& nonparticipating_vertices) {
  DRAKE_DEMAND(!model_->is_linear());
  VectorX<T>& b = scratch_.b;
  VectorX<T>& dz = scratch_.dz;
  BlockSparseSymmetricMatrix3d& tangent_matrix = *scratch_.tangent_matrix;
  FemState<T>& state = *next_state_and_schur_complement_.state;
  model_->ApplyBoundaryCondition(&state);
  model_->CalcResidual(state, plant_data, &b);
  T residual_norm = b.norm();
  const T initial_residual_norm = residual_norm;
  T prev_residual_norm = residual_norm;
  Eigen::ConjugateGradient<EigenBlock3x3SparseSymmetricMatrix,
                           Eigen::Lower | Eigen::Upper>
      cg;
  double prev_cg_tolerance = 0;
  int iter = 0;
  /* For non-linear FEM models, the system of equations is non-linear and we use
   a Newton-Raphson solver. We iterate until any of the following is true:
   1. The max number of allowed iterations is reached;
   2. The norm of the residual is smaller than the absolute tolerance.
   3. The relative error (the norm of the residual divided by the norm of the
      initial residual) is smaller than the dimensionless relative tolerance. */
  while (iter < max_iterations_ &&
         /* On first iteration, this is equivalent to residual_norm < abs_tol */
         !solver_converged(residual_norm, initial_residual_norm)) {
    model_->CalcTangentMatrix(state, &tangent_matrix);
    const EigenBlock3x3SparseSymmetricMatrix wrapper(&tangent_matrix,
                                                     model_->parallelism());
    const double cg_tolerance = ComputeLinearSolverTolerance(
        residual_norm, prev_residual_norm, prev_cg_tolerance);
    cg.setTolerance(cg_tolerance);
    cg.compute(wrapper);
    if (cg.info() != Eigen::Success) {
      return -1;
    }
    dz = cg.solve(-b);
    integrator_->UpdateStateFromChangeInUnknowns(dz, &state);
    prev_residual_norm = residual_norm;
    prev_cg_tolerance = cg_tolerance;
    model_->CalcResidual(state, plant_data, &b);
    residual_norm = b.norm();
    ++iter;
  }
  if (!solver_converged(residual_norm, initial_residual_norm)) {
    /* Solver failed to converge with max number of Newton iterations. */
    return -1;
  }
  /* Build the Schur complement after the Newton iterations have converged. */
  model_->CalcTangentMatrix(state, &tangent_matrix);
  next_state_and_schur_complement_.schur_complement =
      contact_solvers::internal::SchurComplement(tangent_matrix,
                                                 nonparticipating_vertices);
  return iter;
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

template class drake::multibody::fem::internal::FemSolver<double>;
