#include "drake/multibody/fixed_fem/dev/fem_solver.h"

#include <algorithm>
#include <memory>
#include <utility>

namespace drake {
namespace multibody {
namespace fem {

template <typename T>
void FemSolver<T>::Resize() const {
  if (b_.size() != model_->num_dofs()) {
    b_.resize(model_->num_dofs());
    dz_.resize(model_->num_dofs());
    A_ = model_->MakePetscSymmetricBlockSparseTangentMatrix();
  }
}

template <typename T>
int FemSolver<T>::SolveWithInitialGuess(FemStateBase<T>*) const {
  throw std::logic_error(
      "The only scalar type that supports "
      "FemSolver<T>::SolveWithInitialGuess() is T = double.");
  return 0;
}

template <>
int FemSolver<double>::SolveWithInitialGuess(
    FemStateBase<double>* state) const {
  /* Make sure the scratch quantities are of the correct size and apply BC if
   one is specified. */
  Resize();
  model_->ApplyBoundaryCondition(state);
  model_->CalcResidual(*state, &b_);
  int iter = 0;
  /* Newton-Raphson iterations. We iterate until any of the following is true:
   1. The max number of allowed iterations is reached;
   2. The norm of the change in the state in a single iteration is smaller
      than the absolute tolerance.
   3. The relative error (the norm of the change in the state divided by the
      norm of the state) is smaller than the unitless relative tolerance. */
  do {
    model_->CalcTangentMatrix(*state, A_.get());
    /* Solving for A * dz = -b. */
    dz_ = A_->Solve(internal::PetscSymmetricBlockSparseMatrix::SolverType::
                        ConjugateGradient,
                    internal::PetscSymmetricBlockSparseMatrix::
                        PreconditionerType::IncompleteCholesky,
                    -b_);
    model_->UpdateStateFromChangeInUnknowns(dz_, state);
    model_->CalcResidual(*state, &b_);
    ++iter;
  } while (dz_.norm() > std::max(relative_tolerance_ *
                                     model_->GetUnknowns(*state).norm(),
                                 absolute_tolerance_) &&
           iter < kMaxIterations_);
  if (iter == kMaxIterations_) {
    // TODO(xuchenhan-tri): Provide some advice on how to get a "better
    //  initial guess". Or instead, return a status indicating the solver
    //  did not converge and let users decide what to do.
    throw std::runtime_error(
        "The solver did not converge " + std::to_string(kMaxIterations_) +
        " iterations. Please provide a better initial guess.");
  }
  return iter;
}

}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::fem::FemSolver)
