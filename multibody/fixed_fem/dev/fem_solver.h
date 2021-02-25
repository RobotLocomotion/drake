#pragma once

#include <algorithm>
#include <memory>
#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sparse_linear_operator.h"
#include "drake/multibody/fixed_fem/dev/dirichlet_boundary_condition.h"
#include "drake/multibody/fixed_fem/dev/eigen_conjugate_gradient_solver.h"
#include "drake/multibody/fixed_fem/dev/fem_model.h"
#include "drake/multibody/fixed_fem/dev/linear_system_solver.h"
#include "drake/multibody/fixed_fem/dev/state_updater.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
// TODO(xuchenhan-tri): Remove the template by making the model and the state
// abstract. See issue #14667.
/** %FemSolver solves for the state of a given FemModel at which residual of the
 FemModel is sufficiently close to zero. %FemSolver uses a simple Newton-Raphson
 solver to solve for the zero residual state. A common workflow looks like:
 ```
 // Creates a solver for a given FemModel and sets the LinearSystemSolver used
 // in the Newton-Raphson iterations.
 FemSolver<Model> solver(std::move(fem_model), std::move(linear_solver);
 // Optionally, sets the tolerance under which we deem the residual is
 // effectively zero.
 solver.set_tolerance(kTolereance);
 // Optionally, sets the desired boundary condition.
 solver.SetDirichletBoundaryCondition(std::move(bc));
 // Finally, provide an initial guess and solve for the zero residual state.
 solver.SolveWithInitialGuess(state_updater, &state);
 // Now the residual at `state` has 2-norm smaller than `kTolerance` if the
 // solver has converged.
 ```
 @tparam Model    The type of model that `this` %FemSolver solves for. Must be
 derived from FemModel. */
template <class Model>
class FemSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemSolver);
  static_assert(
      std::is_base_of_v<FemModel<typename Model::ElementType>, Model>,
      "The template parameter Model should be derived from FemModel. ");
  using T = typename Model::T;
  using State = FemState<typename Model::ElementType>;

  // TODO(xuchenhan-tri): Consider moving LinearSystemSolver out of internal
  // namespace and allow users to configure the linear solver.
  /** Constructs a new FemSolver with the given FemModel and an Eigen Conjugate
   Gradient solver as the linear solver. */
  explicit FemSolver(std::unique_ptr<Model> model) : model_(std::move(model)) {
    Resize();
    linear_solver_ =
        std::make_unique<internal::EigenConjugateGradientSolver<T>>(&A_op_);
  }

  /** Uses a Newton-Raphson solver to solve for the solution state at which the
   residual of the FemModel is below the set tolerance (see set_tolerance()).
   @param[in] state_updater    The StateUpdater used to update `state` in each
   Newton-Raphson iteration.
   @param[in, out] state    As input, `state` provides an initial guess of the
   solution. As output, `state` reports the solution state at which the residual
   of the FemModel is deemed sufficiently close to zero.
   @pre state != nullptr. */
  int SolveWithInitialGuess(const StateUpdater<State>& state_updater,
                            State* state) const {
    DRAKE_DEMAND(state != nullptr);
    DRAKE_DEMAND(model_ != nullptr);
    DRAKE_DEMAND(linear_solver_ != nullptr);
    /* Make sure the scratch quantities are of the correct size and apply BC if
     one is specified. */
    Resize();
    if (dirichlet_bc_ != nullptr) {
      dirichlet_bc_->ApplyBoundaryConditions(state);
    }

    model_->CalcResidual(*state, &b_);
    if (dirichlet_bc_ != nullptr) {
      dirichlet_bc_->ApplyBcToResidual(&b_);
    }
    int iter = 0;
    /* Newton-Raphson iterations. We iterate until any of the following is true:
     1. The max number of allowed iterations is reached;
     2. The norm of the change in the state in a single iteration is smaller
        than the absolute tolerance.
     3. The relative error (the norm of the change in the state divided by the
        norm of the state) is smaller than the unitless relative tolerance. */
    do {
      model_->CalcTangentMatrix(*state, state_updater.weights(), &A_);
      if (dirichlet_bc_ != nullptr) {
        dirichlet_bc_->ApplyBcToTangentMatrix(&A_);
      }
      linear_solver_->Compute();
      /* Solving for A * dz = -b. */
      linear_solver_->Solve(-b_, &dz_);
      state_updater.UpdateState(dz_, state);
      model_->CalcResidual(*state, &b_);
      if (dirichlet_bc_ != nullptr) {
        dirichlet_bc_->ApplyBcToResidual(&b_);
      }
      ++iter;
    } while (dz_.norm() >
                 std::max(relative_tolerance_ * state->HighestOrderStateNorm(),
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

  /** Returns the FemModel that this solver owns. */
  const Model& model() const {
    DRAKE_THROW_UNLESS(model_ != nullptr);
    return *model_;
  }

  /** Returns the mutable FemModel that this solver owns. */
  Model& mutable_model() {
    DRAKE_THROW_UNLESS(model_ != nullptr);
    return *model_;
  }

  /** Takes ownership of the given Dirichlet boundary condition and applies it
   to the FemModel this solver owns. */
  void SetDirichletBoundaryCondition(
      std::unique_ptr<DirichletBoundaryCondition<State>> dirichlet_bc) {
    dirichlet_bc_ = std::move(dirichlet_bc);
  }

  /** Sets the relative tolerance, unitless. The Newton-Raphson iterations are
   considered as converged if the norm of the change in the state in one
   iteration is smaller than the norm of the current state times the relative
   tolerace. The default value is 1e-6. */
  void set_relative_tolerance(const T& tolerance) {
    relative_tolerance_ = tolerance;
  }

  /** Sets the absolute tolerance which has the same unit as the state with the
   highest order of the model. For example, for dynamic elasticity, it has the
   unit of m/s², and for static elasticity, it has the unit of m. The
   Newton-Raphson iterations are considered as converged if the change in the
   state is smaller than the absolute tolerance. The default value is 1e-6. */
  void set_absolute_tolerance(const T& tolerance) {
    absolute_tolerance_ = tolerance;
  }

  /** Sets the relative tolerance for the linear solver used in `this`
   FemSolver if the linear solver is iterative. No-op if the linear solver is
   direct. */
  void set_linear_solve_tolerance(const T& tolerance) {
    linear_solver_->set_tolerance(tolerance);
  }

 private:
  /* Resize the scratch quantities to the size of the model. */
  void Resize() const {
    if (b_.size() != model_->num_dofs()) {
      b_.resize(model_->num_dofs());
      dz_.resize(model_->num_dofs());
      A_.resize(model_->num_dofs(), model_->num_dofs());
      model_->SetTangentMatrixSparsityPattern(&A_);
    }
  }

  /* The FEM model being solved by `this` solver. */
  std::unique_ptr<Model> model_;
  /* The linear solver used to solve the FEM model. */
  std::unique_ptr<internal::LinearSystemSolver<T>> linear_solver_;
  /* The Dirichlet boundary condition imposed on the model. */
  std::unique_ptr<DirichletBoundaryCondition<State>> dirichlet_bc_;
  /* A scratch sparse matrix to store the tangent matrix of the model. */
  mutable Eigen::SparseMatrix<T> A_;
  /* The operator form of A_. */
  const contact_solvers::internal::SparseLinearOperator<T> A_op_{"A", &A_};
  /* A scratch vector to store the residual of the model. */
  mutable VectorX<T> b_;
  /* A scratch vector to store the solution to A * dz = -b. */
  mutable VectorX<T> dz_;
  /* The relative tolerance for determining the convergence of the Newton
   solver, unitless. */
  T relative_tolerance_{1e-6};
  /* The absolute tolerance for determining the convergence of the Newton
   solver. It has the same unit as the highest order state. */
  T absolute_tolerance_{1e-6};
  // TODO(xuchenhan-tri): Consider making `kMaxIterations_` configurable.
  /* Any reasonable Newton solve should converge within 20 iterations. If it
   doesn't converge in 20 iterations, chances are it will never converge. */
  int kMaxIterations_{20};
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
