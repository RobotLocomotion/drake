#pragma once

#include <algorithm>
#include <memory>
#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sparse_linear_operator.h"
#include "drake/multibody/fixed_fem/dev/eigen_conjugate_gradient_solver.h"
#include "drake/multibody/fixed_fem/dev/fem_model_base.h"
#include "drake/multibody/fixed_fem/dev/fem_state_base.h"

namespace drake {
namespace multibody {
namespace fem {
// TODO(xuchenhan-tri): Move the implementation of this class to a .cc file.
/** %FemSolver solves for the state of a given FemModel at which the residual of
 the model is sufficiently close to zero. %FemSolver uses a simple
 Newton-Raphson solver to solve for the zero residual state. A common workflow
 for solving a static FEM model looks like:
 ```
 // Creates a solver for the given FemModel.
 FemSolver<double> solver(&model));
 // Optionally, sets the tolerances under which we deem the residual is
 // effectively zero.
 solver.set_absolute_tolerance(kAbsoluteTolereance);
 solver.set_relative_tolerance(kRelativeTolereance);
 // Finally, provide an initial guess and solve for the zero residual state.
 solver.SolveStaticModelWithInitialGuess(&state);
 ```
 The workflow for solving dynamics FEM model is similar. AdvanceOneTimeStep()
 should be called in the place of SolveStaticModelWithInitialGuess().
 @tparam_nonsymbolic_scalar T. */
template <typename T>
class FemSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemSolver);

  // TODO(xuchenhan-tri): Consider allowing users to configure the linear
  //  solver or at the very least, note that solver will not converge if the
  //  tangent matrices that show up in the Newton iterations are not SPD.
  /** Constructs a new %FemSolver with the given FemModelBase and an Eigen
   Conjugate Gradient solver as the linear solver. The `model` pointer persists
   in `this` %FemSolver and thus the FemModelBase object must outlive `this`
   %FemSolver.
   @pre model != nullptr. */
  explicit FemSolver(const FemModelBase<T>* model) : model_(model) {
    DRAKE_DEMAND(model_ != nullptr);
    Resize();
    linear_solver_ =
        std::make_unique<internal::EigenConjugateGradientSolver<T>>(&A_op_);
  }

  /** For dynamic models, advances the given FEM state from the previous time
   step to the next time step. If the FEM model associated with `this` solver is
   static, throw an exception.
   @param[in] prev_state The state of the FEM model evaluated at the previous
   time step.
   @param[in, out] next_state As input, `next_state` provides an initial guess
   for the state of the FEM model at the next time step. As output, `state`
   stores the state of the FEM model evaluated at the next time step.
   @pre next_state != nullptr.
   @pre prev_state.num_generalized_positions() ==
   next_state->num_generalized_positions().
   @throw std::exception if the input `prev_state` or `next_state` is
   incompatible with the FEM model associated with `this` %FemSolver, or if the
   model is not dynamic (see is_model_dynamic()). */
  void AdvanceOneTimeStep(const FemStateBase<T>& prev_state,
                          FemStateBase<T>* next_state) const {
    DRAKE_DEMAND(next_state != nullptr);
    if (!is_model_dynamic()) {
      throw std::logic_error(
          fmt::format("{}() can only be called on a dynamic model!", __func__));
    }
    model_->ThrowIfModelStateIncompatible(__func__, prev_state);
    model_->ThrowIfModelStateIncompatible(__func__, *next_state);
    /* Grab the initial guess for the unknown variable. */
    const VectorX<T>& unknown_variable = model_->GetUnknowns(*next_state);
    model_->AdvanceOneTimeStep(prev_state, unknown_variable, next_state);
    SolveWithInitialGuess(next_state);
  }

  // TODO(xuchenhan-tri): Consider making FemSolver only a Newton-Raphson
  //  solver. In other words, make it ignorant of what kind of model it is
  //  solving for and provide a single method that takes an initial guess and
  //  returns the zero-residual state.
  /** For static models, solves for the state at equilibrium given the initial
   guess. If the FEM model associated with `this` solver is dynamic, throw an
   exception.
   @pre next_state != nullptr.
   @throw std::exception if the input `state` is incompatible with the FEM model
   associated with `this` %FemSolver, or if the model is dynamic (see
   is_model_dynamic()). */
  void SolveStaticModelWithInitialGuess(FemStateBase<T>* state) const {
    DRAKE_DEMAND(state != nullptr);
    if (is_model_dynamic()) {
      throw std::logic_error(
          fmt::format("{}() can only be called on a static model!", __func__));
    }
    model_->ThrowIfModelStateIncompatible(__func__, *state);
    SolveWithInitialGuess(state);
  }

  /** Returns true if the FEM model owned by `this` solver has ODE order greater
   than 0. Returns false otherwise. */
  bool is_model_dynamic() const { return model_->ode_order() > 0; }

  /** Returns the FemModel that this solver is solving. */
  const FemModelBase<T>& model() const { return *model_; }

  /** Sets the relative tolerance, unitless. The Newton-Raphson iterations are
   considered as converged if ‖dz‖ < `tolerance`⋅‖z‖ where z is the unknown
   variable defined in FemModelBase. The default value is 1e-6. */
  void set_relative_tolerance(const T& tolerance) {
    relative_tolerance_ = tolerance;
  }

  /** Sets the absolute tolerance which has the same unit as the unknown
   variable z defined in FemModelBase. For example, for dynamic elasticity, it
   has the unit of m/s², and for static elasticity, it has the unit of m. The
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
  /* Uses a Newton-Raphson solver to solve for the equilibrium state that
   satisfies the tolerances. See set_relative_tolerance() and
   set_absolute_tolerance(). The input `state` is non-null and is guaranteed to
   be compatible with the model owned by `this` solver.
   @param[in, out] state  As input, `state` provides an initial guess of
   the solution. As output, `state` reports the equilibrium state. */
  int SolveWithInitialGuess(FemStateBase<T>* state) const {
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
      model_->CalcTangentMatrix(*state, &A_);
      linear_solver_->Compute();
      /* Solving for A * dz = -b. */
      linear_solver_->Solve(-b_, &dz_);
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
  const FemModelBase<T>* model_;
  /* The linear solver used to solve the FEM model. */
  std::unique_ptr<internal::LinearSystemSolver<T>> linear_solver_;
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
   solver. It has the same unit as the unknown variable z. */
  T absolute_tolerance_{1e-6};
  // TODO(xuchenhan-tri): Consider making `kMaxIterations_` configurable.
  /* Any reasonable Newton solve should converge within 20 iterations. If it
   doesn't converge in 20 iterations, chances are it will never converge. */
  int kMaxIterations_{20};
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
