#pragma once

#include <algorithm>
#include <memory>
#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/discrete_time_integrator.h"
#include "drake/multibody/fem/fem_model.h"
#include "drake/multibody/fem/fem_state.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* FemSolver solves discrete dynamic elasticity problems. The governing PDE of
 the dynamics is spatially discretized in FemModel and temporally
 discretized by DiscreteTimeIntegrator. FemSolver provides the
 AdvanceOneTimeStep method that advances the states of the spatially discretized
 FEM model by one time step according to the prescribed discrete time
 integration scheme using a Newton-Raphson solver.
 @tparam_nonsymbolic_scalar. */
template <typename T>
class FemSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemSolver);

  /* Max number of Newton-Raphson iterations the solver takes before it gives
   up. */
  static constexpr int kMaxIterations = 100;

  /* Constructs a new FemSolver that solves the given `model` with the
   `integrator` provided to advance time.
   @note The `model` and `integrator` pointers persist in `this` FemSolver and
   thus the model and the integrator must outlive this solver.
   @pre model != nullptr.
   @pre integrator != nullptr.*/
  FemSolver(const FemModel<T>* model,
            const DiscreteTimeIntegrator<T>* integrator);

  /* Advances the state of the FEM model by one time step with the integrator
   prescribed at construction.
   @param[in] prev_state   The state of the FEM model evaluated at the previous
                           time step.
   @param[out] next_state  The state of the FEM model evaluated at the next time
                           step.
   @returns the number of Newton-Raphson iterations the solver takes to
   converge.
   @pre next_state != nullptr.
   @pre prev_state.num_dofs() == next_state->dofs().
   @throw std::exception if the input `prev_state` or `next_state` is
   incompatible with the FEM model solved by this solver.
   @throw std::exception if the solver doesn't converge after `kMaxIterations`
   Newton-Raphson iterations. */
  int AdvanceOneTimeStep(const FemState<T>& prev_state,
                         FemState<T>* next_state) const;

  /* Returns the FEM model that this solver solves for. */
  const FemModel<T>& model() const { return *model_; }

  /* Returns the discrete time integrator that this solver uses. */
  const DiscreteTimeIntegrator<T>& integrator() const { return *integrator_; }

  /* Sets the relative tolerance, unitless. The Newton-Raphson iterations are
   considered as converged if the norm of the residual is smaller than the
   relative tolerance times the norm of the residual at the start of the
   Newton-Raphson iterations. The default value is 1e-4. */
  void set_relative_tolerance(const T& tolerance) {
    relative_tolerance_ = tolerance;
  }

  const T& relative_tolerance() const { return relative_tolerance_; }

  /* Sets the absolute tolerance with unit N. The Newton-Raphson iterations are
   considered as converged if the norm of the residual is smaller than the
   absolute tolerance. The default value is 1e-6. */
  void set_absolute_tolerance(const T& tolerance) {
    absolute_tolerance_ = tolerance;
  }

  const T& absolute_tolerance() const { return absolute_tolerance_; }

 private:
  /* Uses a Newton-Raphson solver to solve for the equilibrium state that
   satisfies the tolerances. See set_relative_tolerance() and
   set_absolute_tolerance() for convergence criteria. The input FEM state is
   non-null and is guaranteed to be compatible with the FEM model.
   @param[in, out] state  As input, `state` provides an initial guess of
   the solution. As output, `state` reports the equilibrium state. */
  int SolveWithInitialGuess(FemState<T>* state) const;

  /* Reset the scratch data in this class (tangent matrix, residual, and dz) if
   necessary. */
  void ResetScratchDataIfNecessary() const;

  /* Updates the relative tolerance for the linear solver used in the
   Newton-Raphson iterations based on the residual norm if the linear solver is
   iterative. No-op if the linear solver is direct. */
  void set_linear_solve_tolerance(const T& residual_norm) const;

  /* The FEM model being solved by `this` solver. */
  const FemModel<T>* model_;
  /* The discrete time integrator the solver uses. */
  const DiscreteTimeIntegrator<T>* integrator_;
  /* A scratch sparse matrix to store the tangent matrix of the model. We use
   PETSc matrix for T=double and an Eigen::SparseMatrix otherwise. */
  mutable Eigen::SparseMatrix<T> tangent_matrix_eigen_;
  mutable std::unique_ptr<internal::PetscSymmetricBlockSparseMatrix>
      tangent_matrix_petsc_;
  /* Solver for the tangent matrix when T!=double.  */
  mutable Eigen::ConjugateGradient<Eigen::SparseMatrix<T>,
                                   Eigen::Lower | Eigen::Upper>
      eigen_tangent_matrix_solver_;
  /* A scratch vector to store the residual of the model. */
  mutable VectorX<T> b_;
  /* A scratch vector to store the solution to A * dz = -b, where A is the
   tangent matrix. */
  mutable VectorX<T> dz_;

  T relative_tolerance_{1e-4};
  T absolute_tolerance_{1e-6};
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::FemSolver);
