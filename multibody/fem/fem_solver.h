#pragma once

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/discrete_time_integrator.h"
#include "drake/multibody/fem/fem_model.h"
#include "drake/multibody/fem/fem_state.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Holds the scratch data used in the solver to avoid unnecessary
 reallocation.
 @tparam_double_only */
template <typename T>
class FemSolverScratchData {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemSolverScratchData);

  /* Constructs a scratch data that is compatible with the given model. */
  explicit FemSolverScratchData(const FemModel<T>& model) { Resize(model); }

  /* Resizes scratch data to have sizes compatible with the given `model`. */
  void Resize(const FemModel<T>& model);

  std::unique_ptr<FemSolverScratchData<T>> Clone() const;

  int num_dofs() const { return b_.size(); }

  /* Returns the residual of the model. */
  const VectorX<T>& b() const { return b_; }
  /* Returns the solution to A * dz = -b, where A is the tangent matrix. */
  const VectorX<T>& dz() const { return dz_; }
  const internal::PetscSymmetricBlockSparseMatrix& tangent_matrix() const {
    return *tangent_matrix_;
  }

  VectorX<T>& mutable_b() { return b_; }
  VectorX<T>& mutable_dz() { return dz_; }
  internal::PetscSymmetricBlockSparseMatrix& mutable_tangent_matrix() {
    return *tangent_matrix_;
  }

 private:
  /* Private default constructor to facilitate cloning. */
  FemSolverScratchData() = default;

  std::unique_ptr<internal::PetscSymmetricBlockSparseMatrix> tangent_matrix_;
  VectorX<T> b_;
  VectorX<T> dz_;
};

/* FemSolver solves discrete dynamic elasticity problems. The governing PDE of
 the dynamics is spatially discretized in FemModel and temporally discretized by
 DiscreteTimeIntegrator. FemSolver provides the `AdvanceOneTimeStep()` function
 that advances the free-motion states (i.e. without considering contacts or
 constraints) of the spatially discretized FEM model by one time step according
 to the prescribed discrete time integration scheme using a Newton-Raphson
 solver.
 @tparam_double_only */
template <typename T>
class FemSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemSolver);

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
   @param[in] prev_state    The state of the FEM model evaluated at the previous
                            time step.
   @param[out] next_state   The state of the FEM model evaluated at the next
                            time step.
   @param[in, out] scratch  A scratch pad for storing intermediary data used in
                            the computation. We use this scratch only to avoid
                            memory allocation. The actual value of scratch is
                            unused. The size of scratch will be set to be
                            compatible with the model referenced by this solver
                            on output if it's not already appropriately sized.
   @returns the number of Newton-Raphson iterations the solver takes to
   converge if the solver converges or -1 if the solver fails to converge.
   @pre next_state != nullptr.
   @throws std::exception if the input `prev_state` or `next_state` is
   incompatible with the FEM model solved by this solver. */
  int AdvanceOneTimeStep(const FemState<T>& prev_state, FemState<T>* next_state,
                         FemSolverScratchData<T>* scratch) const;

  /* Returns the FEM model that this solver solves for. */
  const FemModel<T>& model() const { return *model_; }

  /* Returns the discrete time integrator that this solver uses. */
  const DiscreteTimeIntegrator<T>& integrator() const { return *integrator_; }

  /* Sets the relative tolerance, unitless. See solver_converged() for how
   the tolerance is used. The default value is 1e-4. */
  void set_relative_tolerance(double tolerance) {
    relative_tolerance_ = tolerance;
  }

  double relative_tolerance() const { return relative_tolerance_; }

  /* Sets the absolute tolerance with unit Newton. See solver_converged() for
   how the tolerance is used. The default value is 1e-6. */
  void set_absolute_tolerance(double tolerance) {
    absolute_tolerance_ = tolerance;
  }

  double absolute_tolerance() const { return absolute_tolerance_; }

  /* The solver is considered as converged if ‖r‖ <= max(εᵣ * ‖r₀‖, εₐ) where r
   and r₀ are `residual_norm` and `initial_residual_norm` respectively, and εᵣ
   and εₐ are relative and absolute tolerance respectively. */
  bool solver_converged(const T& residual_norm,
                        const T& initial_residual_norm) const;

 private:
  /* Uses a Newton-Raphson solver to solve for the unknown z such that the
   residual is zero, i.e. b(z) = 0, up to the specified tolerances. The input
   FEM state is non-null and is guaranteed to be compatible with the FEM model.

   @param[in, out] state  As input, `state` provides an initial guess of
   the solution. As output, `state` reports the equilibrium state.
   @returns the number of iterations it takes for the solver to converge or -1
   if the solver fails to converge. */
  int SolveWithInitialGuess(FemState<T>* state,
                            FemSolverScratchData<T>* scratch) const;

  /* Returns the relative tolerance for the linear solver used in the
   Newton-Raphson iterations based on the residual norm if the linear solver
   is iterative. More specifically, it is set to
   tol = min(k * εᵣ, ‖r‖ / max(‖r₀‖, εₐ)),
   where k is a constant scaling factor < 1, εᵣ and εₐ are the relative and
   absolute tolerances for newton iterations (see set_relative_tolerance() and
   set_absolute_tolerance()), and ‖r‖ and ‖r₀‖ are `residual_norm` and
   `initial_residual_norm`. */
  double linear_solve_tolerance(const T& residual_norm,
                                const T& initial_residual_norm) const;

  /* The FEM model being solved by `this` solver. */
  const FemModel<T>* model_{nullptr};
  /* The discrete time integrator the solver uses. */
  const DiscreteTimeIntegrator<T>* integrator_{nullptr};
  /* Tolerance for convergence. */
  double relative_tolerance_{1e-4};  // unitless.
  double absolute_tolerance_{1e-6};  // unit N.
  /* Max number of Newton-Raphson iterations the solver takes before it gives
   up. */
  int kMaxIterations_{100};
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
