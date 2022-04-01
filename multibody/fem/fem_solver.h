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

  // TODO(xuchenhan-tri): The existance of the mutable scratch_data makes
  // AdvanceOneTimeStep() not thread-safe. As a result FemSolver can't currently
  // be placed in contact managers.
  /* Advances the state of the FEM model by one time step with the integrator
   prescribed at construction.
   @param[in] prev_state   The state of the FEM model evaluated at the previous
                           time step.
   @param[out] next_state  The state of the FEM model evaluated at the next time
                           step.
   @returns the number of Newton-Raphson iterations the solver takes to
   converge if the solver converges or -1 if the solver fails to converge.
   @pre next_state != nullptr.
   @throws std::exception if the input `prev_state` or `next_state` is
   incompatible with the FEM model solved by this solver. */
  int AdvanceOneTimeStep(const FemState<T>& prev_state,
                         FemState<T>* next_state) const;

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
   and r₀ are the residual and residual at the beginning of the iteration
   respectively, and εᵣ and εₐ are relative and absolute tolerance respectively.
  */
  bool solver_converged(const T& residual_norm,
                        const T& initial_residual_norm) const;

 private:
  /* Holds the scratch data used in the solver to avoid unnecessary
   reallocation. */
  struct SolverScratchData {
    /* Constructs a scratch data that is compatible with the given model. */
    explicit SolverScratchData(const FemModel<T>& model) { Resize(model); }

    /* Resizes scratch data to have sizes compatible with the given `model`. */
    void Resize(const FemModel<T>& model) {
      b.resize(model.num_dofs());
      dz.resize(model.num_dofs());
      tangent_matrix = model.MakePetscSymmetricBlockSparseTangentMatrix();
    }

    int num_dofs() const { return b.size(); }

    std::unique_ptr<internal::PetscSymmetricBlockSparseMatrix> tangent_matrix;
    /* Stores the residual of the model. */
    VectorX<T> b;
    /* Stores the solution to A * dz = -b, where A is the tangent matrix. */
    VectorX<T> dz;
  };

  /* Uses a Newton-Raphson solver to solve for the unknown z such that the
   residual is zero, i.e. b(z) = 0, up to the specified tolerances. The input
   FEM state is non-null and is guaranteed to be compatible with the FEM model.

   @param[in, out] state  As input, `state` provides an initial guess of
   the solution. As output, `state` reports the equilibrium state.
   @returns the number of iterations it takes for the solver to converge or -1
   if the solver fails to converge. */
  int SolveWithInitialGuess(FemState<T>* state) const;

  /* Updates the relative tolerance for the linear solver used in the
   Newton-Raphson iterations based on the residual norm if the linear solver
   is iterative. More specifically, it is set to the minimum of 0.5, the
   relative tolerance of the NewtonSolver, and the given `residual_norm`. No-op
   if the linear solver is direct. */
  void set_linear_solve_tolerance(const T& residual_norm) const;

  /* Resets the scratch data to be consistent with the model if the model has
   been modified. */
  void ResetScratchDataIfNecessary() const;

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
  /* Scratch data used in the solver iteration. */
  mutable SolverScratchData scratch_data_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
