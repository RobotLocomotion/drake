#pragma once

#include <memory>
#include <unordered_set>
#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/block_sparse_cholesky_solver.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/contact_solvers/schur_complement.h"
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
 to the prescribed discrete time integration scheme.
 @tparam_double_only */
template <typename T>
class FemSolver {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FemSolver);

  /* Constructs a new FemSolver that solves the given `model` with the
   `integrator` provided to advance time.
   @note The `model` and `integrator` pointers persist in `this` FemSolver and
   thus the model and the integrator must outlive this solver.
   @pre model != nullptr.
   @pre integrator != nullptr.*/
  FemSolver(const FemModel<T>* model,
            const DiscreteTimeIntegrator<T>* integrator);

  ~FemSolver();

  /* Advances the state of the FEM model by one time step with the integrator
   prescribed at construction and computes the Schur complement of the tangent
   matrix of the model at the next time step.
   @param[in] prev_state
     The state of the FEM model evaluated at the previous time step.
   @param[in] plant_data
     Data from the MultibodyPlant that owns the FemModel associated with this
     FemSolver at construction.
   @param[in] nonparticipating_vertices
     The vertices of the FEM model that participate in constraint computation,
     used to compute the Schur complement of the tangent matrix of the FEM
     model.
   @returns  the number of solver iterations the solver takes to converge if the
   solver converges or -1 if the solver fails to converge.
   @pre All entries in `nonparticipating_vertices` are in
   [0, prev_state.num_nodes()).
   @note External forces are always evaluated explicitly at the previous time
   step regardless of the time integration scheme used.
   @throws std::exception if the input `prev_state` is incompatible with the FEM
   model solved by this solver. */
  int AdvanceOneTimeStep(
      const FemState<T>& prev_state, const FemPlantData<T>& plant_data,
      const std::unordered_set<int>& nonparticipating_vertices);

  /* Forces the next fem state to be the given state, and sets the the schur
   complement at the next time step to be empty. */
  void SetNextFemState(const FemState<T>& next_state);

  /* Returns the state of the FEM model after last invocation of
   `AdvanceOneTimeStep()` or `SetNextFemState()`. If neither has been called,
   returns the default FEM state. */
  const FemState<T>& next_fem_state() const {
    return *next_state_and_schur_complement_.state;
  }

  /* Returns the Schur complement of the tangent matrix for the FEM model after
   last invocation of `AdvanceOneTimeStep()`. If `AdvanceOneTimeStep()` has
   never been called, returns an empty Schur complement. */
  const contact_solvers::internal::SchurComplement& next_schur_complement()
      const {
    return next_state_and_schur_complement_.schur_complement;
  }

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

  /* The solver is considered as converged if ‖r‖ < max(εᵣ * ‖r₀‖, εₐ) where r
   and r₀ are `residual_norm` and `initial_residual_norm` respectively, and εᵣ
   and εₐ are relative and absolute tolerance respectively. */
  bool solver_converged(const T& residual_norm,
                        const T& initial_residual_norm) const;

  /* Sets the maximum linear solver tolerance for iterative linear solvers. The
   current solver of choice is Eigen::ConjugateGradient. */
  void set_max_linear_solver_tolerance(double tolerance) {
    max_linear_solver_tolerance_ = tolerance;
  }

  /* Returns the maximum linear solver tolerance for iterative linear solvers.
   The default value is 0.1. */
  double max_linear_solver_tolerance() const {
    return max_linear_solver_tolerance_;
  }

  /* Computes the inexact Newton linear solver tolerance according to eq(2.6)
   from [Eisenstat and Walker, 1996]. We choose γ = 1 and α = 2.

   The linear solver tolerance is the relative tolerance for the reconstruction
   error when solving the linear system Ax = b, defined as |Ax-b|/|b|.

   [Eisenstat and Walker, 1996] Eisenstat, Stanley C., and Homer F. Walker.
   "Choosing the forcing terms in an inexact Newton method." SIAM Journal on
   Scientific Computing 17.1 (1996): 16-32.

   @param[in] current_residual   The residual norm from the current Newton
                                 iteration.
   @param[in] previous_residual  The residual norm from the previous Newton
                                 iteration if this is not the first iteration,
                                 otherwise, the value is ignored.
   @param[in] previous_tolerance The linear solver tolerance from the previous
                                 Newton iteration if this is not the first
                                 iteration. Otherwise, a non-positive value to
                                 indicate this is the first iteration. */
  double ComputeLinearSolverTolerance(double current_residual,
                                      double previous_residual,
                                      double previous_tolerance) const;

 private:
  template <typename U>
  friend class FemSolverTest;

  struct FemStateAndSchurComplement {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FemStateAndSchurComplement);

    /* Constructs an FemStateAndSchurComplement that's compatible with the given
     FEM model. */
    explicit FemStateAndSchurComplement(const FemModel<T>& model);

    ~FemStateAndSchurComplement();

    /* Reinitializes `this` data structure if it's incompatible with the given
     FEM model. */
    void ReinitializeIfNeeded(const FemModel<T>& model) {
      if (!model.is_compatible_with(*state)) {
        state = model.MakeFemState();
        schur_complement = contact_solvers::internal::SchurComplement{};
      }
    }

    copyable_unique_ptr<FemState<T>> state;
    contact_solvers::internal::SchurComplement schur_complement;
  };

  struct Scratch {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Scratch);

    /* Constructs a FemSolverScratch that is compatible with the given model. */
    explicit Scratch(const FemModel<T>& model);

    ~Scratch();

    /* Reinitializes `this` scratch if it's incompatible with the given FEM
     model. */
    void ReinitializeIfNeeded(const FemModel<T>& model);

    copyable_unique_ptr<contact_solvers::internal::BlockSparseSymmetricMatrix3d>
        tangent_matrix;
    VectorX<T> b;
    VectorX<T> dz;
  };

  /* Uses a Newton-Raphson solver to solve for the equilibrium FEM state z
   such that the residual is zero, i.e. b(z) = 0, up to the specified
   tolerances. In addition, computes the Schur complement of the tangent
   matrix of the FEM model at that state z. The results are written to the
   member variable `next_state_and_schur_complement_`.
   @param[in] nonparticipating_vertices
     The vertices of the FEM model that participate in the constraint
     computation, used to compute the Schur complement of the tangent matrix.
   @param[in] plant_data
     Data from the MultibodyPlant that owns the FemModel associated with this
     FemSolver at construction.
   @pre the FEM model is nonlinear.
   @returns the number of iterations it takes for the solver to converge or -1
   if the solver fails to converge. */
  int SolveNonlinearModel(
      const FemPlantData<T>& plant_data,
      const std::unordered_set<int>& nonparticipating_vertices);

  /* For a linear FEM model, solves for the equilibrium FEM state z such that
   the residual is zero, i.e. b(z) = 0. In addition, computes the Schur
   complement of the tangent matrix of the FEM model at that state z. The
   results are written to the member variable
   `next_state_and_schur_complement_`.
   @param[in] plant_data
     Data from the MultibodyPlant that owns the FemModel associated with this
     FemSolver at construction.
   @param[in] nonparticipating_vertices
     The vertices of the FEM model that participate in constraint computation,
     used to compute the Schur complement of the tangent matrix.
   @returns 0 if the `input` state is already at equilibrium, 1 otherwise.
   @pre the FEM model is linear. */
  int SolveLinearModel(
      const FemPlantData<T>& plant_data,
      const std::unordered_set<int>& nonparticipating_vertices);

  /* The FEM model being solved by `this` solver. */
  const FemModel<T>* model_{nullptr};
  /* The discrete time integrator the solver uses. */
  const DiscreteTimeIntegrator<T>* integrator_{nullptr};
  /* Tolerance for convergence. */
  double relative_tolerance_{1e-2};  // unitless.
  // TODO(xuchenhan-tri): Consider using an absolute tolerance with velocity
  // unit so that how stiff the material is doesn't affect the convergence
  // criterion.
  double absolute_tolerance_{1e-6};  // unit N.
  /* Maximum allowed solver tolerance for iterative linear solver, used on the
   first Newton iteration and as a safe-guard to prevent the tolerance from
   being too large (>1). Unitless and default to a loose value to avoid solving
   linear systems to unnecessary accuracy in early stages of Newton solve. */
  double max_linear_solver_tolerance_{0.1};
  /* Max number of Newton-Raphson iterations the solver takes before it gives
   up. */
  int max_iterations_{100};
  FemStateAndSchurComplement next_state_and_schur_complement_;
  Scratch scratch_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
