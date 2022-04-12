#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/multibody/contact_solvers/sap/sap_model.h"
#include "drake/multibody/contact_solvers/sap/sap_solver_results.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// The result from SapSolver::SolveWithGuess() used to report the success or
// failure of the solver.
enum class SapSolverStatus {
  // Successful computation.
  kSuccess = 0,

  // The solver could not find a solution at the specified tolerances.
  kFailure = 1,
};

// SAP solver parameters such as tolerances, maximum number of iterations and
// regularization parameters.
struct SapSolverParameters {
  // Stopping Criteria:
  //   SAP uses two stopping criteria, one on the optimality condition and a
  // second one on the cost, see specifics below for each criteria. SAP
  // terminates with a success status whenever one of these criteria is
  // satisfied first. The criteria on the cost is meant to detect when the
  // solver stalls due to round-off errors. When this happens, the solver
  // terminates and returns the best solution it could find with finite
  // precision arithmetic. The solver cannot do better pass this point. Under
  // normal circumstances, for a well conditioned problem, nominal values
  // documented below are designed so that the solver reaches the optimality
  // condition first. With the values documented below, it is expected that the
  // solver meets the cost condition first only when the problem is
  // ill-conditioned or when the user requests a tight tolerance below the
  // recommended nominal values. Most often SAP returns a solution to a very
  // tight accuracy, with an error significantly below the requested tolerance
  // and often close to machine precision [Castro et al., 2021]. The cost
  // criterion is meant for rare events at which the state of the multibody
  // system leads to an ill-conditioned problem for which reducing the error
  // below the specified tolerance is not possible in practice due to round-off
  // errors.

  // Optimality condition criterion: We monitor the optimality condition (for
  // SAP, balance of momentum), i.e. ‖∇ℓ‖ < εₐ + εᵣ max(‖p‖,‖jc‖),
  // where ∇ℓ = A⋅(v−v*)−Jᵀγ is the momentum balance residual, p = A⋅v and jc =
  // Jᵀ⋅γ. The norms above are defined as ‖x‖ = ‖D⋅x‖₂, where D = diag(A)^(1/2).
  // If p is a generalized momentum, then D⋅p is a scaled generalized momentum
  // where each component has the same units, square root of Joules. Therefore
  // the norms above are used to weigh all components of the generalized
  // momentum equally.
  //
  // Nominal values:
  //  * For rel_tolerance > 1.0e-5 the solver will have no problem reaching this
  //    condition.
  //  * At rel_tolerance ≈ 1.0e-6, the solver might reach the cost condition
  //    (below) first due to round-off errors, but mostly for ill conditioned
  //    problems.
  //  * For rel_tolerance ≲ 1.0e-7 the solver will most likely reach the cost
  //    condition first.
  //
  // SolverStats::optimality_condition_reached indicates if this condition was
  // reached.
  double abs_tolerance{1.e-14};  // Absolute tolerance εₐ, square root of Joule.
  double rel_tolerance{1.e-6};  // Relative tolerance εᵣ.

  // Cost condition criterion: We monitor the decrease of the cost on each
  // iteration. It is not worth it to keep iterating if round-off errors do not
  // allow the cost to keep decreasing. Therefore SAP stops the Newton iteration
  // when the optimality condition OR the cost condition is satisfied. Given the
  // costs ℓᵐ and ℓᵐ⁺¹ at Newton iterations m and m+1 respectively, the cost
  // condition is: |ℓᵐ⁺¹−ℓᵐ| < εₐ + εᵣ (ℓᵐ⁺¹+ℓᵐ)/2.
  //
  // Interaction with the optimality condition (abs_tolerance, rel_tolerance):
  // The purpose of this condition is to detect when the solver reaches the best
  // solution within round-off errors. We expect the optimality condition to be
  // satisfied first for values of the optimality tolerances within nominal
  // values. However, we expect to reach the cost condition when:
  //  1. Optimality condition tolerances are set below nominal values.
  //  2. Ill conditioning of the problem makes reaching the optimality condition
  //     difficult, specially when in the lower range of nominal values.
  //
  // SolverStats::cost_condition_reached indicates if this condition was
  // reached.
  //
  // Nominal values:
  //  * cost_rel_tolerance: we want to detect when changes in the cost are close
  //    to machine epsilon. Since round-off errors are typically larger than
  //    machine epsilon, we typically use larger values.
  //  * cost_abs_tolerance: This has units of Joule. Here we use a value that is
  //    a few orders of magnitude smaller than abs_tolerance (with units of
  //    square root of Joule) squared.
  double cost_abs_tolerance{1.e-30};  // Absolute tolerance εₐ, in Joules.
  double cost_rel_tolerance{1.e-15};  // Relative tolerance εᵣ.
  int max_iterations{100};            // Maximum number of Newton iterations.

  // Line-search parameters.
  double ls_alpha_max{1.5};   // Maximum line search parameter allowed.
  int ls_max_iterations{40};  // Maximum number of line search iterations.
  double ls_c{1.0e-4};        // Armijo's criterion parameter.
  double ls_rho{0.8};         // Backtracking search parameter.

  // Tolerance used in impulse soft norms. In Ns.
  double soft_tolerance{1.0e-7};
};

// This class implements the Semi-Analytic Primal (SAP) solver described in
// [Castro et al., 2021].
//
// SAP uses the convex approximation of contact constraints by [Anitescu, 2006],
// with constraint regularization and analytical inverse dynamics introduced by
// [Todorov, 2014]. However SAP introduces a primal formulation in velocities
// instead of in impulses as done in previous work. This leads to a numerical
// scheme that warm-starts very effectively using velocities from the previous
// time step. In addition, SAP uses regularization to model physical compliance
// rather than to introduce constraint stabilization as previously done by
// [Todorov, 2014]. Please refer to [Castro et al., 2021] for details.
//
// - [Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An
//   Unconstrained Convex Formulation of Compliant Contact. Available online at
//   https://arxiv.org/abs/2110.10107.
// - [Anitescu, 2006] Anitescu M., 2006. Optimization-based simulation of
//   nonsmooth rigid multibody dynamics. Mathematical Programming, 105(1),
//   pp.113-143.
// - [Todorov, 2014] Todorov, E., 2014, May. Convex and analytically-invertible
//   dynamics with contacts and constraints: Theory and implementation in
//   MuJoCo. In 2014 IEEE International Conference on Robotics and Automation
//   (ICRA) (pp. 6054-6061). IEEE.
//
// TODO(amcastro-tri): enable AutoDiffXd support, if only for dense matrices so
// that we can test the long term performant solution.
// @tparam_nonsymbolic_scalar
template <typename T>
class SapSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapSolver);

  // Struct used to store statistics for each solve by SolveWithGuess().
  struct SolverStats {
    // Initializes counters and time statistics to zero.
    void Reset() {
      num_iters = 0;
      num_line_search_iters = 0;
      optimality_criterion_reached = false;
      cost_criterion_reached = false;
    }
    int num_iters{0};              // Number of Newton iterations.
    int num_line_search_iters{0};  // Total number of line search iterations.

    // Indicates if the optimality condition was reached.
    bool optimality_criterion_reached{false};

    // Indicates if the cost condition was reached.
    bool cost_criterion_reached{false};
  };

  SapSolver() = default;

  // Solve the contact problem specified by the input data. Currently, only `T =
  // double` is supported. An exception is thrown if `T != double`.
  //
  // Convergence of the solver is controlled by set_parameters(). Refer to
  // SapSolverParameters for details on the convergence conditions.
  //
  // N.B. SolveWithGuess() is a non-const method and therefore changes to the
  // state of the SapSolver object are allowed. This means that when using this
  // solver in MultibodyPlant (or a DiscreteUpdateManager), it must either be
  // instantiated locally or stored within a Context cache entry to ensure
  // thread safety.
  SapSolverStatus SolveWithGuess(const SapContactProblem<T>& problem,
                                 const VectorX<T>& v_guess,
                                 SapSolverResults<T>* result);

  // New parameters will affect the next call to SolveWithGuess().
  void set_parameters(const SapSolverParameters& parameters);

  // Returns solver statistics from the last call to SolveWithGuess().
  // Statistics are reset with SolverStats::Reset() on each new call to
  // SolveWithGuess().
  const SolverStats& get_statistics() const;

 private:
  friend class SapSolverTester;

  // Struct used to store the result of computing the search direction. We store
  // the search direction in the generalized velocities, dv, as well as
  // additional derived quantities such as the generalized momentum update dp
  // and the constraints' velocities update dvc.
  struct SearchDirectionData {
    SearchDirectionData(int num_velocities, int num_constraint_equations) {
      dv.resize(num_velocities);
      dp.resize(num_velocities);
      dvc.resize(num_constraint_equations);
      d2ellA_dalpha2 = NAN;
    }
    VectorX<T> dv;          // Search direction.
    VectorX<T> dp;          // Momentum update Δp = A⋅Δv.
    VectorX<T> dvc;         // Constraints velocities update, Δvc=J⋅Δv.
    T d2ellA_dalpha2{NAN};  // d²ellA/dα² = Δvᵀ⋅A⋅Δv.
  };

  // Pack solution into SapSolverResults. Where v is the vector of
  // generalized velocities, vc is the vector of contact velocities and gamma is
  // the vector of generalized contact impulses.
  // @pre context was created by the underlying SapModel.
  void PackSapSolverResults(const systems::Context<T>& context,
                            SapSolverResults<T>* results) const;

  // We monitor the optimality condition (for SAP, balance of momentum), i.e.
  // ‖∇ℓ‖ < εₐ + εᵣ max(‖p‖,‖j‖), where ∇ℓ = A⋅(v−v*)−Jᵀγ is the momentum
  // balance residual, p = A⋅v and j = Jᵀ⋅γ. The norms above are weighted as ‖x‖
  // = ‖D⋅x‖₂ where D = diag(A)^(1/2), such that all generalized momenta have
  // the same units (square root of Joules).
  // This method computes momentum_residual = ‖∇ℓ‖ and momentum_scale =
  // max(‖p‖,‖j‖). See [Castro et al., 2021] for further details.
  // @pre context was created by the underlying SapModel.
  void CalcStoppingCriteriaResidual(const systems::Context<T>& context,
                                    T* momentum_residual,
                                    T* momentum_scale) const;

  // Computes the cost ℓ(α) = ℓ(vᵐ + αΔvᵐ) for line search, where vᵐ and Δvᵐ are
  // the last Newton iteration values of generalized velocities and search
  // direction, respectively. This methods uses the O(n) strategy described in
  // [Castro et al., 2021].
  // @pre context was created by the underlying SapModel.
  T CalcCostAlongLine(const systems::Context<T>& context,
                      const SearchDirectionData& search_direction_data,
                      const T& alpha, systems::Context<T>* scratch) const;

  // Approximation to the 1D minimization problem α = argmin ℓ(α) = ℓ(v + αΔv)
  // over α. We define ϕ(α) = ℓ₀ + α c ℓ₀', where ℓ₀ = ℓ(0), ℓ₀' = dℓ/dα(0) and
  // c is the Armijo's criterion parameter. With this definition the Armijo
  // condition reads ℓ(α) < ϕ(α). This approximate method seeks to minimize ℓ(α)
  // over a discrete set of values given by the geometric progression αᵣ =
  // ρʳαₘₐₓ with r an integer, 0 < ρ < 1 and αₘₐₓ the maximum value of α
  // allowed. That is, the exact problem is replaced by a search over the
  // discrete values αᵣ until Armijo's criteria is satisfied. The satisfaction
  // of Armijo's criteria allows to prove the global convergence of SAP.
  // For a good reference on this method, see Section 11.3 of Bierlaire, M.,
  // 2015. "Optimization: principles and algorithms", EPFL Press.
  //
  // @returns A pair (α, num_iterations) where α satisfies Armijo's criterion
  // and num_iterations is the number of backtracking iterations performed.
  // @pre both context and scratch_workspace were created by the underlying
  // SapModel.
  std::pair<T, int> PerformBackTrackingLineSearch(
      const systems::Context<T>& context,
      const SearchDirectionData& search_direction_data,
      systems::Context<T>* scratch_workspace) const;

  // Solves for dv using dense algebra, for debugging.
  // @pre context was created by the underlying SapModel.
  // TODO(amcastro-tri): Add AutoDiffXd support.
  void CallDenseSolver(const systems::Context<T>& context,
                       VectorX<T>* dv) const;

  // This method performs one iteration of the SAP solver. It updates gradient
  // of the primal cost ∇ℓₚ, the cost's Hessian H and solves for the velocity
  // search direction dv = −H⁻¹⋅∇ℓₚ. The result is stored in `data` along with
  // additional derived quantities from dv.
  // @pre context was created by the underlying SapModel.
  void CalcSearchDirectionData(const systems::Context<T>& context,
                               SearchDirectionData* data) const;

  std::unique_ptr<SapModel<T>> model_;
  SapSolverParameters parameters_;
  // Stats are mutable so we can update them from within const methods (e.g.
  // Eval() methods). Nothing in stats is allowed to affect the computation; it
  // is purely a passive observer.
  // TODO(amcastro-tri): Consider moving stats into the solver's state stored as
  // part of the model's context.
  mutable SolverStats stats_;
};

// Forward-declare specializations, prior to DRAKE_DECLARE... below.
template <>
SapSolverStatus SapSolver<double>::SolveWithGuess(
    const SapContactProblem<double>&, const VectorX<double>&,
    SapSolverResults<double>*);

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapSolver);

