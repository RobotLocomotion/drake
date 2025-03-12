#pragma once

#include <limits>
#include <memory>
#include <type_traits>
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
  // Line search method.
  enum LineSearchType {
    // Approximate backtracking method with Armijo criterion.
    kBackTracking,
    // Newton method to find the root of dℓ(α)/dα to high accuracy.
    kExact,
  };

  // Parameters for the backtracking line search.
  // Ignored if line_search_type != LineSearchType::kBackTracking.
  struct BackTrackingLineSearchParameters {
    int max_iterations{40};  // Maximum number of backtracking iterations.
    double armijos_parameter{1.0e-4};  // Armijo's criterion parameter.
    double rho{0.8};                   // Backtracking parameter.
    // Maximum line search step size allowed.
    // Using this value of alpha_max ensures that the backtracking line search
    // uses alpha = 1.0 on the second iteration. This is particularly important
    // to avoid overrelaxation of Newton's method in regions where the cost is
    // quadratic, or close to quadratic.
    double alpha_max{1.0 / rho};
  };

  // Parameters for the exact line search.
  // Ignored if line_search_type != LineSearchType::kExact.
  struct ExactLineSearchParameters {
    // Maximum number of iterations allowed. Since our one-dimensional strategy
    // switches to bisection when Newton convergence is slow, this means that
    // the step size parameter will be computed with precision
    // 0.5^max_iterations. We very seldom need to change this parameter.
    int max_iterations{100};
    // Maximum line search step size allowed.
    double alpha_max{1.5};
  };

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
  // SapStatistics::optimality_condition_reached indicates if this condition
  // was reached.
  double abs_tolerance{1.e-14};  // Absolute tolerance εₐ, square root of Joule.
  double rel_tolerance{1.e-6};   // Relative tolerance εᵣ.

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
  // SapStatistics::cost_condition_reached indicates if this condition was
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

  LineSearchType line_search_type{LineSearchType::kExact};

  BackTrackingLineSearchParameters backtracking_line_search;

  ExactLineSearchParameters exact_line_search;

  // Tolerance used in impulse soft norms. In Ns.
  double soft_tolerance{1.0e-7};

  // Dimensionless number used to allow some slop on the check near zero for
  // certain quantities such as the gradient of the cost.
  // It is also used to check for monotonic convergence. In particular, we allow
  // a small increase in the cost due to round-off errors
  //   ℓᵏ ≤ ℓᵏ⁻¹ + ε
  // where ε = relative_slop*max(1, (ℓᵏ+ℓᵏ⁻¹)/2).
  // If this condition is not satisfied and nonmonotonic_convergence_is_error =
  // true, SapSolver throws an exception.
  double relative_slop{1000 * std::numeric_limits<double>::epsilon()};

  // (For debugging) Even though SAP's convergence in monotonic, round-off
  // errors could cause small cost increases on the order of machine epsilon.
  // SAP's implementation uses a `realtive_slop` so that round-off errors do not
  // cause false negatives. For debugging purposes however, this options allows
  // to trigger an exception if the cost increases. For details, see
  // documentation on `relative_slop`.
  bool nonmonotonic_convergence_is_error{false};

  SapHessianFactorizationType linear_solver_type{
      SapHessianFactorizationType::kBlockSparseCholesky};
};

// Struct used to store SAP solver statistics.
struct SapStatistics {
  // Initializes counters and time statistics to zero.
  void Reset() {
    num_iters = 0;
    num_line_search_iters = 0;
    optimality_criterion_reached = false;
    cost_criterion_reached = false;
    momentum_residual.clear();
    momentum_scale.clear();
    cost.clear();
    alpha.clear();
  }
  int num_iters{0};              // Number of Newton iterations.
  int num_line_search_iters{0};  // Total number of line search iterations.

  // Indicates if the optimality condition was reached.
  bool optimality_criterion_reached{false};

  // Indicates if the cost condition was reached.
  bool cost_criterion_reached{false};

  // Cost at each SAP Newton iteration. cost[0] stores cost at the initial
  // guess.
  std::vector<double> cost;

  // Line search step size at each SAP Newton iteration. alpha[0] stores alpha
  // = 1.
  std::vector<double> alpha;

  // Dimensionless momentum residual at each SAP Newton iteration. Of size
  // num_iters + 1.
  std::vector<double> momentum_residual;

  // Dimensionless momentum scale at each SAP Newton iteration. Of size
  // num_iters + 1.
  std::vector<double> momentum_scale;
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

  SapSolver() = default;

  // Solve the contact problem specified by the input data. Currently, only `T =
  // double` is fully supported. An exception is thrown if `T != double` and the
  // set of constraints is non-empty.
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
  // Statistics are reset with SapStatistics::Reset() on each new call to
  // SolveWithGuess().
  const SapStatistics& get_statistics() const;

 private:
  friend class SapSolverTester;
  template <typename U>
  friend class SapSolver;

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

  // This method takes a full vector of SapContactProblem velocities (including
  // both participating and non-participating DOFs) and stores the participating
  // velocities into the `context` for a `model`.
  void SetProblemVelocitiesIntoModelContext(
      const SapModel<T>& model, const VectorX<T>& v_problem,
      systems::Context<T>* context) const {
    Eigen::VectorBlock<VectorX<T>> v_model =
        model.GetMutableVelocities(context);
    model.velocities_permutation().Apply(v_problem, &v_model);
  }

  // Helper method to implement the SolveWithGuess() public API. This helper
  // takes a `model` and a model `context` used by the solver to work with. On
  // input, `context` stores a guess to the solution. On exit, `context` stores
  // the solution to the problem.
  // @pre context is not nullptr.
  // @pre context was created via a call to model.MakeContext().
  SapSolverStatus SolveWithGuessImpl(const SapModel<T>& model,
                                     systems::Context<T>* context)
    requires std::is_same_v<T, double>;

  // Pack solution into SapSolverResults. Where v is the vector of
  // generalized velocities, vc is the vector of contact velocities and gamma is
  // the vector of generalized contact impulses.
  // @pre context was created by the underlying SapModel.
  void PackSapSolverResults(const SapModel<T>& model,
                            const systems::Context<T>& context,
                            SapSolverResults<T>* results) const;

  // We monitor the optimality condition (for SAP, balance of momentum), i.e.
  // ‖∇ℓ‖ < εₐ + εᵣ max(‖p‖,‖j‖), where ∇ℓ = A⋅(v−v*)−Jᵀγ is the momentum
  // balance residual, p = A⋅v and j = Jᵀ⋅γ. The norms above are weighted as ‖x‖
  // = ‖D⋅x‖₂ where D = diag(A)^(1/2), such that all generalized momenta have
  // the same units (square root of Joules).
  // This method computes momentum_residual = ‖∇ℓ‖ and momentum_scale =
  // max(‖p‖,‖j‖). See [Castro et al., 2021] for further details.
  // @pre context was created by the underlying SapModel.
  void CalcStoppingCriteriaResidual(const SapModel<T>& model,
                                    const systems::Context<T>& context,
                                    T* momentum_residual,
                                    T* momentum_scale) const;

  // Computes the cost ℓ(α) = ℓ(vᵐ + αΔvᵐ) for line search, where vᵐ and Δvᵐ are
  // the last Newton iteration values of generalized velocities and search
  // direction, respectively. This methods uses the O(n) strategy described in
  // [Castro et al., 2021].
  //
  // @param context A SapModel context storing the state of the underlying
  //   model.
  // @param search_direction_data Search direction Δv and derived data.
  // @param alpha Step size α along Δv.
  //   Cost will be computed at ℓ(α) = ℓ(vᵐ + αΔvᵐ).
  // @param scratch A SapModel context used as a scratchpad. Values stored in
  //   this context do not affect the result of the computation. scratch must
  //   not be nullptr and scratch != &context.
  // @param dell_dalpha If not nullptr, on return dell_dalpha contains the value
  //   of the derivative dℓ/dα = ∇ℓ(vᵐ)⋅Δvᵐ.
  // @param d2ell_dalpha2 If not nullptr then on return d2ell_dalpha2 contains
  //   the value of the second derivative d²ℓ/dα².
  // @param d2ell_dalpha2_scratch A scratchpad needed when computing
  //   d2ell_dalpha2. Must not be nullptr if d2ell_dalpha2 != nullptr.
  T CalcCostAlongLine(const SapModel<T>& model,
                      const systems::Context<T>& context,
                      const SearchDirectionData& search_direction_data,
                      const T& alpha, systems::Context<T>* scratch,
                      T* dell_dalpha = nullptr, T* d2ell_dalpha2 = nullptr,
                      VectorX<T>* d2ell_dalpha2_scratch = nullptr) const;

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
  // @param context A SapModel context storing the state of the underlying
  //   model.
  // @param search_direction_data Search direction Δv and derived data.
  // @param scratch_workspace A SapModel context used as a scratchpad. Values
  //   stored in this context do not affect the result of the computation.
  //   scratch_workspace must not be nullptr and scratch_workspace != &context.
  //
  // @returns A pair (α, num_iterations) where α satisfies Armijo's criterion
  // and num_iterations is the number of backtracking iterations performed.
  std::pair<T, int> PerformBackTrackingLineSearch(
      const SapModel<T>& model, const systems::Context<T>& context,
      const SearchDirectionData& search_direction_data,
      systems::Context<T>* scratch_workspace) const;

  // Solves α = argmin ℓ(α) = ℓ(v + αΔv) using a Newton-based method.
  //
  // @param context A SapModel context storing the state of the underlying
  //   model.
  // @param search_direction_data Search direction Δv and derived data.
  // @param scratch_workspace A SapModel context used as a scratchpad. Values
  //   stored in this context do not affect the result of the computation.
  //   scratch_workspace must not be nullptr and scratch_workspace != &context.
  //
  // @returns A pair (α, num_iterations) with α the optimal line search step
  // size and num_iterations is the number of iterations performed.
  std::pair<T, int> PerformExactLineSearch(
      const SapModel<T>& model, const systems::Context<T>& context,
      const SearchDirectionData& search_direction_data,
      systems::Context<T>* scratch_workspace) const
    requires std::is_same_v<T, double>;

  // Computes a dense Hessian H(v) = A + Jᵀ⋅G(v)⋅J for the generalized
  // velocities state stored in `context`.
  MatrixX<T> CalcDenseHessian(const SapModel<T>& model,
                              const systems::Context<T>& context) const;

  // Makes a new SuperNodalSolver compatible with the underlying SapModel.
  std::unique_ptr<SuperNodalSolver> MakeSuperNodalSolver(
      const SapModel<T>& model) const;

  // Evaluates the constraint's Hessian G(v) and updates `supernodal_solver`'s
  // weight matrix so that we can later on solve the Newton system with Hessian
  // H(v) = A + Jᵀ⋅G(v)⋅J.
  void UpdateSuperNodalSolver(const SapModel<T>& model,
                              const systems::Context<T>& context,
                              SuperNodalSolver* supernodal_solver) const;

  // Updates the supernodal solver with the constraint's Hessian G(v),
  // factorizes it, and solves for the search direction `dv`.
  // @pre supernodal_solver and dv are not nullptr.
  // @pre supernodal_solver was created with a call to MakeSuperNodalSolver().
  void CallSuperNodalSolver(const SapModel<T>& model,
                            const systems::Context<T>& context,
                            SuperNodalSolver* supernodal_solver,
                            VectorX<T>* dv) const;

  // Solves for dv using dense algebra, for debugging.
  // @pre context was created by the underlying SapModel.
  // TODO(amcastro-tri): Add AutoDiffXd support.
  void CallDenseSolver(const SapModel<T>& model,
                       const systems::Context<T>& context,
                       VectorX<T>* dv) const;

  // This method performs one iteration of the SAP solver. It updates gradient
  // of the primal cost ∇ℓₚ, the cost's Hessian H and solves for the velocity
  // search direction dv = −H⁻¹⋅∇ℓₚ. The result is stored in `data` along with
  // additional derived quantities from dv.
  // @param supernodal_solver If nullptr, this method uses dense algebra to
  // compute the Hessian and factorize it. Otherwise, this method uses the
  // supernodal solver provided.
  // @pre context was created by the underlying SapModel.
  // @pre supernodal_solver must be a valid supernodal solver created with
  // MakeSuperNodalSolver() when
  // parameters_.linear_solver_type != LinearSolverType::kDense.
  void CalcSearchDirectionData(const SapModel<T>& model,
                               const systems::Context<T>& context,
                               SearchDirectionData* data)
    requires std::is_same_v<T, double>;

  SapSolverParameters parameters_;
  // Stats are mutable so we can update them from within const methods (e.g.
  // Eval() methods). Nothing in stats is allowed to affect the computation; it
  // is purely a passive observer.
  // TODO(amcastro-tri): Consider moving stats into the solver's state stored as
  // part of the model's context.
  mutable SapStatistics stats_;
};

// Forward-declare specializations, prior to DRAKE_DECLARE... below.
// We use these to specialize functions that do not support AutoDiffXd.
template <>
SapSolverStatus SapSolver<double>::SolveWithGuess(
    const SapContactProblem<double>&, const VectorX<double>&,
    SapSolverResults<double>*);
template <>
SapSolverStatus SapSolver<AutoDiffXd>::SolveWithGuess(
    const SapContactProblem<AutoDiffXd>&, const VectorX<AutoDiffXd>&,
    SapSolverResults<AutoDiffXd>*);

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapSolver);
