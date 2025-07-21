#pragma once

#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap.h"
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap_builder.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

using multibody::MultibodyForces;
using multibody::MultibodyPlant;
using multibody::contact_solvers::internal::BlockSparseCholeskySolver;
using multibody::contact_solvers::internal::BlockSparseSymmetricMatrixT;
using multibody::contact_solvers::internal::BlockSparsityPattern;
using multibody::contact_solvers::pooled_sap::PooledSapBuilder;
using multibody::contact_solvers::pooled_sap::PooledSapModel;
using multibody::contact_solvers::pooled_sap::SapData;
using multibody::contact_solvers::pooled_sap::SearchDirectionData;

/**
 * Tolerances and other parameters for the convex integrator's solver.
 */
struct ConvexIntegratorSolverParameters {
  // Maximum outer iterations and linesearch iterations
  int max_iterations{100};
  int max_ls_iterations{100};

  // Maximum line search step size
  double alpha_max{1.0};

  // Tolerance ε for the convergence conditions
  //   ‖D ∇ℓ‖ ≤ ε max(1, ‖D r‖),
  //   η ‖D⁻¹ Δv‖ ≤ ε max(1, ‖D r‖),
  // in fixed-step mode (no error control).
  double tolerance{1e-8};

  // Tolerance for exact line search.
  double ls_tolerance{1e-8};

  // Scaling factor for setting the tolerance ε = κ ⋅ accuracy in
  // error-controlled model.
  double kappa{0.001};

  // Whether hessian reuse between iterations and time steps is enabled.
  bool enable_hessian_reuse{false};
  int max_iterations_for_hessian_reuse{10};  // k_max from [Hairer, 1996]

  // Logging/performance tracking flags
  bool print_solver_stats{false};  // Whether to print stats to console.
  bool log_solver_stats{false};    // Whether to log stats to a file.

  // Dense algebra (LDLT) for solving for the search direction Δv = H⁻¹ g.
  // This is primarily useful for debugging and testing: sparse algebra is
  // generally much faster.
  bool use_dense_algebra{false};

  // How to compute the error estimate ||x̂ₜ₊ₕ - xₜ₊ₕ||.
  // Options are "half_stepping", "sdirk", "implicit_trapezoid".
  std::string error_estimation_strategy{"half_stepping"};
};

/**
 * Statistics to track during the optimization process.
 */
struct ConvexIntegratorSolverStats {
  // The simulation time at which the solve was performed.
  double time;

  // The number of solver iterations
  int iterations;

  // The cost ℓ(v) at each iteration.
  std::vector<double> cost;

  // The gradient norm ||∇ℓ(v)|| at each iteration.
  std::vector<double> gradient_norm;

  // The number of linesearch iterations at each solver iteration.
  std::vector<int> ls_iterations;

  // The linesearch parameter α at each iteration.
  std::vector<double> alpha;

  // The step size at this iteration, ||Δvₖ||
  std::vector<double> step_size;

  // Reset the stats to start a new iteration.
  void Reset(const double t) {
    time = t;
    iterations = 0;
    cost.resize(0);
    gradient_norm.resize(0);
    ls_iterations.resize(0);
    alpha.resize(0);
    step_size.resize(0);
  }

  // Reserve space for the vectors to avoid reallocations.
  void Reserve(int size) {
    cost.reserve(size);
    gradient_norm.reserve(size);
    ls_iterations.reserve(size);
    alpha.reserve(size);
    step_size.reserve(size);
  }
};

/**
 * An experimental implicit integrator that solves a convex SAP problem to
 * advance the state, rather than relying on non-convex Newton-Raphson.
 *
 * N.B. Although this is an implicit integration scheme, we inherit from
 * IntegratorBase rather than ImplicitIntegrator because the way we compute
 * the Jacobian (Hessian) is completely different, and MultibodyPlant
 * specific.
 */
template <class T>
class ConvexIntegrator final : public IntegratorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConvexIntegrator);

  ~ConvexIntegrator() override = default;

  /**
   * Constructs the convex integrator.
   *
   * @param system the overall system diagram to simulate. Must include a
   *               MultibodyPlant and associated SceneGraph.
   * @param context context for the overall system.
   *
   * @note This constructor matches the signature used by other integrators, but
   *       does not specify the MultibodyPlant used to set up the optimization
   *       problem. set_plant() must be called before using the integrator.
   */
  explicit ConvexIntegrator(const System<T>& system,
                            Context<T>* context = nullptr);

  /**
   * Specifies the MultibodyPlant used to set up the optimization problem.
   */
  void set_plant(MultibodyPlant<T>* plant) {
    DRAKE_DEMAND(plant != nullptr);
    plant_ = plant;
  }

  /**
   * Get a reference to the MultibodyPlant used to formulate the convex
   * optimization problem.
   */
  const MultibodyPlant<T>& plant() const {
    DRAKE_ASSERT(plant_ != nullptr);
    return *plant_;
  }

  /**
   * Get a reference to SAP problem builder, used to set up the convex problem.
   */
  const PooledSapBuilder<T>& builder() const {
    DRAKE_ASSERT(builder_ != nullptr);
    return *builder_;
  }

  /**
   * Get a reference to the SAP problem model, used to compute the cost,
   * gradient, Hessian, etc.
   */
  PooledSapModel<T>& get_model() { return model_; }

  /**
   * Get a reference to the SAP problem data, used to store the cost, gradient,
   * Hessian, etc.
   */
  SapData<T>& get_data() { return data_; }

  /**
   * Get the current convex solver tolerances and iteration limits.
   */
  const ConvexIntegratorSolverParameters& get_solver_parameters() const {
    return solver_parameters_;
  }

  /**
   * Set the convex solver tolerances and iteration limits.
   */
  void set_solver_parameters(
      const ConvexIntegratorSolverParameters& parameters) {
    solver_parameters_ = parameters;
  }

  /**
   * Get a mutable reference to the convex solver tolerances and iteration
   * limits.
   */
  ConvexIntegratorSolverParameters& get_mutable_solver_parameters() {
    return solver_parameters_;
  }

  /**
   * Get the current convex solver statistics.
   */
  const ConvexIntegratorSolverStats& get_solver_stats() const { return stats_; }

  /**
   * Get the current total number of solver iterations across all time steps.
   */
  int get_total_solver_iterations() const { return total_solver_iterations_; }

  /**
   * Get the current total number of linesearch iterations, across all time
   * steps and solver iterations.
   */
  int get_total_ls_iterations() const { return total_ls_iterations_; }

  /**
   * Get the current total number of Hessian factorizations performed, across
   * all time steps and solver iterations.
   */
  int get_total_hessian_factorizations() const {
    return total_hessian_factorizations_;
  }

  /**
   * Error estimation is supported via half-stepping.
   */
  bool supports_error_estimation() const final { return true; }

  /**
   * Half-stepping error estimation gives a second-order error estimate. See
   * ImplicitEulerIntegrator for details.
   */
  int get_error_estimate_order() const final { return 2; }

 private:
  friend class ConvexIntegratorTester;

  // Perform final checks and allocations before beginning integration.
  void DoInitialize() final;

  // Perform the main integration step, setting x_{t+h} and the error
  // estimate.
  bool DoStep(const T& h) override;

  // Do the main integration step, using two half-sized steps for error
  // estimation. This requires a total of 3 SAP solves.
  bool StepWithHalfSteppingErrorEstimate(const T& h);

  // Do the main integration step using a 2nd-order L-stable SDIRK scheme. This
  // requires a total of 2 SAP solves, and includes an embedded error estimate.
  bool StepWithSDIRKErrorEstimate(const T& h);

  // Do the main integration step, using the implicit trapezoid rule for error
  // estimation. This requires a total of 2 SAP solves.
  bool StepWithImplicitTrapezoidErrorEstimate(const T& h);

  // Solve the SAP problem to compute x_{t+h} at a given step size. This will be
  // called multiple times for each DoStep to compute the error estimate.
  //
  // @param h the time step to use
  // @param v_guess the initial guess for the MbP plant velocities.
  // @param x_next the output continuous state, includes state for both the
  //               plant and any external systems.
  // @param partial_update if true, reuse the external system linearization
  //                       from the previous solve.
  void ComputeNextContinuousState(const T& h, const VectorX<T>& v_guess,
                                  ContinuousState<T>* x_next,
                                  bool partial_update = false);

  // Advance the plant's generalized positions, q = q₀ + h N(q₀) v, taking care
  // to handle quaternion DoFs properly.
  // N.B. q₀ is stored in this->get_context().
  void AdvancePlantConfiguration(const T& h, const VectorX<T>& v,
                                 VectorX<T>* q) const;

  // Solve the convex SAP problem for next-step velocities v = min ℓ(v).
  // The solution is written back into the initial guess v. Returns true if
  // and only if the optimization converged.
  bool SolveWithGuess(const PooledSapModel<T>& model, VectorX<T>* v_guess);

  // Solve min_α ℓ(v + α Δ v) using a 1D Newton method with bisection fallback.
  // Returns the linesearch parameter α and the number of iterations taken.
  std::pair<T, int> PerformExactLineSearch(const PooledSapModel<T>& model,
                                           const SapData<T>& data,
                                           const VectorX<T>& dv);

  // Returns the root of the quadratic equation ax² + bx + c = 0, x ∈ [0, 1].
  T SolveQuadraticInUnitInterval(const T& a, const T& b, const T& c) const;

  // Solve for the Newton search direction Δv = −H⁻¹g, with flags for several
  // levels of Hessian reuse:
  //  - reuse_factorization: reuse the exact same factorization of H as in the
  //                         previous iteration. Do not compute the new Hessian
  //                         at all. This is the fastest option, but gives a
  //                         lower-quality search direction.
  //  - reuse_sparsity_pattern: recompute H and its factorization, but reuse the
  //                            stored sparsity pattern. This gives an exact
  //                            Newton step, but avoids some allocations.
  void ComputeSearchDirection(const PooledSapModel<T>& model,
                              const SapData<T>& data, VectorX<T>* dv,
                              bool reuse_factorization = false,
                              bool reuse_sparsity_pattern = false);

  // Indicate whether a change in problem structure requires a Hessian with a
  // new sparsity pattern.
  bool SparsityPatternChanged(const PooledSapModel<T>& model) const;

  // Compute external forces τ = τₑₓₜ(x) from the plant's spatial and
  // generalized force input ports.
  void CalcExternalForces(const Context<T>& context, VectorX<T>* tau);

  // Compute actuator forces τ = B u(x) from the plant's actuation input
  // ports (including the general actuation input port and any
  // model-instance-specific ports).
  void CalcActuationForces(const Context<T>& context, VectorX<T>* tau);

  // (Partially) linearize all the external (controller) systems connected to
  // the plant with finite differences.
  //
  // Torques from externally connected systems are given by
  //     τ = B u(x) + τₑₓₜ(x),
  // which we will approximate as
  //     τ ≈ clamp(-Kᵤ v + bᵤ) - Kₑ v + bₑ,
  // where Kᵤ, Kₑ are diagonal and positive semi-definite.
  //
  // Note that contributions due to controls u(x) will be clamped to effort
  // limits, while contributions due to external generalized and spatial forces
  // τₑₓₜ(x) will not be.
  void LinearizeExternalSystem(const T& h, VectorX<T>* Ku, VectorX<T>* bu,
                               VectorX<T>* Ke, VectorX<T>* be);

  // The multibody plant used as the basis of the convex optimization problem.
  MultibodyPlant<T>* plant_{nullptr};

  // Pre-allocated objects used to formulate and solve the optimization problem.
  std::unique_ptr<PooledSapBuilder<T>> builder_;
  PooledSapModel<T> model_;
  SapData<T> data_;
  SapData<T> scratch_data_;
  std::unique_ptr<BlockSparseSymmetricMatrixT<T>> hessian_;
  BlockSparseCholeskySolver<Eigen::MatrixXd> hessian_factorization_;
  Eigen::LDLT<Eigen::MatrixXd> dense_hessian_factorization_;
  SearchDirectionData<T> search_direction_data_;

  // Pre-allocated scratch space for intermediate calculations.
  struct Scratch {
    VectorX<T> v_guess;
    VectorX<T> search_direction;

    // Next-step state
    VectorX<T> v;
    VectorX<T> q;
    VectorX<T> z;

    // External forces
    std::unique_ptr<MultibodyForces<T>> f_ext;

    // External system linearization
    VectorX<T> Ku;
    VectorX<T> bu;
    VectorX<T> Ke;
    VectorX<T> be;
    VectorX<T> gu0;
    VectorX<T> ge0;
    VectorX<T> gu_prime;
    VectorX<T> ge_prime;
    VectorX<T> x_prime;
    MatrixX<T> N;
  } scratch_;

  // Track previous sparsity pattern for hessian reuse
  std::unique_ptr<BlockSparsityPattern> previous_sparsity_pattern_;

  // Flag for Hessian factorization re-use (changes between iterations)
  bool reuse_hessian_factorization_{true};

  // Solver tolerances and other parameters
  ConvexIntegratorSolverParameters solver_parameters_;

  // Logging/performance tracking utilities
  std::ofstream log_file_;
  ConvexIntegratorSolverStats stats_;
  int total_solver_iterations_{0};
  int total_ls_iterations_{0};
  int total_hessian_factorizations_{0};

  // Intermediate states for error control, which compares a single large
  // step (x_next_full_) to the result of two smaller steps (x_next_half_2_).
  std::unique_ptr<ContinuousState<T>> x_next_full_;    // x_{t+h}
  std::unique_ptr<ContinuousState<T>> x_next_half_1_;  // x_{t+h/2}
  std::unique_ptr<ContinuousState<T>> x_next_half_2_;  // x_{t+h/2+h/2}
};

// Forward-declare specializations to double, prior to DRAKE_DECLARE... below.
template <>
bool ConvexIntegrator<double>::SolveWithGuess(const PooledSapModel<double>&,
                                              VectorX<double>*);
template <>
std::pair<double, int> ConvexIntegrator<double>::PerformExactLineSearch(
    const PooledSapModel<double>&, const SapData<double>&,
    const VectorX<double>&);

template <>
void ConvexIntegrator<double>::ComputeSearchDirection(
    const PooledSapModel<double>&, const SapData<double>&, VectorX<double>*,
    bool, bool);

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::ConvexIntegrator);
