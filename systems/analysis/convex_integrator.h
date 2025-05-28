#pragma once

#include <fstream>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap.h"
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap_builder.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"

namespace drake {
namespace systems {

using multibody::MultibodyPlant;
using multibody::contact_solvers::pooled_sap::PooledSapBuilder;
using multibody::contact_solvers::pooled_sap::PooledSapModel;
using multibody::contact_solvers::pooled_sap::SapData;
using multibody::contact_solvers::internal::BlockSparseCholeskySolver;
using multibody::contact_solvers::internal::BlockSparseSymmetricMatrix;

/**
 * Tolerances and other parameters for the convex integrator's solver.
 */
struct ConvexIntegratorSolverParameters {
  // Maximum outer iterations and linesearch iterations
  int max_iterations{100};
  int max_ls_iterations{100};

  // Maximum line search step size
  double alpha_max{1.0};

  // Absolute tolerance for the stricter gradient-based convergence check.
  double tolerance{1e-6};

  // Tolerance for exact line search.
  double ls_tolerance{1e-6};

  // Scaling factor for the relaxed convergence check (θ method of Hairer 1996)
  // used to exit early under loose accuracies.
  double kappa{0.05};
};

/**
 * Statistics to track during the optimization process.
 */
struct ConvexIntegratorSolverStats {
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
  void Reset() {
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
   * Constructs the convex integrator, specifying the MultibodyPlant used to
   * formulate the convex optimization problem.
   */
  ConvexIntegrator(const System<T>& system, MultibodyPlant<T>* plant,
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
   * Get the current convex solver statistics.
   */
  const ConvexIntegratorSolverStats& get_solver_stats() const { return stats_; }

  /**
   * Set whether to record solver statistics to a CSV file.
   */
  void set_log_solver_stats(bool log_stats) { log_solver_stats_ = log_stats; }

  /**
   * Set whether to print solver statistics to the console.
   */
  void set_print_solver_stats(bool print_stats) {
    print_solver_stats_ = print_stats;
  }

  /**
   * Get the current total number of solver iterations across all time steps.
   */
  int get_total_solver_iterations() const { return total_solver_iterations_; }

  /**
   * Get the current total number of linesearch iterations, across all time
   * steps and solver iterations.
   */
  int get_total_ls_iterations() const { return total_ls_iterations_; }

  // TODO(vincekurtz): add support for error estimation.
  bool supports_error_estimation() const final { return false; }
  int get_error_estimate_order() const final { return 0; }

 private:
  // Perform final checks and allocations before beginning integration.
  void DoInitialize() final;

  // Perform the main integration step, setting x_{t+h} and the error
  // estimate.
  bool DoStep(const T& h) override;

  // Solve the convex SAP problem for next-step velocities v = min ℓ(v).
  // The solution is written back into the initial guess v. Returns true if
  // and only if the optimization converged.
  bool SolveWithGuess(const PooledSapModel<T>& model, VectorX<T>* v_guess);

  // Solve min_α ℓ(v + α Δ v) using a 1D Newton method with bisection fallback.
  void PerformExactLineSearch(const PooledSapModel<T>& model,
                              const VectorX<T>& v, const VectorX<T>& dv,
                              T* alpha, int* num_iterations);

  // Returns the largest root of the quadratic equation ax² + bx + c = 0.
  T SolveQuadraticForLargestRoot(const T& a, const T& b, const T& c) const;

  // Print solver statistics to the console for debugging.
  void PrintSolverStats() const;

  // Log solver statistics to a CSV file for later analysis.
  void LogSolverStats();

  // The multibody plant used as the basis of the convex optimization problem.
  MultibodyPlant<T>* plant_{nullptr};

  // Objects used to formulate and solve the convex optimization problem.
  std::unique_ptr<PooledSapBuilder<T>> builder_;
  PooledSapModel<T> model_;
  SapData<T> data_;
  BlockSparseCholeskySolver<Eigen::MatrixXd> hessian_factorization_;

  // Solver tolerances and other parameters
  ConvexIntegratorSolverParameters solver_parameters_;

  // Logging/performance tracking utilities
  bool print_solver_stats_{true};  // Whether to print stats to console.
  bool log_solver_stats_{true};    // Whether to log stats to a file.
  std::ofstream log_file_;         // CSV file for logging stats.
  ConvexIntegratorSolverStats stats_;
  int total_solver_iterations_{0};
  int total_ls_iterations_{0};
};

// Forward-declare specializations to double, prior to DRAKE_DECLARE... below.
template <>
bool ConvexIntegrator<double>::SolveWithGuess(const PooledSapModel<double>&,
                                              VectorX<double>*);
template <>
void ConvexIntegrator<double>::PerformExactLineSearch(
    const PooledSapModel<double>&, const VectorX<double>&,
    const VectorX<double>&, double*, int*);

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::ConvexIntegrator);
