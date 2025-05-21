#pragma once

#include <limits>
#include <memory>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap.h"
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap_builder.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

using multibody::MultibodyPlant;
using multibody::contact_solvers::pooled_sap::PooledSapBuilder;
using multibody::contact_solvers::pooled_sap::PooledSapModel;
using multibody::contact_solvers::pooled_sap::SapData;

/**
 * Tolerances and other parameters for the convex integrator's solver.
 */
struct ConvexIntegratorSolverParameters {
  // Maximum outer iterations and linesearch iterations
  int max_iterations{100};
  int max_ls_iterations{10};

  // Absolute and relative tolerances for the gradient-based convergence check
  // (similar to discrete SAP).
  double abs_tolerance{1e-14};
  double rel_tolerance{1e-6};

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
  // The iteration number that we're on
  int k;

  // The cost ℓ(v)
  double cost;

  // The gradient norm ||∇ℓ(v)||
  double gradient_norm;

  // Number of line search iterations taken in this solver iteration
  int num_ls_iterations;

  // The linesearch parameter α
  double alpha;

  // The step size at this iteration, ||Δvₖ||
  double step_size;

  // The step size at the previous iteration, ||Δvₖ₋₁||
  double last_step_size;

  // The ratio of current and previous step sizes, θ = ||Δvₖ|| / ||Δvₖ₋₁||
  double theta;

  // Reset the stats to start a new iteration.
  void Reset() {
    k = 0;
    cost = std::numeric_limits<double>::quiet_NaN();
    gradient_norm = std::numeric_limits<double>::quiet_NaN();
    num_ls_iterations = 0;
    alpha = 1.0;
    step_size = std::numeric_limits<double>::quiet_NaN();
    last_step_size = std::numeric_limits<double>::quiet_NaN();
    theta = std::numeric_limits<double>::quiet_NaN();
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

  // The multibody plant used as the basis of the convex optimization problem.
  MultibodyPlant<T>* plant_{nullptr};

  // Objects used to formulate and solve the convex optimization problem.
  std::unique_ptr<PooledSapBuilder<T>> builder_;
  PooledSapModel<T> model_;
  SapData<T> data_;

  // Solver tolerances and other parameters
  ConvexIntegratorSolverParameters solver_parameters_;

  // Solver statistics
  ConvexIntegratorSolverStats stats_;
};

// Forward-declare specializations to double, prior to DRAKE_DECLARE... below.
template <>
bool ConvexIntegrator<double>::SolveWithGuess(const PooledSapModel<double>&,
                                              VectorX<double>*);

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::ConvexIntegrator);
