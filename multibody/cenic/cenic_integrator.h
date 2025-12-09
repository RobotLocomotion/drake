#pragma once

#include <memory>
#include <tuple>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/multibody/contact_solvers/block_sparse_cholesky_solver.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/contact_solvers/icf/icf_builder.h"
#include "drake/multibody/contact_solvers/icf/icf_model.h"
#include "drake/multibody/contact_solvers/icf/icf_solver.h"
#include "drake/multibody/contact_solvers/icf/icf_solver_parameters.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/framework/diagram_continuous_state.h"

namespace drake {
namespace multibody {

using contact_solvers::icf::IcfSolverParameters;
using contact_solvers::icf::internal::IcfBuilder;
using contact_solvers::icf::internal::IcfData;
using contact_solvers::icf::internal::IcfModel;
using contact_solvers::icf::internal::IcfSolver;
using contact_solvers::icf::internal::IcfSolverStats;
using contact_solvers::icf::internal::LinearFeedbackGains;

/**
 * An experimental implicit integrator that solves a convex ICF problem to
 * advance the state, rather than relying on non-convex Newton-Raphson.
 *
 * N.B. Although this is an implicit integration scheme, we inherit from
 * IntegratorBase rather than ImplicitIntegrator because the way we compute
 * the Jacobian (Hessian) is completely different, and MultibodyPlant
 * specific.
 */
template <class T>
class CenicIntegrator final : public systems::IntegratorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CenicIntegrator);

  ~CenicIntegrator() override = default;

  /**
   * Constructs the integrator.
   *
   * @param system The overall system diagram to simulate. Must include a
   *               MultibodyPlant and associated SceneGraph, with the plant
   *               found as a direct child of the `system` diagram using the
   *               subsystem name `"plant"`. This system is aliased by this
   *               object so must remain alive longer than the integrator.
   * @param context context for the overall system.
   */
  explicit CenicIntegrator(const systems::System<T>& system,
                           systems::Context<T>* context = nullptr);

  /**
   * Get a reference to the MultibodyPlant used to formulate the convex
   * optimization problem.
   */
  const MultibodyPlant<T>& plant() const { return plant_; }

  /**
   * Get a reference to the ICF builder, used to set up the convex problem.
   *
   * N.B. this is not const because the builder caches geometry data.
   */
  IcfBuilder<T>& builder() {
    DRAKE_ASSERT(builder_ != nullptr);
    return *builder_;
  }

  /**
   * Get the current convex solver tolerances and iteration limits.
   */
  const IcfSolverParameters& get_solver_parameters() const {
    return solver_.get_parameters();
  }

  /**
   * Set the convex solver tolerances and iteration limits.
   */
  void set_solver_parameters(const IcfSolverParameters& parameters) {
    solver_.set_parameters(parameters);
  }

  /**
   * Get the current convex solver statistics.
   */
  const IcfSolverStats& get_solver_stats() const { return solver_.stats(); }

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
  friend class CenicTester;

  // Preallocated scratch space
  struct Scratch {
    // Resize to accommodate the given plant.
    void Resize(const MultibodyPlant<T>& plant);

    // State-sized variables, x = [q; v] for the plant (not the diagram).
    VectorX<T> v;
    VectorX<T> q;

    // External forces
    std::unique_ptr<MultibodyForces<T>> f_ext;

    // Linearized external system gains (sized to the plant's num_velocities):
    LinearFeedbackGains<T> actuation_feedback;  // τ = clamp(−Ku⋅v + bu)
    LinearFeedbackGains<T> external_feedback;   // τ = −Ke⋅v + be

    // External system linearization
    VectorX<T> v0;        // Unperturbed plant velocities.
    VectorX<T> gu0;       // Actuation gu(x) = B u(x) at x₀
    VectorX<T> ge0;       // External forces ge(x) = τ_ext(x) at x₀
    VectorX<T> gu_prime;  // Perturbed actuation gu(x') = B u(x')
    VectorX<T> ge_prime;  // Perturbed external forces ge(x') = τ_ext(x')
    VectorX<T> x_prime;   // Perturbed plant state x' for finite differences
    MatrixX<T> N;         // Kinematic map q̇ = N v
  };

  // Perform final checks and allocations before beginning integration.
  void DoInitialize() final;

  // Perform the main integration step, setting x_{t+h} and the error
  // estimate.
  bool DoStep(const T& h) override;

  /**
   * Solve the ICF problem to compute x_{t+h} at a given step size. This will be
   * called multiple times for each DoStep to compute the error estimate.
   *
   * @param model the ICF model for the convex problem min_v ℓ(v; q₀, v₀, h).
   * @param v_guess the initial guess for the MbP plant velocities.
   * @param x_next the output continuous state, includes state for both the
   *               plant and any external systems.
   */
  void ComputeNextContinuousState(const IcfModel<T>& model,
                                  const VectorX<T>& v_guess,
                                  systems::DiagramContinuousState<T>* x_next);

  // Advance the plant's generalized positions, q = q₀ + h N(q₀) v, taking care
  // to handle quaternion DoFs properly.
  // N.B. q₀ is stored in this->get_context().
  void AdvancePlantConfiguration(const T& h, const VectorX<T>& v,
                                 VectorX<T>* q) const;

  // Compute external forces τ = τₑₓₜ(x) from the plant's spatial and
  // generalized force input ports.
  void CalcExternalForces(const systems::Context<T>& context, VectorX<T>* tau);

  // Compute actuator forces τ = B u(x) from the plant's actuation input
  // ports (including the general actuation input port and any
  // model-instance-specific ports).
  void CalcActuationForces(const systems::Context<T>& context, VectorX<T>* tau);

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
  //
  // @return [has_actuation_forces, has_external_forces] where a given tuple
  //   element is true iff any input is connected for the corresponding type.
  std::tuple<bool, bool> LinearizeExternalSystem(
      const T& h, LinearFeedbackGains<T>* actuation_feedback,
      LinearFeedbackGains<T>* external_feedback);

  // Overrides the typical state change norm (weighted infinity norm) to use
  // just the infinity norm of the position vector.
  T CalcStateChangeNorm(
      const systems::ContinuousState<T>& dx_state) const final;

  // The multibody plant used as the basis of the convex optimization problem.
  const MultibodyPlant<T>& plant_;
  const systems::SubsystemIndex plant_subsystem_index_;
  // Which subsystems in our Diagram have continuous state beyond the MbP.
  const std::vector<int> non_plant_xc_subsystem_indices_;

  // Pre-allocated objects used to formulate and solve the optimization problem.
  std::unique_ptr<IcfBuilder<T>> builder_;
  IcfSolver solver_;
  IcfModel<T> model_at_x0_;  // for the full step and first half-step
  IcfModel<T> model_at_xh_;  // for the second half-step (at t + h/2)
  IcfData<T> data_;          // reusable data for the solver

  // Track whether solves are initialized at the same time as a previous
  // rejected step, to enable model (e.g., constraints, geometry) reuse.
  T time_at_last_solve_{NAN};

  // Pre-allocated scratch space for intermediate calculations.
  Scratch scratch_;

  // Logging/performance tracking utilities
  int total_solver_iterations_{0};
  int total_ls_iterations_{0};
  int total_hessian_factorizations_{0};

  // Intermediate states for error control, which compares a single large
  // step (x_next_full_) to the result of two smaller steps (x_next_half_2_).
  // x_{t+h}
  std::unique_ptr<systems::DiagramContinuousState<T>> x_next_full_;
  // x_{t+h/2}
  std::unique_ptr<systems::DiagramContinuousState<T>> x_next_half_1_;
  // x_{t+h/2+h/2}
  std::unique_ptr<systems::DiagramContinuousState<T>> x_next_half_2_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::CenicIntegrator);
