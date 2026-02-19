#pragma once

#include <memory>
#include <vector>

#include "drake/multibody/contact_solvers/icf/icf_builder.h"
#include "drake/multibody/contact_solvers/icf/icf_external_systems_linearizer.h"
#include "drake/multibody/contact_solvers/icf/icf_model.h"
#include "drake/multibody/contact_solvers/icf/icf_solver.h"
#include "drake/multibody/contact_solvers/icf/icf_solver_parameters.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/framework/diagram_continuous_state.h"

namespace drake {
namespace multibody {

/** Convex Error-controlled Numerical Integration for Contact (CENIC) is a
specialized error-controlled implicit integrator for contact-rich robotics
simulations [Kurtz and Castro, 2025].

CENIC provides variable-step error-controlled integration for multibody systems
with stiff contact interactions, while maintaining the high speeds
characteristic of discrete-time solvers required for modern robotics workflows.

Benefits of CENIC include:

- **Guaranteed convergence**. Unlike traditional implicit integrators that rely
  on non-convex Newton-Raphson solves, CENIC's convex formulation eliminates
  step rejections due to convergence failures.

- **Guaranteed accuracy**. CENIC inherits the well-studied accuracy guarantees
  associated with error-controlled integration [Hairer and Wanner, 1996],
  avoiding discretization artifacts common in fixed-step discrete-time methods.

- **Automatic time step selection**. Users specify a desired accuracy rather
  than a fixed time step, eliminating a common pain point in authoring multibody
  simulations.

- **Implicit treatment of external systems**. This means that users can connect
  arbitrary stiff controllers (e.g., a custom `LeafSystem`) to the
  `MultibodyPlant` and have them treated implicitly in CENIC's convex
  formulation. This allows for larger time steps, leading to faster and more
  stable simulations.

- **Principled static/dynamic friction modeling**. Unlike discrete solvers,
  CENIC can simulate frictional contact with different static and dynamic
  friction coefficients.

- **Speed**. CENIC consistently outperforms general-purpose integrators by
  orders of magnitude on contact-rich problems. Error-controlled CENIC is often
  (but not always) faster than discrete-time simulation, depending on the
  simulation in question and the requested accuracy.

CENIC works by solving a convex Irrotational Contact Fields (ICF) optimization
problem [Castro et al., 2023] to advance the system state at each time step. A
simple half-stepping strategy provides a second-order error estimate for
automatic step-size selection.

Because CENIC is specific to multibody systems, this integrator requires a
system diagram with a `MultibodyPlant` subsystem named `"plant"`.

Running CENIC in fixed-step mode (with error-control disabled) recovers the
"Lagged" variant of discrete-time ICF simulation from [Castro et al., 2023].

References:

  [Castro et al., 2023] Castro A., Han X., and Masterjohn J., 2023. Irrotational
  Contact Fields. https://arxiv.org/abs/2312.03908.

  [Hairer and Wanner, 1996] Hairer E. and Wanner G., 1996. Solving Ordinary
  Differential Equations II: Stiff and Differential-Algebraic Problems. Springer
  Series in Computational Mathematics, Vol. 14. Springer-Verlag, Berlin, 2nd
  edition.

  [Kurtz and Castro, 2025] Kurtz V. and Castro A., 2025. CENIC: Convex
  Error-controlled Numerical Integration for Contact.
  https://arxiv.org/abs/2511.08771.

@tparam_nonsymbolic_scalar */
template <class T>
class CenicIntegrator final : public systems::IntegratorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CenicIntegrator);

  /** This target accuracy is established in the constructor, but may be
  changed by @ref integrator-accuracy methods. CENIC works best at loose
  accuracy. */
  static constexpr double kDefaultAccuracy = 1e-3;

  /** Constructs the integrator.
  @param system The overall system diagram to simulate. Must include a
                MultibodyPlant and associated SceneGraph, with the plant
                found as a direct child of the `system` diagram using the
                subsystem name `"plant"`. The plant must be a continuous-time
                plant. This system is aliased by this object so must remain
                alive longer than the integrator.
  @param context context for the overall system.  */
  explicit CenicIntegrator(const systems::System<T>& system,
                           systems::Context<T>* context = nullptr);

  ~CenicIntegrator() final;

  /** Gets a reference to the MultibodyPlant used to formulate the convex
  optimization problem. */
  const MultibodyPlant<T>& plant() const { return plant_; }

  /** Gets the current convex solver tolerances and iteration limits. */
  const contact_solvers::icf::IcfSolverParameters& get_solver_parameters()
      const {
    return solver_.get_parameters();
  }

  /** Sets the convex solver tolerances and iteration limits. */
  void SetSolverParameters(
      const contact_solvers::icf::IcfSolverParameters& parameters);

  bool supports_error_estimation() const final;

  int get_error_estimate_order() const final;

 private:
  /* Preallocated scratch space. */
  struct Scratch {
    /* Resizes the scratch space to accommodate the given plant and enclosing
    diagram. */
    void Resize(const MultibodyPlant<T>& plant,
                const systems::System<T>& diagram);

    /* State-sized variables, x = [q; v] for the plant (not the diagram). */
    VectorX<T> v_guess;
    VectorX<T> q;

    /* Linearized external system gains (sized to the plant's num_velocities):
    Torque-limited actuation, τᵤ(v) ≈ clamp(-Kᵤ⋅v + bᵤ, e). */
    contact_solvers::icf::internal::IcfLinearFeedbackGains<T>
        actuation_feedback_storage;
    /* Non-limited external forces, τₑ(v) ≈ −Kₑ⋅v + bₑ. */
    contact_solvers::icf::internal::IcfLinearFeedbackGains<T>
        external_feedback_storage;

    /* Intermediate states for error control, which compares a single large
    step (x_next_full_) to the result of two smaller steps (x_next_half_2_). */
    /* x_{t+h}. */
    std::unique_ptr<systems::DiagramContinuousState<T>> x_next_full;
    /* x_{t+h/2}. */
    std::unique_ptr<systems::DiagramContinuousState<T>> x_next_half_1;
    /* x_{t+h/2+h/2}. */
    std::unique_ptr<systems::DiagramContinuousState<T>> x_next_half_2;
  };

  /* Data for PrintSimulatorStatistics(). */
  struct Stats {
    int total_solver_iterations{0};
    int total_hessian_factorizations{0};
    int total_ls_iterations{0};
  };

  void DoResetStatistics() final;

  std::vector<systems::NamedStatistic> DoGetStatisticsSummary() const final;

  T CalcStateChangeNorm(
      const systems::ContinuousState<T>& dx_state) const final;

  void DoInitialize() final;

  bool DoStep(const T& h) final;

  /* Solves the ICF problem to compute x_{t+h}.

  @param model The ICF model for the convex problem min_v ℓ(v; q₀, v₀, h).
  @param v_guess The initial guess for the MbP plant velocities.
  @param[out] x_next The output continuous state, includes state for both the
                     plant and any external systems. */
  void ComputeNextContinuousState(
      const contact_solvers::icf::internal::IcfModel<T>& model,
      const VectorX<T>& v_guess, systems::DiagramContinuousState<T>* x_next);

  /* Advances the plant's generalized positions, q = q₀ + h N(q₀) v, taking care
  to handle quaternion DoFs properly.

  @param h The time step size.
  @param v The next-step generalized velocities v.
  @param[out] q The output next-step generalized positions q.

  N.B. q₀ is stored in this->get_context(). */
  void AdvancePlantConfiguration(const T& h, const VectorX<T>& v,
                                 VectorX<T>* q) const;

  /* The plant used as the basis of the convex optimization problem. */
  const MultibodyPlant<T>& plant_;
  const systems::SubsystemIndex plant_subsystem_index_;
  /* Which subsystems in our Diagram have continuous state other than the
  plant. */
  const std::vector<int> non_plant_xc_subsystem_indices_;

  /* Helper class that linearizes torques dτ/dv from plant input ports. */
  const contact_solvers::icf::internal::IcfExternalSystemsLinearizer<T>
      external_systems_linearizer_;

  /* ICF integrator state and storage. */
  std::unique_ptr<contact_solvers::icf::internal::IcfBuilder<T>> builder_;
  contact_solvers::icf::internal::IcfSolver solver_;
  /* For the full step and first half-step. */
  contact_solvers::icf::internal::IcfModel<T> model_at_x0_;
  /* For the second half-step (at t + h/2). */
  contact_solvers::icf::internal::IcfModel<T> model_at_xh_;
  contact_solvers::icf::internal::IcfData<T> data_;

  /* Track whether solves are initialized at the same time as a previous
  rejected step, to enable model (e.g., constraints, geometry) reuse. */
  T time_at_last_solve_{NAN};

  /* Preallocated scratch space for intermediate calculations. */
  Scratch scratch_;

  /* Data for PrintSimulatorStatistics(). */
  Stats stats_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::CenicIntegrator);
