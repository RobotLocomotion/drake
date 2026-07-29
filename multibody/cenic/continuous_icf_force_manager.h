#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/icf/icf_builder.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"
#include "drake/multibody/contact_solvers/icf/icf_model.h"
#include "drake/multibody/plant/continuous_contact_force_reporter.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

/* Reports contact and reaction forces for a continuous-mode MultibodyPlant that
is being integrated with the Irrotational Contact Fields (ICF) convex contact
model (e.g., by CenicIntegrator), rather than with the plant's compliant
point/hydroelastic continuous contact model.

Motivation
----------
A continuous MultibodyPlant's force-reporting output ports (reaction_forces,
contact_results) and its continuous ODE (EvalForwardDynamics) are defined by the
compliant point/hydroelastic contact model. CenicIntegrator, however, does not
integrate that ODE: each step it solves a convex ICF problem, whose contact and
constraint force law differs from the compliant model. Reporting forces from the
compliant model is therefore inconsistent with the trajectory that was actually
simulated.

This manager evaluates the ICF constraint force law at the *current* context
state (q, v, u), so that reporting is:

  - a pure function of the context (no dependence on the integrator's adaptive
    step size, sub-steps, or transient last-solve impulses), and
  - consistent with the ICF dynamics that CenicIntegrator integrates.

It is the continuous-reporting analog of DiscreteUpdateManager. Because the ICF
solver library depends on the full MultibodyPlant, the plant cannot depend on it
directly (a Bazel cycle); this manager therefore lives downstream of both and is
injected into the plant via MultibodyPlant::SetContinuousContactForceReporter()
(see AddIcfContinuousForceReporting() below). CenicIntegrator is unaware of it —
CenicIntegrator integrates, this manager reports.

Force vs. impulse and the reporting time step
---------------------------------------------
The ICF constraint quantities γ are *impulses* (force × δt), so this manager
builds the ICF model at a small fixed "reporting time step" kReportingTimeStep
and reports forces as (Σ Jᵀγ) / kReportingTimeStep.

Two distinct time steps appear in the ICF force law and, crucially, they
decouple here:
  - The *position prediction* baked into the compliant law (e.g. for contact,
    fe = fe0 − δt·k·vₙ, γₙ = δt·fe·damping) uses IcfModel::time_step().
  - The near-rigid *regularization* (stiffness cap, holonomic/limit/coupler
    regularization, SAP stiction) floors on IcfModel::effective_time_step() =
    max(time_step, kHMin).
Because reporting only evaluates the force law (IcfModel::CalcData at the current
v) and never solves the optimization, we use a δt far below kHMin without any
conditioning concern. That sends the position-prediction bias to zero —
recovering the true continuous force γ(q₀, v₀) — while the regularization stays
pinned at the kHMin values that CenicIntegrator uses. See kReportingTimeStep.

Active contact set and geometry caching
---------------------------------------
The active constraints are determined by a SceneGraph query at q₀. This manager's
builder sources that query from MultibodyPlant's position-keyed
geometry_contact_data cache (EvalGeometryContactData), the same cache used by
CenicIntegrator's builder and the plant's own contact path, so the query is
computed once per q₀ and reused by the next integration step.

@tparam_nonsymbolic_scalar */
template <typename T>
class ContinuousIcfForceManager final
    : public ContinuousContactForceReporter<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContinuousIcfForceManager);

  /* Constructs a manager for the given continuous, finalized `plant`, which is
  aliased and must outlive this object.
  @pre plant != nullptr && plant->is_finalized() && !plant->is_discrete(). */
  explicit ContinuousIcfForceManager(const MultibodyPlant<T>* plant);

  ~ContinuousIcfForceManager() final;

  /* The fixed time step used to convert ICF impulses to reported forces. Chosen
  small enough that the position-prediction term (e.g. δt·k·vₙ in the contact
  law) falls below the ULP of the elastic force fe0, so the force is evaluated
  effectively at (q₀,v₀) exactly, while ICF's regularization stays floored at
  kHMin.

  This is NOT finite differencing: each impulse is the analytic product γ = δt·f
  with f formed independently of δt, so force = γ/δt cancels δt exactly and
  carries only relative-ε round-off, independent of δt. Any value in roughly
  [1e-16, 1e-40] is equally accurate and FP-safe (γ nowhere near underflow; the
  1/δt regularization terms nowhere near overflow); the exact value is not
  sensitive. Must be > 0 (IcfModel::VerifyInvariants). */
  static constexpr double kReportingTimeStep = 1e-30;

  /* Implements ContinuousContactForceReporter. */
  void CalcAppliedForces(const systems::Context<T>& context,
                         MultibodyForces<T>* forces) const final;

 private:
  /* Builds the ICF model at (q, v, u) from `context` using kReportingTimeStep,
  solves no optimization but evaluates the constraint force law at the current v,
  populating the preallocated model_ and data_ members for force extraction.
  Actuation/external forces are NOT modeled as gain constraints here (they are
  included among the non-contact forces instead), so the ICF model carries only
  contact, joint-limit, coupler, weld, and ball constraints. */
  void UpdateModelAndDataAtCurrentState(
      const systems::Context<T>& context) const;

  /* Accumulates the ICF constraint contribution into `forces`:
    tau       += Σ Jᵀγ / kReportingTimeStep            (all constraints)
    F_Bo_W[b] += (per-body spatial impulse) / kReportingTimeStep  (contact)
  using model_ and data_ populated by UpdateModelAndDataAtCurrentState(). */
  void AddInIcfConstraintForces(MultibodyForces<T>* forces) const;

  const MultibodyPlant<T>* const plant_;

  // ICF machinery, mirroring CenicIntegrator's members but owned here for
  // reporting. Mutable because evaluation is logically const (results are
  // cached on the plant context by the output-port infrastructure).
  std::unique_ptr<contact_solvers::icf::internal::IcfBuilder<T>> builder_;
  mutable contact_solvers::icf::internal::IcfModel<T> model_;
  mutable contact_solvers::icf::internal::IcfData<T> data_;
};

}  // namespace internal

/** Configures `plant` so that its `reaction_forces` and `contact_results`
output ports report values consistent with the Irrotational Contact Fields (ICF)
convex contact model — the model that CenicIntegrator uses to integrate the
plant — rather than the plant's compliant point/hydroelastic continuous contact
model.

Without this call, a continuous MultibodyPlant reports contact and reaction
forces from its compliant contact model, which is inconsistent with the
trajectory produced by CenicIntegrator. Call this once, after `plant` is
finalized, when you intend to integrate `plant` with CenicIntegrator and want
its force-reporting ports to reflect the simulated (ICF) dynamics.

The reported forces are evaluated as a pure function of the plant Context, so
the ports remain well-defined independent of the integrator's internal state.

@throws std::exception if `plant` is not finalized or is not a continuous-time
    (time_step == 0) plant.
@tparam_nonsymbolic_scalar */
template <typename T>
void AddIcfContinuousForceReporting(MultibodyPlant<T>* plant);

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::ContinuousIcfForceManager);
