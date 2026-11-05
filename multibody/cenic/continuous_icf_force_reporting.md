# Force reporting for a continuous MultibodyPlant under CenicIntegrator

This note describes how a continuous-time `MultibodyPlant` integrated by
`CenicIntegrator` reports forces (`reaction_forces`, `contact_results`,
`generalized_acceleration`, `generalized_contact_forces`, `net_actuation`)
that are consistent with the ICF (Irrotational Contact Fields) convex problem
CENIC actually solves, rather than with the plant's own compliant contact
model.

## 1. The problem

CENIC does not integrate the plant's continuous ODE. Each step it solves a
fully implicit convex problem whose solution `v₁` satisfies the discrete
momentum balance

    A (v₁ − v*) = Jᵀ γ,        A = M + δt·D,

where `γ` are the constraint impulses (contact, weld, ball, limit, coupler,
actuation gain) and `δt` is the step. The plant's live output ports know
nothing about this; they evaluate the compliant point/hydroelastic model at the
current `(q, v)` and disagree with the simulated trajectory.

A previous attempt *evaluated* the ICF force law at the current state with a
vanishing `δt`. That is wrong for near-rigid constraints: ICF is implicit, so
the impulse it applies is whatever is needed to satisfy the constraint at the
*end* of the step, which is small. Evaluating a stiff law at the *start*-of-step
configuration, where the constraint is violated by O(1e-8 m), produces forces
of O(1e4 N). The mismatch is structural, not a tuning issue.

## 2. What we report

The forces of the solve that produced the current state: `γ/δt`, where `γ`
and `δt` come from the last accepted ICF solve.

- `Jᵀγ/δt` is exactly the constraint force in the momentum balance that took
  `x₀ → x₁`. It is first-order consistent, the same order as the integrator.
- Within a step the constraint impulse is bounded by the drift accumulated
  during that step, O(δt²·a), so `γ/δt = O(M·a)`. There is no `1/δt²`
  amplification, because the stiff law is never evaluated at a stale state.
- In error-controlled mode CENIC propagates the two-half-step result, so the
  relevant solve is the second half step (`δt = h/2`). In fixed-step mode it is
  the full step (`δt = h`). The sample's kinematics (contact points, surface
  centroids, reaction force frames) are those at the start of the recorded
  solve, so in error-controlled mode they are at q(t₀+h/2) and can lag the
  Context's poses at t₀+h by up to half a step. Rejected steps need care:
  `IntegratorBase` restores only time and continuous state, but every trial
  records a sample into abstract state. The final accepted `DoStep` always
  overwrites the sample, so sample and state stay paired. However, sampled
  ports feed external systems (e.g., a controller using the sampled
  acceleration), and a retry would see the rejected trial's sample. So
  `DoStep` remembers the sample as of the start of each fresh step and puts it
  back when it detects a retry (a step starting at the time of the last solve),
  before linearizing the external systems.

These are the same semantics as Drake's existing *sampled output ports* for a
discrete plant: the dynamics ports report the sample recorded by the most
recent step; before the first step they are zero/empty; editing the state by
hand between steps does not change them until the next step.

## 3. Architecture

```
                 pre-Finalize opt-in
  user ───────────────────────────────▶ MultibodyPlant::SetUseSampledOutputPorts(true)
                                           │ Finalize() declares abstract state 0:
                                           │   Value<DiscreteStepMemory>  (no update events)
                                           ▼
  CenicIntegrator::DoStep                MultibodyPlant
  ───────────────────────                ──────────────
  solve ICF (model_at_x0_ or              dynamics ports declared with
  model_at_xh_) → v₁, data_               DeclareSampledOutputPort(calc, abstract_state_ticket(0))
        │                                        ▲
        │ context still at solve start x_s       │ read sample (or zero/empty if none)
        ▼                                        │
  RecordStepMemory(model, ...)  ─── writes ──▶  DiscreteStepMemory::Data<T>
        │                                          acceleration_kinematics_cache
        │ uses plant helpers via                   reaction_forces
        │ MultibodyPlantIcfAttorney                contact_solver_results.tau_contact
        │                                          net_actuation
        ▼                                          geometry_contact_data
  then x_next → context, t → t + h                 contact_results   (new field)
```

**Plant side** (`multibody/plant`). The discrete plant already stores a
`DiscreteStepMemory` in abstract state slot 0 and wires every dynamics port to
that ticket, with zero/empty fallbacks when no sample exists. This feature
reuses that machinery for continuous plants:

- `SetUseSampledOutputPorts(true)` is now permitted on a continuous plant.
  It must be called before `Finalize()` because that is when the abstract
  state is declared and the ports get their tickets. It stays opt-in because
  any abstract state makes `Linearize` reject the plant. The config flag
  `MultibodyPlantConfig::use_sampled_output_ports` still applies to discrete
  plants only.
- The `sampled` calc paths for contact results, net actuation, generalized
  contact forces, and the acceleration cache serve continuous plants from the
  memory without touching the discrete update manager.
- `DiscreteStepMemory::Data<T>` gained a `ContactResults<T> contact_results`
  field, used only by continuous plants (the discrete manager rebuilds results
  from pairs).
- `CalcReactionForces` is split so the inverse-dynamics and joint-frame mapping
  half is a reusable helper. A new private
  `CalcAccelerationKinematicsAndReactionForces(context, applied_forces, vdot,
  ac, reaction_forces)` fills the acceleration cache and reaction forces from a
  given `vdot` and applied `MultibodyForces`. Both are exposed to CENIC through
  `MultibodyPlantIcfAttorney`. Taking the two outputs rather than the
  `DiscreteStepMemory::Data` they live in keeps the internal
  `discrete_step_memory.h` out of the public `multibody_plant.h`.

**Integrator side** (`multibody/cenic`). `RecordStepMemory` runs in `DoStep`
after the accepted solve and before the new state is written, so the plant
context still holds the solve's start state and its kinematics caches are
valid. It:

1. Evaluates the constraint impulses at `v₁` with `IcfModel::CalcAllData`.
   Gain, limit and coupler constraints come from the **full** (unreduced) model:
   their regularization depends only on the diagonal of the mass matrix, which
   reduction doesn't change, and this keeps the actuation of locked joints.
   Contact patches and weld, ball and distance constraints come from the
   **reduced** model that was solved when joint locking reduced the problem.
   Reduction treats locked bodies as anchored and re-derives these
   constraints' regularization from the remaining bodies (e.g., with one of two
   equal masses locked, `R` is half the full model's), so the full model's
   impulses at the same `v₁` are *not* the ones that advanced the state. The
   reduced pools keep the original body indices, so their impulses also load
   the locked bodies. Constraints with no surviving body (e.g., a weld between
   two locked bodies) are not in the reduced model and report nothing; the
   locked joints carry their loads. Contact patches are the exception: the plant
   filters contact by spanning tree but reduction works per clique, so a patch
   whose bodies are all locked can still be in the geometry list. Such patches
   (the reduced pool records `original_patches()` to tell) are reported from
   the full model, so contact results keep one entry per point pair or surface.
   The generalized contact forces are the same patch impulses mapped through
   the *full* model's Jacobians, so a locked coordinate still carries the load
   its body feels (expanding the reduced gradient would put zeros there).
2. Builds the applied `MultibodyForces`: force elements (gravity, springs,
   damping) and the `applied_*` input ports at the start state; actuation as
   the gain-constraint impulse `γ_u/δt` when effort limits are active,
   otherwise `B·u` with `u` clamped; patch, weld, ball, and distance
   constraints as spatial forces `Γ/δt` on both bodies; limit and coupler
   constraints as `Jᵀγ/δt`. Spatial forces are kept spatial because joint
   reaction forces need the body-level loads, not just generalized forces.
   Bodies welded to the world are not part of the ICF problem, but their weld
   joints still transmit the loads applied to them, so their impulses are kept;
   only the world body itself is skipped.

   The `applied_*` input ports are evaluated at the start state of the
   recorded solve, explicitly. This is only first-order consistent with what
   the solve applied, for two reasons. First, ICF applies external forces
   through a gain constraint, implicitly (`−Ke·v + be`), so the two differ by
   O(δt·Ke·v̇) when the forces depend on velocity. Second, in error-controlled
   mode the recorded solve is the second half step, which starts at x(t₀+h/2)
   but reuses the external-system linearization taken at x₀. The gain impulse
   cannot be used instead, because it is a generalized force and joint
   reaction forces need the per-body spatial forces. Both errors are O(h), the
   same order as the integrator, but the first can be large for stiff
   velocity-dependent forces: it is about `Ke·(v₁−v₀)`, so a 1 kg free body at
   1 m/s with `F = −1000·v` and `h = 0.01` reports a 909 N reaction on its
   floating joint, which should transmit nothing. Reporting this exactly would
   require linearizing the spatial force port per body. Only the spatial forces affect the reported
   reaction forces: inverse dynamics determines each joint's reaction from
   body spatial forces and `v̇`, and applied generalized forces only enter its
   residual.

   The actuation gain constraints act per dof, with a single effort limit
   each, so `IcfBuilder` rejects plants where several actuators share a joint.
   Each actuated dof therefore has exactly one actuator, and `net_actuation`
   is `Bᵀ·γ_u/δt`.
3. Sets `vdot = (v₁ − v₀)/δt` and `tau_contact = Jᵀγ_contact/δt`.
4. Builds `ContactResults` from the patch pool. Patch ordering is fixed by
   `IcfBuilder`: point pair `p` is patch `p` (one pair each); hydroelastic
   surface `s` is patch `num_point_pairs + s`. The pool orders bodies by
   anchorage, so the reported force is negated when the pool's body B is not
   the body of `point_pair.id_B` (or of the surface's `id_M`). Hydroelastic
   spatial forces are shifted to the surface centroid.
5. Allocates the memory in abstract state slot 0 and fills it via the attorney.

**Shared geometry data.** `IcfBuilder` now holds the plant's cached
`GeometryContactData<T>` handle instead of private vector copies. The
`HydroelasticContactInfo` objects in `ContactResults` keep raw pointers to the
contact surfaces, so the results need shared ownership of those surfaces; the
handle's `Share()` provides that as the results' backing store at no copy
cost.

## 4. Behavioral notes

- Other integrators never write the sample, so with sampled ports enabled and
  a non-CENIC integrator the dynamics ports report zero/empty. This is
  documented on `SetUseSampledOutputPorts` and in the CENIC class docs.
- The plant filters out contacts whose two bodies both have zero DoFs (locked
  trees or the world). A body whose whole tree is locked therefore reports no
  contacts, and its locked joint carries the load in `reaction_forces`. This
  matches discrete-plant behavior.
- The locking filter in `MultibodyPlant::CalcGeometryContactData` previously
  dereferenced an invalid tree index for world-anchored geometry whenever a
  joint was locked; that is fixed here by treating the world as zero-dof.

### Deprecating CENIC without sampled output ports

Under CENIC the plant's live dynamics output ports report the plant's own
continuous contact model, not the forces of CENIC's solve. CENIC will
eventually refuse a plant without sampled output ports. For now,
`CenicIntegrator::DoInitialize()` emits a once-per-process
`drake::internal::WarnDeprecated` (removal date 2027-03-01) telling users to
call `SetUseSampledOutputPorts(true)` before `Finalize()`;
`DRAKE_DEPRECATION_RUNTIME_SEVERITY=error` turns it into the future refusal.
Note that `MultibodyPlantConfig::use_sampled_output_ports` is currently ignored
for continuous plants, so a YAML-only user cannot comply until the config
change (a separate PR) lands; the removal date must follow that.

### Fail-fast without a recording integrator

A continuous plant with sampled output ports under an integrator that never
records would report zeros forever. `CenicIntegrator::DoInitialize()` sets
`DiscreteStepMemory::recorder_attached` in the plant's state, and the plant
declares an initialization publish event (only in this configuration) that
throws if the flag is missing. `Simulator::Initialize()` runs the integrator's
initialization before the initialization events, so a Simulator with any other
integrator fails at `Initialize()` with a message naming both remedies. Users
who build contexts and read ports without a Simulator, or drive an integrator
directly, don't get the check. A system with no continuous state at all is
rejected by CENIC itself (see above), since it is never stepped.

## 5. Tests

- `multibody/plant/test/multibody_plant_test.cc`:
  `ContinuousSampledOutputPorts` checks declaration, zero/empty fallbacks, live
  kinematic ports, and the post-Finalize throw.
- `multibody/cenic/test/cenic_integrator_test.cc`: sampled ports before and
  after steps, exact `vdot`, reaction forces against a live reference plant,
  the motivating weld with a 1e-8 m violation (forces stay O(weight)),
  joint locking (locked joint carries the contact load), and point and
  hydroelastic ball-on-table contact results with body-ordering signs, for
  fixed-step and error-controlled modes; sampled feedback that must not see a
  rejected trial's sample; a ball on a shelf welded to the world (the weld
  reports both weights); a whole locked tree whose ball constraint to a free
  body must report the impulse that was solved (not the full model's), and the
  contact analog; reinitializing with a fresh context; a locked sphere whose contact patch reduction drops, next to an unlocked
  branch (one contact result per point pair) and the locked slider's generalized
  contact force; the deprecation warning for CENIC without sampled ports (own test binary, run
  at error severity); sampled ports requiring a recording integrator (and the recorder attaching again after a context
  reset); a system with no continuous state; and the stiff external
  force limitation above.
