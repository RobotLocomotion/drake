# ICF-consistent reaction-force reporting for continuous MultibodyPlant

This document describes how a continuous-time `MultibodyPlant` reports **joint
reaction forces** that are consistent with the Irrotational Contact Fields (ICF)
convex contact model used by `CenicIntegrator`, rather than with the plant's own
compliant point/hydroelastic contact model. It records the design, the
mathematical justification for each choice, and the (deliberate) scope
limitations.

> **Scope.** This feature reroutes only the `reaction_forces` output port.
> `contact_results` and the acceleration/`vdot` ports are intentionally left on
> the compliant model (see [Scope and limitations](#scope-and-limitations)).

---

## 1. Motivation

A continuous `MultibodyPlant` (`time_step == 0`) exposes force-reporting output
ports (`reaction_forces`, `contact_results`) and a continuous ODE
(`EvalForwardDynamics`) whose contact contribution is the plant's *compliant*
point/hydroelastic model. `CenicIntegrator` does **not** integrate that ODE.
Each step it solves a convex ICF optimization problem whose contact and
constraint force law is a *different* physical model (regularized compliant
contact with convex friction and near-rigid stabilization). Consequently, the
forces the plant reports from its compliant model are inconsistent with the
trajectory that was actually simulated.

The goal is to make `reaction_forces` reflect the ICF forces — the forces
consistent with the dynamics `CenicIntegrator` integrates — while keeping the
output port a **pure function of the plant `Context`** (so it is well defined
independent of the integrator's internal state).

---

## 2. The ICF model

At a state $(q_0, v_0)$ and time step $\delta t$, ICF defines the convex problem

$$
\min_{v}\ \ell(v;\,q_0,v_0,\delta t)
   = \tfrac12 v^\top A\,v - r^\top v + \ell_c(v),
$$

with

$$
A = M(q_0) + \delta t\,D, \qquad r = A\,v_0 - \delta t\,k_0,
$$

where $M$ is the mass matrix, $D$ the (implicit) joint damping, and $k_0$ the
bias term (Coriolis, gyroscopic, gravity, and applied non-constraint forces).
$\ell_c(v)$ is the sum of per-constraint convex cost terms whose negated
gradient is the generalized constraint impulse:

$$
\nabla \ell(v) = A v - r - J^\top\gamma(v),
$$

so the stationarity condition $\nabla\ell(v^\*)=0$ is the **discrete momentum
balance**

$$
\boxed{\,M\big(v^\* - v_0\big) + \delta t\,k_0 = J^\top\gamma(v^\*)\,}
\tag{2.1}
$$

Here $\gamma$ are constraint **impulses** (units of force $\times$ time). For a
single contact pair, the normal impulse from the lagged discrete Hunt–Crossley
model is

$$
\gamma_n(v_n) = \delta t\,\big(f_{e0} - \delta t\,k\,v_n\big)_+\,\big(1 - d\,v_n\big)_+,
\qquad f_{e0} = -k\,\phi_0,
\tag{2.2}
$$

where $\phi_0$ is the signed distance at $q_0$, $k$ the stiffness, $d$ the
dissipation, $v_n$ the normal contact velocity, and $(x)_+ = \max(0,x)$
(`patch_constraints_pool.cc`).

Two distinct time steps appear in the ICF force law and, importantly, they are
independent knobs:

- The **position prediction** inside the force law — e.g. the
  $f_{e0} - \delta t\,k\,v_n$ term in (2.2), which evaluates the elastic force
  at the *predicted* penetration $\phi_0 + \delta t\,v_n$ — uses
  `IcfModel::time_step()` $=\delta t$.
- The **near-rigid regularization** (stiffness cap, holonomic/limit/coupler
  regularization, SAP stiction) floors on
  `IcfModel::effective_time_step()` $= \max(\delta t,\ k_{H\!\min})$ with
  $k_{H\!\min} = 10^{-4}\,\mathrm{s}$.

---

## 3. What we report and why it is well posed

### 3.1 The reported quantity

We report the ICF constraint force law **evaluated at the current state**
$(q_0, v_0)$, converted from impulse to force by dividing by a fixed reporting
time step:

$$
f_{\text{constraint}} \;=\; \frac{J^\top\gamma(v_0)}{\delta t_{\text{report}}},
\qquad
\delta t_{\text{report}} = 10^{-30}\ \text{(fixed)}.
\tag{3.1}
$$

This is computed by building the ICF model at $(q_0,v_0)$ and calling
`IcfModel::CalcData(v0, &data)` — an **evaluation only, no optimization solve**.
It is a pure function of the `Context`, with no dependence on the integrator's
adaptive step size, sub-steps, or transient last-solve impulses.

Actuation and other external forces are handled among the *non-contact* forces
(via `CalcNonContactForcesContinuous`), so the ICF model is built with **null
feedback gains** and carries no gain (actuation) constraints — avoiding any
double counting. Continuous plants carry no joint-limit penalty forces, so
there is likewise no double counting with the ICF joint-limit constraints.

### 3.2 Force decomposition (spatial vs. generalized)

For correct inverse-dynamics distribution, constraint contributions must enter
in their natural representation, mirroring
`SapDriver::CalcDiscreteUpdateMultibodyForces`:

| Constraint type            | Representation        | Source                                            |
| -------------------------- | --------------------- | ------------------------------------------------- |
| Contact patch, weld, ball  | per-body **spatial** $F_{B_o}$ | patch $\Gamma_{B_o}$ pool; weld/ball `CalcSpatialImpulses` |
| Joint limit, coupler       | **generalized** $\tau$ | pool `AccumulateGradient`                          |

- For a contact patch, the pool stores the spatial impulse $\Gamma_{B_o}$ on
  body $B$; the reaction on body $A$ is $-\operatorname{Shift}(\Gamma_{B_o},\,p_{AB})$.
  Each is placed on the body's `MultibodyForces` slot as
  $\Gamma/\delta t_{\text{report}}$.
- For weld/ball, `CalcSpatialImpulses` returns the impulses already resolved on
  each body.
- For limit/coupler, accumulating each pool's `AccumulateGradient` into a zeroed
  vector yields $-\sum J^\top\gamma$ in generalized space, so the generalized
  constraint force is $\tau \mathrel{-}= g/\delta t_{\text{report}}$.

Body indices stored by the ICF pools are plant `BodyIndex` values; they map to
`MobodIndex` via `plant.get_body(b).mobod_index()` for indexing
`MultibodyForces::mutable_body_forces()`.

### 3.3 Reaction forces

The full applied force is

$$
F_{\text{applied}} \;=\; F_{\text{non-contact}}(q_0,v_0) \;+\; f_{\text{constraint}}(q_0,v_0),
$$

with $F_{\text{non-contact}}$ from `CalcNonContactForcesContinuous` (gravity,
force elements incl. joint damping, actuation, other input-port forces).

Because `EvalForwardDynamics` would use the *compliant* contact model (which is
inconsistent with $F_{\text{applied}}$), we compute the generalized
acceleration directly from $F_{\text{applied}}$ via the articulated body
algorithm:

```cpp
CalcArticulatedBodyForceCache(context, F_applied, &aba_force_cache);
CalcArticulatedBodyAccelerations(context, aba_force_cache, &ac);
vdot = ac.get_vdot();
```

so that

$$
M(q_0)\,\dot v + C(q_0,v_0) = F_{\text{applied}}.
\tag{3.2}
$$

Reaction forces then come from the existing inverse-dynamics path
(`CalcInverseDynamics` with $\dot v$ and $F_{\text{applied}}$), which yields the
per-mobilizer spatial reactions $F_{BM_o}$ and finally the joint reactions
$F_{CJc}$ after the standard frame conversions. Since $\dot v$ is obtained from
$F_{\text{applied}}$ by (3.2), the inverse-dynamics residual is $\approx 0$ and
the reported $\big(F_{\text{applied}},\ \dot v,\ \text{reactions}\big)$ triple
satisfies Newton's continuous momentum balance **exactly**.

---

## 4. The reporting time step $\delta t_{\text{report}} = 10^{-30}$

The single fixed constant $\delta t_{\text{report}}$ is doing three jobs. Each
is justified below.

### 4.1 It sends the position-prediction bias to zero

From (2.2), the reported normal force is

$$
\frac{\gamma_n}{\delta t_{\text{report}}}
 = \big(f_{e0} - \delta t_{\text{report}}\,k\,v_n\big)_+\,(1 - d\,v_n)_+
 \;\xrightarrow[\ \delta t_{\text{report}}\to 0\ ]{}\;
   (f_{e0})_+\,(1 - d\,v_n)_+,
$$

i.e. the exact continuous Hunt–Crossley force at $(q_0,v_0)$. The
$\delta t_{\text{report}}\,k\,v_n$ term is the "$q_0 + \delta t\,v$" prediction;
driving $\delta t_{\text{report}}$ small removes it. Crucially, because the
prediction rides on `time_step()` while the regularization floors on
`effective_time_step()` $=k_{H\!\min}$, shrinking $\delta t_{\text{report}}$
below $k_{H\!\min}$ removes the prediction bias **without** changing the
regularization — the reported force is the compliant law regularized at the same
$k_{H\!\min}$ that `CenicIntegrator` uses. All constraint types share this
structure (holonomic/limit/coupler use `dt` for the prediction and `dt_eff` for
regularization).

### 4.2 The impulse→force conversion is round-off safe (it is not a finite difference)

The classic "$h\to 0$ destroys precision" intuition comes from finite
differencing, where two nearly-equal $O(1)$ values are subtracted and the tiny
remainder divided by a tiny $h$, amplifying round-off as $O(\varepsilon/h)$.
Nothing here is differenced. Each impulse is an **analytic product**
$\gamma = \delta t\cdot f$, with $f$ an $O(1)$ quantity formed independently of
$\delta t$. Hence

$$
\text{force} = \frac{\gamma}{\delta t_{\text{report}}} = f\,(1 + O(\varepsilon)),
$$

with round-off $O(\varepsilon)$ **independent of $\delta t_{\text{report}}$** —
the $\delta t$ cancels analytically. Every constraint impulse scales as
$O(\delta t)$ (required for a finite continuous-force limit to exist): contact
$\gamma_n \propto \delta t$; for bilateral/limit constraints $R^{-1}\propto\delta t$
while the stabilization bias $\hat v = -g_0/(\delta t + \tau)$ is $O(1)$, so
$\gamma = R^{-1}(\hat v - Jv) \propto \delta t$ with a finite $\gamma/\delta t$
limit. The only genuine cancellation site, $f_e = f_{e0} - \delta t\,k\,v_n$,
gets *cleaner* as $\delta t\to 0$ (the subtracted term shrinks below the ULP of
$f_{e0}$) — and this is the *one place we deliberately drop information*, since
that dropped term is exactly the $q_0 + \delta t\,v$ position-prediction bias we
want gone (§4.1).

**The intermediate shift does not lose precision either.** Between $\gamma$ and
the final $\gamma/\delta t$, the contact impulse at point $C$ is shifted to the
body origin as a spatial impulse
$\Gamma_{B_o} = \big[\,p_{BC}\times\gamma_{Bc};\ \gamma_{Bc}\,\big]$ (and the
$B\!\to\!A$ reaction shift is analogous). One might worry that shifting a
$\sim\!10^{-30}$ impulse before rescaling loses accuracy — it does not, because
floating-point *relative* precision is scale-invariant across the normal range.
Concretely:

- Multiplying/dividing by a scalar only shifts the exponent; $\mathrm{fl}(c\,x)=c\,x(1+\delta)$,
  $|\delta|\le\varepsilon$, whether $c$ is $10^{-30}$ or $10^{30}$. So $\gamma$
  (formed as $\delta t\cdot f$ with $f$ at relative $\varepsilon$), the shift, and
  the later $/\delta t$ each carry the same $\sim\varepsilon$ relative error.
- $\gamma\sim10^{-30}$ and the moment term $p_{BC}\times\gamma\sim10^{-31}$ are
  $\sim\!278$ decades above the denormal floor, so both keep a full 52-bit
  mantissa. A `Vector6` is six *independent* doubles (no shared exponent), so the
  smaller torque component is not absorbed by the force component; the later
  per-component $/\delta t$ restores each to $O(1)$ at relative $\varepsilon$.
- The moment-arm cross product's only precision hazard — cancellation when
  $p_{BC}$ is nearly parallel to $\gamma_{Bc}$ (a small torque relative to
  $|p||\gamma|$) — has a cancellation ratio $|p\gamma|/|\text{result}|$ that is
  purely geometric: scaling $\gamma$ by $10^{-30}$ scales numerator and result
  together, leaving it unchanged. It is exactly the error one would incur at
  $O(1)$ force, neither created nor worsened by the small $\delta t$.

The only way the tiny magnitude *would* bite is cross-scale absorption — adding
the $O(10^{-30})$ impulse to an $O(1)$ quantity before rescaling. The ordering
avoids this: each spatial impulse is divided by $\delta t$ (to $O(1)$) *before*
being summed with the $O(1)$ non-contact body forces, so every addition is
$O(1)+O(1)$; the $10^{-30}$ values only ever meet each other.

**Safety envelope.** The binding limits are under/overflow of intermediates,
which are far away: impulses $\gamma\sim\delta t\cdot f$ are $\sim 10^{-49}$ for
even a $10^{-9}\,\mathrm N$ force (vs. the $10^{-308}$ denormal floor), and the
$1/\delta t$-type regularization terms don't overflow until
$\delta t \lesssim 10^{-302}$. Any value in roughly $[10^{-16},\,10^{-40}]$ is
equally accurate and safe; $10^{-30}$ sits comfortably in the middle. The ICF
code is *designed* to accept $\delta t < k_{H\!\min}$ (that is the entire purpose
of `effective_time_step`), so sub-$k_{H\!\min}$ steps are an anticipated input,
not an abuse.

### 4.3 It makes $v_0$ a machine-precision stand-in for the equilibrated $v^\*$

We evaluate $\gamma$ at $v_0$, **not** at the ICF optimum $v^\*$, so in general
$\gamma(v_0)$ does not satisfy the stationarity condition
$\nabla\ell(v)=0$; that holds at $v^\*$ (and at steady state, where $v^\*=v_0$).
The residual, in force units, is

$$
\frac{\nabla\ell(v_0)}{\delta t} = k_0 - J^\top f_c(v_0) \approx -M\,\dot v,
$$

i.e. exactly the inertial term — the forces are not meant to sum to zero while
the system accelerates; their net *is* $M\dot v$.

The error from evaluating at $v_0$ instead of $v^\*$ is negligible because
$\delta t_{\text{report}}$ is tiny. Over a step, the impulses are
$\gamma=\delta t\,f$ with bounded (regularized) $f$, so the Newton step is

$$
v^\* - v_0 = -H^{-1}\,\nabla\ell(v_0),\qquad
H = A + (\text{constraint Hessian}) \approx M + O(\delta t),\qquad
\nabla\ell(v_0) = O(\delta t),
$$

giving $v^\* - v_0 = O(\delta t)\approx 10^{-30}$. The constraint Hessian is
itself $O(\delta t)$ — for contact,
$G = \mu n_0/(\cdot)\,\mathbf M - (\mathrm d\gamma_n/\mathrm dv_n)P_n$ with
$n_0 = f_{n0}\,\delta t$ and $\mathrm d\gamma_n/\mathrm dv_n = O(\delta t)$ — so
even stiff near-rigid constraints keep $H\approx M$ and $v^\* - v_0 = O(\delta t)$.
Since $\partial\gamma/\partial v = O(\delta t)$, the reported force differs from
the equilibrated one by

$$
\frac{\gamma(v^\*) - \gamma(v_0)}{\delta t} = O(\delta t) \approx 10^{-30}.
$$

So evaluating the force law at $v_0$ (and skipping the convex solve entirely)
returns the momentum-balancing impulses to machine precision.

> **Corollary (why not default to $k_{H\!\min}$?).** If
> $\delta t_{\text{report}} = k_{H\!\min}$, then $v^\* - v_0 = O(k_{H\!\min})$ and
> the reported forces would differ from the balanced ones at $O(10^{-4})$, *and*
> the position-prediction bias (4.1) would return. The $10^{-30}$ choice is what
> makes "evaluate at $v_0$, don't solve" legitimate.

---

## 5. Architecture

### 5.1 Why an abstract base plus downstream injection

The ICF solver library depends on the full `//multibody/plant` target
(`IcfBuilder` includes `multibody_plant.h`). A plant-owned manager that used
`IcfBuilder` would therefore create a Bazel cycle
(`plant → icf → plant`). This is unlike SAP, which depends only on the leaf
`//multibody/plant:slicing_and_indexing`, allowing `CompliantContactManager` to
live inside the plant target and be constructed at `Finalize()`.

The resolution:

- **`internal::ContinuousContactForceReporter<T>`** — an abstract base in
  `//multibody/plant` (`continuous_contact_force_reporter.h`) with a single
  virtual, `CalcAppliedForces(context, MultibodyForces*)`. It has no ICF types,
  so the plant does not depend on ICF.
- **`internal::ContinuousIcfForceManager<T>`** — the concrete implementation in
  `//multibody/cenic` (`continuous_icf_force_manager.{h,cc}`), downstream of both
  plant and ICF.
- **Injection.** The plant holds a nullable
  `unique_ptr<ContinuousContactForceReporter<T>>` and a setter
  `SetContinuousContactForceReporter()`. The presence of a reporter *is* the
  switch: no configuration flag. `CalcReactionForces` routes through the reporter
  when one is set (and the plant is continuous), else uses the compliant path.

### 5.2 Public API

```cpp
plant.Finalize();
// ... build the diagram, create contexts ...
AddIcfContinuousForceReporting(&plant);   // reaction_forces now ICF-consistent
auto integrator = MakeCenicIntegrator(diagram, &context);
```

`drake::multibody::AddIcfContinuousForceReporting(plant)` constructs the manager
and injects it. It requires a finalized, continuous plant.

### 5.3 Geometry query sharing (active contact set)

The active constraint set is determined by a SceneGraph query at $q_0$.
`IcfBuilder::CalcGeometryContactData` was changed to source that query from the
plant's position-keyed `geometry_contact_data` cache
(`MultibodyPlantIcfAttorney::EvalGeometryContactData`) instead of issuing its own
`ComputePointPairPenetration` / `ComputeContactSurfaces`. The same
`ContactModel`-dependent query is thus computed once per configuration and shared
across the reporter, `CenicIntegrator`'s builder, and the plant's own contact
path — and reused by the next integration sub-step at the same $q_0$.

---

## 6. Scope and limitations

- **Only `reaction_forces` is rerouted.** `contact_results` remains on the
  compliant model; reporting per-contact ICF results (point-pair and
  hydroelastic infos with contact point, slip, and separation speed) is future
  work. The acceleration / `vdot` output ports also remain compliant.
- **`vdot` is computed locally** inside `CalcReactionForces` from the reporter's
  forces; it is not published on any port.
- **Reported forces are exact at/near steady state**, where $v^\*=v_0$ and the
  ICF balance is satisfied. In transients they are the instantaneous
  continuous-limit force law at $(q_0,v_0)$ — which, per §4.3, equals the
  equilibrated ICF impulse to $O(\delta t_{\text{report}})$.
- **The ICF force law is regularized at $k_{H\!\min}$.** Reported forces are the
  near-rigid/compliant forces at that time scale — consistent with what
  `CenicIntegrator` integrates, but not a $\delta t$-independent "rigid" force.
- **Sanity invariant.** With no contact (and no other ICF constraints), the ICF
  applied forces and the resulting $\dot v$ reduce to the compliant continuous
  path, so `reaction_forces` is unchanged by adding the reporter — this is a
  regression test (`NoContactMatchesCompliantModel`).

---

## 7. Files

| File | Role |
| ---- | ---- |
| `multibody/plant/continuous_contact_force_reporter.{h,cc}` | abstract base interface (plant core) |
| `multibody/cenic/continuous_icf_force_manager.{h,cc}` | concrete ICF reporter + `AddIcfContinuousForceReporting` |
| `multibody/plant/multibody_plant.{h,cc}` | reporter member, setter, `CalcReactionForces` reroute + ABA `vdot` |
| `multibody/plant/multibody_plant_icf_attorney.h` | `EvalGeometryContactData`, `CalcNonContactForcesContinuous` accessors |
| `multibody/contact_solvers/icf/icf_builder.cc` | geometry query sourced from the plant cache |
| `bindings/pydrake/multibody/cenic_py.cc` | Python binding of `AddIcfContinuousForceReporting` |
| `multibody/cenic/test/continuous_icf_force_manager_test.cc` | C++ unit tests |
| `bindings/pydrake/multibody/test/cenic_test.py` | Python binding test |

---

## 8. References

- A. Castro, X. Han, J. Masterjohn, *Irrotational Contact Fields*, 2023.
  <https://arxiv.org/abs/2312.03908>
- V. Kurtz, A. Castro, *CENIC: Convex Error-controlled Numerical Integration for
  Contact*, 2025. <https://arxiv.org/abs/2511.08771>
