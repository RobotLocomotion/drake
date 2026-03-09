/** @file
 Doxygen-only documentation for @ref multibody_solvers.  */

// clang-format off (to preserve link to images)

/** @addtogroup multibody_solvers
@{
@section multibody_simulation Simulation of Multibody Systems with Frictional Contact

@ref drake::multibody::MultibodyPlant "MultibodyPlant" offers two different
modalities to model mechanical systems in time. These are:
  - @ref mbp_discrete "Discrete Models for Simulation" (currently the preferred
     method for robustness and speed),
  - @ref mbp_continuous "Continuous Models for Simulation",

In the following sections we'll only provide a very limited distinction between
a continuous and a discrete system. A complete discussion of the subject,
including the modeling of hybrid systems is beyond the scope of this section and
thus interested readers are referred to the documentation for
@ref drake::systems::Simulator "Simulator".

As of today (May 2024), for multibody problems involving contact with friction,
discrete solvers in Drake perform significantly better than error-controlled
integration methods.
@}
*/

/** @addtogroup mbp_discrete Discrete Models for Simulation
@{
@section mbp_discrete_intro Discrete Models for Simulation

Currently, this is the preferred modality given its speed and robustness. In
this modality, the system is updated through periodic updates of length
`time_step`. This can essentially be seen as a time-stepping strategy with a
fixed `time_step`. The value of `time_step` is provided at construction of the
@ref drake::multibody::MultibodyPlant "MultibodyPlant".

Drake provides two very different simulation technologies:
  1. TAMSI, which formulates non-linear compliant contact with regularized
     friction using a Newton-Raphson solver with a custom "transition aware"
     line search, see @ref Castro2019 "[Castro et al., 2019]"
  2. SAP, a convex formulation originally developed in
     @ref Castro2022 "[Castro et al., 2022]" as an extension to the work in
     @ref Anitescu2006 "[Anitescu, 2006]" and @ref Todorov2014 "[Todorov, 2014]"
     to resolve physical compliance, providing a performant implementation in
     primal coordinates.

To choose different approximations of discrete contact refer to @ref
drake::multibody::DiscreteContactApproximation "DiscreteContactApproximation".
Today SAP is our preferred discrete solver for its robustness, speed, and rich
feature set of constraints. A novel extension to SAP presented in @ref
Castro2023 "[Castro et al., 2023]", still convex, allows for the introduction of
complex models of compliant contact, such as
@ref HuntCrossley1975 "[Hunt and Crossley 1975]". The Hunt & Crossley model is
based on physics, it is continuous and has been experimentally validated.
Therefore it is the preferred model to capture the physics of contact.

@note Discrete models, formulated at the velocity level, cannot differentiate
 between static (μₛ) and dynamic (μₖ) friction; only dynamic friction can be
 resolved. Therefore the static coefficient of friction is ignored.
@}
*/

/** @addtogroup mbp_continuous Continuous Models for Simulation
@{
@section mbp_continuous_intro Continuous Models for Simulation

If the `time_step` defined above is specified to be exactly zero at
@ref drake::multibody::MultibodyPlant "MultibodyPlant" construction, the system
is modeled as continuous. What that means is that the system is modeled to
follow continuous dynamics of the form `ẋ = f(t, x, u)`, where `x` is the state
of the plant, t is time, and u is the set of externally applied inputs (either
actuation or external body forces). In this mode, any of Drake's integrators can
be used to advance the system forward in time. The following text outlines the
implications of using particular integrators.

Integrators can be broadly categorized as one of:
  1. Implicit/Explicit integrators.
  2. Fixed time step/error controlled integrators.

While fixed time step integrators often provide faster simulations, they can
miss slip/stick transitions when the stiction tolerance `vₛ` of the Stribeck
approximation is small (say, smaller than 1e-3 m/s). In addition, they might
suffer from stability problems when the penalty forces are made stiff (see
@ref drake::multibody::MultibodyPlant::set_penetration_allowance()
"set_penetration_allowance()").

Error controlled integrators such as @ref drake::systems::RungeKutta3Integrator
"RungeKutta3Integrator" offer a stable integration scheme by adapting the time
step to satisfy a pre-specified accuracy tolerance. However, this stability
comes with the price of slower simulations given that often these integrators
need to take very small time steps in order to resolve stiff contact dynamics.
In addition, these integrators usually perform additional computations to
estimate error bounds used to determine step size.

Implicit integrators have the potential to integrate stiff continuous systems
forward in time using larger time steps and therefore reduce computational cost.
Thus far, with our @ref drake::systems::ImplicitEulerIntegrator
"ImplicitEulerIntegrator" we have not observed this advantage for multibody
systems using the Stribeck approximation.

@subsection stribeck_approximation Continuous Approximation of Coulomb Friction

Static friction (or stiction) arises due to surface characteristics at the
microscopic level (e.g., mechanical interference of surface imperfections,
electrostatic, and/or Van der Waals forces). Two objects in static contact
need to have a force `fₚ` applied parallel to the surface of contact sufficient
to _break_ stiction. Once the objects are moving, kinetic (dynamic) friction
takes over. It is possible to accelerate one body sliding
across another body with a force that would have been too small to break
stiction. In essence, stiction will create a contrary force canceling out
any force too small to break stiction (see Figure 2).
<!-- This is illustrated much better in the formatted doxygen image, but in
case you are too lazy to look there:
    Pushing Force vs Tangent Force
     |     stiction
  Fₛ |   |
     |   |      kinetic friction
     |   |______________________
 fₜ  |   |
     |   |
     |   |
     |   |
   0 |___|________________________
         0                      Fₚ
                     fₚ
     Figure 2: Idealized Stiction/Sliding Friction Model
-->
@image html drake/multibody/plant/images/ideal_stiction.png "Figure 2: Idealized Stiction/Sliding Friction Model"
In _idealized_ stiction, tangent force `fₜ` is equal and opposite
to the pushing force `fₚ` up to the point where that force is sufficient to
break stiction (the red dot in Figure 2). At that point, the tangent force
immediately becomes a constant force based on the _kinetic_ coefficient of
friction. There are obvious discontinuities in this function which do not occur
in reality, but this can be a useful approximation and can be implemented in
this form using constraints that can be enabled and disabled discontinuously.
However, here we are looking for a continuous model that can produce reasonable
behavior using only unconstrained differential equations. With this model we
can also capture the empirically-observed Stribeck effect where the friction
force declines with increasing slip velocity until it reaches the kinetic
friction level.
<!-- This is illustrated much better in the formatted doxygen image, but in
case you are too lazy to look there:
  Stribeck function: μ vs. vₛ
     |
     |
  μₛ |     **
     |    *  *
     |    *   *
  μₖ |   *      **********
     |   *
     |   *
     |   *
     |  *
     |*____________________
     0     1     2     3
         multiple of vₛ
  Figure 3: Stribeck function for stiction.
-->
@image html drake/multibody/plant/images/stribeck.png "Figure 3: Stribeck function for stiction"
The Stribeck model is a variation of Coulomb friction, where the frictional
(aka _tangential_) force is proportional to the normal force as:
`fₜ = μ⋅fₙ`,
In the Stribeck model, the coefficient of friction, μ, is replaced with a
slip speed-dependent function:
`fₜ = μ(s)⋅fₙ`,
where `s` is a unitless multiple of a _new_ parameter: _slip tolerance_ (`vₛ`).
Rather than modeling _perfect_ stiction, it makes use of an _allowable_ amount
of relative motion to approximate stiction.  When we refer to
"relative motion", we refer specifically to the relative translational speed of
two points `Ac` and `Bc` defined to instantly be located at contact point
`Co` and moving with bodies A and B, respectively.
The function, as illustrated in Figure 3, is a function of the unitless
_multiple_ of `vₛ`. The domain is divided into three intervals:
   - `s ∈ [0, 1)`: the coefficient of friction rises smoothly from zero to the
   static coefficient of friction, μs.
   - `s ∈ [1, 3)`: The coefficient of friction falls smoothly from
   μs to the kinetic (dynamic) coefficient of friction, μd.
   - `s ∈ [3, ∞)`: Coefficient of friction is held constant at μd.
Other than the residual "creep" velocity limited by `vₛ`, which can be
arbitrarily small (in theory; see @ref
drake::multibody::MultibodyPlant::set_stiction_tolerance()
"set_stiction_tolerance()" for practical considerations),
this model produces a reasonably good approximation of
Coulomb friction. Its primary drawback is that the model is numerically
very stiff in the stiction region, which requires either small step sizes
with an explicit integrator, or use of a more-stable implicit integrator.

@note When modeling the multibody system as discrete (refer to the @ref
multibody_simulation "Choice of Time Advancement Strategy" section), the
model assumes both static and kinetic coefficients of friction to be equal and
the static coefficient of friction is ignored.
@}
*/
