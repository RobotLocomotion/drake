/** @defgroup drake_contacts   Contact in Drake
    @ingroup multibody

 Drake is concerned with the simulation of _physical_ phenomena, including
 contact between simulated objects.
 Drake approximates real-world physical contact phenomena with a combination
 of geometric techniques and response models. Here we discuss the
 parameterization and idiosyncracies of a particular contact response model,
 based on point contact with compliance and dissipation, and a Stribeck friction
 model approximating Coulomb stiction and sliding friction effects.

 This document gives an overview of the state of the implementation of compliant
 contact in Drake (as of Q2 2019) with particular emphasis on how to account for
 its particular quirks in a well-principled manner. What works in one simulation
 scenario, may not work equally well in another scenario. This discussion will
 encompass:

 - @ref contact_geometry "properties of the geometric contact techniques",
 - @ref contact_engineering "techniques for teasing out desirable behavior",
 - @ref contact_model "details of the contact response model", and
 - @ref drake_per_object_material "per-object contact materials".

 @anchor point_contact
 <h2>Point contact in Drake</h2>

 As of Q2 of 2019, MultibodyPlant implements a point contact model. In a
 point contact model two bodies A and B are considered to be in contact if the
 geometrical intersection of their original rigid geometries is non-empty. That
 is, there exists a non-empty overlap volume. In a point contact model, this
 overlap region is simply characterized by a pair of points `Ac` and `Bc` on
 bodies A and B, respectively, such that `Ac` is a point on the surface of A
 that lies the farthest from the surface of B. Similarly for point `Bc`.
 In the limit to rigid, bodies do never interpenetrate, the intersection
 volume shrinks to zero and in this limit points `Ac` and `Bc` coincide with
 each other. In Drake we enforce such a rigid constraint using a penalty
 force. This penalty force introduces a "numerical" compliance such that,
 within this approximation, rigid bodies are allowed to overlap with a
 non-empty intersection. The strength of the penalty can be adjusted so that, in
 the limit to a very stiff penalty force we recover rigid contact.
 In this limit, for which `Ac ≡ Bc`, a contact point C is
 defined as `C ≜ Ac (≡ Bc)`. In practice, with a finite numerical stiffness of
 the penalty force, we define `C = 1/2⋅(Ac + Bc)`.

 At point C we define a contact frame; we'll just refer to that frame as `C`
 when it is clear we mean the frame rather than the point.
 We define the normal `n̂` as pointing outward from the surface of
 `B` towards the interior of `A`. That is, if we denote with `d` the
 (positive) penetration depth, we have that `Bc = d⋅n̂ + Ac`.
 The `C` frame's z-axis is aligned along this normal (with arbitrary x- and
 y-axes). Because the two forces are equal and opposite, we limit our discussion
 to the force `f` acting on `A` at `Ac` (such that `-f` acts on `B` at `Bc`).

 @image html multibody/plant/images/simple_contact.png "Figure 1: Illustration of contact between two spheres."

 The computation of the contact force is most naturally discussed in the
 contact frame `C` (shown in Figure 1).

 The contact force, `f`,  can be decomposed into two components: normal, `fₙ`,
 and tangential, `fₜ` such that `f=fₙ+fₜ`. The normal force lies in the
 direction of the contact frame's z-axis.  The tangential
 component lies parallel to the contact frame's x-y plane.  In Drake's compliant
 contact model, the tangential force is a function of the normal force.

 The detailed discussion of the contact force computation is decomposed into
 two parts: a high-level engineering discussion addressed to end users who care
 most about working with the model and a further detailed discussion of the
 mathematical underpinnings of the implemented model. The practical guide should
 be sufficient for most users.

 Next topic: @ref contact_geometry
*/

/** @defgroup contact_geometry Detecting Contact
 @ingroup drake_contacts

 Given two posed geometric shapes in a common frame, the collision detection
 system is responsible for determining if those shapes are penetrating and
 characterizing that penetration. We won't go into the details of the how and
 why these techniques work the way they do, but, instead, focus on _what_ the
 properties of the results of the _current implementation_ are.  It is worth
 noting that some of these properties are considered _problems_ yet to be
 resolved and should not necessarily be considered desirable.

 -# Between any two collision geometries, only a _single_ contact will be
 reported. This pair will contain points `Ac` and `Bc` as defined in @ref
 point_contact.
 -# Contacts are reported as a point pair. A PenetrationAsPointPair in Drake.
 -# Surface-to-surface contacts (such as a block sitting on a plane) are
 unfortunately still limited to a single contact point, typically located at
 the point of deepest penetration. (That point will necessarily change from step
 to step in an essentially non-physical manner that is likely to cause
 difficulties.) Our contact solver has shown to be stable even under these
 conditions. However it is recommended to emulate multi-point contact by adding
 a collection of spheres covering the contact surfaces of interest. Refer to
 the example in `drake/examples/inclined_plane_with_body.cc` for a demonstration
 of this strategy.
 -# A contact _normal_ is determined that approximates the mutual normal of
 the contacting surfaces at the contact point.

 Next topic: @ref contact_engineering
 */

/** @defgroup contact_engineering Working with Contacts in Drake
 @ingroup drake_contacts

 The behavior of a simulation with contact will depend on three factors:

 - contact parameters,
 - the time advancement strategy,
 - nature of collision geometry.

 The three factors are interdependent; specific choices for one factor may
 require supporting changes in the other factors.

 @anchor contact_parameters
 <h2>Contact Parameters</h2>

 Contact modeling in Drake is controlled by a small set of parameters:
   1. Per-geometry coefficients of friction. Refer to the documentation for
      CoulombFriction for details, which also includes a description for
      modeling the interaction between two surfaces with different coefficients
      of friction.
   2. Global parameters controlling the stiffness of normal penalty forces.
      MultibodyPlant offers a single global parameter, the "penetration
      allowance", described in detail in section @ref mbp_penalty_method.
   3. Global parameter controlling the Stribeck approximation of Coulomb
      friction, refer to section @ref stribeck_approximation for details.
      MultibodyPlant::set_stiction_tolerance() provides additional information
      and guidelines on how to select this parameter.

 @anchor time_advancement_strategy
 <h2>Choice of Time Advancement Strategy</h2>

 MultibodyPlant offers two different modalities to model mechanical sytems in
 time. These are:
   1. As a discrete system with periodic updates (the prefered method for
      robustness and speed).
   2. As a continuous system.

 In this section we'll only provide a very limited distinction between a
 continuous and a discrete system. A complete discussion of the subject,
 including the modeling of hybrid systems is beyond the scope of this section
 and thus the interested is referred to the documentation for Simulator.

 <h3>Discrete MultibodyPlant</h3>
 Currently, this is method the prefered method given its speed and robustness.
 In this modality the system is modeled as a system with periodic updates of
 length `time_step`. This can essentially be seen as a time stepping strategy
 with a fixed `time_step`. The value of `time_step` is provided at construction
 of the MultibodyPlant.
 In Drake we use a custom semi-implicit Euler scheme for multibody systems
 using the Stribeck approximation of Coulomb friction. Details for this solver
 are provided in the documentation for ImplicitStribeckSolver.

 <h3>Continuous MultibodyPlant</h3>
 If `time_step` defined above is specified to be exactly zero at the time of
 instantiating a MultibodyPlant, the system is modeled as continuous. What that
 means is that the system is modeld to follow a continuous dynamics of the form
 `ẋ = f(t, x, u)`, where `x` is the state of the plant, t is time and u are
 externally applied inputs (either actuation or external body forces).
 In this mode, Drake allows chose from a variety of integrators to advance the
 continuos system forward in time. Integrators can, in a broad sense, be
 classified according to:
   1. Implicit/Explicit integrators.
   2. Fixed time step/error controlled integrators.

 Fixed time step integratos often provide faster simulations however they often
 miss slip/stick transitions when the stiction tolerance `vₛ` of the Stribeck
 approximation is small (say smaller than 1e-3 m/s). In addition, they might
 suffer of stability problems when the penalty forces are made stiff (see
 MultibodyPlant::set_penetration_allowance()).

 Error controlled integrators such as @ref drake::systems::RungeKutta3Integrator
 "RungeKutta3Integrator" offer a stable integration scheme by adapting the time
 step to satisfy a pre-specified accuracy tolerance. However, this stability
comes with the price of slower simulations given that often these integrators
need to take very small time steps in order to resolve the stiff contact
dynamics. In addition, these integrators usually perform additional computations
to estimate error bounds used to determine step size.

 Implicit integrators have the potential to integrate stiff continuous systems
 forward in time using larger time steps and therefore reduce computational
 cost. Thus far, with our @ref drake::systems::ImplicitEulerIntegrator
 "ImplicitEulerIntegrator" we observed that the additional cost of performing a
 Newton-Raphson iteration at each time step is cost prohibitive for multibody
 systems using the Stribeck approximation.

 @anchor crafting_collision_geometry
 <h2>Choosing the Right Collision Geometry</h2>

 The compliant point contact model only reports a single contact between bodies.
 More particularly, the contact is characterized by a single point. The point is
 associated with a characteristic area in the model (see above). This has two
 implications:

 - **Sampled contact area**

   Increasing the area associated with the point will increase the magnitude of
   the contact force. An alternative solution is to increase the number of
   collision geometries associated with a body such that each point corresponds
   to a smaller portion of the body's surface area. This essentially changes the
   interpretation of the point to be closer in line with a smaller
   characteristic area.

   This would have particular value if your simulation scenario has contacts of
   disparate scales. A single, global characteristic radius may be insufficient.
   By increasing the samples on large contact patches, those contact points will
   be compatible with the smaller characteristic area which works for the small
   contact patches.

 - **Surface-on-surface contacts**

   If the contact between bodies is better characterized as a
   surface instead of a point (such as one box lying on another), the contact
   point will
     - not be guaranteed to be at the center of pressure, possibly inducing
       unrealistic torque, and
     - not be temporally coherent. This will lead to instability artifacts which
       can only be addressed through smaller time steps.

   Both of these issues can be addressed by changing the geometry that
   represent the body's contact surface. For some shapes (e.g., boxes), we can
   introduce two sets of collision elements: discrete "points" at the corners,
   and a box capturing the volume (see `block_for_pick_and_place.urdf` as an
   example). With this strategy, the contact "points" are actually small-radius
   spheres. The volume-capturing box should actually be inset from those spheres
   such that when the box is lying on a plane (such that the logical contact
   manifold would be a face), only the contact points make contact, providing
   reliable points of contact. However, for arbitrary configurations contact
   with the box will provide more general contact.

 Next topic: @ref contact_model_background
 */

/** @defgroup contact_model_background The Details of Computing Contact Forces
 @ingroup drake_contacts

 Drake includes a compliant contact model. Compliant models determine contact
 forces by assuming that all bodies are, to some extent, pliable. As two bodies
 collide, they deform and that deformation produces the contact normal force.
 In practice, this is achieved by allowing _undeformed_ geometry to penetrate
 and then infer the deformation which would eliminate the penetration and
 compute the force from that inferred deformation.
 More particularly, Drake's compliant model is adapted from Simbody's
 Hertz/Hunt & Crossley/Stribeck model described in [Sherman 2011] with further
 details drawn directly from [Hunt 1975].

 - [Sherman 2011] M. Sherman, et al. Procedia IUTAM 2:241-261 (2011), Section 5.
   http://dx.doi.org/10.1016/j.piutam.2011.04.023
 - [Hunt 1975] K. H. Hunt and F. R. E. Crossley, "Coefficient of Restitution
   Interpreted as Damping in Vibroimpact," ASME Journal of Applied Mechanics,
   pp. 440-445, June 1975. http://dx.doi.org/10.1115/1.3423596

 The model naturally decomposes into the normal and tangential components of the
 contact force and the discussion follows this decomposition:
 @ref contact_normal_force "normal force"
 @ref tangent_force "tangent force".

 @anchor contact_normal_force
 <h2>Computing the Normal Component of the Contact Force</h2>

 @anchor compliant_normal_force_overview
 <h3>Overview of Compliant Normal Force</h3>

 Generally, the component of the contact force due to pressure (i.e., _not_
 frictional forces) is simply the pressure on the contact surface integrated
 over that surface's area. If we assume the contact surface is _planar_ then
 we have a well defined normal direction, and the force due to pressure is
 parallel with that normal and can be defined precisely as:

   `fₙ = p(q, v)·A(q)`

 where `A(q)` is the contact patch area, and
 `p(q, v)` is the average contact pressure on that patch, the details of which
 depend on how the contact is characterized (e.g., penetration depth and
 approaching speed), and the geometry and material parameters of the contacting
 bodies. Notice there is no approximation in this equation; it is always true
 since this equation _is_ the definition of the average contact pressure
 `p(q, v)`. Thus, determining the contact normal force consists of determining
 the geometry of the contact patch, and the average pressure on that patch.

 @anchor hunt_crossley
 <h3>Hunt-Crossley Model</h3>

 Drake uses the Hunt-Crossley model [Hunt 1975] for computing a normal force
 `fₙ` that accounts for both elasticity and dissipation effects. This is a
 continuous model based on Hertz elastic contact theory. We'll examine the
 underlying Hertz contact model and then show its extension.

 __Hertz Contact Model__

 The Hertz contact model applies to scenarios where the contacting surfaces can
 be _locally_ represented by two principal axes of curvature (e.g., spheres,
 ellipsoids, cylinders, planes, etc.) For the sake of simplicity, this
 discussion focuses on spheres where a _single_ radius of curvature (per
 surface) is sufficient to describe the contacting geometry. The primary
 advantage of this model is that the magnitude of the normal force can be
 expressed in closed form.

 Consider two spheres made of the same material and of the same size (with
 Young's modulus E and radius R). If the amount that the two spheres are
 penetrating is x, then the resultant normal force (according to the Hertz
 contact model) would be:

   `fₙ = ⁴/₃⋅E⋅√R⋅√x³`.

 The two spheres compress such that they are touching along a disk with radius
 `√(R⋅x)`.  Based on this, we can equate it to the earlier function:

   - `fₙ = p(q, v)·A(q) = ⁴/₃⋅E⋅√R⋅√x³`
   - `p(q, v) = 4/(3π)·E·√(x/R)`
   - `A(q) = π⋅R⋅x`

 This can be generalized to spheres of different sizes and different materials
 by creating _effective_ radii of curvature and Young's modulus. Assuming body i
 has radius Rᵢ and Young's modulus Eᵢ, we can define an effective radius of
 curvature and Young's modules as functions of the contacting geometries:
 `R(R₁, R₂)` and `E(E₁, E₂)`, respectively. Generally, the details of those
 functions depend on the geometry and scenario. See the
 @ref drake_contact_model_impl "implementation details" below for more details.

 This pattern can be extended to contact between other shapes with
 characteristic radii of curvature:

 - Sphere and plane: same as sphere and sphere with the plane treated as a
   sphere with infinite radius of curvature.
 - Two perpendicularly crossed cylinders of _equal_ radius R: same as sphere and
   sphere of radius R.
 - Vertical cylinder of radius R and a plane: `fₙ = 2⋅E⋅R⋅x`.
 <!--
 TODO(amcastro-tri): This math doesn't seem to work out. Given the equation
 `kxᵐ`, and values `m = 1` and `k = 2⋅E⋅R⋅x`, the previous documentation
 suggested the resultant equation was as shown below. However, the L term in the
 k factor becomes R²/L in the final expression. Things don't cancel out right.
 This math needs to be confirmed so that this example can be included again.

 - Two cylinders of radii R₁ and R₂, length L and with parallel axes:
   `fₙ = π/4⋅E⋅R²⋅(R/L)⋅(x/R)`.
   -->

 __Hunt-Crossley Extension__

 The Hunt-Crossley model extends the Hertz contact by adding a dissipation
 factor which depends on the rate of change of penetration. The resultant force
 is:

 `fₙ = H(x)(1 + ³/₂⋅d⋅ẋ)`,

 where `H(x)` is the Hertz factor appropriate for the contact
 geometry, `x` is the penetration depth, `ẋ` is penetration rate (positive for
 increasing penetration and negative during rebound), and `d` is a dissipation
 term that captures the empirically observed velocity dependence of the
 coefficient of restitution, `e = (1 - d⋅v)`, for (small) impact velocity `v`.
 `d` has units of 1/velocity and, in theory at least,  can be measured right off
 the coefficient of restitution-vs.-impact velocity curves; it is the negated
 slope at low impact velocities.

 Please note, `d` is not a _damping_ coefficient (which would have units of
 force/velocity), but is a _dissipation_ factor, with units of 1/velocity,
 modeling power loss as a rate-dependent fraction of the conservative
 deformation-dependent force. For steel, bronze or ivory, [Hunt 1975] reports
 values of d between 0.08-0.32 sec/m.

 By definition `fₙ` should always be positive, so that the contact force is
 a repulsive force. Mathematically, for arbitrary x and ẋ, it is possible for
 `fₙ` to become negative, creating an attractive or "sucking" force. This case
 will be achieved if `ẋ < -1/d`. To prevent sucking forces, the normal
 component is clamped to zero. In this regime, it is still possible for there to
 be a repulsive force for bodies that are drawing apart (`ẋ < 0`), as long as
 the relative velocities are small. This approximately models recovery of energy
 in the deformed material with observed hysteresis effects as described in
 [Hunt 1975]. However, a surface can't return to its undeformed state
 arbitrarily quickly. If the bodies are pulled apart *faster* than the surface
 can recover, the bodies will separate before the potential energy of
 deformation can be converted to kinetic energy, resulting in energy loss (to
 heat, vibration, or other unmodeled effects).

 @anchor tangent_force
 <h2>Stribeck Friction Tangential Force</h2>

 Static friction (or stiction) arises due to surface characteristics at the
 microscopic level (e.g., mechanical interference of surface imperfections,
 electrostatic, and/or Van der Waals forces). Two objects in static contact
 need to have a force `fₚ` applied parallel to the surface of contact sufficient
 to _break_ stiction. Once the objects are moving, dynamic (kinetic) friction
 takes over. It is possible to accelerate one body sliding
 across another body with a force that would have been too small to break
 stiction. In essence, stiction will create a contrary force canceling out
 any force too small to break stiction (see Figure 2).

 <!-- This is illustrated much better in the formatted doxygen image, but in
 case you are too lazy to look there:

     Pushing Force vs Tangent Force

      |     stiction
   Fₛ |   |
      |   |      dynamic friction
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
 @image html multibody/plant/images/ideal_stiction.png "Figure 2: Idealized Stiction/Sliding Friction Model"

 In _idealized_ stiction, tangent force `fₜ` is equal and opposite
 to the pushing force `fₚ` up to the point where that force is sufficient to
 break stiction (the red dot in Figure 2). At that point, the tangent force
 immediately becomes a constant force based on the _dynamic_ coefficient of
 friction. There are obvious discontinuities in this function which do not occur
 in reality, but this can be a useful approximation and can be implemented in
 this form using constraints that can be enabled and disabled discontinuously.
 However, here we are looking for a continuous model that can produce reasonable
 behavior using only unconstrained differential equations. With this model we
 can also capture the empirically-observed Stribeck effect where the friction
 force declines with increasing slip velocity until it reaches the dynamic
 friction level.

 <!-- This is illustrated much better in the formatted doxygen image, but in
 case you are too lazy to look there:

   Stribeck function: μ vs. vₛ

      |
      |
   μs |     **
      |    *  *
      |    *   *
   μd |   *      **********
      |   *
      |   *
      |   *
      |  *
      |*____________________
      0     1     2     3
          multiple of vₛ

   Figure 3: Stribeck function for stiction.
 -->
 @image html multibody/plant/images/stribeck.png "Figure 3: Stribeck function for stiction"

 <!-- TODO(SeanCurtis-TRI,sherm1) Consider using "static" and "kinetic"
 coefficients of friction so we can write μₛ and μₖ in Unicode ("d" isn't
 available as a subscript). This isn't simply a change in this file; the
 code should also reflect this nomenclature change. -->

 The Stribeck model is a variation of Coulomb friction, where the frictional
 (aka _tangential_) force is proportional to the normal force as:

 `fₜ = μ⋅fₙ`

 In the Stribeck model, the coefficient of friction, μ, is replaced with a
 slip speed-dependent function:

 `fₜ = μ(s)⋅fₙ`,

 where `s` is a unitless multiple of a _new_ parameter: _slip tolerance_ (`vₛ`).
 Rather than modeling _perfect_ stiction, it makes use of an _allowable_ amount
 of relative motion to approximate stiction.  When we refer to
 "relative motion", we refer specifically to the relative motion of the two
 points `Ac` and `Bc` on the corresponding bodies that are coincident in space
 with the contact point `C`.

 The function, as illustrated in Figure 3, is a function of the unitless
 _multiple_ of `vₛ`. The domain is divided into three intervals:

    - `s ∈ [0, 1)`: the coefficient of friction rises smoothly from zero to the
    static coefficient of friction, μs.
    - `s ∈ [1, 3)`: The coefficient of friction falls smoothly from
    μs to the dynamic (kinetic) coefficient of friction, μd.
    - `s ∈ [3, ∞)`: Coefficient of friction is held constant at μd.

 Other than the residual "creep" velocity limited by `vₛ`, which can be
 arbitrarily small (in theory; see next section for practical considerations),
 this model produces a reasonably good approximation of
 Coulomb friction. Its primary drawback is that the model is numerically
 very stiff in the stiction region, which requires either small step sizes
 with an explicit integrator, or use of a more-stable implicit integrator.

 Next topic: @ref drake_contact_implementation
*/
