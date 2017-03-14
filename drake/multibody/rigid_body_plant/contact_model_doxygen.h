/** @defgroup drake_contacts   Compliant Point Contacts in Drake

 Drake is concerned with the simulation of _physical_ phenomena, including
 contact between simulated objects.
 Drake approximates real-world physical contact phenomena with a combination
 of geometric techniques and response models. Here we discuss the
 parameterization and idiosyncracies of a particular contact response model,
 based on point contact with compliance and dissipation, and a Stribeck friction
 model approximating Coulomb stiction and sliding friction effects.

 This document gives an overview of the _current_, very limited state of the
 implementation of compliant contact in Drake with particular emphasis on how to
 account for its particular
 quirks in a well-principled manner. What works in one simulation scenario, may
 not work equally well in another scenario. This discussion will encompass:

 - @ref contact_geometry "properties of the geometric contact techniques",
 - @ref contact_model "details of the contact response model", and
 - @ref contact_engineering "techniques for teasing out desirable behavior".

 @section contact_spec  Definition of contact

 Before getting into the details of how contacts are detected and responses are
 modeled, it is worthwhile to define what a contact means to Drake.

 First, contact _conceptually_ occurs between two _surfaces_, one on each of
 two independently moving _bodies_. The contact produces _forces_ on those two
 surfaces, which can then affect the motion of the bodies. In practice, surfaces
 are represented by one or more DrakeCollision::Element objects -- geometric
 shapes rigidly affixed to a body, whose surfaces can engage in contact.

 This document discusses a _compliant_ contact model. In compliant models, the
 bodies are considered deformable. The
 collision geometry represents an object in its undeformed state. As two objects
 collide, the contact forces cause them to deform. Compliant models compute
 the forces that would cause the deformation. The deformed geometry is _not_
 modeled explicitly. Instead, the contact forces are computed based on the
 degree of penetration of the non-deforming collision geometry; greater
 penetration implies larger contact forces.
 One can think of largely rigid objects which have slightly deformable surfaces.
 For this model to be useful in practice, the deformations should be small
 relative to the whole body, so that we can (a) use simple models for the
 relationship between deformation and forces, and (b) neglect the change in mass
 properties induced by the deformation. As such, all discussion of collision
 geometry/elements refers to this _undeformed_ geometry.

 Contacts are defined in terms of these collision Element instances and _not_
 RigidBody instances. For Drake's purposes, a "contact":

 - describes a relationship between two DrakeCollision::Element instances,
   denoted elements `A` and `B`,
 - only exists if the Element instances overlap,
 - quantifies the degree that the two Element instances are overlapping,
 - is characterized by a single contact point and a normal direction that are
   used to define a _contact frame_ `C`, and
 - leads to the generation of two equal-but-opposite forces acting on the
   RigidBody instances to which the corresponding Elements belong.

 Handling physical contact is decomposed into (1) the detection and
 quantification of overlap (penetration) through the use of geometric techniques
 applied to collision geometry, and (2) the generation of the resultant
 forces based on models of material and surface properties resulting in
 deformation and frictional forces.

 Next topic: @ref contact_geometry
*/

/** @defgroup contact_geometry Detecting Contact
 @ingroup drake_contacts

 Given two posed geometric shapes in a common frame, the collision detection
 system is responsible for determining if those shapes are penetrating and
 characterizing that penetration. We won't go into the details of the how and
 why these techniques work the way they do, and, instead, focus on _what_ the
 properties of the results of the _current implementation_ are.  It is worth
 noting that these some of these properties are considered _problems_ yet to be
 resolved and should not necessarily be considered desirable.

 -# Between any two collision Elements, only a _single_ contact will be
 reported.
 -# Contacts are reported as contact at a _point_. (This is a very reasonable
 assumption for smooth convex shapes such as spheres and ellipsoids where
 relative motion must inevitably lead to initial contact at a single point.)
 -# Surface-to-surface contacts (such as a block sitting on a plane) are
 unfortunately still limited to a single contact point, typically located at
 the point of deepest penetration. (That point will necessarily change from step
 to step in an essentially non-physical manner that is likely to cause
 difficulties.)
 -# A contact _normal_ is determined that approximates the mutual normal of
 the contacting surfaces at the contact point.

 Next topic: @ref contact_model
 */

/** @defgroup contact_model Computing Contact Forces
 @ingroup drake_contacts

 Consider @ref contact_spec "the definition of a contact" with
 interpenetrating collision elements `A` and `B`. The single, computed contact
 point serves as the origin of a contact frame `C`, so is designated `Co`; we'll
 shorten that to just `C` when it is clear we mean the point rather than the
 frame. We define the normal as pointing outward from the deformed surface of
 `B` towards the deformed interior of `A`. The `C` frame's z-axis is aligned
 along this normal (with arbitrary x- and y-axes). We are also interested in the
 points of bodies `A` and `B` that are coincident with `Co`, which we call
 `Ac` and `Bc`, respectively. Because the two
 forces are equal and opposite, we limit our discussion to the force `f` acting
 on `A` at `Ac` (such that `-f` acts on `B` at `Bc`).

 @image html simple_contact.png "Figure 1: Illustration of contact between two spheres."

 The computation of the contact force is most naturally discussed in the
 contact frame `C` (shown in Figure 1).

 The contact force, `f`,  can be decomposed into two components: normal, `fₙ`,
 and tangential, `fₜ` such that `f=fₙ+fₜ`. The normal force lies in the
 direction of the contact frame's z-axis.  The tangential
 component lies in the contact frame's x-y plane.  In Drake's compliant contact
 model, although these components are orthogonal, they are _not_ independent;
 the tangential force is a function of the normal force.

 The model described here is adapted from Simbody's
 Hertz/Hunt & Crossley/Stribeck model described in [Sherman 2011]. We will
 summarize the elements of this model below.

 - [Sherman 2011] M. Sherman, et al. Procedia IUTAM 2:241-261 (2011), Section 5.
   http://dx.doi.org/10.1016/j.piutam.2011.04.023

 @section normal_force Hunt-Crossley Normal Force

 Drake uses the Hunt-Crossley model [Hunt 1975] for computing a normal force
 `fₙ` that accounts for both stiffness and dissipation effects. This is a
 continuous model based on Hertz elastic contact theory, which correctly
 reproduces the empirically observed velocity dependence of coefficient of
 restitution, where `e=(1-dv)` for (small) impact velocity `v` and a material
 property `d` with units of 1/velocity. In theory, at least, `d` can be
 measured right off the coefficient of restitution-vs.-impact velocity curves:
 it is the negated slope at low impact velocities.

 Given a collision between two spheres, or a sphere and a plane, we can generate
 a contact force from this equation `fₙ = kxᵐ(1 + mdẋ)` where `k` is a stiffness
 constant incorporating material properties and geometry (to be defined below),
 `x` is penetration depth and `ẋ` is penetration rate (positive during
 penetration and negative during rebound). Exponent `m` depends on the surface
 geometry and captures the change in contact patch area with penetration. For
 Hertz contact where the geometry can be approximated by sphere (or
 sphere-plane) interactions, `m=3/2`. For contact geometry where the patch area
 is independent of penetration depth, `m=1`.

 For Drake, we liberally extend the application of this model to point contact
 and (somewhat implausibly) take `m=1`. So, for Drake, the normal component of
 the contact force is currently:

   `fₙ = kx(1 + dẋ)`,

 where `k > 0` and `d > 0`. Please note, `d` is not a _damping_ coefficient
 (which would have units of force/velocity), but is a _dissipation_ factor,
 with units of 1/velocity, modeling power loss as a rate-dependent fraction
 of the conservative deformation-dependent force.

 By definition `fₙ` should always be positive, so that the contact force is
 a repulsive force. Mathematically, for arbitrary x and ẋ, it is possible for
 `fₙ` to become negative, creating an attractive or "sucking" force. This case
 will be achieved if `ẋ < -1 / d`. To prevent sucking forces, the normal
 component is clamped to zero. In this regime, it is still possible for there to
 be a repulsive force for bodies that are drawing apart (`ẋ < 0`), as long as
 the relative velocities are small. This approximately models recovery of energy
 in the deformed material with observed hysteresis effects as described in
 [Hunt 1975]. However, a surface can't return to its undeformed state
 arbitrarily quickly. If the bodies are pulled apart *faster* than the surface
 can recover, the bodies will separate before the potential energy of
 deformation can be converted to kinetic energy, resulting in energy loss (to
 heat, vibration, or other unmodeled effects).

 - [Hunt 1975] K. H. Hunt and F. R. E. Crossley, "Coefficient of Restitution
   Interpreted as Damping in Vibroimpact," ASME Journal of Applied Mechanics,
   pp. 440-445, June 1975. http://dx.doi.org/10.1115/1.3423596

 @section tangent_force Stribeck Friction Tangential Force

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
 @image html ideal_stiction.png "Figure 2: Idealized Stiction/Sliding Friction Model"

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
 @image html stribeck.png "Figure 3: Stribeck function for stiction"

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

 Next topic: @ref contact_engineering
*/

/** @defgroup contact_engineering Working with Contacts in Drake
 @ingroup drake_contacts

 The behavior of a simulation with contact will depend on three factors:

 - the choice of integrator,
 - contact parameters,
 - nature of collision geometry.

 The three factors are interdependent; specific choices for one factor may
 require supporting changes in the other factors.

 Issues:
 - **Picking a good value for `vₛ`**

   In selecting a value for `vₛ`, you must ask yourself the question, "When
   two objects are ostensibly in stiction, how much slip am I willing to allow?"
   There are two opposing design issues in picking a value for `vₛ`.  On the one
   hand, small values of `vₛ` make the problem numerically stiff during
   stiction. Stable integration then requires either _very_ small step sizes
   when using an explicit integrator, or use of an implicit integrator. Implicit
   integration is under development but not yet available in Drake, so a small
   `vₛ` will require small steps, or high accuracy for the error-controlled RK3.
   On the other hand, it
   should be picked to be appropriate for the scale of the problem. For example,
   a car simulation could allow a "large" value for `vₛ` of 1 cm/s (1e-2 m/s),
   but reasonable stiction for grasping a 10 cm box might require limiting
   residual slip to a mm/s or less, 1e-3 or 1e-4 m/s. Ultimately,
   picking the largest viable value will allow your simulation to run faster.

 - **Picking values for the other contact parameters**

   The contact model provides five parameters:
     - Stiffness `k` in N/m,
     - dissipation `d` in s/m (1/velocity),
     - static coefficient of friction `μs`, unitless,
     - dynamic (kinetic) coefficient of friction `μd`, unitless,
     - and stiction slip speed tolerance `vₛ` in m/s.

   In a compliant model, deformation (which appears as
   penetration of the undeformed geometry) is part of the stable equilibrium
   state. Imagine a box sitting on a half plane.
   The stable penetration depth will, in principle, be equal to the box's weight
   divided by the stiffness. Appropriate stiffness for a 1 kg box is not the
   same as for a 1000 kg car. (In fact, with a small stiffness, the car will
   pass right through the ground while attempting to find the equilibrium
   distance.) Stiffness is the most important parameter for capturing realistic
   deformation of softer materials. The parameter `k` is used to capture both
   the surface material's inherent stiffness per unit area, and the area of
   contact. The dissipation `d` is significant primarily for impacts, where
   there are rapid changes in deformation.

   Simulation speed using an explicit integrator is likely to be most affected
   by `k` and `vₛ`. In cases where more penetration is acceptable, you can
   soften `k` and get better performance in exchange for less-realistic
   deformation. The total contact force at equilibrium is not very sensitive
   to `k` since the penetration will be adjusted as necessary to achieve
   force balance. For stiction behavior, increasing the coefficients of friction
   to unrealistic levels seems counterintuitively to degrade the results. The
   previous note discusses the importance of `vₛ`.

 - **Global contact parameters**

   In its current incarnation, Drake does _not_ support per-object mechanical
   material properties. That means whatever parameter values you select will
   be the same for all contacts in the world.

 - **Surface-on-surface contacts**

   Remember that the contact detection computation produces a single point to
   represent contact between two collision elements. If the contact is a
   surface instead of a point (such as one box lying on another), the contact
   point will *not* be temporally coherent. This will lead to instability
   artifacts which can only be addressed through smaller time steps.

   An alternative is to represent the body's contact differently. For some
   shapes (e.g., boxes), we can introduce two sets of collision elements:
   discrete "points" at the corners, and a box capturing the volume (see
   `block_for_pick_and_place.urdf` as an example). With this strategy, the
   contact "points" are actually small-radius spheres. The volume-capturing
   box should actually be inset from those spheres such that when the box is
   lying on a plane (such that the logical contact manifold would be a face),
   only the contact points make contact, providing reliable points of contact.
   However, for arbitrary configurations contact with the box will provide
   more general contact.

 - **Contact samples**

   Because of weaknesses in the current implementation of contact detection and
   characterization, we get best numerical performance when explicitly
   enumerating contact points on a body (see previous issue). However, each of
   those contact points are processed without any knowledge of the others.
   Thus, for a fixed contact force between two objects, increasing the number
   of explicit contact points reduces the amount of penetration needed to
   generate that force. If you use this technique be sure to consider that each
   explicit point represents a fraction of the contact area, and adjust the
   point-contact stiffness to reflect that.

 - **Choice of integrator**

   Empirical evidence suggests that any integrator _except_
   ExplicitEulerIntegrator can work with this contact model. Generally, the
   RungeKutta2Integrator and SemiExplicitEulerIntegrator require similar time
   steps to produce equivalent behavior. Generally, for a `vₛ` value of
   1e-2 m/s, a timestep on the order of 1e-4 is required for both of these.
   The error-controlled RungeKutta3Integrator will choose very small steps and
   accuracy must be set tight enough to ensure stability. An implicit integrator
   is currently in development and should perform much better on
   stiction-dominated problems such as manipulator grasping.
 */
