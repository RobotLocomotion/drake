/** @file
 Doxygen-only documentation for @ref drake_contacts.  */

/** @defgroup drake_contacts   Contact Modeling in Drake
    @ingroup multibody

Drake is concerned with the simulation of _physical_ phenomena, including
contact between simulated objects.
Drake approximates real-world physical contact phenomena with a combination
of geometric techniques and response models. Here we discuss the
parameterization and idiosyncrasies of a particular contact response model,
based on point contact, non-penetration imposed with a penalty force, and a
continuous model of friction approximating Coulomb stiction and sliding friction
effects.
This document gives an overview of the state of the implementation of
contact in Drake (as of Q2 2019) with particular emphasis on how to account for
its particular quirks in a well-principled manner. What works in one simulation
scenario, may not work equally well in another scenario. This discussion will
encompass:
- @ref contact_geometry "properties of the geometric contact techniques",
- @ref contact_parameters "choosing appropriate modeling parameters",
- @ref time_advancement_strategy "choosing a time advancement strategy", and
- @ref stribeck_approximation "details of the friction model".

<h2>Mechanics of Solids</h2>
All objects in real life are compliant and deform under the action of external
loads. Deformations, as well as motion, of real life solids are well described
by the theory of _continuum mechanics_, which provides a complete description
of the full three-dimensional state of stress and deformation given a set of
external loads and boundary conditions.
Broadly, how compliant a solid is depends on the material (or materials) it is
made of. Factors such as microstructure, imperfections, anisotropy, etc. affect
the behavior of a material. Ultimately, in continuum mechanics, the behavior of
a particular material can be boiled down to its stress-strain curve,
or constitutive law describing the stress/strain relationship at each point
within a solid composed of that material. The simplest constitutive law is that
of Hooke's law, describing the behaviour of materials which exhibit a
linear-elastic region in the stress-strain curve. Materials such as steel and
aluminum can be modeled as Hookean, as long as strain is within the elastic
range (< 1% strain for steel for example). Other materials such as rubber are
hyperelastic and are best described by a Neo-Hookean law.
The _stiffness_ of a material essentially refers to how steep this
stress-strain curve is. The steeper the slope of the stress-strain curve, the
stiffer a material is. For Hookean materials, this translates to a larger
Young's modulus (the slope of the stress-strain curve in the linear regime).

<h2>Contact Mechanics</h2>
Contact mechanics refers to the study of the deformations that solids undergo as
the result of contact loads. The study of contact mechanics dates back to the
pioneering work of Heinrich Hertz in 1882. The Hertzian theory of contact
describes the loads and deformations when strains are small, the contacting
surfaces are much smaller than the overall dimensions of the contacting bodies
and the materials are linear (i.e. are described by the Hookean law).

In general, when two solids come into contact, they inevitably must undergo
deformation in order to avoid the physical impossibility of interpenetration.
This constraint is described by the Signorini boundary condition, which at each
point in the contact surface imposes a complementarity constraint between
normal stress and penetration (described by a gap function).
Stresses on the contact surface are the result of these deformations.

Ultimately, contact forces are the result of integrating these contact
stresses on the contact surface between the solids.

<h2>Rigid Approximation to Contact</h2>
A rigid body is an _approximation_ to the general mechanics of solids in the
mathematical limit to infinite stiffness.
This mathematical approximation has been used extensively in the past for the
simulation of multibody systems with contact. As an approximation to the real
physical system, it leads to known problems such as indeterminate systems.
Consider for instance a rigid beam supported by its two end points and by a
third point of support right in the middle, through its center of gravity. Under
the action of its own weight, infinite solutions to the static problem exist.

Rigid contact with Coulomb friction is a common approximation used in
simulation as well. It has its own limitations, most notable the possible
non-existence of a solution. An example of this is the well known Painlevé
paradox, a one degree of freedom system with contact that, depending on the
state, has an infinite number of solutions or even no solution.

It should be noted that these artifacts are a consequence of the mathematical
approximations adopted and not a flaw of the original continuum mechanics theory
and even much less, a flaw in the physics itself.


<h3>%Point Contact</h3>
In the limit to rigid contact, often the region of contact, or contact surface,
shrinks to a single point. Notable exceptions are planar boundaries coming into
contact (box on a plane for instance) and rigid bodies with conforming
boundaries. Even in those cases it is common to model the surface contact by
selecting a few points on the surface and generating contact forces only at
those points.

@anchor point_contact_approximation
<h3>Numerical Approximation of %Point Contact</h3>
Contact determination in the limit to rigid is not feasible with floating point
arithmetic given that rigid bodies either lie exactly on top of each other or
are not in contact. In order for a practical implementation to detect contact
the bodies must be in an overlapping configuration, even if by a negligible
small amount, see Figure 1.

In a point contact model two bodies A and B are considered to be in contact if
the geometrical intersection of their original rigid geometries is non-empty.
That is, there exists a non-empty overlap volume. In a point contact model, this
overlap region is simply characterized by a pair of witness points `Aw` and `Bw`
on bodies A and B, respectively, such that `Aw` is a point on the surface of A
that lies the farthest from the surface of B. Similarly for point `Bw`. In the
limit to rigid, bodies do not interpenetrate, the intersection volume shrinks to
zero and in this limit witness points `Aw` and `Bw` coincide with each other.

<h3>Enforcing Non-Penetration with Penalty Forces</h3>
In Drake we enforce the non-penetration constraint for rigid contact using a
penalty force. This penalty force
introduces a "numerical" compliance such that, within this approximation, rigid
bodies are allowed to overlap with a non-empty intersection. The strength of
the penalty can be adjusted so that, in the limit to a very stiff penalty force
we recover rigid contact. In this limit, for which `Aw ≡ Bw`, a contact point
`Co` is defined as `Co ≜ Aw (≡ Bw)`. In practice, with a finite numerical
stiffness of the penalty force, we define `Co = 1/2⋅(Aw + Bw)`. Notice that the
1/2 factor is arbitrary and it's chosen for symmetry.
We define the normal `n̂` as pointing outward from the surface of
`B` towards the interior of `A`. That is, if we denote with `d` the
(positive) penetration depth, we have that `Bw = d⋅n̂ + Aw`. We define a contact
frame C with origin at `Co`. The C frame's z-axis is aligned along the normal n̂
(with arbitrary x- and y-axes). Because the two forces are equal and opposite,
we limit our discussion to the force `f` acting on `A` at `Co` (such that `-f`
acts on `B` at `Co`).

@image html drake/multibody/plant/images/simple_contact.png "Figure 1: Illustration of contact between two spheres."

The computation of the contact force is most naturally discussed in the
contact frame `C` (shown in Figure 1).
The contact force, `f`,  can be decomposed into two components: normal, `fₙ`,
and tangential, `fₜ` such that `f=fₙ+fₜ`. The normal force lies in the
direction of the contact frame's z-axis.  The tangential
component lies parallel to the contact frame's x-y plane.  In Drake's
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
-# Between any two collision geometries, there is a single contact point
`Co`. This point is characterized by two _witness points_ `Aw` and `Bw` on the
geometry surfaces at the points of deepest penetration, see
@ref point_contact_approximation "Numerical Approximation of Point Contact".
-# Contacts are reported as the pair of deepest-penetration witness points
`Aw` and `Bw`, a @ref drake::geometry::PenetrationAsPointPair
"PenetrationAsPointPair" in Drake.
-# Surface-to-surface contacts (such as a block sitting on a plane) are
unfortunately still limited to a single contact point, typically located at
the point of deepest penetration. That point will necessarily change from step
to step in an essentially non-physical manner. Our contact solver has generally
exhibited stable behavior, even under these adversarial conditions. However, we
recommend emulating multi-point contact by adding a collection of spheres
covering the contact surfaces of interest. Refer to the example in
%drake/examples/multibody/inclined_plane_with_body/inclined_plane_with_body.cc
for a demonstration of this strategy.
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
      @ref drake::multibody::CoulombFriction "CoulombFriction" for details,
      which also includes a description for modeling the interaction between two
      surfaces with different coefficients of friction.
   2. Global parameters controlling the stiffness of normal penalty forces.
      @ref drake::multibody::MultibodyPlant "MultibodyPlant" offers a single
      global parameter, the "penetration allowance", described in detail in
      section @ref mbp_penalty_method "Contact by penalty method".
   3. Global parameter controlling the Stribeck approximation of Coulomb
      friction, refer to section @ref stribeck_approximation for details.
      @ref drake::multibody::MultibodyPlant::set_stiction_tolerance()
      "set_stiction_tolerance()" provides additional information and guidelines
      on how to set this parameter.

 @note When modeling the multibody system as discrete (refer to
 the @ref time_advancement_strategy "Choice of Time Advancement Strategy"
 section), only the dynamic coefficient of friction is used while the static
 coefficient of friction is ignored.

 @anchor time_advancement_strategy
 <h2>Choice of Time Advancement Strategy</h2>

 @ref drake::multibody::MultibodyPlant "MultibodyPlant" offers two different
 modalities to model mechanical systems in time. These are:
   1. As a discrete system with periodic updates (the preferred method for
      robustness and speed).
   2. As a continuous system.

 In this section we'll only provide a very limited distinction between a
 continuous and a discrete system. A complete discussion of the subject,
 including the modeling of hybrid systems is beyond the scope of this section
 and thus the interested is referred to the documentation for
 @ref drake::systems::Simulator "Simulator".

 <h3>Discrete MultibodyPlant</h3>
 Currently, this is the preferred modality given its speed and robustness.
 In this modality, the system is updated through periodic updates of length
 `time_step`. This can essentially be seen as a time-stepping strategy with a
 fixed `time_step`. The value of `time_step` is provided at construction of the
 @ref drake::multibody::MultibodyPlant "MultibodyPlant". In Drake we use a
 custom semi-implicit Euler scheme, TAMSI, for multibody systems with
 regularized friction. Details for this solver are provided in the documentation
 for @ref drake::multibody::TamsiSolver "TamsiSolver" and in
 [Castro et al., 2019].

 [Castro et al., 2019] Castro, A.M, Qu, A., Kuppuswamy, N., Alspach, A., Sherman,
   M.A., 2019. A Transition-Aware Method for the Simulation of Compliant Contact
   with Regularized Friction. arXiv:1909.05700 [cs.RO].

 @note For better numerical stability, the discrete model assumes
 both static and kinetic coefficients of friction to be equal, the kinetic
 coefficient of friction is ignored.

 <h3>Continuous MultibodyPlant</h3>
 If the `time_step` defined above is specified to be exactly zero at
 @ref drake::multibody::MultibodyPlant "MultibodyPlant" construction, the
 system is modeled as continuous. What that means is that the system is modeled
 to follow continuous dynamics of the form `ẋ = f(t, x, u)`, where `x` is the
 state of the plant, t is time, and u is the set of externally applied inputs
 (either actuation or external body forces). In this mode, any of Drake's
 integrators can be used to advance the system forward in time. The following
 text outlines the implications of using particular integrators.

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
 forward in time using larger time steps and therefore reduce computational
 cost. Thus far, with our @ref drake::systems::ImplicitEulerIntegrator
 "ImplicitEulerIntegrator" we have not observed this advantage for multibody
 systems using the Stribeck approximation.

 @anchor crafting_collision_geometry
 <h2>Choosing the Right Collision Geometry</h2>

 The compliant point contact model only reports a single contact between bodies.
 More particularly, the contact is characterized by a single point. The point is
 associated with a characteristic area in the model (see above). This has two
 implications:

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

 Next topic: @ref stribeck_approximation
 */

/** @defgroup stribeck_approximation Continuous Approximation of Coulomb
 Friction
 @ingroup drake_contacts

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
 time_advancement_strategy "Choice of Time Advancement Strategy" section), we
 regularize Coulomb friction and only use the static coefficient of friction μₛ
 for better numerical stability.
*/
