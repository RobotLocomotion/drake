/** @defgroup drake_contacts Contacts in Drake

 Drake is concerned with the simulation of _physical_ phenomena.
 Contact is what makes the difference between physical simulation and astral
 simulation. The former being characterized by objects with extent and mass
 physically interacting with each other in a manner such that energy and
 momentum are conserved and no two entities can occupy the same space-time
 position, and the latter being characterized by massless entities in a higher
 plane of existence whose projection into our own dimension allows for the
 suspension of basic physical law.

 Drake approximates the real-world physical contact phenomena with a combination
 of geometric techniques and response models. The encoded approximations
 themselves have unique idiosyncracies and parameters. What works in one
 simulation scenario, may not work equally well in another scenario.

 This document gives an overview of the _current_ state of the implementation of
 contact in Drake with particular emphasis on how to account for its particular
 quirks in a well-principled manner.  This discussion will encompass:

 - @ref contact_geometry "properties of the geometric contact techniques",
 - @ref contact_model "details of the contact response model", and
 - @ref contact_engineering "techniques for teasing out desirable behavior".

 @section contact_spec  Definition of contact

 Before getting into the details of how contacts are detected and responses are
 modeled, it is worthwhile to define what a contact means to Drake.

 First, contact _conceptually_ occurs between two _bodies_.  The contact
 produces _forces_ on those two bodies.  In practice, bodies are
 represented by one or more DrakeCollision::Elements -- geometric shapes which
 define the extent of the body's volume, rigidly affixed to the body.  Contacts
 are defined in terms of these collision Element instances and _not_ RigidBody
 instances. For Drake's purposes, a "contact":

 - describes a relationship between two DrakeCollision::Element instances,
 denoted elements `A` and `B`,
 - only exists if the Element instances overlap (i.e., are _penetrating_),
 - quantifies the degree that the two Element instances are overlapping,
 - is characterized by a single contact point and a normal direction (both the
 point and the normal vector are measured and expressed in a common frame --
 typically the world frame), and
 - leads to the generation of two equal-but-opposite forces acting on the
 RigidBody instances to which the corresponding elements belong.

 Second, Drake uses a compliant contact model.  It does not strictly enforce
 the tenant that no two objects can occupy the same space.  Instead, it
 implements the idea that, in the real world, no objects are perfectly rigid
 and all deform, even if only slightly, upon contact.  The nature of the forces
 acting on the colliding bodies are related to this deformation. This idea is
 modeled by allowing the collision Elements to overlap, quantifying the degree
 of penetration such that increased penetration implies increased deformation.
 One can think of largely rigid objects which have slightly deformable surface.
 The deformations, relative to the whole body, are so small, that we can use
 a rigid model for the body's mass properties (e.g., inertia tensor).

 Thus, handling physical contact is decomposed into the detection and
 quantification of penetration through the use of geometric techniques and the
 generation of the resultant forces.

 Next topic: @ref contact_geometry
*/

/** @defgroup contact_geometry Detecting Contact
 @ingroup drake_contacts

 Given two posed geometric shapes in a common frame, the collision detection
 system is responsible for determining if those shapes are penetrating and
 characterizing that penetration. We won't go into the details of the how and
 why these techniques work the way they do, and, instead, focus on _what_ the
 properties of the results are.

 -# Between any two collision Elements, only a _single_ contact will be
 reported.
 -# Contacts are reported as contact at a _point_.  (This is a very reasonable
 assumption for smooth convex shapes such as spheres and ellipses where relative
 motion must inevitably lead to initial contact at a single point.)
 -# Surface-to-surface contacts are still modeled as contact at a point, but
 there are no guarantees on that point exhibiting temporal coherence. In fact,
 we're guaranteed that there will _not_ be temporal coherence.
 <!--
 TODO(SeanCurtis-Tri): decide if this is relevant.
 -# As previously indicated, the contact also has a _normal_ -- ostensibly the
 surface normal at the point of contact.
 -->

 Next topic: @ref contact_model
 */

/** @defgroup contact_model Computing Contact Forces
 @ingroup drake_contacts

 Given @ref contact_spec "the definition of a contact", we can name the
 penetrating collision elements `A` and `B` and define the normal as pointing
 from `B` to `A`.  Furthermore, because the two forces are equal but opposite,
 we can discuss only the force acting on `A`.

 @image html simple_contact.png "Figure 1: Illustration of contact scenario between two spheres."

 The computation of the contact force is most cleanly discussed in the
 _contact frame_, `C` (shown in Figure 1).  We define the contact frame such
 that its z-axis is aligned with the contact normal `N_C` but place no
 constraints on the directions of its x- and y-axes.  The contact frame's origin
 is at the contact point `P_C`.

 The contact force, `f`,  can be decomposed into two components: normal, `fₙ`,
 and tangential, `fₜ`. The normal force lies in the normal direction and,
 therefore, in the direction of the contact frame's z-axis.  The tangential
 component lies on the contact frame's x-y plane.  In Drake's compliant contact
 model, although these components are orthogonal, they are _not_ independent
 (the tangential force is a function of the normal force.)

 @section normal_force Hunt-Crossley Normal Force

 Drake uses the Hunt-Crossley model for computing the normal force. We borrow
 the general description of this model from
 <a href="https://simtk.org/api_docs/simtkcore/api_docs20/classSimTK_1_1HuntCrossleyForce.html">
 SimTK</a> (with some formatting modifications):

> The normal component of the contact force, `fₙ`, is based on a model due to
> Hunt & Crossley: K. H. Hunt and F. R. E. Crossley, "Coefficient of Restitution
> Interpreted as Damping in Vibroimpact," ASME Journal of Applied Mechanics, pp.
> 440-445, June 1975. This is a continuous model based on Hertz elastic contact
> theory, which correctly reproduces the empirically observed dependence on
> velocity of coefficient of restitution, where e=(1-dẋ) for (small) impact
> velocity v and a material property `d` with units 1/ẋ. Note that `d` can be
> measured right off the coefficient of restitution-vs.-velocity curves: it is
> the absolute value of the slope at low velocities.

> Given a collision between two spheres, or a sphere and a plane, we can generate
> a contact force from this equation `fₙ = kxᵐ(1 + mdẋ)` where `k` is a stiffness
> constant incorporating material properties and geometry (to be defined below),
> `x` is penetration depth and ẋ is penetration rate (positive during
> penetration and negative during rebound). Exponent `m` depends on the surface
> geometry. For Hertz contact where the geometry can be approximated by sphere
> (or sphere-plane) interactions, which is all we are currently handling here,
> `m=3/2`.

 For Drake, we liberally extend the application of this model in a more
 generic way across arbitrary contact.  One implication is that the exponent,
 `m` becomes 1.  So, for Drake, the normal component of the contact force is:

   `fₙ = kx(1 + dẋ)`,

 where `k > 0` and `d > 0`. Please note, `d` is not a _damping_ factor, but as
 _dissipation_ factor.

 By definition `fₙ` should always be positive, so that the contact force is
 a repulsive force. Mathematically, for arbitrary x and ẋ, it is possible for
 `fₙ` to become negative, creating an attractive or "sucking" force. This case
 will be achieved if `ẋ < -1 / d`.  In this case, the normal component is
 clamped to zero.

 One of the implications of this, is that there will still be a repulsive force
 between two objects when they are drawing apart, as long as the relative
 velocities are small. This approximately models storing energy in the
 deformed material and then later restoring it.  However, the material can't
 restore arbitrarily fast, and if the bodies are pulled apart *faster* than
 the material can restore, the deformation will not have the chance to put the
 energy back into the system, and the dissipated energy will be lost with
 respect to the bodies' kinetic energy.

 @section tangent_force Stribeck Friction Tangential Force

 Static friction (or stiction) arises due to surface characteristics at the
 microscopic level (e.g., electrostatic and/or Van der Waals forces). Two
 objects in static contact need to have a force applied parallel to the surface
 of contact sufficient, `fₚ`, to _break_ stiction.  Once the objects are moving,
 dynamic friction takes over.  It is possible to accelerate one body sliding
 across another body with a force that would have been too small to break
 stiction.  In essence, stiction will create a contrary force canceling out
 any force too small to break stiction (see Figure 2).

 <!--
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
      Figure 2: Ideal Stiction
 -->
 @image html ideal_stiction.png "Figure 2: Ideal stiction"

 In _ideal_ stiction, the tangent force `fₜ` has magnitude equal to the pushing
 force `fₚ` up to the point where the force is sufficient to break stiction
 (the red dot in Figure 2).  At that point, the tangent force immediately
 becomes a constant force based on the _dynamic_ coefficient of friction. There
 are obvious discontinuities in this function which would wreak havoc with
 numerical integrators.  We use a Stribeck model to approximate this behavior
 in a way compatible with numerical integrators.

 <!--
   Stribeck function: u vs v_s

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

 The Stribeck model is a variation of Coulomb friction, where the frictional
 (aka _tangential_) force is proportional to the normal force as:

 `fₜ = μ⋅fₙ`

 In the Stribeck model, the coefficient of friction, μ, is replaced with a
 speed-dependent function:

 `fₜ = μ(v)⋅fₙ`,

 where `v` is a unitless multiple of a _new_ parameter: _slip tolerance_ (`vₛ`).
 Rather than modeling _perfect_ stiction, it makes use of an _allowable_ amount
 of relative motion to approximate stiction.  When we refer to
 "relative motion", we refer specifically to the relative motion of the two
 points in the corresponding bodies that are coincident with the contact point
 `P_C`, denoted as `rv_BcAc_C` for bodies `A` and `B`, respectively
 (and, in this case, measured and expressed in the contact frame, `C`).

 The function, as illustrated in Figure 3, is a function of the unitless
 _multiple_ of `vₛ`. The domain is divided into three intervals:

    - `v ∈ [0, 1)`: the coefficient of friction rises smoothly from zero to the
    static coefficient of friction, μs.
    - `v ∈ [1, 3)`: The coefficient of friction smooth falls from
    μs to the dynamic coefficient of friction, μd.
    - `v ∈ [3, ∞)`: Coefficient of friction is held constant at μd.

 Next topic: @ref contact_engineering
*/

/** @defgroup contact_engineering Working with Contacts in Drake
 @ingroup drake_contacts

 The behavior of a simulation with contact will depend on three factors:

 - the choice of integrator,
 - contact parameters,
 - nature of collision geometry.

 The three factors are inter-dependent; specific choices for one factor may
 require supporting changes in the other factors.

 Issues:
 - **Picking a good value for `vₛ`**

   In selecting a value for `vₛ`, you must ask yourself the question, "When
   two objects are ostensibly in stiction, how much slip am I willing to allow?"
   There are two opposing design issues in picking a value for `vₛ`.  On the one
   hand, small values of `vₛ` require integrators with _very_ small time steps
   (or high accuracy for the error-controlled RK3).  On the other hand, it
   should be picked to be appropriate for the scale of the problem. For example,
   a car simulation could allow a "large" value for `vₛ` of 1e-2 m/s, but
   grasping a 10 cm box might have to be at 1e-3 or 1e-4 m/s. Ultimately,
   picking the largest viable value will allow your simulator to run faster.

 - **Picking values for the contact parameters**

   The contact model provides five parameters:
     - Stiffness (`k`)
     - Dissipation (`d`)
     - static coefficient of friction (`μs`)
     - dynamic coefficient of friction (`μd`)
     - stiction slip tolerance (`vₛ`)
     .
     Empirical evidence suggests that the most important properties to tune will
     be `k` and `vₛ`.  Modifying `d` can have subtle influence over softening
     the behavior, but doesn't obviously contribute to improved stiction.
     Increasing the coefficients of friction to unrealistic levels seems to
     counterintuitively degrade the results. The previous note discusses the
     importance of `vₛ`.

     Tuning stiffness _does_ have value.  In a compliant model, penetration is
     part of the stable equilibrium state. Imagine a box sitting on a half plane.
     The stable penetration depth will, in principle, be equal to the box's mass
     divided by the stiffness.  Appropriate stiffness for a 1 kg box is not the
     same as for a 1000 kg car.  (In fact, with a small stiffness, the car will
     pass right through the ground while attempting to find the equilibrium
     distance).

 - **Global contact parameters**

   In its current incarnation, Drake does _not_ support per-object mechanical
   material properties.  That means whatever parameter values you select will
   be the same for all contacts in the world.

 - **Surface-on-surface contacts**

   Remember that the contact detection work produces a single, point to
   represent contact between two collision elements.  If the contact is a
   surface instead of a point (such as one box lying on another), the contact
   point will *not* be temporally coherent.  This will lead to instability
   artifacts which can only be addressed through smaller time steps.

   An alternative is to represent the body's contact differently. For some
   shapes (e.g., boxes), we can introduce two sets of collision elements:
   discrete "points" at the corners, and a box capturing the volume (see
   block_for_pick_and_place.urdf as an example). With this strategy, the
   contact "points" are actually small-radius spheres.  The volume-capturing
   box should actually be inset from those spheres such that when the box is
   lying on a plane (such that the logical contact manifold would be a face),
   only the contact points make contact, providing reliable points of contact.
   However, for arbitrary configurations contact with the box will provide
   more general contact.

 - **Contact samples**

   Because of weaknesses in the contact detection and characterization, we get
   bet numerical performance in explicitly enumerating contact points on a body
   (see previous issue).  However, each of those contact points are processed
   without any knowledge of the others or the relationship.  Thus, for a fixed
   penetration depth between two objects, increasing the number of explicit
   contact spheres will increase the magnitude of the _overall_ contact force.

 - **Choice of integrator**

   Empirical evidence suggests that any integrator _except_ ExplicitEulerIntegrator
   can work with the new contact model.  Generally, the RungeKutta2Integrator
   and SemiExplicitEulerIntegrator require similar time steps to produce
   equivalent behavior.  Generally, for a `vₛ` value of 1e-2 m/s, a timestep on
   the order of 1e-4 is required for both of these.

   Similarly, the error-controlled integrator, RungeKutta3Integrator, requires
 */
