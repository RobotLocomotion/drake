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

 @image html multibody/rigid_body_plant/images/simple_contact.png "Figure 1: Illustration of contact scenario between two spheres."

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
> velocity of coefficient of restitution, where e=(1-cẋ) for (small) impact
> velocity v and a material property c with units 1/ẋ. Note that c can be
> measured right off the coefficient of restitution-vs.-velocity curves: it is
> the absolute value of the slope at low velocities.

> Given a collision between two spheres, or a sphere and a plane, we can generate
> a contact force from this equation `fₙ = kxᵐ(1 + mcẋ)` where `k` is a stiffness
> constant incorporating material properties and geometry (to be defined below),
> `x` is penetration depth and ẋ is penetration rate (positive during
> penetration and negative during rebound). Exponent `m` depends on the surface
> geometry. For Hertz contact where the geometry can be approximated by sphere
> (or sphere-plane) interactions, which is all we are currently handling here,
> `m=3/2`.

 For Drake, we don't liberally extend the application of this model in a more
 generic way across arbitrary contact.  One implication is that the exponent,
 `m` becomes 1.  So, for Drake, the normal component of the contact force is:

   `fₙ = kx(1 + cẋ)`,

 where `k > 0` and `c > 0`.

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
 of contact sufficient to _break_ stiction.  Once the objects are moving,
 dynamic friction takes over.  It is possible to accelerate one body sliding
 across another body with a force that would have been too small to break
 stiction.  In essence, stiction will create a contrary force canceling out
 any force too small to break stiction; the size of the stiction force depends
 on the size of the parallel pushing force.

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
 -->
 @image html multibody/rigid_body_plant/images/ideal_stiction.png "Figure 2: Ideal stiction"

 In _ideal_ stiction, the tangent force `fₜ` has magnitude equal to the pushing
 force `fₚ` up to the point where the force is sufficient to break stiction
 (the red dot in Figure 2).  At that point, the tangent force immediately
 becomes a constant force based on the _dynamic_ coefficient of friction. There
 are obvious discontinuities in this function which would wreak havoc with
 numerical integrators.  We use a Stribeck model to approximate this behavior
 in a way compatible with numerical integrators.


 @image html multibody/rigid_body_plant/images/stribeck.png "Figure 3: Stribeck function for stiction"
*/

/** @defgroup contact_engineering Working with Contacts in Drake
 @ingroup drake_contacts

 Issues:
 - global contact parameters
 - point contact only fights with surface-to-surface contact.
 */