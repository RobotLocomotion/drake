/** @defgroup drake_contacts   Compliant Contact in Drake
    @ingroup collision_concepts

 Drake is concerned with the simulation of _physical_ phenomena, including
 contact between simulated objects.
 Drake approximates real-world physical contact phenomena with a combination
 of geometric techniques and response models. Here we discuss the
 parameterization and idiosyncracies of a particular contact response model,
 based on point contact with compliance and dissipation, and a Stribeck friction
 model approximating Coulomb stiction and sliding friction effects.

 <!-- TODO(SeanCurtis-TRI): Update this as contact models update. -->
 This document gives an overview of the state of the implementation of compliant
 contact in Drake (as of Q4 2017) with particular emphasis on how to account for
 its particular quirks in a well-principled manner. What works in one simulation
 scenario, may not work equally well in another scenario. This discussion will
 encompass:

 - @ref contact_geometry "properties of the geometric contact techniques",
 - @ref contact_engineering "techniques for teasing out desirable behavior",
 - @ref contact_model "details of the contact response model", and
 - @ref drake_per_object_material "per-object contact materials".

 @anchor contact_spec
 <h2>Definition of contact</h2>

 Before getting into the details of how contacts are detected and responses are
 modeled, it is worthwhile to define what a contact means to Drake.

 First, contact _conceptually_ occurs between two _surfaces_, one on each of
 two independently moving _bodies_. The contact produces _forces_ on those two
 surfaces, which can then affect the motion of the bodies. In practice, surfaces
 are represented by one or more drake::multibody::collision::Element objects --
 geometric shapes rigidly affixed to a body, whose surfaces can engage in
 contact.

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

 - describes a relationship between two drake::multibody::collision::Element
   instances, denoted elements `A` and `B`,
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
 why these techniques work the way they do, but, instead, focus on _what_ the
 properties of the results of the _current implementation_ are.  It is worth
 noting that some of these properties are considered _problems_ yet to be
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
 component lies parallel to the contact frame's x-y plane.  In Drake's compliant
 contact model, the tangential force is a function of the normal force.

 The detailed discussion of the contact force computation is decomposed into
 two parts: a high-level engineering discussion addressed to end users who care
 most about working with the model and a further detailed discussion of the
 mathematical underpinnings of the implemented model. The practical guide should
 be sufficient for most users.

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

 @anchor contact_parameters
 <h2>Contact Parameters</h2>

 @anchor contact_parameter_lists
 <h3>Parameters</h3>

 The determination of contact forces is a combination of per-object contact
 _material_ parameters and global model parameters. See
 @ref contact_model_background for elaboration on the parameters.

 __Per-object Contact Material Parameters__

 - Young's modulus, `E`, (with units of pascals, i.e., N/m²) is the tensile
   elasticity of the material. It may also be referred to generally as
   "elasticity" or colloquially as "stiffness".
    - Generally, these values can be looked up in a table. Typical values are
      quite large ranging from soft rubber at 1×10⁷ N/m² to diamond at
      1.2×10¹² N/m².
    - The default value is that of medium rubber: 1×10⁸ N/m².
 - dissipation, `d`, (with units of 1/velocity) is a material property that
   correctly reproduces the empirically observed velocity dependence of the
   coefficient of restitution, where `e = (1-d⋅v)` for small impact velocity
   `v`.
     - In theory, at least, `d` can be measured right off the coefficient of
       restitution-vs.-impact velocity curves; it is the negated slope at low
       impact velocities. In practice, the curve is difficult to produce and
       very little data exists to provide physical values.
     - [Hunt 1975] reports values of `d` between 0.08-0.32 sec/m for steel,
       bronze or ivory. From this, we advocate relatively small values.
     - The default value is 0.32 sec/m.
 - coefficient of static friction, `μ_s`, (unitless) is the standard Coulomb
   coefficient of friction for a body with zero relative motion.
     - The coefficient of static friction must be greater than or equal to the
       coefficient of dynamic friction.
     - The default value 0.9.
 - coefficient of dynamic friction, `μ_d`, (unitless) is the standard Coulomb
   coefficient of friction for a body with non-zero relative motion.
     - The coefficient of dynamic friction must be less than or equal to the
       coefficient of static friction.
     - The default value is 0.5.

 __Global Model Parameters__

 - slip velocity, `vₛ`, (with units of m/s) determines the behavior of contact
   bodies during _stiction_. Essentially, stiction is approximated. When bodies
   should be in stiction, the model allows the contacting points a relative
   slip velocity _up to_ this value. See @ref tangent_force for details.
     - The default value is 0.01 m/s.
 - characteristic area, `A`, (with units of m²) defines a characteristic contact
   patch scale. See @ref drake_contact_implementation for details.
     - The default value is 2 cm² (e.g., 2×10⁻⁴ m²).

 @anchor contact_parameter_choices
 <h3>Issues with Parameter Values</h3>

  - **Contact Material Parameters**

    With the exception of the dissipation parameter, the values for material
    parameters should be predicated on realistic values. The majority of these
    values can and should be looked up in tables. The goal is for these
    physical quantities to remain meaningful and constant while the underlying
    contact model implementation improves and makes better use of them.

    As for the dissipation parameter, we generally recommend leaving it at the
    default value unless there's particular insight and principle in changing
    it.

  - **Picking a good value for `vₛ`**

    In selecting a value for `vₛ`, you must ask yourself the question, "When
    two objects are ostensibly in stiction, how much slip am I willing to
    allow?" There are two opposing design issues in picking a value for `vₛ`.
    On the  one hand, small values of `vₛ` make the problem numerically stiff
    during stiction, potentially increasing the integration cost.
    On the other hand, it should be picked to be appropriate for the scale of
    the problem. For example, a car simulation could allow a "large" value for
    `vₛ` of 1 cm/s (1×10⁻² m/s), but reasonable stiction for grasping a 10 cm
    box might require limiting residual slip to 1×10⁻³ m/s or less. Ultimately,
    picking the largest viable value will allow your simulation to run faster.

  - **Picking a good value for `A`**

    One of the quirks of the implemented contact model, is that is largely
    unaware of the size of the contact surface between bodies. However, the
    contact force _depends_ on that area. This _global_ setting allows you to
    hint to the model the size of the contact patches; larger contact patches
    produce larger contact forces. For example, a 1 cm rubber ball on a table
    has a much smaller contact patch than the tires of a car would have on the
    road. Even if we assume the ball and tires are made of the same rubber, the
    forces in play are quite different.
    Rather than increasing the Young's modulus for the car's tires (as compared
    to the rubber ball), we recommend increasing the characteristic area in the
    contact model.

 @anchor integrator_choice
 <h2>Choice of Integrator</h2>

   Empirical evidence suggests that any integrator _except_
   @ref drake::systems::ExplicitEulerIntegrator "ExplicitEulerIntegrator" can
   work with this contact model. Generally, the
   @ref drake::systems::RungeKutta2Integrator "RungeKutta2Integrator" and
   @ref drake::systems::SemiExplicitEulerIntegrator "SemiExplicitEulerIntegrator"
   require similar time steps to produce equivalent behavior. Generally, for a
   `vₛ` value of 1e-2 m/s, a timestep on the order of 1e-4 is required for both
   of these. The error-controlled
   @ref drake::systems::RungeKutta3Integrator "RungeKutta3Integrator" will
   choose very small steps and accuracy must be set tight enough to ensure
   stability.
   There is also a first-order, fully-implicit integrator:
   @ref drake::systems::ImplicitEulerIntegrator "ImplicitEulerIntegrator" that
   can be considered for handling stiff systems.

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
   disparate scales. A single, global characteristic area may be insufficient.
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
 since this equation _is_ the definition of the average contact pressure `p(q)`.
 Thus, determining the contact normal force consists of determining the geometry
 of the contact patch, and the average pressure on that patch.

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
 penetrating are x, then the resultant normal force (according to the Hertz
 contact model) would be:

   `fₙ = ⁴/₃⋅E⋅√R⋅√x³`.

 The two spheres compress such that they are touching along a disk with radius
 `√(R⋅x)`.  Based on this, we can equate it to the earlier function:

   - `fₙ = p(q)·A(q) = ⁴/₃⋅E⋅√R⋅√x³`
   - `p(q) = 4/(3π)·E·√(x/R)`
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

 Next topic: @ref drake_contact_implementation
*/

/** @defgroup drake_contact_implementation Drake Contact Implementation
 @ingroup drake_contacts

 Drake's compliant point contact model is a coarse approximation of the
 previous discussion. This section outlines the simplifications. With time, the
 contact model will grow more and offer more sophisticated and _accurate_
 models.

 @anchor drake_collision_detection
 <h2>Collision Detection and Characterization</h2>

 When the collision geometry of two bodies penetrate, the penetration is
 characterized by:

   - a pair of points: the points on each geometry that lies most deeply in the
     other geometry,
   - a vector: the contact normal direction, and
   - the penetration depth.

 The volume and domain of the penetration is conspicuously absent. As such, it
 is impossible for a contact model to calculate a contact force based on the
 specific details of that unreported volume.

 Future versions of Drake will support additional characterizations of geometry
 penetration in support of volume-based contact force calculation.

 @anchor drake_contact_model_impl
 <h2>Contact Force Computation</h2>

 Given the characterization of penetration outlined above, a full implementation
 of the contact normal force would be impossible. Instead, Drake employs a
 corruption of the Hertz model. The best analogy would be to think of
 the contact as between a vertical cylinder and a plane: `fₙ = 2⋅E⋅R⋅x`. In
 other words, it assumes that the contact area is independent of the depth and
 that pressure is proportional to the penetration depth. The radius R is a
 tunable "global" parameter of the
 @ref drake::systems::CompliantContactModel "CompliantContactModel".
 (See @ref contact_engineering on how to work with this property.)

 Based on this Hertzian value, the full Hunt-Crossley normal force is computed
 as defined above. Furthermore, the tangential component of the contact force
 is also computed as outlined above.

 @anchor drake_per_object_material
 <h2>Per-object Contact Material</h2>

 Drake supports defining compliant contact materials on a per-collision geometry
 basis. It has several mechanisms in place to facilitate working with
 per-collision object contact materials:

 - Universal default values (all objects default to the universal values if none
   have been explicitly specified).
 - Parsing per-collision element material parameters from URDF and SDF files
   using extended tags (formatted identically for both source files types).
 - Runtime query which admits the ability to provide a custom default value.

 @anchor mat_parameters_and_contact
 <h3>Material parameters and evaluating contact</h3>

 The per-object material parameters consist of:

 - Elastic modulus (aka stiffness) (E) with units of pascals,
 - dissipation (d) with units of 1/velocity, and
 - static and dynamic friction (unitless μ_s and μ_d, respectively).

 The parameters outlined in @ref contact_model_background are derived from the
 material values for the two colliding bodies. Consider two colliding bodies M
 and N. The contact values E, d, μ_s, and μ_d used to compute the contact force
 are defined in the following way:

 - sₘ ∈ [0, 1] is the "squish" factor of body M. It represents the amount of
   total deformation experienced by body M. Consider contact between a steel
   body and foam ball; the foam ball would experience the entire deformation and
   the squish factors for the foam ball and steel plate would be 1 and 0,
   respectively. The squish value is defined as sₘ = kₙ / (kₘ + kₙ), with
   sₙ = 1 - sₘ.
 - E = sₘEₘ = sₙEₙ. The effective Young's modulus of the _contact_ will
   generally not be the Young's modulus of either constituent material (unless
   one were infinite). If Eₘ = Eₙ, then E would be Eₘ/2.
 - d = sₘdₘ + sₙdₙ. Again, the dissipation of the contact is simply a linear
   interpolation of the two bodies' dissipation values.
 - μ_s (and μ_d) are defined as 2μₘμₙ / (μₘ + μₙ).

 Finally, the contact point is also defined with respect to the "squish"
 factors. For penetrating bodies M and N, there is a point on the surface of M
 that _most_ deeply penetrates into N (and vice versa). We will call those
 points `Mc` and `Nc`, respectively. The contact point, is
 `C = Mc * sₙ + Nc * sₘ`.
 We draw _particular_ attention to the fact that the point on M's surface is
 weighted by N's squish factor and vice versa. That is because, if body M
 experiences all of the deformation, it will be deformed all the way to the
 point of deepest penetration _in_ M, which was the definition of `Nc`.

 @anchor material_defaults
 <h3>Contact material default values</h3>

 Every collision element has a compliant material. If the values of that
 material have not been explicitly set (i.e., via calls to the API or specified
 in a URDF/SDF file), the the value is configured to use a "default" value.
 What the actual default value is depends on how the material property is
 accessed (see the documentation for
 @ref drake::systems::CompliantMaterial "CompliantMaterial" for further
 elaboration).

 Consider a box used as collision geometry with all of its compliant material
 parameters set to default. Consider querying for the box's dissipation. The
 value returned could be different values based on invocation:

 1. Querying directly (e.g., `box.compliant_material().dissipation()`) will
    return the hard-coded, Drake-wide default value.
 2. Alternatively, when the
    @ref drake::systems::CompliantContactModel "CompliantContactModel"
    used by a @ref drake::systems::RigidBodyPlant "RigidBodyPlant"
    evaluates it, the default value will be the
    @ref drake::systems::CompliantContactModel "CompliantContactModel's" default
    material dissipation value. Which _may_ be different from the hard-coded
    globals or from any other instance of
    @ref drake::systems::CompliantContactModel "CompliantContactModel".
 3. Alternatively, user-code could provide a preferred default which will be
    returned iff the property is default configured
    (e.g., `box.compliant_material().dissipation(0.125)`)

 The point that needs to be emphasized is that configuring a material property
 to be default is not a one-time operation. It defines a relationship that
 can be determined at evaluation time.

 __A word of warning__

 It might be tempting to write code akin to this pseudo-code:

 ```c
 CompliantMaterial mat;
 RigidBodyPlant plant;
 mat.set_dissipation(0.1);
 plant.set_default_contact_material(mat);
 ParseUrdf(plant, "my_robot.urdf");
 mat.set_(5e6);
 mat.set_dissipation(0.2);
 plant.set_default_contact_material(mat);
 ParseUrdf(plant, "other_robot.urdf");
 ```

 At first glance, one might be inclined to believe that the any collision
 elements without specified contact materials in the file `my_robot.urdf` would
 have dissipation value of 0.1, whereas those in the file `other_robot.urdf`
 would have dissipation of 0.2. This is _not_ the case. The collision elements
 of both robots are configured to use the default value. And they will report a
 dissipation of 5e6 if the `plant` evaluates it, or some other value in other
 contexts.

 @anchor material_urdf_sdf
 <h3>Specifying contact parameter values in URDF/SDF.</h3>

 We are exploiting the fact that URDF and SDF are XML files and choose to
 naively extend the specification to include a custom tag. Although there are
 numerous differences between the two formats, there is fortunate similarity
 in declaring collision geometries. For simplicity's sake, we expect identically
 formatted contact material format in both formats that look something like
 this:

 ```xml
 ...
 <collision ...>
   <geometry ...>
   </geometry>

   <drake_compliance>
     <youngs_modulus>##</youngs_modulus>
     <dissipation>##</dissipation>
     <static_friction>##</static_friction>
     <dynamic_friction>##</dynamic_friction>
   </drake_compliance>

 </collision>
 ...
 ```

 Differences between URDF and SDF are dismissed with ellipses. What is
 significant is that the `<drake_compliance>` tag should be introduced as a
 child of the `<collision>` tag (common to both formats) and should be
 formatted as shown.

 The following rules are applied for parsing:

 - If no `<drake_compliance>` tag is found, the element uses the global default
   parameters.
 - Not all parameters are required; explicitly specified parameters will be
   applied to the corresponding element and omitted parameters will map to the
   default values.
 - Friction values must be defined as a pair, or not at all. When defined as a
   pair, the `static_friction` value must be greater than or equal to the
   `dynamic_friction` value. Failure to meet these requirements will cause a
   runtime exception.

 */
