/** @file
 Doxygen-only documentation for @ref drake_contacts.  */

// clang-format off (to preserve link to images)

/** @addtogroup drake_contacts

@section solid_mechanics Mechanics of Solids

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


@section contact_mechanics Contact Mechanics

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

@section rigid_approximation Rigid Approximation of Contact

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
paradox @ref PfeifferGlocker1996 "[Pfeiffer & Glocker, 1996]", a one degree of
freedom system with only one contact that, depending on the state, has an
infinite number of solutions or even no solution.

It should be noted that these artifacts are a consequence of the mathematical
approximations adopted and not a flaw of the original continuum mechanics theory
and even much less, a flaw in the physics itself.

@section why_rigid Why so Rigid? Embracing Compliance

The rigid modeling of contact coupled with Coulomb friction is plagued with a
wide variety of technical issues:
 - Even without friction, the system can be overconstrained and the contact
   forces are indeterminate.
 - As described above, rigid contact coupled with Coulomb friction might lead to
   the Painlevé paradox, where the system has no solution or infinite solutions.
 - Simulating these systems requires the formulation of Non-Linear
   Complementarity problems (NCPs).  NCPs are believed to be NP-hard
   @ref Kaufman2008 "[Kaufman et al., 2008]" (i.e. very difficult to deal with
   in practice) and are often very ill conditioned numerically, leading to
   brittle solutions in practical software.

The literature on this topic is very rich, and there has been a great deal of
effort to solve many of these problems. Unfortunately, most solutions involve
the introduction of heuristics or tuning parameters hidden within the solvers
that introduce yet new artifacts or trading off robustness/speed for physical
accuracy.

As an example, *constraint softening* is introduced "...to control the
sponginess and springiness..." (quoting from the @ref ODEUserGuide
"[ODE User Guide]"). This strategy is used to improve the numerics of the
solver, which no longer approximates a true rigid system, but a compliant one.
Similarly, Interior Point (IP) methods for general NCPs or more recently
Incremental Potential Contact (IPC) in @ref Li2020 "[Li et al., 2020]",
introduce barrier functions to handle constraints. These barriers can only be
solved to a finite accuracy, at which the solution is essentially compliant
(even if stiff). This is discussed in more detail in
@ref Castro2023 "[Castro et al., 2023]".

@subsection compliance_in_robotics Compliance in Robotics

Many robots are actually compliant in the real hardware --- robotic grippers and
fingers are padded to improve grasping efficiency, and robot feet as those
commonly found in quadrupeds and humanoid robots have rubber surfaces.
Therefore, compliance *must* be modeled if our goal is physical accuracy to
close the sim2real gap.

@subsection embracing_compliance Embracing Compliance

The discussion above can be probably summarized as follows:
 1. The rigid approximation of contact leads to fundamental problems in the
    mathematics and ill-conditioned numerics in software.
 2. Real physical systems are compliant, even if stiff.
 3. Numerical techniques to deal with ill conditioning essentially boil down to
    introducing compliance at some level deep within the solver.

It is for these reasons that in Drake we embrace compliance, to provide accurate
modeling of the physics of contact but also as a means to mitigate numerical
problems associated with rigid approximations of contact. Within this framework,
<em>rigid contact</em> can be approximated with very stiff compliance. As shown
in @ref Castro2023 "[Castro et al., 2023]" this technique is very effective even
at very high stiffness values, beyond what it is really needed to model hard
materials such as steel. In this way the modeling of contact is transparent to
users, with physical parameters to control compliance and no need for hidden
parameters in our solvers.

Read more about Drake's modeling of compliant contact in the next section, @ref
compliant_contact.

<b>Next topic:</b> @ref compliant_contact

@section contact_model_references References

- @anchor Anitescu2006 [Anitescu, 2006] Anitescu, M., 2006. Optimization-based
  simulation of nonsmooth rigid multibody dynamics. Mathematical Programming,
  105, pp.113-143.
- @anchor Castro2019 [Castro et al., 2019] Castro, A.M., Qu, A., Kuppuswamy, N.,
  Alspach, A. and Sherman, M., 2020. A transition-aware method for the
  simulation of compliant contact with regularized friction. IEEE Robotics and
  Automation Letters, 5(2), pp.1859-1866.
- @anchor Castro2022 [Castro et al., 2022] Castro, A.M., Permenter, F.N. and
  Han, X., 2022. An unconstrained convex formulation of compliant contact. IEEE
  Transactions on Robotics, 39(2), pp.1301-1320.
- @anchor Castro2023 [Castro et al., 2023] Castro, A., Han, X. and Masterjohn,
  J., 2023. A Theory of Irrotational Contact Fields. arXiv preprint
  https://arxiv.org/abs/2312.03908
- @anchor PfeifferGlocker1996 [Pfeiffer & Glocker, 1996]
  Pfeiffer, F., Glocker, C. (1996). Multibody Dynamics with Unilateral
  Contacts. Germany: Wiley.  
- @anchor HuntCrossley1975 [Hunt and Crossley 1975] Hunt, K.H. and Crossley,
  F.R.E., 1975. Coefficient of restitution interpreted as damping in
  vibroimpact. Journal of Applied Mechanics, vol. 42, pp. 440–445.
- @anchor Kaufman2008 [Kaufman et al., 2008] Kaufman, D.M., Sueda, S., James,
  D.L. and Pai, D.K., 2008. Staggered projections for frictional contact in
  multibody systems. In ACM SIGGRAPH Asia 2008 papers (pp. 1-11).
- @anchor Li2020 [Li et al., 2020] Li, M., Ferguson, Z., Schneider, T.,
  Langlois, T.R., Zorin, D., Panozzo, D., Jiang, C. and Kaufman, D.M., 2020.
  Incremental potential contact: intersection-and inversion-free,
  large-deformation dynamics. ACM Trans. Graph., 39(4), p.49.
- @anchor ODEUserGuide [ODE User Guide] Smith, R., 2005. Open dynamics engine.
  https://ode.org/ode-latest-userguide.pdf
- @anchor Todorov2014 [Todorov, 2014] Todorov, E., 2014, May. Convex and
  analytically-invertible dynamics with contacts and constraints: Theory and
  implementation in mujoco. In 2014 IEEE International Conference on Robotics
  and Automation (ICRA) (pp. 6054-6061). IEEE.
*/

/** @addtogroup compliant_contact Modeling Compliant Contact

@section compliant_point_contact Compliant Point Contact

The point contact model defines the contact force by determining the minimum
translational displacement (MTD). The minimum translational displacement is the
smallest relative displacement between two volumes that will take them from
intersecting to just touching. This quantity need not be unique (if two spheres
have coincident centers, any direction will serve). Once we have this
displacement, we get three quantities that we use to define the contact force:
  - The direction of the displacement vector is the contact normal.
  - The magnitude is a measure of the amount of penetration (and is correlated
    with the magnitude of the normal component of the contact force).
  - Two witness points for the MTD. The witness points comprise one point on
    the surface of each volume such that when we apply the minimum
    translational displacement, they are coincident. We use the witness points
    (and other physical parameters) to define the point at which the contact
    force is applied.

This model is simple to implement and cheap to compute, but has some drawbacks.

- A single measure of “maximum penetration” cannot distinguish between a large
  intersecting volume and a small intersecting volume (see Figure 1). The two
  intersections would produce the same amount of contact force despite the fact
  that one is clearly compressing more material.

- Contact along a large interface is treated as contact at a single point (see
  Figure 2). Effects that depend on a contact interface over a domain with
  non-zero area disappear (e.g., torsional friction).

- The witness points are not necessarily unique. This means,
  generally, there is no guarantee that the witness points will be consistent
  from frame to frame, which means that the point at which the force is applied
  will not be consistent. This can produce non-physical artifacts like sudden
  changes in force direction.

@image html "multibody/hydroelastics/images/contact-fig-01.png"

Figure 1: Two intersections with significantly different intersecting volumes
characterized with the same measure: d.

@image html "multibody/hydroelastics/images/contact-fig-02.png"

Figure 2: Modeling contact forces with point contact (considering the blue half
space as rigid). (a) the actual intersection of the simulated bodies. (b) the
conceptual deformation of the orange body creating a large area of contact. (c)
how point contact sees the deformation: contact at a single point.

@subsection point_forces_modeling Modeling of Forces

Given the "maximum penetration" x, we compute the normal component of the
contact force according to a simple compliant law of the form: <pre>
   fₙ = k(x)₊(1 + dẋ)₊
</pre>
with `(a)₊ = max(0, a)`. The normal contact force `fₙ` is made a continuous
function of the penetration distance x between the bodies (defined to be
positive when the bodies are in contact) and the penetration distance rate ẋ
(with ẋ > 0 meaning the penetration distance is increasing and therefore the
interpenetration between the bodies is also increasing). Stiffness `k` and
dissipation `d` are the combined "effective" stiffness and dissipation for the
pair of contacting geometries. Dissipation is modeled using a Hunt & Crossley
model of dissipation, see @ref mbp_dissipation_model for details.  For
flexibility of parameterization, stiffness and dissipation are set on a
per-geometry basis
(@ref accessing_contact_properties "Accessing point contact parameters"). Given
two geometries with individual stiffness and dissipation parameters (k₁, d₁) and
(k₂, d₂), we define the rule for combined stiffness (k) and dissipation (d) as:
<pre>
  k = (k₁⋅k₂)/(k₁+k₂)
  d = (k₂/(k₁+k₂))⋅d₁ + (k₁/(k₁+k₂))⋅d₂
</pre>
These parameters are optional for each geometry. For any geometry not assigned
these parameters by a user Pre-Finalize, %MultibodyPlant will assign default
values such that the combined parameters of two geometries with default values
match those estimated using the user-supplied "penetration allowance", see
@ref point_contact_defaults.

@note When modeling stiff materials such as steel or ceramics, these model
parameters often need to be tuned as a trade-off between numerical stiffness and
physical accuracy. Stiffer materials lead to a harder to solve system of
equations, affecting the overall performance of the simulation. The convex
approximations provided in Drake are very robust even at high stiffness values,
please refer to @ref Castro2023 "[Castro et al., 2023]" for a study on the
effect of stiffness on solver performance.

@subsubsection point_contact_defaults Default Parameters

@note The treatment of default parameters is undergoing a major revision. Please
refer to the documentation for
@ref drake::geometry::DefaultProximityProperties "DefaultProximityProperties"
and for AddMultibodyPlant(). For now, we still support the "penetration
allowance" workflow outlined below, but that might change.

While we strongly recommend setting these parameters appropriately for your
model,
@ref drake::multibody::MultibodyPlant "MultibodyPlant" aids the estimation of
these coefficients using a heuristic function based on a user-supplied
"penetration allowance", see
@ref drake::multibody::MultibodyPlant::set_penetration_allowance() "set_penetration_allowance()".
 This heuristics offers a good starting point when creating a simulation for the
first time. Users can then set material properties for specific geometries once
they observe the results of a first simulation with these defaults. The
penetration allowance is a number in meters that specifies the order of
magnitude of the average penetration between bodies in the system that the user
is willing to accept as reasonable for the problem being solved. For instance,
in the robotic manipulation of ordinary daily objects the user might set this
number to 1 millimeter. However, the user might want to increase it for the
simulation of heavy walking robots for which an allowance of 1 millimeter would
result in a very stiff system.

As for the dissipation coefficient in the simple law above,
@ref drake::multibody::MultibodyPlant "MultibodyPlant" chooses the dissipation
coefficient d to model inelastic collisions and therefore sets it so that the
penetration distance x behaves as a critically damped oscillator. That is, at
the limit of ideal rigid contact (very high stiffness k or equivalently the
penetration allowance goes to zero), this method behaves as a unilateral
constraint on the penetration distance, which models a perfect inelastic
collision. For most applications, such as manipulation and walking, this is the
desired behavior.

When
@ref drake::multibody::MultibodyPlant::set_penetration_allowance() "set_penetration_allowance()"
 is called,
@ref drake::multibody::MultibodyPlant "MultibodyPlant" will estimate reasonable
stiffness and dissipation coefficients as a function of the input penetration
allowance. Users will want to run their simulation a number of times to
determine if they are satisfied with the level of inter-penetration actually
observed in the simulation; if the observed penetration is too large, the user
will want to set a smaller penetration allowance. If the system is too stiff and
the time integration requires very small time steps while at the same time the
user can afford larger inter-penetrations, the user will want to increase the
penetration allowance. Typically, the observed penetration will be proportional
to the penetration allowance. Thus scaling the penetration allowance by say a
factor of 0.5, would typically results in inter-penetrations being reduced by
the same factor of 0.5. In summary, users should choose the largest penetration
allowance that results in inter-penetration levels that are acceptable for the
particular application (even when in theory this penetration should be zero for
perfectly rigid bodies.)

For a given penetration allowance, the contact interaction that takes two bodies
with a non-zero approaching velocity to zero approaching velocity, takes place
in a finite amount of time (for ideal rigid contact this time is zero.) A good
estimate of this time period is given by a call to
@ref drake::multibody::MultibodyPlant::get_contact_penalty_method_time_scale() "get_contact_penalty_method_time_scale()".
 Users might want to query this value to
either set the maximum time step in error-controlled time integration or to set
the time step for fixed time step integration. As a guidance, typical fixed time
step integrators will become unstable for time steps larger than about a tenth
of this time scale.

@subsection crafting_collision_geometry Choosing the Right Collision Geometry

The compliant point contact model only reports a single contact between bodies.
More particularly, the contact is characterized by a single point. In some cases
(e.g., a steel ball on a table surface), that is perfectly fine. For objects
that contact across a large surface area (such as a box on the table), this
characterization has two negative implications:
 - the contact point will not be guaranteed to be at the center of pressure,
   possibly inducing unrealistic torque, and
 - not be temporally coherent. This will lead to instability artifacts which
   can only be addressed through smaller time steps.

Both of these issues can be addressed by changing the geometry that
represent the body's contact surface. For some shapes (e.g., boxes), we can
introduce two sets of collision elements: discrete "points" at the corners,
and a box capturing the volume (see
<a href="https://github.com/RobotLocomotion/drake/blob/master/examples/kuka_iiwa_arm/models/objects/block_for_pick_and_place.urdf">
block_for_pick_and_place.urdf</a> and
<a href="https://github.com/RobotLocomotion/drake/blob/master/examples/multibody/inclined_plane_with_body/inclined_plane_with_body.cc">
inclined_plane_with_body.cc</a> in Drake's examples).
With this strategy, the contact "points" are actually small-radius spheres. The
volume-capturing box should actually be inset from those spheres such that when
the box is lying on a plane (such that the logical contact manifold would be a
face), only the contact points make contact, providing reliable points of
contact. However, for arbitrary configurations contact with the box will provide
more general contact.

The Hydroelastic contact model described below in @ref hydro_contact
"Hydroelastic Contact", was designed to resolve these issues in a principled
manner. We recommend switching to this model unless other requirements such as
simulation speed are a constraint.

@section hydro_contact Hydroelastic Contact

The purpose of this documentation is to provide a quick overview of the
Hydroelastic contact model and its modeling parameters. More details are provided
in Drake's
@ref hydroelastic_user_guide "Hydroelastic Contact User Guide" and references
therein.

Hydroelastic contact was created to address some of the short-comings in point
contact. In fact, one might argue that many of the strategies used to mitigate
the shortcomings of point contact (such as using lots of points) push it closer
and closer to hydroelastic contact.

Hydroelastic Contact was originally introduced by
@ref Elandt2019 "[Elandt et al., 2019]". Modifications and further development
of the model can be found in @ref Masterjohn2022 "[Masterjohn et al., 2022]". In
Drake, we refer to this model as the “hydroelastic” model. It differs from point
contact in how it characterizes the contact. Rather than a single point, it
imagines an entire contact surface. This surface is an approximation of the
contact surface as visualized in Figure 2(b).

When two objects come into contact, forces are produced due to deformation
("strain") of the objects at the contact interface. At first touch, there are no
forces but as the objects are pressed further they deform to produce a region
over which contact pressure is non-zero, causing equal-and-opposite forces to
act on the objects. Calculating the actual deformations is expensive.
Hydroelastic contact is based on the idea that for relatively small deformations
we can approximate the resulting contact interface and pressure distribution
without having to compute the actual deformations. We do that by precalculating
a "pressure field" on the interior of compliant objects (see Figure 3). The
pressure field approximates the pressure that would result from deforming the
surface to some point within the field. A point on the surface (that is, no
deformation) experiences zero pressure, but as it is pressed inward, it
experiences an increase in pressure (up to a maximum pressure on the interior of
the body). When two bodies are colliding, we look for a surface in the
intersecting volume where the pressure on the surface is the same in each
object; it’s an equilibrium surface (see Figure 4). There is pressure defined
across the entire contact surface. It is integrated to define the resultant
contact force and moment.

@image html "multibody/hydroelastics/images/contact-fig-03.png"

Figure 3: Three shapes and possible pressure fields in the interior of the
object. Pressure is zero at the outer boundary, and maximum on the interior.

@image html "multibody/hydroelastics/images/contact-fig-04.png"

Figure 4: The equilibrium contact surface (pale green) between two bodies where
the left-hand, yellow body has (a) less compliance, (b) equal compliance, and
(c) greater compliance.

This equilibrium surface has important properties:
- The contact surface will always be contained within the intersecting volume.
- The surface’s edge will always lie on the surface of both objects and have
  zero pressure.
- The location of the surface depends on the relative compliance of the two
  objects. As one object becomes more rigid than the other, the contact surface
  begins to converge to its surface (see Figure 4). As one surface becomes
  perfectly rigid, the other object deforms completely to conform to its shape.
- The contacting bodies need not be convex, nor will the contact surface
  between two objects necessarily be a single contiguous patch. For non-convex
  geometries, the contact can be meaningfully represented by multiple disjoint
  patches. The resultant contact force will still be meaningful.
- The resultant contact force is continuous with respect to the relative pose
  between bodies. In fact, the contact surface’s mesh, its area, and the
  pressures measured on the surface are likewise continuous.

@subsection hydro_model_parameters Model Parameters

The hydroelastic modulus has units of pressure, i.e. `Pa (N/m²)`. The
hydroelastic modulus is often estimated based on the Young's modulus of the
material though in the hydroelastic model it represents an effective elastic
property. For instance, @ref Elandt2019 "[Elandt et al., 2019]" chooses to use
`E = G`, with `G` the P-wave elastic modulus `G = (1-ν)/(1+ν)/(1-2ν)E`, with ν
the Poisson ratio, consistent with the theory of layered solids in which plane
sections remain planar after compression. Another possibility is to specify `E =
E*`, with `E*` the effective elastic modulus given by the Hertz theory of
contact, `E* = E/(1-ν²)`. In all of these cases a sound estimation of
`hydroelastic_modulus` starts with the Young's modulus of the material. However,
due to numerical conditioning, much smaller values are used in practice for hard
materials such as steel. While Young's modulus of steel is about 200 GPa (2×10¹¹
Pa), hydroelastic modulus values of about 10⁵−10⁷ Pa lead to good approximations
of rigid contact, with no practical reason to use higher values.

@note Although the hydroelastic modulus carries the same units as the more
familiar elastic moduli mentioned above, it is qualitatively different. Do not
expect to use values for those moduli for the hydroelastic modulus to good
effect.

As with point contact, the hydroelastic contact model uses a Hunt & Crossley
dissipation model (see @ref mbp_dissipation_model). It differs in defining the
combined effective dissipation parameter; for hydroelastic contact, the
combined dissipation depends on the hydroelastic moduli of the contacting
geometries. For two hydroelastic bodies A and B, with hydroelastic moduli `Eᵃ`
and `Eᵇ`, respectively, and dissipation `dᵃ` and `dᵇ`, respectively, the
effective dissipation is defined according to the combination law: <pre>
  E = Eᵃ⋅Eᵇ/(Eᵃ + Eᵇ),
  d = E/Eᵃ⋅dᵃ + E/Eᵇ⋅dᵇ = Eᵇ/(Eᵃ+Eᵇ)⋅dᵃ + Eᵃ/(Eᵃ+Eᵇ)⋅dᵇ
</pre>
thus dissipation is weighted in accordance with the fact that the softer
material will deform more and faster and thus the softer material dissipation is
given more importance.

The @ref hug_geometry_properties "hydroelastic user guide" shows how values for
these properties can be assigned to geometries.

@subsection hydro_practice Hydroelastic Contact in practice

The theory operates on arbitrary geometries and pressure fields. In practice,
we operate on discrete representations of both the geometry and the field.

Compliant objects are represented by tetrahedral meshes. The Drake data
type is @ref drake::geometry::VolumeMesh "VolumeMesh". The pressure fields on
those meshes are piecewise linear.

The resulting contact surface is likewise a discrete mesh. This mesh may be
built of nothing but triangles or polygons. The choice of representation has
various implications on the computation of contact forces (see below). The
pressure on the contact surface is likewise a piecewise linear function.

Objects with very large hydroelastic modulus can introduce stiffness into the
numerics of the contact resolution computation and lead to ill conditioning,
affecting simulation performance and robustness. For these cases, Drake permits
declaring an object as "rigid hydroelastic" (see @ref creating_hydro_reps.) When
a *rigid* hydroelastic object interacts with
other *compliant* hydroelastic objects, the contact surface always follows the
surface of the rigid object. Think of it as the compliant object doing 100% of
the deformation, so it conforms to the shape of the rigid object. However,
rigid hydroelastic objects have limitations. There is no hydroelastic contact
surface between two rigid hydroelastic representations -- when contact is
observed between two such objects, the contact will be characterized with
point contact (or maybe even throw), depending on how the
@ref hug_enabling "contact model" has been configured. For
this reason, the use of the rigid hydroelastic declaration should be used
judiciously -- making everything compliant can be simpler in reasoning about the
scene.

Important points to note:
- The time cost of resolving contact scales with the complexity of the contact
  surface (number of faces).
- The complexity of the contact surface is a function of the complexity of the
  contacting geometries.
- The best performance comes from the lowest resolution meshes that produce
  “acceptable” behaviors.
- It is not universally true that every geometry is represented discretely. The
  implementation may represent some shapes differently when it allows
  performance improvements. The canonical example would be a half space
  geometry. As a shape with infinite volume, it would be infeasible to create a
  finite, discrete representation. It is also unnecessary. Intersecting meshes
  directly with a half space is far more efficient.

@section mbp_dissipation_model Modeling Dissipation

We use a dissipation model inspired by the model in
@ref HuntCrossley1975 "[Hunt and Crossley 1975]", parameterized by a dissipation
constant with units of inverse of velocity, i.e. `s/m`.

To be more precise, compliant point contact forces are modeled as a function of
state x: <pre>
  f(x) = fₑ(x)⋅(1 - d⋅vₙ(x))₊
</pre>
where here `fₑ(x)` denotes the elastic forces, vₙ(x) is the contact velocity in
the normal direction (negative when objects approach) and `(a)₊` denotes "the
positive part of a". The model parameter `d ` is the Hunt & Crossley dissipation
constant, in s/m. The Hunt & Crossley term `(1 - d⋅vₙ(x))₊` models the effect of
dissipation due to deformation.

Similarly, Drake's hydroelastic contact model incorporates dissipation at the
stress level, rather than forces. That is, pressure `p(x)` at a specific point
on the contact surface replaces the force `f(x)` in the point contact model:
<pre>
  p(x) = pₑ(x)⋅(1 - d⋅vₙ(x))₊
</pre>
where `pₑ(x)` is the (elastic) hydroelastic pressure and once more the term
`(1 - d⋅vₙ(x))₊` models Hunt & Crossley dissipation.

This is our preferred model of dissipation for several reasons:
1. It is based on physics and has been developed based on experimental
   observations.
2. It is a continuous function of state, as in the real physical world.
   Moreover, this continuity leads to better conditioned systems of
   equations.
3. The bounce velocity after an impact is bounded by 1/d, giving a quick
   physical intuition when setting this parameter.
4. Typical values are in the range [0; 100] s/m, with a value of 20 s/m being
   typical.
5. Values larger than 500 s/m are unphysical and usually lead to numerical
   problems when using discrete approximations given how time is
   discretized.

The Hunt & Crossley model is supported by the
drake::multibody::DiscreteContactApproximation::kTamsi,
drake::multibody::DiscreteContactApproximation::kSimilar, and
drake::multibody::DiscreteContactApproximation::kLagged model approximations. In
particular, drake::multibody::DiscreteContactApproximation::kSimilar and
drake::multibody::DiscreteContactApproximation::kLagged are convex
approximations of contact, using a solver with theoretical and practical
convergence guarantees.
*/

/** @addtogroup  friction_model Modeling of Dry Friction
 @section friction_physical_model Physical Model

 Despite several @ref friction_numerical_approximations
 "numerical approximations" that Drake provides, the underlying physical "model"
 is the same; Coulomb's law of friction coupled with the maximum dissipation
 principle (MDP).

 Coulomb's law of friction can be stated in terms of forces for
 @ref compliant_point_contact "point contact" as:<pre>
  ‖fₜ‖ ≤ μₛfₙ,
 </pre>
 where μₛ is the "static" coefficient of friction and fₜ and fₙ are the
 tangential (friction) and normal components of the contact force, respectively.

 For @ref hydro_contact "hydroelastic contact" the same law can be stated in
 terms of stresses as:<pre>
  ‖Tₜ‖ ≤ μₛpₙ,
 </pre>
 where Tₜ and pₙ are the tangential (friction) and normal stresses,
 respectively.

 The friction force fₜ (or stress Tₜ) is perpendicular to the contact normal.
 It's direction within a plane perpendicular to the contact normal, is
 determined by MDP. The MDP states that the direction of the friction force is
 such that it maximizes the amount of dissipation due to friction. Therefore,
 for a sliding contact with a non-zero slip velocity vector vₜ, the friction
 force directly opposes vₜ and its magnitude is μₖfₙ (or μₖpₙ), where μₖ is the
 "dynamic" (or kinetic) coefficient of friction, with μₖ ≤ μₛ. This can be
 written as:
 <pre> 
  fₜ = -μₖ vₜ/‖vₜ‖ fₙ.
 </pre> 

 @section friction_numerical_approximations Numerical Approximations

 Drake can model multibody systems as being @ref mbp_continuous "continuous" or
 @ref mbp_discrete "discrete". Each of these represent different numerical
 approximations of the same underlying physics.

 In a nutshell, @ref mbp_continuous "continuous" models formulate the multibody
 physics at the acceleration level and use a @ref stribeck_approximation
 "regularized model of Coulomb friction". This model is a continuous function of
 state that can be integrated in time using standard error-controlled numerical
 methods.

 @ref mbp_discrete "Discrete" models on the other hand, formulate the multibody
 physics at the velocity level, and incorporate friction as constraints. Now,
 these constraints can be regularized similarly to continuous models, to improve
 numerics and robustness. Drake's discrete non-convex model with regularized
 friction is presented in @ref Castro2019 "[Castro et al., 2019]", and novel
 convex approximations are presented @ref Castro2022 "[Castro et al., 2022]" and
 @ref Castro2023 "[Castro et al., 2023]".

 Please refer to @ref multibody_solvers for further details.

 @note Discrete models, formulated at the velocity level, cannot differentiate
 between static (μₛ) and dynamic (μₖ) friction; only dynamic friction can be
 resolved. Therefore the static coefficient of friction is ignored.
*/

/** @addtogroup hydro_margin Margin for Hydroelastic Contact

In Drake we introduce the concept of "margin" to mitigate instability problems
caused by the discrete nature of time stepping schemes. 

While margin is not a new concept (see Kato's presentation and Mirtich's
papers), the way margin is used in Drake differs from its traditional purpose in
game engines. Game engines such as Bullet (cite manual and Chapter) and Unity
(cite online manual page) introduce margin mostly as a mechanism to mitigate
pass through problems that can arise when objects move at high speeds and/or the
simulation time step sizes are large in order to meet real-time requirements
within a limited computational budget. In Drake however, margin is used to
mitigate instability problems inherent of discrete schemes.

In the following sections we describe why we introduce margin, how we do it and
its limitations.

@section discrete_contact_instabilities Discrete Contact Instabilities

We motivate the concept of margin by considering an instability problem with the
simulation in Fig. 1. Two boxes (of FxFxF and mass of XX kg) are stacked on top
of each other. Their dynamics is simulated at a relatively large time step of 10
ms (large for robotics applications). Only their surface meshes are shown as
wire frames so that we can appreciate the contact surface, colored by pressure.
While we'd expect the upper box to rest stably on top of the box at the bottom,
we observe a wobbling instability where the contact surface shifts back and
forth between two corners. 

@image html drake/multibody/plant/images/unstable_box_box_contact.gif "Figure 1: Unstable flat-on-flat contact." width=50%

This wobbling is not physical, but rather a numerical instability. To gain
insight into this problem, we analyze the simplified system in Fig. 2. This
figure shows a box in contact with the ground (in gray) at three consecutive
time steps, labeled tₙ, tₙ₊₁, and , tₙ₊₂. In this simplified system we use a
point contact model in which only the corners marked in red can make contact
with the ground. At time tₙ only the left corner is in penetration. For such a
configuration the contact resolution phase will predict a contact force that
will attempt to minimize this interpenetration. If the time step size is large
enough, the box's configuration can actually overshoot into the state sketched
for tₙ₊₁. Now the box is in a new state where the right corner is in contact and
the left corner is out of contact. In this configuration, the box can overshoot
once again to the state at tₙ₊₂. Even more, the motion of the box can fall into
a limit cycle with intermittent contact between the two corners. 

The situation is more complex in Fig. 1, where we have continuous (in space)
contact surfaces instead of discrete points between fully three dimensional
objects. Still, the underlying principle of this instability remains.

@image html drake/multibody/plant/images/discrete_instabilities.png "Figure 2: Intermittent contact results in wobbling instabilities." width=35%

While the instability tends to show more frequently for the case of contact
between flat surfaces, it can also happen between complex geometries. Figure 3
shows an example of this instability for the contact between a mug placed on top
of a plate. When the visuals for the geometries are turned off, we can see the
two contact surfaces (table/plate and plate/table) rocking back and forth. The
visualization also includes the contact forces (red) and moments (blue).

@image html drake/multibody/plant/images/mug_on_plate.gif "Figure 3: Instability on complex geometries. Mug on a plate." width=50%

@section speculative_constraints Speculative Constraints

Lets consider once again the schematic in Fig. 2. At each time step, only one
constraint is added corresponding to the one contact point (left or right) that
is in penetration. If instead the set of contact constraints is augmented to
include both points, the solver can now use this information to compute a force
that can avoid the overshooting that leads to the instability.  We are
essentially adding contact constraints before contact actually happens. Whether
this constraints become active or not, will ultimately depend on the contact
resolution phase, which considers balance of momentum and the physics of
contact. We refer to this new set of constraints as "speculative constraints",
see (add Kato's reference). Speculative constraints are highlighted with blue
boxes in Fig. 2.

To provide more context, we'll discuss briefly how the solver can use
information from speculative constraints. Each contact pair will be
associated with a signed distance ϕ (defined negative if objects overlap). For
a compliant contact with stiffness k, the normal force is modeled according to:
<pre>
  fₙ = k (-ϕ)₊,                                                             (1)
</pre>
where (a)₊ = max(0, a). Notice that fₙ > 0 (i.e. repulsive) always, even if ϕ₀ >
0 (speculative constraint). At each time step Drake solvers user a first order
approximation of the signed distance function as ϕ ≈ ϕ₀ + δt⋅vₙ, where ϕ₀ is the
signed distance from the previous time step, vₙ the normal component of the
relative velocity between the two participating bodies at the contact point and
δt is the time step size. With this approximation the normal force is modeled as
<pre>
  fₙ ≈ k (-ϕ₀ - δt⋅vₙ)₊,  ϕ₀ < δ,                                           (2)
</pre>
where, to be more precise, we added the condition for speculative constraints,
with δ a pre-specified margin value. Large enough values of δ allow to recover
the original model (1). In practice however, we cannot have δ = ∞ since this
would incur in expensive 𝒪(n²) geometry queries. Moreover, the linear
approximation used for ϕ is no longer accurate for large extrapolations.
Notice that with δ = 0 (no margin) we only consider a subset of all
desired contact constraints.

Notice that while speculative constraints will allow the solver to avoid
overshoots and mitigate instabilities, this is done at the expense of having to
solve a contact problem with a larger number of constraints. While generally
negligible, this will impact performance.

The precise value of margin is therefore determined as a trade-off between
accuracy/stability and performance. Section @ref margin_how_much discusses how
we determine values of margin in detail.

@section margin_for_hydroelastics Hydroelastic Contact

Speculative constraints for contacts within a given margin fit well within a
framework in terms of distance constraints — contact pairs with a distance
within the margin threshold are included in the contact resolution phase. In
contrast, how to introduce margin within the hydroelastic contact framework is
not as straightforward.

In the @ref hydro_contact "hydroelastic contact model", each geometry is
assigned a body centric "pressure" field. The contact surface between two
overlapping geometries is computed as the surface of equal hydroelastic
pressure. 

To include the concept of margin, we need to extend hydroelastic contact. In our
approach we first "inflate" geometries such that their boundary is moved a
margin distance in the outwards direction. This creates a thin layer of margin
thickness all around the object. While this operation is well defined for convex
shapes, it requires a more careful consideration for non-convex shapes (this is
discussed in further detail in @ref margin_non_convex). We then extend the
pressure field within this new "margin layer" with negative values at the
boundary of the extended region, in such a way that the zero level set of the
pressure field is good approximation to the original geometry (exact for convex
shapes).

Figure 4 exemplifies this process for the case of a cylinder. Iso-surfaces of
constant pressure, colored by pressure, are shown. The extended pressure is
defined such that the zero pressure level set corresponds to the original
cylinder.

@image html drake/multibody/plant/images/extended_cylinder.png "Figure 4: Inflated cylinder and its pressure field." width=35%

The newly inflated geometries and their extended pressure fields are then
processed as any other hydroelastic fields. When the geometries (including their
margin region) overlap, the contact surface is computed as the surface of equal
pressure. This computation will now lead to triangulated contact surfaces with a
thin "skirt", of thickness in the order of the margin, all around the contact
surface where pressures are negative. Figure 5 shows peppers modeled with a very
large margin value of 1 cm to exaggerate the visualization of this "skirt" of
negative pressure values. The margin layer or skirt is clearly visible as a
halo all around the peppers.

@image html drake/multibody/plant/images/peppers_margin_1cm.png "Figure 5: Pepper models with a very large margin of δ = 1 cm." width=30%

Similarly to the compliant point contact model in terms of signed distance, we
use a first order approximation of the pressure field as p ≈ (p₀ + δt⋅vₙ⋅g)₊,
where p₀ is the previous time step pressure and g = ∂p/∂n is the gradient of the
pressure field in the normal direction 
(see @ref Masterjohn2022 "[Masterjohn et al., 2022]"). With this, the normal
force contribution from a triangle of area Δ is:
<pre>
  fₙ = Δ⋅(p₀ + δt⋅vₙ⋅g)₊,
</pre>
where for speculative constraints p₀ < 0.

Therefore, triangles with a negative pressure are processed by the contact
solver as speculative constraints, and will only generate forces if the new
configuration pushes this triangles into the positive pressure region. Triangles
that after the resolution phase still have a negative pressure will not produce
forces. As with the example of Fig. 5, the margin layer effectively increases
the number of constraints, increasing the contact problem size.

For more on hydroelastic contact, including the modeling of dissipation, refer
to @ref hydro_contact "Hydroelastic Contact".

@section margin_how_much But, How Much Margin?

How much margin is enough to mitigate stability problems? and how does this
quantity scales or depends on problem parameters such stiffness, size, etc.?

To answer this question we consider the simplified two-dimensional system in
Fig. 6. This is a massless rod of length `L` connecting two points of mass
`m/2`. We assume the rod to be in a limit cycle between two states, left (State
I) and right (State II) in Fig. 6. Only one point is in penetration in each
state. In the discrete setting, the rod is assumed to go back and forth between
these two states in a single time step. The states are assumed symmetric, with
the rod rotated an angle θ in State I and an angle -θ in State II. Due to this
symmetry, the angular velocity to go from State II to State I is ω and the
angular velocity to go from State I to State II is -ω. 

@image html drake/multibody/plant/images/margin_stability.png "Figure 6: Simplified system for stability analysis." width=50%

Since the system is in this symmetric limit cycle, the rod does not move in the
vertical direction. Therefore the mean normal force due to contact must balance
gravity, i.e. `<fₙ> = m⋅g`. Since only one point is in contact at each state, we
have that `<fₙ> = fₙ`, where we denoted with fₙ the instantaneous normal force
at one of these points. Thus fₙ = m⋅g. Notice this force value is independent of
stiffness.

The rotational inertia of this system is `I = m⋅L²/4`. In the discrete system,
the system moves between State I and State II according to the balance of
momentum:
<pre>
  I⋅(ω − ω₀)=δt⋅fₙ⋅L/2,
  θ = θ₀ + δt⋅ω,
</pre>
with θ₀ and ω₀ the previous angle and angular velocity, respectively. Since the
states are symmetric, ω₀ = -ω and θ₀ = -θ. Using the expressions for `I` and
`fₙ` we can determine the angular velocity to be `ω = δt⋅g/2`. The angle is then
θ = δt²⋅g/(2L). The total angle change is Δθ = 2⋅θ = δt²⋅g/L. We can now
determine the amplitude of these oscillations in signed distance as Δϕ =
L⋅sin(Δθ) ≈ L⋅Δθ. Putting it all together we obtain:
<pre>
  Δϕ = δt²⋅g.                                                               (3)
</pre>

This result reveals a number of important properties for the amplitude of these
oscillations:
 1. independent of mass m and of size L,
 2. independent of contact stiffness k,
 3. scale with δt²,

This is a very important result, because if it holds for more complex and
arbitrary geometries, it tells us that we can use a single value of the margin δ
for all of our simulations (for as long as we are on the same planet).

To verify the validity of this scaling result on a more complex situation, we
run the simulation of Fig. 1, with different problem parameters, sizes and
margin. Notice the case is now three-dimensional, with contact modeled using the
hydroelastic contact model instead of point contact, which provides continuous
surfaces rather than discrete points. We run the simulation with the box stacked
on top of each other, no margin, and let the system reach its limit cycle in
which the boxes rattle back and forth. Unlike the simplified system of Fig. 6,
the limit cycle typically include several time  steps and the boxes rotate about
an axes usually not aligned with their axes of symmetry.

The results are shown in Fig. 7. We plot mean penetration dimensionless with
`δt²⋅g`. On the horizontal axes with plot a "semi-dimensionless" time step. With
that we mean that if hydro modulus E was a point contact stiffness k, this
quantity would be dimensionless. It is not, but it still scales data properly
such that curves overlap in the region at the left of the plot. We observe two
regions. On the right, data scales as `y ~ x⁰ᐧ¹`, which essentially confirms the
quotient `Δϕ / (δt²⋅g)` is approximately constant as predicted by Eq. 3. On the
left, penetration is dominated by compliance, and it simply increases as the
hydroelastic modulus is lowered.

@image html drake/multibody/plant/images/box_on_box_mean_penetration.png "Figure 7: Scaling of the mean penetration with problem parameters." width=30%

For non-zero margin, we now expect that if these vibrations are in the oder of
the margin δ, they'll be mitigated. We test this by running the same simulation
and plotting the standard deviation of the vibrations vs margin (dimensionless
with δt²⋅g) in Fig. 8. We observe in Fig. 8 that indeed for `Δϕ / (δt²⋅g) < 1`
the instabilities die off.

@image html drake/multibody/plant/images/instability_vs_margin.png "Figure 8: Standard deviation of the vibrations vs. margin." width=30%

Notice how remarkable these results are. Hydroelastic modulus spans five orders
of magnitude, geometry sizes and aspect ratios change, and δt² spans four orders
of magnitude. The fact these curves collapse in such tight regions is an
excellent confirmation of the scaling law from Eq. (3). Moreover, additional
experimentation with complex geometries such as those in Figs. 3 and 5 provide
additional confirmation.

Finally, based on this plots, we can say that setting `δ = δt²⋅g` is quite
conservative and will mitigate instabilities effectively. The largest time step
typically expected in simulations of robotics system seldom exceeds 10 ms.
Therefore a value of `δ = 10⁻⁴ m` will be more than enough in all robotics
applications. While incredibly effective, notice how small this value is.

@section margin_contact_results Contact Results

TODO: write.

@section margin_non_convex Non-Convex Geometries

TODO: Write this.

 */