/** @file
 Doxygen-only documentation for @ref hydroelastic_user_guide,
 */

/**
@defgroup hydroelastic_user_guide Hydroelastic Contact User Guide
@ingroup multibody

@section hug_title Hydroelastic Contact User Guide

@subsection hug_introduction Introduction

There are many ways to model contact between rigid bodies. Drake uses an
approach we call “compliant” contact. In compliant contact, nominally rigid
bodies are allowed to penetrate slightly, as if the rigid body had a slightly
deformable layer, but whose compression has no appreciable effect on the body’s
mass properties. The contact force between two deformed bodies is distributed
over a contact patch with an uneven pressure distribution over that patch. It
is common in robotics to model that as a single point contact or a set of point
contacts. Hydroelastic contact instead attempts to approximate the patch and
pressure distribution to provide much richer and more realistic contact
behavior. For a high-level overview, see [this blog post]
(https://medium.com/toyotaresearch/rethinking-contact-simulation-for-robot-manipulation-434a56b5ec88).

Drake implements two models for resolving contact to forces: point contact and
hydroelastic contact. See @ref hydro_appendix_a for a fuller discussion of the
theory and practice of contact models. For notes on implementation status, see
@ref hydro_appendix_b. Also see @ref hydro_appendix_examples_and_tutorials.

@subsection hug_working_with_hydro Working with Hydroelastic Contact

It is relatively simple to enable a simulation to use hydroelastic
contact. However, using it effectively requires some experience and
consideration. This section details the mechanisms and choices involved in
enabling hydroelastic contact and the next section helps guide you through some
common tips, tricks, and traps.

Using hydroelastic contact requires two things:
- Configuring the system (drake::multibody::MultibodyPlant) to use hydroelastic
  contact.
- Applying appropriate properties to the collision geometries to enable
  hydroelastic contact calculations.

Collision geometries for hydroelastic contact can be either
"compliant-hydroelastic" with tetrahedral meshes describing an
internal pressure field or "rigid-hydroelastic" with triangle
surface meshes that are regarded as infinitely stiff.
The figure below shows examples of a compliant hydroelastic box
and a rigid hydroelastic box.
<!-- N.B. This image is also used by hydroelastic_contact_basics.ipynb. -->
@image html "multibody/hydroelastics/images/HydroelasticTutorialCompliantRigidOutsideInside800x669.jpg"

The figure below shows a contact surface between a compliant-hydroelastic cylinder
and a compliant-hydroelastic sphere. The contact surface is internal to both
solids and is defined as the surface where the two pressure fields are equal.
<!-- N.B. This image is also used by hydroelastic_contact_basics.ipynb. -->
@image html "multibody/hydroelastics/images/HydroelasticTutorialContactSurfaceCompliantCompliant.png"

Pictured below, a rigid-hydroelastic cylindrical spatula handle is grasped
by two hydroelastic-compliant ellipsoidal bubble grippers.
The contact surface between the spatula handle's rigid-hydroelastic
geometry and either gripper is on the <b>surface</b> of the
rigid-hydroelastic geometry.

<!-- N.B. This image is also used by hydroelastic_contact_basics.ipynb. -->
@image html "multibody/hydroelastics/images/HydroelasticTutorialContactSurfaceRigidCompliantBubble.png"

@subsubsection hug_enabling Enabling Hydroelastic contact in your simulation

Because @ref drake::multibody::MultibodyPlant "MultibodyPlant"
is responsible for computing the dynamics of the
system (and the contact forces are part of that), the ability to enable/disable
the hydroelastic contact model is part of `MultibodyPlant`’s API:
drake::multibody::MultibodyPlant::set_contact_model().

There are three different options:
- drake::multibody::ContactModel::kPoint
- drake::multibody::ContactModel::kHydroelastic
- drake::multibody::ContactModel::kHydroelasticWithFallback

The default model is
@ref drake::multibody::ContactModel::kHydroelasticWithFallback
"kHydroelasticWithFallback".

@ref drake::multibody::ContactModel::kPoint "kPoint" is the implementation of
the point contact model (see @ref hydro_appendix_a).

Models @ref drake::multibody::ContactModel::kHydroelastic "kHydroelastic" and
@ref drake::multibody::ContactModel::kHydroelasticWithFallback
"kHydroelasticWithFallback" will both enable the hydroelastic contact. For
forces to be created from hydroelastic contact, geometries need to have
hydroelastic representations (see @ref creating_hydro_reps).

@ref drake::multibody::ContactModel::kHydroelastic "kHydroelastic" is a strict
contact model that will attempt to create a hydroelastic contact surface
whenever a geometry with a hydroelastic representation appears to be in contact
(based on broad-phase bounding volume culling). With this contact model, the
simulator will throw an exception if:

- The contact is between two geometries with rigid hydroelastic representation,
or
- The contact is between a geometry with a hydroelastic representation and one
without.

Collisions between two geometries where neither has a hydroelastic
representation are simply ignored.

@ref drake::multibody::ContactModel::kHydroelasticWithFallback
"kHydroelasticWithFallback" provides “fallback” behavior for when
@ref drake::multibody::ContactModel::kHydroelastic "kHydroelastic" would throw.
When those scenarios are detected, it uses the point contact model between the
colliding geometries so that the contact can be accounted for and produce
contact forces. As the implementation evolves, more and more contact will be
captured with hydroelastic contact and the circumstances in which the
point-contact fallback is applied will decrease.

@subsubsection creating_hydro_reps Creating hydroelastic representations of collision geometries

By default no geometry in drake::geometry::SceneGraph has a hydroelastic
representation. So, enabling hydroelastic contact in `MultibodyPlant`, but
forgetting to configure the geometries will lead to either simulation crashes
or point contact-based forces.

In order for a mesh to be given a hydroelastic representation, it must be
assigned certain properties. The exact properties it needs depends on the Shape
type and whether it is rigid or compliant. Some properties can be defined, but
if they are absent they’ll be provided by `MultibodyPlant`. First we’ll discuss
each of the properties and then discuss how they can be specified.

@paragraph hug_properties Properties for hydroelastic contact
<!-- TODO(rpoyner-tri): consider restructuring this section to paragraphs,
     rather than a nested list -->
- Hydroelastic classification
   - To have a hydroelastic representation, a shape needs to be classified as
     either “compliant” or “rigid”. This must be explicit -- there are no
     implicit assumptions. Otherwise, it will participate in point contact only.
- Resolution hint (meters)
   - A positive real value in meters.
   - Most shapes (capsules, cylinders, ellipsoids, spheres) need to be
     tessellated into meshes. The resolution hint controls the fineness of the
     meshes.
   - For all tessellated geometries, Drake puts hard limits on the effect of
     the resolution hint property. There is a limit to how fine Drake will
     tessellate a primitive. Drake has hard-coded internal limits that are
     still very generous (approximately 100 million tetrahedra or triangles)
     which should be more than enough for any application. In practice,
     resolution hints will be quite “large” as coarser meshes are typically
     more desirable.
   - Notes on choosing a value:
     - Generally, the coarsest mesh that produces desirable results will allow
       the simulation to run as efficiently as possible.
       - The coarsest mesh will typically be generated when the resolution is
         equal to the geometry’s maximum measure (see the details below).
     - If tessellation artifacts become obvious in the simulation, either make
       the object more compliant (see hydroelastic modulus) or increase the
       resolution (as discussed for the various shapes below).
   - Geometric detail:
     - For each geometry type, the resolution hint works with some (maybe
       abstract) choice of a representative circle, and its effect is defined
       in terms of the circumference of that circle. The resolution hint is the
       target length of each edge of the tessellated mesh around the
       circle. The circle will have N = ⌈2πr/h⌉ edges, where r is the capsule
       radius and h is the resolution hint. The number N changes
       discontinuously as h changes. However, generally, decreasing the
       resolution hint by some factor will increase the number of edges by that
       same factor. The number of elements will grow quadratically.
       - Sphere
         - The representative circle is the great circle of the sphere.
       - Cylinder
         - The representative circle is the base of the cylinder.
       - Capsule
         - The representative circle is the base of the cylinder, which is also
           the great circle of each hemisphere.
       - Ellipsoid
         - The representative circle is an abstract circle whose radius is that
           of the largest semi-axis of the elllipsoid.
     - Resolution hint has no effect on Box.
- Hydroelastic modulus (Pa (N/m²))
  - This is the measure of how stiff the material is. It directly defines how
    much pressure is exerted given a certain amount of penetration. More
    pressure leads to greater forces. Larger values create stiffer objects. An
    infinite modulus is mathematically equivalent to a “rigid”
    object. (Although, it is definitely better, in that case, to simply declare
    it “rigid”.)
  - This is *only* required for shapes declared to be “compliant”. It can be
    defined for shapes declared as “rigid”, but the value will be ignored. (It
    can be convenient to always define it to enable tests where one might
    toggle the classification between rigid and compliant such that the
    compliant declaration is always well formed.)
  - Declaring a geometry to be compliant but not providing a valid hydroelastic
    modulus will produce an error at the time the geometry is added to
    `SceneGraph`.
  - This is similar to something like Young’s modulus but it is not identical.
    - The primary distinction is this isn’t purely a material property. The
      value is entangled with the geometry. It essentially scales the force
      based on the *percentage* of penetration into an object (i.e.,
      strain). So, a fixed modulus value might lead to a 1cm penetration on one
      sphere. But if the sphere were scaled up by a factor of 10, the
      penetration would become 10cm. So, if there are constraints on
      penetration depth, the modulus would have to increase with the scale of
      the geometry.
  - Notes on choosing a value:
    - Starting with the value for Young’s modulus is not
      unreasonable. Empirical evidence suggests that the hydroelastic modulus
      tends to be smaller depending on the size of the objects.
    - For large modulus values, the resolution of the representations matter
      more. A very high modulus will keep the contact near the surface of the
      geometry, exposing tessellation artifacts. A smaller modulus has a
      smoothing effect.
- Hunt-Crossley dissipation (s/m)
  - A non-negative real value.
  - This gives the contact an energy-damping property.
  - The larger the value, the more energy is damped.
  - If no value is provided, `MultibodyPlant` provides a default value of zero.
  - Notes on choosing a value:
    - For objects that are nominally rigid, starting with zero-damping is good.
    - If the behavior seems “jittery”, gradually increase the amount of
      dissipation. Typical values would remain in the range [0, 100] with 10 to
      50 being a typical damping amount.
    - The inverse of the Hunt-Crossley dissipation is the maximum bounce
      velocity between two objects. Therefore, if maximum bounce velocities of
      10 cm/s are acceptable, a dissipation of 10 s/m will keep bounce
      velocities bounded to this range.
    - Warning: values larger than about 500 s/m are unphysical and typically
      lead to numerical problems. Due to time discretization errors, the user
      will see objects that take a long time to go back to their rest state.
    - Remember, as this value increases, your simulation will lose energy at a
      higher rate.
  - See @ref mbp_dissipation_model "Modeling Dissipation" for details.
- Slab thickness
  - A positive real value in meters
  - Only used for compliant half spaces. Declaring a half space to be compliant
    but lacking a value for slab thickness will cause the simulation to error
    out.
  - A measure of the thickness of the compliant material near the boundary of
    the half space.
    - A compliant half space can be made more rigid by:
      - Increasing the hydroelastic modulus and fixing slab thickness.
      - Fixing the hydroelastic modulus and decreasing the slab thickness.
      - Increasing the hydroelastic modulus and decreasing the slab thickness.

@subsubsection hug_geometry_properties Assigning hydroelastic properties to geometries

Properties can be assigned to geometries inside a URDF or SDFormat file using
some custom Drake XML tags. Alternatively, the properties can be assigned
programmatically.

@paragraph hug_file_specs Assigning hydroelastic properties in file specifications

Drake has introduced a number of custom tags to provide a uniform language to
specify hydroelastic properties in both URDF and SDFormat. The tag names are
the same but the values are expressed differently (to align with the practices
common to URDF and SDFormat). For example, consider a custom tag <drake:foo>
that takes a single integer value. In an SDFormat file it would look like:

    <drake:foo>17</drake:foo>

But in a URDF file it would look like this:

    <drake:foo value=”17”/>

Both URDF and SDFormat files define a <collision> tag. This tag contains the
tag for specifying the geometry type and associated properties. The
hydroelastic properties can be specified as a child of the <collision>
tag. Consider the following SDFormat example:

    …
      <collision name="body1_collision">
        <geometry>
          ...
        </geometry>
        <drake:proximity_properties>
          <drake:compliant_hydroelastic/>
          <drake:mesh_resolution_hint>0.1</drake:mesh_resolution_hint>
          <drake:hydroelastic_modulus>5e7</drake:hydroelastic_modulus>
          <drake:hunt_crossley_dissipation>1.25</drake:hunt_crossley_dissipation>
        </drake:proximity_properties>
      </collision>
    …
and the following equivalent URDF example:

    …
      <collision name="body1_collision">
        <geometry>
          ...
        </geometry>
        <drake:proximity_properties>
          <drake:compliant_hydroelastic/>
          <drake:mesh_resolution_hint value="0.1"/>
          <drake:hydroelastic_modulus value="5e7/>
          <drake:hunt_crossley_dissipation value="1.25/>
        </drake:proximity_properties>
      </collision>
    …

For a body, we’ve defined a collision geometry called “body1_collision”. Inside
the <collision> tag (as a sibling to the <geometry> tag), we’ve introduced the
Drake-specific <drake:proximity_properties> tag. In it we’ve defined the
geometry to be compliant for hydroelastic contact and specified its resolution
hint, its hydroelastic modulus, and its dissipation.

Let’s look at the specific tags:

- <@ref tag_drake_proximity_properties>: This is the container for all
  Drake-specific proximity property values.
- The following tags define hydroelastic properties for the collision geometry
  and would be contained in the <@ref tag_drake_proximity_properties> tag.
  - <@ref tag_drake_rigid_hydroelastic> or
    <@ref tag_drake_compliant_hydroelastic>
  - <@ref tag_drake_mesh_resolution_hint>
  - <@ref tag_drake_hydroelastic_modulus> (for compliant geometry only)
  - <@ref tag_drake_hunt_crossley_dissipation>
- In the name of completeness, the following tags would be children of the
  <@ref tag_drake_proximity_properties> tag as well, but are not restricted to
  use with hydroelastic contact.
  - <@ref tag_drake_point_contact_stiffness>: Only used when this geometry is
    for point contact. Defines the compliance of the point contact. This
    property is allowed to exist even if the geometry has been specified to be
    of hydroelastic type, it will have no effect on hydroelastic computations.
  - <@ref tag_drake_mu_static> The coefficient for static friction; this
    applies to both point contact and hydroelastic contact.
  - <@ref tag_drake_mu_dynamic> The coefficient for dynamic friction; this
    applies to both point contact and hydroelastic contact.

@paragraph hug_code_properties Assigning hydroelastic properties in code

Hydroelastic properties can be set to objects programmatically via the following
APIs (see their documentation for further details):

- AddContactMaterial()
- AddRigidHydroelasticProperties()
- AddCompliantHydroelasticProperties()
- AddCompliantHydroelasticPropertiesForHalfSpace()

Some extra notes:

AddContactMaterial() isn’t hydroelastic contact specific, but does provide a
mechanism for setting the friction coefficients that hydroelastic and point
contact models both use.

AddRigidHydroelasticProperties() is overloaded. One version accepts a value for
resolution hint, one does not. When in doubt which to use, it is harmless to
provide a resolution hint value that would be ignored for some geometries, but
failing to provide one where necessary will cause the simulation to throw an
exception.  Note the special case for declaring a compliant half space -- it
takes the required slab thickness parameter in addition to the hydroelastic
modulus value.

@subsection hug_visualizing Visualizing hydroelastic contact

Start Meldis by:

    $ bazel run //tools:meldis -- --open-window &

Run a simulation with hydroelastic contact model; for example,

    $ bazel run //examples/hydroelastic/ball_plate:ball_plate_run_dynamics -- \
       --mbp_dt=0.001 --x0=0.10 --z0=0.15 --vz=-7.0 \
       --simulation_time=0.015 --simulator_publish_every_time_step

At the time of this writing, the Meldis view of this example looks like this:

@image html "multibody/hydroelastics/images/drake-vis-01.png" width=90%

In the above picture, we see two red force vectors acting at the centroids of
the two contact surfaces and also the two blue moment vectors. One contact
surface represents the ball pushing the dinner plate down. The other contact
surface represents the rectangular floor pushing the dinner plate up. (Gravity
forces are not shown, nor is the contact between the ball and the floor.) Notice
that only approximately half of the bottom of the plate makes contact with the
floor.

The Scene panel can toggle on/off many different aspects of the visualization
using checkboxes, including: which contacts are shown, whether to view the
illustration (visual) geometry and/or collision (proximity) geometry and/or the
bodies' inertia, as well as sliders for controlling transparency to better view
overlapping shapes.

When viewing collisions, note that red arrows indicate a hydroelastic contact,
with an associated contact patch. Green arrows indicate point contact, which
does not have any contact patch.

@subsection hug_pitfalls Pitfalls/Troubleshooting

Here are various ways that hydroelastic contact may surprise you.
- By default, a rigid body is not a rigid hydroelastic body until users say
  so. Otherwise, it is ignored by the hydroelastic contact model and might get
  point contact instead.
  - Users need to call AddRigidHydroelasticProperties() or use the URDF or
    SDFormat tag <drake:rigid_hydroelastic/>.
- Backface culling -- the visualized contact surface is not what you
  expect. Or, “Why are there holes?”
  <!-- TODO(rpoyner-tri): This should come with some illustrations to make this
       clear. -->
- Hydroelastic modulus is not properly a material property; it combines
  material and geometry.
  - Make the sphere bigger, the force gets weaker for the same contact surface.
- “I don’t seem to be getting any contact surfaces although my spheres are
  clearly overlapping!” Depending on how coarsely you tessellated your geometry
  (a sphere, at its coarsest, is an octahedron), there can be a large amount of
  error between the primitive surface and discrete mesh’s surface. Make sure
  you visualize the hydro meshes and have what you expect.
- Stiff and coarse is seldom a good thing. The stiffer an object is, the more
  the details of the discrete tessellation will directly contribute to the
  dynamics.
  <!-- TODO(rpoyner-tri): Ideally, there should be a demo that shows a coarse
       sphere with decreasing elasticity rolling would be good. One rolls like
       an 8-sided dice.  One more closely approximates a sphere. -->
- Getting moments out of the contact depends on how many elements are in the
  contact surface. If the elements of the two contributing meshes are much
  larger than the actual contact surface, the contact can become, in essence,
  point contact.
- Half spaces are not represented by meshes. They don’t contribute *any*
  geometry to the resultant contact surface. The contact surface’s refinement
  will depend on the other geometry’s level of refinement.

@subsubsection hug_tips Tips and Tricks

This is a random collection of things we can do to maximize the benefits of the
hydroelastic contact model. Some of these tricks accommodate current
implementation limitations, and some are to work within the theoretical
framework.

@paragraph hug_gaming Gaming the model

- Rigid meshes need not be closed. You can use this to provide fine grained
  control over where a contact surface can actually exist. Be careful to have a
  consistent direction of surface normals on the triangles. They should point
  outward.

- For a given relative configuration between bodies, the magnitude of force can
  be tuned in two ways: increasing the hydroelastic modulus and/or scaling the
  geometry up (in essence, increasing the average penetration depth).


@section hydro_appendix_a Appendix A: Compliant Contact Models

@subsection hug_point_contact_theory Compliant Point Contact

Before we get into hydroelastic contact, it’s worth describing the compliant
point contact model as a reference model. The point contact model defines the
contact force by determining the minimum translational displacement (MTD). The
minimum translational displacement is the smallest relative displacement
between two volumes that will take them from intersecting to just
touching. This quantity need not be unique (if two spheres have coincident
centers, any direction will serve). Once we have this displacement, we get
three quantities that we use to define the contact force:
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

Hydroelastic contact was created to address some of the short-comings in point
contact. In fact, one might argue that many of the strategies used to mitigate
the shortcomings of point contact (such as using lots of points) push it closer
and closer to hydroelastic contact.

@subsection hug_hydro_theory Hydroelastic Contact

Hydroelastic Contact is another compliant contact formulation. It was
originally introduced by @ref Elandt2019 "[Elandt 2019]". Modifications and
further development of the model can be found in
@ref Masterjohn2022 "[Masterjohn 2022]". In Drake, we refer to this model as
the “hydroelastic” model. It differs from point contact in how it characterizes
the contact. Rather than a single point, it imagines an entire contact
surface. This surface is an approximation of the contact surface as visualized
in Figure 2(b).

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


@subsection hug_hydro_practice Hydroelastic Contact in practice

The theory operates on arbitrary geometries and pressure fields. In practice,
we operate on discrete representations of both the geometry and the field.

Compliant objects are represented by tetrahedral meshes. The Drake data
type is @ref drake::geometry::VolumeMesh "VolumeMesh". The pressure fields on
those meshes are piecewise linear.

The resulting contact surface is likewise a discrete mesh. This mesh may be
built of nothing but triangles or polygons. The choice of representation has
various implications on the computation of contact forces (see below). The
pressure on the contact surface is likewise a piecewise linear function.

We model perfectly rigid objects (objects with no compliance at all) as
triangle surface meshes. They have no pressure field associated with them
because any penetration from the surface instantaneously produces infinite
pressure. When colliding a rigid object against a compliant object, the contact
surface always follows the surface of the rigid object. Think of it as the
compliant object doing 100% of the deformation, so it conforms to the shape of
the rigid object.

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
- When an object is very near rigid it is more efficient to model it as rigid
  than as a very stiff compliant object. Modeling as rigid allows us to take
  advantage of the fact that the contact surface will lie on the surface of the
  rigid object.

@section hydro_appendix_b Appendix B: Current state of implementation

This section will be updated as the scope and feature set of
hydroelastic contact is advanced. The main crux of this section is to clearly
indicate what can and cannot be done with hydroelastic contact.

@subsection hug_implemented What can you do with hydroelastic contact?

- Hydroelastic contact can be used with MultibodyPlant in either continuous or
  discrete mode.
- You can visualize the contact surfaces, their pressure fields, and the
  resultant contact forces in MeshCat as the simulation progresses. However,
  for playing back recordings in MeshCat, only the contact forces and moments
  are available (see issue
  [19142](https://github.com/RobotLocomotion/drake/issues/19142)).
- All Drake Shape types can be used to create rigid hydroelastic bodies (this
  includes arbitrary meshes defined as OBJs).
- Drake primitive Shape types (Box, Capsule, Cylinder, Ellipsoid, HalfSpace,
  and Cylinder) can all be used to create both compliant hydroelastic bodies
  and rigid hydroelastic bodies.
- The Drake Convex Shape type can be used both as a compliant hydroelastic body
  and a rigid hydroelastic body. To use Convex, add the custom tag
  `<drake:declare_convex/>` tag under the `<mesh>` tag in either an SDFormat
  or URDF file.
- The Drake Mesh Shape type can be used as a compliant hydroelastic body
  using a tetrahedral mesh in VTK file.
  See
  [examples/hydroelastic/python_nonconvex_mesh.]
  (https://github.com/RobotLocomotion/drake/tree/master/examples/hydroelastic/python_nonconvex_mesh)

@subsection hug_not_yet_implemented What can’t you do with hydroelastic contact?

- You can’t get contact surfaces between two rigid hydroelastic geometries.
  The contact surface between rigid geometries is ill defined and will most
  likely never be supported.
  The default drake::multibody::ContactModel::kHydroelasticWithFallback uses
  point contact model for the rigid-rigid contact.
  See @ref hug_enabling.
- Hydroelastics cannot model true deformations given the model does not
  introduce state. Therefore effects such as tangential compliance or
  short time scale waves are not captured by the model.
  Currently we are actively developing code for deformable bodies in
  @ref drake::multibody::DeformableModel.

@subsection hug_dissipation_and_solver Current dissipation models

- The TAMSI (DiscreteContactApproximation::kTamsi), Similar
  (DiscreteContactApproximation::kSimilar), and Lagged
  (DiscreteContactApproximation::kLagged) model approximations use a
  Hunt-Crossley model of dissipation, parameterized by
  `hunt_crossley_dissipation`, for both point and hydroelastic contact. The SAP
  approximation parameter `relaxation_time` is ignored by these approximations.
- The SAP model approximation (DiscreteContactApproximation::kSap) uses a linear
  Kelvin–Voigt model of dissipation parameterized by `relaxation_time`, for both
  point and hydroelastic contact. The Hunt-Crossley dissipation parameter is
  ignored by this approximation.
- We allow the user to specify both `hunt_crossley_dissipation` (TAMSI, Similar
  and Lagged discrete approximations as well as continuous plant model) and
  `relaxation_time` (SAP approximation specific parameter) on the model, but the
  parameter may be ignored depending on your plant configuration. See the
  documentation for that in the [MultibodyPlant documentation.]
  (https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html#:~:text=%E2%81%B4%20We%20allow%20to,will%20be%20ignored.)

@section hydro_appendix_examples_and_tutorials Appendix C: Examples and Tutorials
- Example
  [Contact between a ball, dinner plate, and floor in C++]
  (https://github.com/RobotLocomotion/drake/tree/master/examples/hydroelastic/ball_plate)
  uses the hydroelastic contact model for a convex ball and non-convex
  dinner plate.

- Example
  [Contact between a ball and paddle (box) in Python]
  (https://github.com/RobotLocomotion/drake/tree/master/examples/hydroelastic/python_ball_paddle)
  uses the hydroelastic contact model for a ball on stationary paddle (box).

- Example
  [Contact between a bell pepper, dinner bowl, and floor in Python]
  (https://github.com/RobotLocomotion/drake/tree/master/examples/hydroelastic/python_nonconvex_mesh)
  uses the hydroelastic contact model for a non-convex bell pepper in
  a non-convex dinner bowl.

- Example
  [Slip control for a spatula handle between finger grippers in C++]
  (https://github.com/RobotLocomotion/drake/tree/master/examples/hydroelastic/spatula_slip_control)
  uses the hydroelastic contact model for a spatula handle between finger
  grippers.

- Tutorial
  [Hydroelastic Contact Basics with Jupyter Python notebook]
  (https://github.com/RobotLocomotion/drake/blob/master/tutorials/hydroelastic_contact_basics.ipynb)
  shows how to simulate hydroelastic contact for a compliant block dropped
  on a rigid rectangular plate.
  The below-left picture shows one frame of the block's contact with
  the contact force as a red arrow and the contact torque as a blue arrow.
  The below-right picture zooms into the hydroelastic contact surface showing
  pressure distribution.
  <!-- N.B. This image is also used by hydroelastic_contact_basics.ipynb. -->
  @image html "multibody/hydroelastics/images/HydroelasticTutorialImage600x388.jpg"


@section hydro_references Sources referenced within this documentation

- @anchor Elandt2019 [Elandt 2019] Elandt, R., Drumwright, E., Sherman, M.,
  & Ruina, A. (2019, November). A pressure field model for fast, robust
  approximation of net contact force and moment between nominally rigid
  objects. In 2019 IEEE/RSJ International Conference on Intelligent Robots
  and Systems(IROS) (pp. 8238-8245). IEEE. https://arxiv.org/abs/1904.11433 .

- @anchor Masterjohn2022 [Masterjohn 2022] Masterjohn, J., Guoy, D., Shepherd,
  J., & Castro, A. (2022). Velocity Level Approximation of Pressure Field
  Contact Patches. In 2022 IEEE/RSJ International Conference on Intelligent
  Robots and Systems(IROS). https://arxiv.org/abs/2110.04157 .

- @anchor Sherman2022 [Sherman 2022] Sherman, M. (2022, June). Rethinking
  Contact Simulation for Robot Manipulation. Blog post in Medium.
  https://medium.com/toyotaresearch/rethinking-contact-simulation-for-robot-manipulation-434a56b5ec88 .

 */
