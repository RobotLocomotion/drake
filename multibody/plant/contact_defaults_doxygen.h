/** @file
 Doxygen-only documentation for @ref drake_contacts.  */

// clang-format off (to preserve link to images)

/** @addtogroup contact_defaults Default Contact Parameters
@{
@brief  <a><!-- no brief line please --></a>

In Drake, @ref why_rigid "contacts are modeled as compliant". While some may
view the absence of rigid contact modeling as a limitation, it's essential to
understand that rigid contact is itself an approximation; real physical objects
are not truly rigid but simply very stiff. Compliant contact offers an excellent
approximation of rigid contact while often providing better numerical stability
@ref Castro2023 "[Castro et al., 2023]".

Although our solvers can handle stiffness values far exceeding those of real
materials @ref Castro2023 "[Castro et al., 2023]", excessively high stiffness
can lead to numerical ill-conditioning, degraded performance, or even solver
failure. This underscores the importance of carefully selecting contact
parameters—particularly stiffness—to balance robustness and accuracy in
simulations.

To address this, Drake provides default contact parameters specifically designed
to approximate rigid contact while maintaining numerical stability. Below, we
summarize the key parameters most relevant to stiffness and numerical
performance:

|Property [units]                  |  Units | Default value |
|:---------------------------------|:------:|:-------------:|
|Point stiffness                   |   N/m  | @ref drake::geometry::DefaultProximityProperties::point_stiffness "Defined here"           |
|Hydroelastic modulus              |   Pa   | @ref drake::geometry::DefaultProximityProperties::hydroelastic_modulus "Defined here"      |
|Hunt & Crossley dissipation       |   s/m  | @ref drake::geometry::DefaultProximityProperties::hunt_crossley_dissipation "Defined here" |
|Friction                          |    -   | @ref drake::geometry::DefaultProximityProperties::dynamic_friction "Defined here"          |
|Stiction tolerance                |   m/s  | @ref drake::multibody::MultibodyPlantConfig::stiction_tolerance "Defined here"             |
|Marginᵃ                           |   m    | @ref drake::geometry::DefaultProximityProperties::margin "Defined here"                    |

ᵃ Margin has no default yet, this is the recommended value. Refer to
@ref margin_how_much for details.

Users can change these defaults in
drake::geometry::SceneGraphConfig::default_proximity_properties, with the
exception being the stiction tolerance, defined in
drake::multibody::MultibodyPlantConfig.

<h3>Margin</h3>

Though margin and stiction tolerance are not physical parameters, we include
them here since its a set of parameters users should be aware of. For most cases
users will not need to change them. Refer to
@ref hydro_margin for a thorough discussion of margin and to
@ref margin_how_much for a discussion on estimation and recommended values.

<h3>Stiction tolerance</h3>

The stiction tolerance parameterizes our model of regularized friction, see
@ref Castro2023 "[Castro et al., 2023]" for details. While a smaller value
produces a tighter approximation of stiction, it can lead to numerical stiffness
and ill conditioning. The recommended default works for most robotics
applications including the modeling of challenging manipulation tasks, without
sacrificing performance.

<h3>Friction</h3>

We like to think that friction coefficients can generally be categorized into
three ranges: less than 0.2 for slippery surfaces, 0.2 - 0.4 for moderately
smooth surfaces, and greater than 0.5 for rough surfaces. While there is no
theoretical upper limit to the friction coefficient, we'll rarely need a value
larger than 1.0, a good estimate to model very rough or rubber-like surfaces. We
summarize our guideline as follows:
 1. <b>\< 0.2</b> for slippery surfaces,
 2. <b>0.2-0.4</b> for medium-smooth surfaces,
 3. <b>\>0.5</b> for rough surfaces.

<h3>Contact dissipation</h3>

The Hunt & Crossley dissipation parameter is theoretically linked to energy
dissipation during impact and the coefficient of restitution,
@ref HuntCrossley1975 "[Hunt and Crossley 1975]". Practically, we estimate this
parameter using a simple guideline: the inverse of the dissipation parameter, d,
is the maximum bounce speed after contact. For instance, to limit the bounce
speed to 0.1 m/s, set d = 10 s/m.

In robotics, bouncing is often undesirable, and hardware typically favors
inelastic contact. While purely inelastic contact requires d = ∞, values around
40 - 50 s/m effectively approximate this behavior in most applications.

@section Proposed Modeling Workflow

When authoring a new model, this is the workflow we encourage:

1. Author a first pass of your model without specifying contact parameters.
   Drake will default to parameters in drake::geometry::SceneGraphConfig.
2. Simulate your model and verify the behavior. For many cases, this will be
   enough (e.g. manipulands, anchored bodies, robots, etc.)
3. Adjust model-specific parameters. For instance, rubber pads in a gripper or
   robotic feet are better modeled using lower contact stiffnesses. A good way
   to estimate stiffness is from known values of deformation (penetration) for a
   given force (say weight or gripper effort limit).
4. Friction coefficients might need to be adjusted for modeling smoother
   surfaces
5. Iterate between 2 and 4 to better match your application.

For completeness, the next section shows how default contact parameters are
estimated. These notes might be useful for the customized parameter
estimations in your models.

@section contact_defaults_estimating Estimating Contact Stiffness

This section shows how we can estimate stiffness. These guidelines are used to
provide Drake's default values above, but you can also use them to estimate
custom values for your application.

Experience shows that for very stiff objects, increasing material stiffness
beyond a certain large value results in negligible changes in behavior. For
example, when modeling a household object like a mug, the stiffness of diamond,
steel, or wood often yields indistinguishable results for most applications.
However, extremely high stiffness values can cause numerical issues such as
ill-conditioning, poor performance, or even solver failure. As a result,
selecting stiffness to approximate rigid objects involves balancing model
accuracy with numerical stability.

Drake’s default material stiffness values are chosen to provide a practical
approximation of rigid contact. Rigidity is measured based on an acceptable
level of interpenetration, with reasonable bounds set to balance precision and
performance. For instance, there is no benefit in limiting interpenetration to
atomic scales, as it exceeds practical measurement capabilities and
significantly impacts solver performance.

We consider both @ref compliant_point_contact "compliant point contact" and
@ref hydro_contact "hydroelastic contact". For these estimates, we consider a
half-space in contact with a sphere of radius R and water's density, and a
cylinder of radius R and length 4/3⋅R so that it has the same mass as the
sphere.

Algebraic formulas for the contact force and volume in these configurations is
given below:

|  Geometry  |     Volume V     |  Point Contact |  Hydroelastic Contact
|:-----------|:----------------:|:- ------------:|:----------------------
|  Cylinder  |   V = 4/3⋅π⋅R³   |    fₙ = k⋅x    |   fₙ=π⋅E⋅R⋅x
|  Sphere    |   V = 4/3⋅π⋅R³   |    fₙ = k⋅x    |   fₙ=π⋅E⋅x²

The volume (and mass) of these objects is the same since the length of the
cylinder is 4/3⋅R. For the point contact model, the contact force is independent
of contact area and it is always linear with penetration depth x. For
hydroelastic contact, the area of the contact patch is considered and thus the
force is linear with penetration for the cylinder (constant area at small
penetrations) and quadratic for the sphere, refer to the section on @ref
hydro_params_formulas "analytical formulas for hydroelastic" for derivations and
details.

Therefore, for these objects in contact with a flat surface, the expected amount
of penetration under the influence of their own weight is:

|  Geometry  |     Point Contact      |   Hydroelastic Contact
|:-----------|:----------------------:|:------------------------
|  Cylinder  |    x = 4/3⋅π⋅ρ⋅g/k⋅R³  |   x = (4/3⋅ρ⋅g/E)  ⋅ R²
|  Sphere    |    x = 4/3⋅π⋅ρ⋅g/k⋅R³  |   x = (4/3⋅ρ⋅g/E)¹⸍²⋅R³⸍²

where g is the acceleration of gravity.

As examples, we consider an object of radius 0.05 m, a typical household object
of about half a kilogram, and an object of radius 0.30 m, that with the density
of water, has about 110 kilograms (the mass of a typical humanoid robot).

For point contact, the table below shows the amount of penetration (in meters)
for point contact stiffness k = 10⁶ N/m (Drake's default) and k = 10⁷ N/m (in
parentheses):

|  Geometry  |     R = 0.05 m      |      R = 0.3 m
|:-----------|:-------------------:|:---------------------
|  Cylinder  | 5.1⋅10⁻⁶ (5.1⋅10⁻⁷) |  1.1⋅10⁻³ (1.1⋅10⁻⁴)
|  Sphere    | 5.1⋅10⁻⁶ (5.1⋅10⁻⁷) |  1.1⋅10⁻³ (1.1⋅10⁻⁴)

For hydroelastic contact, the table below shows the amount of penetration (in
meters) for hydroelastic modulus E = 10⁷ Pa (Drake's default) and E = 10⁸ Pa (in
parentheses):

|  Geometry  |      R = 0.05 m      |       R = 0.3 m
|:-----------|:--------------------:|:---------------------
|  Cylinder  |  3.3⋅10⁻⁶ (3.3⋅10⁻⁷) |   1.2⋅10⁻⁴ (1.2⋅10⁻⁵)
|  Sphere    |  4.0⋅10⁻⁴ (1.3⋅10⁻⁴) |   5.9⋅10⁻³ (1.9⋅10⁻³)

From these numerical values we see that Drake's defaults are chosen so that
penetrations are a few submillimeters for household objects and in the
millimeters range for large mobile robots the size of a humanoid. Stiffer values
can be used, but this set of parameters is a good starting point that
effectively trades off a practical approximation of rigid contact with numerical
stiffness.
@}
*/
