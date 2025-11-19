/** @file
 Doxygen-only documentation for @ref drake_contacts.  */

// clang-format off (to preserve link to images)

/** @addtogroup hydro_params Estimation of Hydroelastic Parameters
@{
@brief  <a><!-- no brief line please --></a>

Similarly to Hertz theory of contact mechanics, in this section we derive
analytical formulas to estimate the elastic force that establishes between two
bodies in contact. As in Hertz theory, we assume small deformations, allowing
us to introduce geometrical approximations.

The next section presents a table of analytical formulas to estimate the contact
force between two compliant bodies given their geometry and elastic moduli. A
Drake user can use these formulas to estimate hydroelastic parameters for their
particular model. Below we provide a @ref hydro_params_study_case "Study Case"
to show with an example how to estimate hydroelastic moduli using these
formulas.

Those readers interested on the theory behind, can keep reading further into the
sections that follow.

@section hydro_params_formulas Analytical Formulas

In @ref hydro_params_analytical we show that a hydroelastic object of modulus E
behaves as an @ref hydro_params_efm "Elastic Foundation"  of depth H
(established by the particular geometry). At small penetrations, the
hydroelastic contact force fₙ between two compliant objects can be estimated
from
<pre>
  fₙ = κ⋅V,
</pre>
where `V` is the overlap volume or volume of intersection and κ = κᵃ⋅κᵇ/(κᵃ+κᵇ)
is the effective stiffness, with κᵃ = Eᵃ/Hᵃ and κᵇ = Eᵇ/Hᵇ the stiffness of
bodies A and B respectively.

The table below summarizes analytical formulas to compute the contact force
between a body of a given geometric shape with a half-space. These formulas are
obtained by substituting an estimate of the intersection volume as a function of
a small interpenetration distance `x`, see column "Overlap Volume V" in the
table below. Column "Foundation Depth H" shows the value used by Drake given its
choice of hydroelastic pressure field (where pressure equals E at the point of
maximum signed distance). For a conic indenter of radius R and apex angle θ, the
signed distance is maximum at the incenter of its triangular cross section (an
isosceles triangle of base 2R and height R/tan(θ)). With a pressure field linear
with the signed distance and with maximum value E at the incenter, the
foundation depth equals the radius of the incircle.

Geometry           |                        Foundation Depth H                       |      Overlap Volume V       | Contact Force fₙ
-------------------|:---------------------------------------------------------------:|:---------------------------:|:------------------------
Cylindrical punchᵃ |                                R                                |@f$\pi R^2 x@f$              |@f$\pi E R x@f$
Sphereᵇ            |                                R                                |@f$\pi R x^2@f$              |@f$\pi E x^2@f$
Conic indenterᶜ    | @f$R\left(\frac{1-\sin(\theta)}{1+\sin(\theta)}\right)^{1/2}@f$ |@f$\pi/3\tan^2(\theta)x^3@f$ |@f$\frac{\pi}{3}\frac{E}{R}x^3\tan^2(\theta)\left(\frac{1+\sin(\theta)}{1-\sin(\theta)}\right)^{1/2}@f$

ᵃ Compliant cylinder of radius R and length L > R. Rigid half-space.

ᵇ Compliant sphere of radius R. Rigid half-space.

ᶜ Rigid conic indenter of radius R and apex angle θ. Compliant half-space. The
  foundation depth H equals the radius of the incircle of its triangular cross
  section.

One final observation. As with Hertz contact theory, the contact force between
two compliant spheres A and B can be computed by considering the contact
between a sphere of effective radius R = Rᵃ⋅Rᵇ/(Rᵃ+Rᵇ) and a half-space.

@subsection hydro_params_study_case Study Case: Manipuland resting on a table

We estimate the penetration `x` for a typical household object of mass 1 kg
(9.81 N). We make this estimation for two extreme geometric cases: a cylinder, a
blunt object generating a flat contact surface, and a sphere, a smooth object
generating a smoother (spherical) contact surface.

The table below summarizes penetration `x` in millimeters for different values
of hydroelastic modulus E.

  E [Pa]  |   Sphere   |  Cylinder
----------|:----------:|:-----------
  10⁶     | 1.767      | 6.2 × 10⁻²
  10⁷     | 0.559      | 6.2 × 10⁻³
  10⁸     | 0.177      | 6.2 × 10⁻⁴
  10⁹     | 0.056      | 6.2 × 10⁻⁵

While the Hydroelastic modulus is not the Young's modulus, we can still use
values of Young's modulus as a reference. For instance, steel has a Young's
modulus of about E ≈ 200 GPa (2 × 10¹¹ Pa), while E ≈ 1 GPa would correspond to
a soft wood.

For simulation of rigid (stiff in reality) objects, the choice of hydroelastic
modulus is a tradeoff between an _acceptable_ amount of penetration and
numerical conditioning. Good numerical conditioning translates in practice to
better performance and robustness. Very large values of hydroelastic modulus can
degrade numerical conditioning and therefore users must choose a value that is
acceptable for their application. Experience shows that for modeling "rigid"
(once again, stiff) objects, there is not much gain in accurately resolving
penetrations below the submillimeter range (~10⁻⁴ m). Therefore moduli in the
order of 10⁷-10⁸ Pa will be more than enough, with no good practical reason to
use larger values.

Notice that for the very typical flat contact case (a mug, a plate, the foot of
a robot), the cylinder estimation with E = 10⁷ Pa leads to penetrations well
under submillimeter scales. We find this value a good starting point for most
models. Most often, users won't use a larger value but will want instead to
lower this value to model rubber padded surfaces (robotic feet or grippers).

@section hydro_params_efm Winkler's Elastic Foundation

Before describing our derivation, we first need to understand how Hydroelastic
Contact relates Winkler's Elastic Foundation Model (EFM),
@ref Johnson1987 "[Johnson, 1987; §4.3]". This will allows us to make a
connection to previous work by @ref Gonthier2007 "[Gonthier, 2007; §4.3]", who
derived a model of compliant contact with EFM as the starting point.

EFM provides a simple approximation for the contact pressure distribution. The
key idea is to model a compliant layer or _foundation_ of thickness H over a
rigid core, see Fig. 1. This compliant layer can be seen as a bed of linear
springs that gets pushed when a second object comes into contact. Integrating
the effect of all springs over the contact area produces the net force among the
two contacting bodies.

@image html drake/multibody/plant/images/elastic_foundation.svg "Figure 1: Elastic Foundation Model." width=35%

Denoting with `ϕ(r)` the penetrated distance at a location `r`, Fig. 1, EFM
models the pressure due to contact at `r` as
<pre>
  p(r) = E⋅ϕ(r)/H,                                                          (1)
</pre>
where H is the _elastic foundation_ depth (the thickness of the compliant layer)
and `E` is the elastic modulus. We use E for the elastic modulus in analogy to
Young's modulus from elasticity theory (we do the same for Hydroelastic
contact). Keep in mind that these models are different and their moduli
parameters are not expected to match, though we often use Young's modulus as a
starting point for estimation. Though an approximation of reality, the model (1)
has the nice property that is fully algebraic, not requiring the solution of
complex integral equations as with Hertz contact theory.

@section hydro_params_efm_analogy Hydroelastic Contact and Elastic Foundation

Similarly to EFM, the @ref hydro_contact "Hydroelastic Contact Model" also
proposes an approximation of the contact pressure. In fact, we can see
hydroelastic contact as a modern rendition of EFM in which the pressures
generated by (1) are precomputed into a _hydroelastic pressure field_, see @ref
hydro_contact. These precomputed pressure fields are then used at runtime for an
efficient computation of contact surface and forces.

Designing the hydroelastic pressure field allows for some flexibility, but the
EFM approximation (1) is typically used, especially in Drake. This choice is
supported by extensive literature and practical experience with EFM.
Additionally, the coarse meshes used to discretize hydroelastic geometries yield
only linear approximations, making a pressure field linear in the distance
function a natural choice.

While this connection to EFM is strong, Hydroelastic contact presents several
advantages over traditional EFM
  - Hydroelastic contact generalizes to arbitrary non-convex geometry
  - Large "deformations" (interpenetration) are allowed
  - Efficient algorithm to compute continuous contact patches
  - Continuously differentiable contact forces

@section hydro_params_analytical Analytical Solutions

Given the clear connection between hydroelastic contact and EFM, we use this
analogy to derive an analytical formula to estimate contact forces. More
precisely, we follow the work by @ref Gonthier2007 "[Gonthier, 2007; §4.3]", who
by using the approximation in (1) derives analytical formula for the computation
of contact forces between two compliant bodies A and B, Fig. 2.

@image html drake/multibody/plant/images/hydro_analytic.svg "Figure 2: Hydroelastic contact. Signed distances and balance of pressure." width=35%

@subsection hydro_params_gonthier Gonthier's derivation

At small deformations, @ref Gonthier2007 "[Gonthier, 2007]" assumes a planar
contact surface. This assumption allows to compute the elastic forces on each
body separately, as if they interacted with a rigid separating plane. The true
contact force is obtained by making both forces equal. Using this assumption and
the EFM model (1),
@ref Gonthier2007 "[Gonthier, 2007]" finds a generic expression for the elastic
force pushing bodies A and B. The result is remarkably simple and beautiful,
only involving the overlapping volume of the original undeformed geometries
<pre>
  fₙ = κ⋅V,                                                                 (2)
</pre>
where κ = κᵃ⋅κᵇ/(κᵃ+κᵇ) is the effective stiffness and κᵃ = Eᵃ/Hᵃ and κᵇ = Eᵇ/Hᵇ
are the stiffness of bodies A and B respectively. V is the volume the original
geometries would overlap if they did not deform.

@subsection hydro_params_analytical Alternative derivation

We present an alternative derivation that more closely resembles the
Hydroelastic contact model, where the balance of normal stresses (pressure)
determines the location of the contact surface. Still, we find that this
derivation leads to exactly the same result in (2).

At small deformations, Fig. 2, we approximate the distance between the two
bodies as
<pre>
  ϕ(r) = ϕᵃ(r) + ϕᵇ(r),                                                     (3)
</pre>

We use the EFM approximation (1) to model the normal stress (pressure) that
results from deforming each body A and B amounts ϕᵃ(r) and ϕᵇ(r) respectively.
With this, the balance of pressure at each point on the contact surface is
<pre>
  p(r) = pᵃ(r) = pᵇ(r), or
  p(r) = κᵃ⋅ϕᵃ(r) = κᵇ⋅ϕᵇ(r).                                               (4)
</pre>

We can solve for the contact pressure p(r) from (3) and (4) as
<pre>
  p(r) = κ⋅ϕ(r),                                                            (5)
</pre>
with the same stiffness κ found by @ref Gonthier2007 "[Gonthier, 2007]"'s
alternative method in (2).

The total contact force is found by integrating (5) over the contact surface
<pre>
  fₙ(r) = ∫p(r)d²r = κ⋅∫ϕ(r)d²r.                                            (6)
</pre>

Given the small deformations assumption we see in Fig. 2 that this last integral
can be approximated as the volume of intersection between the two undeformed
geometries, leading to the same result found by
@ref Gonthier2007 "[Gonthier, 2007]" in Eq. (2).
@}
*/
