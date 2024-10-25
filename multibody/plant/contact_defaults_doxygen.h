/** @file
 Doxygen-only documentation for @ref drake_contacts.  */

// clang-format off (to preserve link to images)

/** @addtogroup contact_defaults Default Contact Parameters

In Drake all contacts are compliant. However, while true "rigid constraints" are
not supported, our solvers can handle stiffness values far beyond those of real
physical materials @ref Castro2023 "[Castro et al., 2023]". Still, such high
stiffness values can lead to numerical ill conditioning and are thus not
recommended. 

Many who belong to the "rigid contact" school of thought might see this as a
modeling weakness. Consider that, after all, "rigid contact" is only an
approximation of reality. True physical objects are not rigid, though they might
be very stiff.

Working with compliant objects does have a practical weakness; we must provide a
value of compliance for each object in our simulation. Experience shows however
that for very stiff objects, increasing the stiffness leads to negligible
changes in behavior past a certain (large) value. Moreover, there is a practical
limit --- very large values of stiffness can lead to numerical conditioning, bad
performance, and even solver failure.

In practice, the modeling of stiff bodies (as an approximation to "rigid
bodies") boils down to establishing an amount of penetration that is enough as
an approximation of our real objects but that requires stiffness values that can
be safely handled by our solvers. In @ref Castro2023 "[Castro et al., 2023]" we
found that often times we can use stiffness values several orders of magnitude
larger than the stiffness of a stiff material such as steel. However in practice
using a stiffness two and up to three orders of magnitude lower leads to a good
approximation of rigid bodies that can be handled very robustly without
incurring on numerical problems.


@section foo1 First Foo

In @ref hydro_params_analytical we show that a hydroelastic object of modulus E
behaves as an @ref hydro_params_efm "Elastic Foundation"  of depth H
(established by the particular geometry). At small penetrations, the
hydroelastic contact force fₙ between two compliant objects can be estimated
from
<pre>
  fₙ = κ⋅V,
</pre>

@subsection foo2 Second Foo

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

  */
