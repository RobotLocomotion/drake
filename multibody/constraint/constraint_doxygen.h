/** @file
 Doxygen-only documentation for @ref constraint_overview */

/** @addtogroup constraint_overview

This documentation describes the types of multibody constraints supported in
Drake, including specialized constraint types- namely point-based contact
constraints that allow Drake's constraint solver to readily incorporate the
Coulomb friction model.

Drake's constraint system helps solve computational dynamics problems with
algebraic constraints, like differential algebraic equations:<pre>
ẋ = f(x, t, λ)
0 = g(x, t, λ)
</pre>
where `g()` corresponds to one or more constraint equations and λ
is typically interpretable as "forces" that enforce the constraints. Drake's
constraint system permits solving initial value problems subject to such
constraints and does so in a manner very different from "smoothing methods"
(also known as "penalty methods"). Smoothing methods have traditionally required
significant computation to maintain `g = 0` to high accuracy (and typically
yielding what is known in ODE and DAE literature as a "computationally stiff
system").

We also provide the core components of an efficient first-order integration
scheme for a system with both compliant and rigid unilateral constraints. Such
a system arises in many contact problems, where the correct modeling parameters
would yield a computationally stiff system. The integration scheme is
low-accuracy but very stable for mechanical systems, even when the algebraic
variables (i.e., the constraint forces) are computed to low accuracy.

This discussion will provide necessary background material in:

 - @ref constraint_types
 - @ref constraint_stabilization
 - @ref constraint_Jacobians
 - @ref contact_surface_constraints

and will delve into the constraint solver functionality in:

 - @ref discretization

 A prodigious number of variables will be referenced throughout the discussion
 on constraints. Variables common to both acceleration-level constraints and
 velocity-level constraints will be covered in @ref constraint_variable_defs.

 References for this discussion will be provided in @ref constraint_references.
*/

/** @defgroup constraint_variable_defs Variable definitions
@ingroup constraint_overview
- nb      The number of bilateral constraint equations (nb ≥ 0)
- nk      The number of edges in a polygonal approximation to a friction
          cone (nk ≥ 4 for contacts between three-dimensional bodies, nk = 2 for
          contacts between two-dimensional bodies). Note that nk = 2nr (where
          nr is defined immediately below).
- nr      *Half* the number of edges in a polygonal approximation to a
          friction cone. (nr ≥ 2 for contacts between three-dimensional bodies,
          nr = 1 for contacts between two-dimensional bodies).
- nc      The number of contact surface constraint equations.
- nck     The total number of edges in the polygonal approximations to the
          nc friction cones corresponding to the nc point contacts. Note that
          nck = 2ncr (where ncr is defined immediately below).
- ncr     *Half* the total number of edges in the polygonal approximations to the
          nc friction cones corresponding to the nc point contacts.
- nv      The dimension of the system generalized velocity / force.
- nq      The dimension of the system generalized coordinates.
- v       The system's generalized velocity vector (of dimension nv), which is a
          linear transformation of the time derivative of the system's
          generalized coordinates.
- q       The generalized coordinate vector of the system (of dimension nq).
- t       The system time variable (a non-negative real number).
- nu      The number of "generic" (non-contact related) unilateral constraint
          equations.
- α       A non-negative, real valued scalar used to correct the time derivative
          of position constraint errors (i.e., "stabilize" the constraints) via
          an error feedback process (Baumgarte Stabilization).
- β       A non-negative, real valued scalar used to correct position constraint
          errors via the same error feedback process (Baumgarte
          Stabilization) that uses α.
- γ       Non-negative, real valued scalar used to regularize constraints.
*/

/** @defgroup constraint_types Constraint types
@ingroup constraint_overview

Constraints can be categorized as either bilateral ("two-sided" constraints,
e.g., g(q) = 0) or unilateral ("one-sided" constraints, e.g., g(q) ≥ 0).
Although the former can be realized through the latter using a pair of
inequality constraints, g(q) ≥ 0 and -g(q) ≥ 0; the constraint problem structure
distinguishes the two types to maximize computational efficiency in the solution
algorithms. It is assumed throughout this documentation that g(.) is a vector
function. In general, the constraint functions do not maintain consistent units:
the units of the iᵗʰ constraint function are not necessarily equivalent to the
units of the jᵗʰ dimension of g(.).

Constraints may be posed at the position level:<pre>
gₚ(t;q)
</pre>
at the velocity level:<pre>
gᵥ(t,q;v)
</pre>
or at the acceleration level:<pre>
gₐ(t,q,v;v̇)
</pre>
Additionally, constraints posed at any level can impose constraints on the
constraint forces, λ. For example, the position-level constraint:<pre>
gₚ(t,q;λ)
</pre>
could be used to enforce q ≥ 0, λ ≥ 0, and qλ = 0, a triplet of conditions
useful for, e.g., ensuring that a joint range-of-motion limit is respected
(q ≥ 0), guaranteeing that no constraint force is applied if the constraint
is not active (q > 0 implies λ = 0), and guaranteeing that the constraint force
can only push the connected links apart at the joint rather than act to "glue"
them together (λ > 0).

Note the semicolon in all of the above definitions of g() separates general
constraint dependencies (e.g., t,q,v in gₐ) from variables that must be
determined using the constraints (e.g., v̇ in gₐ).

<h4>Constraints with velocity-level unknowns</h4>
This documentation distinguishes
between equations that are posed at the position level but are
differentiated once with respect to time (i.e., to reduce the Differential
Algebraic Equation or Differential Complementarity Problem index
@ref Ascher1998 "[Ascher 1998]"):
<pre>
d/dt gₚ(t; q) = ġₚ(t, q; v)
</pre>
vs. equations that **must** be posed at the velocity-level
(i.e., nonholonomic constraints):<pre>
gᵥ(t, q; v).
</pre>
Both cases yield a constraint with velocity-level unknowns, thereby
allowing the constraint to be used in a velocity-level constraint formulation
(e.g., an Index-2 DAE or Index-2 DCP). *Only the former constraint may "drift"
from zero over time*, due to truncation and discretization errors, unless
corrected. See @ref constraint_stabilization for further information.

<h4>Constraints with force and acceleration-level unknowns</h4>
A bilateral constraint equation with force and acceleration-level unknowns will
take the form:
<pre>
g̈ₚ(t,q,v;v̇) = 0
</pre>
if gₚ(.) has been differentiated twice with respect to time (for DAE/DCP index
reduction), or<pre>
ġᵥ(t,q,v;v̇) = 0
</pre>
if gᵥ(.) is nonholonomic and has been differentiated once with respect to time
(again, for DAE/DCP index reduction). As noted elsewhere, constraints can also
incorporate terms dependent on constraint forces, like:<pre>
g̈ₚ + λ = 0
</pre>
Making the constraint type above more general by permitting unknowns over λ
allows constraints to readily express, e.g., sliding Coulomb friction
constraints (of the form fₜ = μ⋅fₙ).

<h4>Complementarity conditions</h4>
Each unilateral constraint comprises a triplet of equations. For example:<pre>
gₐ(t,q,v;v̇,λ) ≥ 0
λ ≥ 0
gₐ(t,q,v;v̇,λ)⋅λ = 0
</pre>
which we will typically write in the common shorthand notation:<pre>
0 ≤ gₐ  ⊥  λ ≥ 0
</pre>
Interpreting this triplet of constraint equations, two conditions become
apparent: (1) when the constraint is inactive (gₐ > 0), the constraint force
must be zero (λ = 0) and (2) the constraint force can only act in one
direction (λ ≥ 0). This triplet is known as a *complementarity constraint*.
*/

/** @defgroup constraint_stabilization Constraint stabilization
@ingroup constraint_overview

Both truncation and rounding errors can prevent constraints from being
exactly satisfied. For example, consider the bilateral holonomic constraint
equation gₚ(t,q) = 0. Even if
gₚ(t₀,q(t₀)) = ġₚ(t₀,q(t₀),v(t₀)) = g̈ₚ(t₀,q(t₀),v(t₀),v̇(t₀)) = 0, it is often
true that gₚ(t₁,q(t₁)) will be nonzero for sufficiently large Δt = t₁ - t₀.
One way to address this constraint "drift" is through *dynamic stabilization*.
In particular, holonomic unilateral constraints that have been differentiated
twice with respect to time can be modified from:<pre>
0 ≤ g̈ₚ  ⊥  λ ≥ 0
</pre>
to:<pre>
0 ≤ g̈ₚ + 2αġₚ + β²gₚ  ⊥  λ ≥ 0
</pre>
and holonomic bilateral constraints can be modified from:<pre>
g̈ₚ = 0
</pre>
to:<pre>
g̈ₚ + 2αġₚ + β²gₚ = 0
</pre>
for non-negative scalar α and real β (α and β can also represent diagonal
matrices for greater generality). α and β, which both have units of 1/sec
(i.e., the reciprocal of unit time) are described more fully in
@ref Baumgarte1972 "[Baumgarte 1972]". The use of α and β above make correcting
position-level (gₚ) and velocity-level (gₚ̇) holonomic constraint errors
analogous to the dynamic problem of stabilizing a damped harmonic oscillator.
Given that analogy, 2α is the viscous damping coefficient and β² the stiffness
coefficient. These two coefficients make predicting the motion of the oscillator
challenging to interpret, so one typically converts them to undamped angular
frequency (ω₀) and damping ratio (ζ) via the following equations:<pre>
ω₀² = β²
ζ = α/β
</pre>

To eliminate constraint errors as quickly as possible, one strategy used in
commercial software uses ζ=1, implying *critical damping*, and undamped angular
frequency ω₀ that is high enough to correct errors rapidly but low enough to
avoid computational stiffness. Picking that parameter is considered to be more
art than science (see @ref Ascher1995 "[Ascher 1995]"). Given desired ω₀ and ζ,
α and β are set using the equations above.
*/

/** @defgroup constraint_Jacobians Constraint Jacobian matrices
@ingroup constraint_overview

Much of the problem data necessary to account for constraints in dynamical
systems refers to particular Jacobian matrices. These Jacobian matrices arise
through the time derivatives of the constraint equations, e.g.:<pre>
ġₚ = ∂gₚ/∂q⋅q̇
</pre>
where we assume that gₚ above is a function only of position (not time) for
simplicity. The constraint solver currently operates on generalized velocities,
requiring us to leverage the relationship:<pre>
q̇ = N(q)⋅v
</pre>
using the left-invertible matrix N(q) between the time derivative of generalized
coordinates and generalized velocities (see @ref quasi_coordinates). This yields
the requisite form:<pre>
ġₚ = ∂gₚ/∂q⋅N(q)⋅v
</pre>

Fortunately, adding new constraints defined in the form `gₚ(t,q)` does not
require considering this distinction using the operator paradigm (see, e.g.,
`N_mult` in @ref drake::multibody::constraint::ConstraintAccelProblemData
"ConstraintAccelProblemData" and
@ref drake::multibody::constraint::ConstraintVelProblemData
"ConstraintVelProblemData"). Since the Jacobians are described
completely by the equation `ġₚ = ∂gₚ/∂q⋅N(q)⋅v + ∂c/∂t`, one can simply
evaluate `ġₚ - ∂g/∂t` for a given `v`; no Jacobian matrix need be formed
explicitly.
*/

/** @defgroup contact_surface_constraints Contact surface constraints
@ingroup constraint_overview

Consider two points pᵢ and pⱼ on rigid bodies i and j, respectively, and
assume that at a certain configuration of the two bodies, ᶜq, the two
points are coincident at a single location in space,
p(ᶜq). To constrain the motion of pᵢ and pⱼ to the contact
surface as the bodies move, one can introduce the constraint
g(q) = n(q)ᵀ(pᵢ(q) - pⱼ(q)), where n(q) is the common surface normal
expressed in the world frame. gₚ(q) is a unilateral constraint, meaning that
complementarity constraints are necessary (see @ref constraint_types):<pre>
0 ≤ gₚ  ⊥  λ ≥ 0
</pre>

<h4>Differentiating gₚ(.) with respect to time</h4>
As usual, gₚ(.) must be differentiated (with respect to time) once to use the
contact constraint in velocity-level constraint formulation or twice to use
it in a force/acceleration-level formulation. Differentiating gₚ(q) once with
respect to time yields:<pre>
ġₚ(q,v) = nᵀ(ṗᵢ - ṗⱼ) + ṅᵀ(pᵢ - pⱼ);
</pre>
one more differentiation with respect to time yields:<pre>
g̈ₚ(q,v,v̇) = nᵀ(p̈ᵢ - p̈ⱼ) + 2ṅᵀ(ṗᵢ - ṗⱼ) + n̈ᵀ(pᵢ - pⱼ).
</pre>

The non-negativity condition on the constraint force magnitudes (λ ≥ 0)
keeps the contact force along the contact normal compressive, as is consistent
with a non-adhesive contact model.

A more substantial discussion on the kinematics of contact can be found in
@ref Pfeiffer1996 "[Pfeiffer 1996]", Ch. 4.

@image html drake/multibody/constraint/images/colliding-boxes.png "Figure 1: Illustration of the interpretation of non-interpenetration constraints when two boxes are interpenetrating (right). The boxes prior to contact are shown at left, and are shown in the middle figure at the initial time of contact; the surface normal n̂ is shown in this figure as well. The bodies interpenetrate over time as the constraint becomes violated (e.g., by constraint drift). Nevertheless, n̂ is tracked over time from its initial direction (and definition relative to the blue body). σ represents the signed distance the bodies must be translated along n̂ so that they are osculating (kissing)."

<h4>Constraint softening</h4>
As discussed in @ref discretization, the non-interpenetration
constraint can be softened by adding a term to the time derivative of the
equation:
<pre>
0 ≤ ġₚ  ⊥  λ ≥ 0
</pre>
resulting in:<pre>
0 ≤ ġₚ + γλ  ⊥  λ ≥ 0
</pre>
for γ ≥ 0.

<h4>Constraint stabilization</h4>
As discussed in @ref constraint_stabilization, the drift induced by solving an
Index-0 DAE or DCP (that has been reduced from an Index-3 DAE or DCP) can be
mitigated through one of several strategies, one of which is Baumgarte
Stabilization. The typical application of Baumgarte Stabilization will use the
second time derivative of the complementarity condition:<pre>
0 ≤ g̈ₚ  ⊥  λ ≥ 0
</pre>
Baumgarte Stabilization would be layered on top of this equation, resulting in:
<pre>
0 ≤ g̈ₚ + 2αġₚ + β²gₚ  ⊥  λ ≥ 0
</pre>

<h4>Effects of constraint softening and stabilization on
constraint problem data</h4>

  - Constraint regularization/softening is effected through `gammaN`
  - Constraint stabilization is effected through `kN`
  - Note that *neither Baumgarte Stabilization nor constraint regularization/
    softening affects the definition of g(.)'s Jacobian* operators, `N_mult` and
    either `N_minus_muQ_transpose_mult` (for
    @ref drake::multibody::constraint::ConstraintAccelProblemData
    "ConstraintAccelProblemData") or `N_transpose_mult` (for
    @ref drake::multibody::constraint::ConstraintVelProblemData
    "ConstraintVelProblemData")
*/

/** @defgroup discretization A stable discretization strategy
@ingroup constraint_overview

To be written. Refer to
https://github.com/RobotLocomotion/drake/pull/7055
for a preview.
*/

/** @defgroup constraint_references References
 @ingroup constraint_overview

 Sources referenced within the multibody constraint documentation.

 - @anchor Anitescu1997 [Anitescu 1997]  M. Anitescu and F. Potra. Formulating
   Dynamic Multi-Rigid-Body Contact Problems with Friction as Solvable Linear
   Complementarity Problems. Nonlinear Dynamics, 14, pp. 231-247. 1997.
 - @anchor Ascher1995 [Ascher 1995]  U. Ascher, H. Chin, L. Petzold, and
   S. Reich. Stabilization of constrained mechanical systems with DAEs and
   invariant manifolds. J. Mech. Struct. Machines, 23, pp. 135-158. 1995.
 - @anchor Ascher1998 [Ascher 1998]  U. Ascher and L. Petzold. Computer Methods
   for Ordinary Differential Equations and Differential Algebraic Equations.
   SIAM, Philadelphia. 1998.
 - @anchor Baumgarte1972 [Baumgarte 1972]  J. Baumgarte. Stabilization of
   constraints and integrals of motion in dynamical systems. Comp. Math. Appl.
   Mech. Engr., 1, pp. 1-16. 1972.
 - @anchor Catto2011 [Catto 2011]  E. Catto. Soft Constraints: Reinventing the
   Spring. Game Developers Conference presentation, 2011.
 - @anchor Cottle1992 [Cottle 1992]  R. Cottle, J-S. Pang, and R. Stone. The
   Linear Complementarity Problem. Academic Press, Boston. 1992.
 - @anchor Hairer1996 [Hairer 1996]  E. Hairer and G. Wanner. Solving ordinary
   differential equations II: stiff and differential algebraic problems, 2nd ed.
   Springer-Verlag, Berlin. 1996.
 - @anchor Lacoursiere2007 [Lacoursiere 2007]  C. Lacoursière. Ghosts and
   Machines: Regularized Variational Methods for Interactive Simulations of
   Multibodies with Dry Frictional Contacts. Umeå University. 2007.
 - @anchor Pfeiffer1996 [Pfeiffer 1996]  F. Pfeiffer and C. Glocker. Multibody
   Dynamics with Unilateral Contacts, John Wiley & Sons, New York. 1996.
 - @anchor Sciavicco2000 [Sciavicco 2000]  L. Sciavicco and B. Siciliano.
   Modeling and Control of Robot Manipulators, 2nd ed. Springer-Verlag, London.
   2000.
 */
