/** @defgroup constraint_overview Multibody dynamics constraints in Drake

This documentation describes the types of multibody constraints supported in
Drake, including specialized constraint types- namely point-based contact
constraints- which allow Drake's constraint solver to readily incorporate the
Coulomb friction model.

Drake allows constraints to be imposed at both the acceleration-level- for when
constraint forces are non-impulsive- and at the velocity-level, for admitting
impulsive forces. The latter formulation is useful for both modeling impacts
and implementing "time stepping" discretizations of the continuous time
dynamics. We caution the reader that these distinctions are orthogonal to
whether a constraint is holonomic or nonholonomic. This issue will be discussed
further in @ref constraint_types.

 This discussion will encompass:
 - @ref constraint_types
 - @ref constraint_stabilization
 - @ref constraint_regularization
 - @ref constraint_Jacobians
 - @ref contact_surface_constraints
 - @ref frictional_constraints
 - @ref generic_bilateral
 - @ref generic_unilateral

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
- γ,ε     Non-negative, real valued scalars used to regularize constraints.
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
or at the force/acceleration level:<pre>
gₐ(t,q,v;v̇,λ)
</pre>
where λ is a vector of *constraint-space forces*. Note the semicolon in these
definitions separates general constraint dependencies (t,q,v in gₐ) from
variables that must be determined using the constraints (v̇,λ in gₐ). *The three
constraint equations listed above can then be categorized as having
position-level unknowns, velocity-level unknowns, or force/acceleration-level
unknowns*, respectively.

<h4>Constraints with velocity-level unknowns</h4>
This documentation distinguishes
between equations that are posed at the position level but are
differentiated once with respect to time (i.e., to reduce the Differential
Algebraic Equation or Differential Complementarity Problem index [Ascher 1998]):
<pre>
d/dt gₚ(t; q) = ġₚ(t, q; v)
</pre>
vs. equations that **must** be posed at at the velocity-level
(i.e., nonholonomic constraints):<pre>
gᵥ(t, q; v).
</pre>
Both cases yield a constraint with velocity-level unknowns, thereby
allowing the constraint to be used in a velocity-level constraint formulation
(e.g., an Index-2 DAE or Index-2 DCP). *Only the former constraint may "drift"
from zero over time*, due to truncation and discretization errors, unless
corrected. See @ref constraint_stabilization for further information.

<h4>Constraints with force/acceleration-level unknowns</h4>
A bilateral constraint equation with force/acceleration-level unknowns will take
the form:
<pre>
g̈ₚ(t,q,v;v̇) = 0
</pre>
if gₚ(.) has been differentiated twice with respect to time (for DAE/DCP index
reduction), or<pre>
ġᵥ(t,q,v;v̇) = 0
</pre>
if gᵥ(.) is nonholonomic and has been differentiated once with respect to time
(again, for DAE/DCP index reduction). Constraints with acceleration-level
unknowns can be also incorporate terms dependent on constraint forces,
like:<pre>
g̈ₚ + λ = 0
</pre>
Making the constraint type above more general by permitting unknowns over λ
allows constraints to readily express, e.g., sliding Coulomb friction
constraints (of the form fₜ = μ⋅fₙ). It also admits more sophisticated
constraints that are functions of both v̇ and λ (simultaneously), like the
illustrative example above, which will be examined further in
@ref constraint_regularization.

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
[Baumgarte 1972]. The use of α and β above make correcting position-level
(gₚ) and velocity-level (gₚ̇) holonomic constraint errors analogous to the dynamic
problem of stabilizing a damped harmonic oscillator. Given that analogy, 2α is
the viscous damping coefficient and β² the stiffness coefficient. These two
coefficients make predicting the motion of the oscillator challenging to
interpret, so one typically converts them to undamped angular frequency (ω₀) 
and damping ratio (ζ) via the following equations:<pre>
ω₀² = β²
ζ = α/β
</pre>

To eliminate constraint errors as quickly as possible, one will typically use
ζ=1, implying *critical damping*, and undamped angular frequency ω₀ that is
high enough to correct errors rapidly but low enough to avoid computational
stiffness. Picking that parameter is currently considered to be more art
than science (see [Ascher 1995]). Given desired ω₀ and ζ, α and β are set using
the equations above.
*/

/** @defgroup constraint_regularization Constraint regularization and softening
@ingroup constraint_overview

It can be both numerically advantageous and a desirable modeling feature to
regularize constraints. For example, consider modifying a 
unilateral complementarity constraint from:<pre>
0 ≤ gₐ(t,q,v;v̇,λ)  ⊥  λ ≥ 0
</pre>
to:<pre>
0 ≤ gₐ(t,q,v;v̇,λ) + ελ  ⊥  λ ≥ 0
</pre>
where ε is a non-negative scalar; alternatively, it can represent a diagonal
matrix with the same number of rows/columns as the dimension of gₐ and λ,
permitting different coefficients for each constraint equation.
With ελ > 0, it becomes easier to satisfy the constraint
gₐ(t,q,v;v̇,λ) + ελ ≥ 0, though the resulting v̇ and λ may not quite
satisfy gₐ ≥ 0: gₐ can become slightly negative. As hinted above,
regularization can confer two benefits. First, the resulting complementarity
problems become easier to solve; more importantly, any complementarity problem
becomes solvable given sufficient regularization [Cottle 1992].

<h4>Regularization introduces compliance</h4>
Second (in concert with constraint stabilization and a particular discretization
of the constrained multibody dynamics equations), regularization introduces
compliant effects, e.g., at joint stops and between contacting bodies; such
"softening" effects are often desirable.

Unfortunately, not all constraints can be intuitively softened. While
[Lacoursiere 2007] convincingly argues for softening interpenetration
constraints between rigid bodies (and even provides an algorithmic mechanism
for doing so), users typically expect stiction and Coulomb friction constraints
to be maintained to high accuracy. Without softening *all* constraints, the
regularization benefits noted above disappear.

<h4>Regularizing at the velocity level</h4>
[Catto 2011] showed how the combination of constraint softening, constraint
stabilization, and a particular integration scheme results in numerically
stable spring-like constraints. For a one-dimensional particle with dynamics
defined by: 
<pre>
mẍ = f
</pre>
The "hard constraint" x = 0 can be added, which Catto differentiates once with
respect to time, resulting in the equations:
<pre>
mẍ = f + λ
ẋ = 0
</pre>
This hard constraint can be regularized and stabilized using γ, which takes the
place of ε but provides identical functionality (albeit not identical units),
resulting in:
<pre>
mẍ = f + λ
ẋ + xϱ/h + γλ = 0
</pre>
where `h` will be used for discretization (see immediately below) and ϱ ∈ [0,1].
[Catto 2011] showed that a particular discretization of this system yields the
dynamics of a harmonic oscillator by solving the following system of equations
for ẋ(t+h), x(t+h), and λ (thereby yielding an integration scheme).
<pre>
ẋ(t+h) = ẋ(t) + hf/m + hλ/m
ẋ(t+h) + x(t)ϱ/h + γλ = 0
x(t+h) = x(t) + hẋ(t+h)
</pre>
γ and ϱ can be selected to effect the desired undamping angular frequency and
damping ratio (a process also described in @ref constraint_stabilization) using
the formula:<pre>
γ = 1 / (2m̂ζω + hm̂ω²)
ϱ = hm̂ω²γ
</pre>
where m̂ is the *effective inertia* of the constraint and is determined
by 1/(GM⁻¹Gᵀ), where G = ∂c/∂q̅, G ∈ ℝ¹ˣⁿ is the partial derivative of the
constraint function with respect to the quasi-coordinates (see
@ref quasi_coordinates; equivalently, G maps generalized velocities to the time
derivative of the constraints, i.e., ġₚ) and M is the generalized inertia
matrix. Considering gₚ(.) to be a scalar function for simplicity of presentation
should make it clear that GM⁻¹Gᵀ would be a scalar as well.

While Catto studied a mass-spring system, these results apply to general
multibody systems as well, as discussed in [Lacoursiere 2007]. Implementing a
time stepping scheme in Drake using
@ref drake::multibody::constraint::ConstraintSolver "ConstraintSolver", one
would use ω and ζ to correspondingly set
@ref drake::multibody::constraint::ConstraintVelProblemData::gammaN "gammaN" (or
@ref drake::multibody::constraint::ConstraintVelProblemData::gammaL "gammaL")
to γ and
@ref drake::multibody::constraint::ConstraintVelProblemData::kN "kN"
(@ref drake::multibody::constraint::ConstraintVelProblemData::kL "kL") to
ϱ/h times the signed constraint distance (using, e.g., signed distance for the
point contact non-interpenetration constraint).

<h4>Regularization at the acceleration-level</h4>
Starting from the same stabilized and regularized spring mass system:
<pre>
mẍ = f + λ
ẍ + ελ = 0
</pre>
the softening benefits are not realized, but the linear equations and linear
complementarity problems become easier to solve (albeit at the expense of
larger constraint errors to be stabilized).

<h4>Bilateral constraints</h4>
Drake does not regularize bilateral constraints.
*/

/** @defgroup constraint_Jacobians Constraint Jacobian matrices
@ingroup constraint_overview

Much of the problem data necessary to account for constraints in dynamical
systems refers to particular Jacobian matrices. If the time derivatives of the 
system's generalized coordinates are equal to the system's generalized
velocities (i.e., if q̇ = v), these Jacobian matrices
can be defined simply as the partial derivatives of the constraint equations
taken with respect to the partial derivatives of the generalized
coordinates (i.e., ∂g/∂q).

The time derivative of the generalized coordinates need not equal the
generalized velocities, however,
which leads to a general, albeit harder to describe definition of the
Jacobian matrices as the partial derivatives of the constraint equations
taken with respect to the quasi-coordinates (see @ref quasi_coordinates);
using the notation within the citation for
quasi-coordinates means we write the Jacobian as ∂g/∂q̅ (quasi-coordinates
possess the property that ∂q̅/∂v = Iₙₓₙ, the n × n identity matrix). In
robotics literature, ∂g/∂q̅ is known as a *geometric Jacobian* while
∂g/∂q is known as an *analytical Jacobian* [Sciavicco 2000].

Fortunately, for constraints defined in the form gₚ(t,q), the distinction is
moot: the Jacobians are described completely by the equation ġₚ = ∂g/∂q̅⋅v +
∂c/∂t, where v are the generalized velocities of the system. Since the problem
data calls for operators (see
@ref drake::multibody::constraint::ConstraintAccelProblemData
"ConstraintAccelProblemData" and
@ref drake::multibody::constraint::ConstraintVelProblemData
"ConstraintVelProblemData" that compute (∂g/∂q̅⋅v), one can simply
evaluate ġ - ∂g/∂t for a given v: no Jacobian need be formed explicitly.
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

@image html multibody/constraint/images/surface_contact.png "Figure 1: Illustration of the interpretation of contact surface constraints when two spheres are interpenetrating. The contact surface, normal, and signed distance are determined by mapping the undeformed intersecting configurations to the minimum-potential-energy deformed 'kissing' configuration."

<h4>Constraint regularization and softening</h4>
As discussed in @ref constraint_regularization, the contact surface
constraint can be regularized or softened by adding a term to, e.g., the
second time derivative of the equation:
<pre>
0 ≤ g̈ₚ  ⊥  λ ≥ 0
</pre>
resulting in:<pre>
0 ≤ g̈ₚ + ελ  ⊥  λ ≥ 0
</pre>
for ε ≥ 0.

<h4>Constraint stabilization</h4>
As discussed in @ref constraint_stabilization, the drift induced by solving an
Index-0 DAE or DCP (that has been reduced from an Index-3 DAE or DCP) can be
mitigated through one of several strategies, one of which is Baumgarte
Stabilization. The typical application of Baumgarte Stabilization will use the
second time derivative of the complementarity condition (the regularization
term from above is incorporated as well for purposes of illustration):<pre>
0 ≤ g̈ₚ + ελ  ⊥  λ ≥ 0
</pre>
Baumgarte Stabilization would be layered on top of this equation, resulting in:
<pre>
0 ≤ g̈ₚ + 2αġₚ + β²gₚ + ελ  ⊥  λ ≥ 0
</pre>

<h4>Effects of constraint regularization/softening and stabilization on
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
 
/** @defgroup frictional_constraints Frictional constraints
@ingroup constraint_overview
The frictional aspects of the contact model used by Drake's constraint solver
are modeled by the Coulomb friction model, which requires setting few
parameters (sticking and sliding friction coefficients) and captures important
stick-slip transition phenomena.

<h4>The Coulomb friction model (for sliding)</h4>
Incorporating the Coulomb friction model for sliding contact between dry
surfaces introduces the constraint equation:<pre>
μλᴺ = λᶜ
</pre>
where λᶜ is the force due to Coulomb friction, μ is the (dimensionless)
Coulomb coefficient of friction, and λᴺ corresponds to the constraint force
applied along the surface normal. The frictional force λᶜ is applied against the
direction of sliding between two bodies. When there is no sliding, this equation
does not apply; those cases can be treated using special equations for stiction
(see the next section), by applying no force at all, or by using a
"regularized" Coulomb friction model (using e.g., the hyperbolic tangent
function).

Drake's constraint solver solves for λᶜ by substituting it with μλᴺ (i.e., the
coefficient of friction times the force applied along the surface normal) in the
multi-body dynamics equations. As a concrete example, consider
a system consisting for two bodies sliding at a single point of contact with
surface normal (n̂) and sliding direction q̂. If all other (non-contact related)
forces are captured by the generalized force f, the dynamics of that system
could be written
as:<pre>
Mv̇ = f + Nᵀλᴺ - (Qᵀ)λᶜ
</pre>
where M is the generalized inertia matrix, Nᵀ is the wrench on the bodies that
results from applying a unit force along n̂ to the bodies at the point of
contact, and Qᵀ is the wrench on the bodies that results from applying a unit
force along q̂. Instead, Drake uses the equivalent equation:
<pre>
Mv̇ = f + (Nᵀ - μQᵀ)λᴺ
</pre>
which allows the constraint solver to avoid explicitly solving for λᶜ.
This optimization explains the provenance of `N_minus_muQ_transpose_mult` (see
@ref drake::multibody::constraint::ConstraintAccelProblemData
"ConstraintAccelProblemData").

Note that *this aspect of the Coulomb friction model does not apply in
velocity-level constraint equations*, in which contacts can transition from
sliding to sticking (and vice versa) instantaneously.
<h4>Non-sliding constraints</h4>
Non-sliding constraints for bodies can correspond to constraints introduced
for bodies in stiction or rolling at a point of contact. These
particular constraints act to minimize the acceleration, to the extent
permitted by the contact normal force and friction coefficient, in the 
contact tangent plane (i.e., plane passing through the point of contact and
with normal parallel to the contact surface normal.

The bodies are, by definition, not sliding at a point, a condition described
by the following pair of constraints:<pre>
((ṗᵢ - ṗⱼ)ᵀbₛ) = 0
((ṗᵢ - ṗⱼ)ᵀbₜ) = 0
</pre>
where bₛ and bₜ are basis vectors in ℝ³ that span the contact tangent
plane and pᵢ, pⱼ ∈ ℝ³ represent a point of contact between bodies i and j.

The non-sliding constraints can be categorized into kinematic constraints on
tangential motion and frictional force constraints. Such a grouping for a 3D
contact is provided below:<pre>
(0) ⁰ĝₐ = μ⋅λᴺ - ||λˢ λᵗ|| ≥ 0
(1) ¹ĝₐ = ((p̈ᵢ - p̈ⱼ)ᵀbₛ) = 0
(2) ²ĝₐ = ((p̈ᵢ - p̈ⱼ)ᵀbₜ) = 0
</pre>
where μ is the coefficient of "static" friction, λᴺ is the magnitude of the
force applied along the contact normal, and λˢ and λᵗ are scalars
corresponding to the frictional forces applied along the basis vectors.
Since nonlinear equations are typically challenging to solve, the ĝ equations
are often transformed to linear approximations:<pre>
(0')      ⁰g̅ₐ = μ⋅λᴺ - 1ᵀλᵇ
(1')      ¹g̅ₐ = (p̈ᵢ - p̈ⱼ)ᵀb₁
...
(nr')     ⁿʳg̅ₐ = (p̈ᵢ - p̈ⱼ)ᵀbₙᵣ
(nr+1') ⁿʳ⁺¹g̅ = -(p̈ᵢ - p̈ⱼ)ᵀb₁
...
(nk')    ⁿᵏg̅ₐ = -(p̈ᵢ - p̈ⱼ)ᵀbₙᵣ
</pre>
where b₁,...,bₙᵣ ∈ ℝ³ (nk = 2nr) are a set of spanning vectors in the contact
tangent plane (the more vectors, the better the approximation to the
nonlinear friction cone), and λᵇ ≥ 0 (which will conveniently allow us
to pose these constraints within the linear complementarity problem
framework), where λᵇ ∈ ℝⁿᵏ are non-negative scalars that represent the
frictional force along the spanning vectors and the negated spanning
vectors. Equations 0'-nk' cannot generally be satisfied simultaneously:
maximizing λᵇ (i.e., λᵇ = μ⋅λᴺ) may be insufficient to keep the relative
tangential acceleration at the point of contact zero. Accordingly, we
transform Equations 0'-nk' to the following:<pre>
(0*)      ⁰gₐ = μ⋅λᴺ - 1ᵀλᵇ
(1*)      ¹gₐ = (p̈ᵢ - p̈ⱼ)ᵀb₁ - Λ
...
(nr*)     ⁿʳgₐ = (p̈ᵢ - p̈ⱼ)ᵀbₙᵣ - Λ
(nr+1*) ⁿʳ⁺¹gₐ = -(p̈ᵢ - p̈ⱼ)ᵀb₁ - Λ
...
(nk*)     ⁿᵏgₐ = -(p̈ᵢ - p̈ⱼ)ᵀbₙᵣ - Λ
</pre>
which lead to the following complementarity conditions:<pre>
0 ≤ ⁰gₐ     ⊥      Λ ≥ 0
0 ≤ ¹gₐ     ⊥    λᵇ₁ ≥ 0
...
0 ≤ ⁿʳgₐ    ⊥    λᵇₙᵣ ≥ 0
0 ≤ ⁿʳ⁺¹gₐ  ⊥  λᵇₙᵣ₊₁ ≥ 0
0 ≤ ⁿᵏgₐ    ⊥    λᵇₙₖ ≥ 0
</pre>
where Λ is roughly interpretable as the remaining tangential acceleration
at the contact after constraint forces have been applied.  From this
construction, the frictional force to be applied along direction
i will be equal to λᵇᵢ - λᵇₙᵣ₊ᵢ.

<h4>Friction constraints at the velocity-level</h4>
Since modifying velocity variables can cause constraints to change from sticking
and sliding and vice versa, Drake's velocity-level formulation treats both
sliding and non-sliding contact constraints indistinguishably using a
modification to the treatment above (the sliding Coulomb friction constraints
μλᴺ = λᶜ do not apply in this formulation). The constraints now act to maximize
negative work in the contact tangent plane [Anitescu 1997]. Reflecting this
change at the velocity level, Equations (0*)-(nk*) are modified to:
<pre>
(0⁺)       ⁰gᵥ = μ⋅λᴺ - 1ᵀλᵇ
(1⁺)       ¹gᵥ = (ṗᵢ - ṗⱼ)ᵀb₁ - Λ
...
(nr⁺)     ⁿʳgᵥ = (ṗᵢ - ṗⱼ)ᵀbᵣ - Λ
(nr+1⁺) ⁿʳ⁺¹gᵥ = -(ṗᵢ - ṗⱼ)ᵀb₁ - Λ
...
(nk⁺)     ⁿᵏgᵥ = -(ṗᵢ - ṗⱼ)ᵀbₙᵣ - Λ
</pre>
λᴺ and λᵇ now reflect impulsive forces, and, similarly, Λ now corresponds to
residual tangential velocity post-impact (i.e., after those impulsive forces
are applied). The remainder of the discussion in the previous section, apart
from these modifications to the constraints, still applies.

<h4>Effects of constraint regularization/softening and stabilization on
constraint problem data</h4>

It is expected both that users will not wish to soften the frictional
constraints and that constraint drift will remain sufficiently small that
constraint stabilization will not be required. Nevertheless, Drake provides
infrastructure that permits using these capabilities:

  - Regularization/softening of the no-slip constraint is effected through
    `gammaF`
  - Regularization/softening of the linearized version of the Coulomb friction
    relationship constraint (i.e., μ⋅λᴺ = 1ᵀλᵇ) is effected through
    `gammaE`. A positive value of `gammaE` corresponds to permitting the
    linearized frictional force to lie outside of the friction cone
    (μ⋅λᴺ < 1ᵀλᵇ); a zero value of `gammaE` requires the linearized
    frictional force force to lie inside or on the friction cone.
  - Constraint stabilization of the no-slip constraint is effected through `kF`
  - Note that *neither Baumgarte Stabilization nor constraint regularization/
    softening affects the definition of g(.)'s Jacobian* operators, `F_mult` and
    `F_transpose_mult`.
*/

/** @defgroup generic_bilateral Generic bilateral constraints
@ingroup constraint_overview

The bilateral constraint functions supported includes, among others, holonomic
constraints, which are constraints posable as gₚ(t, q). An example such
holonomic constraint function is the transmission (gearing) constraint:<pre>
qᵢ - rqⱼ = 0
</pre> where
gₚ(q) = qᵢ - rqⱼ
</pre>
and where `r` is the gear ratio (for simplicity, this equation does not
incorporate a constant angular offset between the rotational joints).
The first derivative of this function with respect to time:<pre>
ġₚ(q;v) = q̇ᵢ - rq̇ⱼ
</pre>
would allow this constraint to be used with a velocity-level constraint
formulation and can be read as the velocity at joint i (q̇ᵢ) must equal `r`
times the velocity at joint j (q̇ⱼ). The second derivative of gₚ(q) with respect
to time,<pre>
g̈ₚ(q,v;v̇) = q̈ᵢ - rq̈ⱼ
</pre>
would allow this constraint to be used with an acceleration-level constraint
formulation. For this simple transmission constraint example, it will generally
be the case that q̇ᵢ = vᵢ and q̇ⱼ = vⱼ.
*/ 

/** @defgroup generic_unilateral Generic unilateral constraints
@ingroup constraint_overview

The unilateral constraint functions supported includes, among others,
holonomic constraints, which are constraints posable as gₚ(t, q). An example
such unilateral holonomic constraint function is a joint range-of-motion
limit:<pre>
0 ≤ gₚ(q)  ⊥  λᵢ ≥ 0
</pre> where
gₚ(q) = qᵢ.
</pre>
This limit range of motion limit requires joint qᵢ to be non-negative.
The force limit (λᵢ ≥ 0) requires the applied force to also be non-negative
(i.e., it acts against any force that would lead to make gₚ(q)
negative). And, the complementarity constraint g(q)λᵢ = 0 implies that
force can only be applied when the joint is at the limit.

The first derivative of this function with respect to time:<pre>
ġₚ(q;v) = q̇ᵢ
</pre>
yields the complementarity condition:<pre>
0 ≤ ġₚ(q)  ⊥  λᵢ ≥ 0
</pre>
and would allow this constraint to be used with a velocity-level constraint
formulation. The second derivative of gₚ(q) with respect to
time,<pre>
g̈ₚ(q,v;v̇) = q̈ᵢ
</pre>
would allow this constraint to be used with an acceleration-level constraint
formulation and yields the complementarity condition:<pre>
0 ≤ g̈ₚ(q,v;v̇)  ⊥  λᵢ ≥ 0
</pre>
For this simple range of motion constraint example, it will generally be
the case that q̇ᵢ = vᵢ.

<h4>Constraint regularization and softening</h4>
As discussed in @ref constraint_regularization, generic unilateral
constraints can be regularized or softened by adding a term to, e.g., the
second time derivative of the equation:
<pre>
0 ≤ g̈ₚ  ⊥  λᵢ ≥ 0
</pre>
resulting in:<pre>
0 ≤ g̈ₚ + ελᵢ  ⊥  λᵢ ≥ 0
</pre>
for ε ≥ 0. This same concept applies equally well to the first time derivative
of gₚ(.), as also discussed in @ref constraint_regularization.

<h4>Constraint stabilization</h4>
As discussed in @ref constraint_stabilization, the drift induced by solving an
Index-0 DAE or DCP (that has been reduced from an Index-3 DAE or DCP) can be
mitigated through one of several strategies, one of which is Baumgarte
Stabilization. The typical application of Baumgarte Stabilization will use the
second time derivative of the complementarity condition (the regularization
term from above is incorporated as well for purposes of illustration):<pre>
0 ≤ g̈ₚ + ελᵢ  ⊥  λᵢ ≥ 0
</pre>
Baumgarte Stabilization would be layered on top of this equation, resulting in:
<pre>
0 ≤ g̈ₚ + 2αġₚ + β²gₚ + ελᵢ  ⊥  λᵢ ≥ 0
</pre>

<h4>Effects of constraint regularization/softening and stabilization on
constraint problem data</h4>

  - Constraint regularization/softening is effected through `gammaL`
  - Constraint stabilization is effected through `kL`
  - Note that *neither Baumgarte Stabilization nor constraint regularization/
    softening affects the definition of gₚ(.)'s Jacobian* operators, `L_mult`
    and `L_transpose_mult`.
*/

/** @defgroup constraint_references References
 @ingroup constraint_overview

 Sources referenced within the multibody constraint documentation.

 - [Anitescu 1997]  M. Anitescu and F. Potra. Formulating Dynamic Multi-Rigid-
   Body Contact Problems with Friction as Solvable Linear Complementarity
   Problems. Nonlinear Dynamics, 14, pp. 231-247. 1997.
 - [Ascher 1995]  U. Ascher, H. Chin, L. Petzold, and S. Reich. Stabilization
   of constrained mechanical systems with DAEs and invariant manifolds. J.
   Mech. Struct. Machines, 23, pp. 135-158. 1995.
 - [Ascher 1998]  U. Ascher and L. Petzold. Computer Methods for Ordinary
   Differential Equations and Differential Algebraic Equations. SIAM,
   Philadelphia. 1998.
 - [Baumgarte 1972]  J. Baumgarte. Stabilization of constraints and integrals of
   motion in dynamical systems. Comp. Math. Appl. Mech. Engr., 1, pp. 1-16.
   1972.
 - [Catto 2011]  E. Catto. Soft Constraints: Reinventing the Spring.
   Game Developers Conference presentation, 2011.
 - [Cottle 1992]  R. Cottle, J-S. Pang, and R. Stone. The Linear Complementarity
   Problem. Academic Press, Boston. 1992.
 - [Hairer 1996]  E. Hairer and G. Wanner. Solving ordinary differential
   equations II: stiff and differential algebraic problems, 2nd ed.
   Springer-Verlag, Berlin. 1996.
 - [Lacoursiere 2007]  C. Lacoursière. Ghosts and Machines: Regularized
   Variational Methods for Interactive Simulations of Multibodies with Dry
   Frictional Contacts. Umeå University. 2007.
 - [Sciavicco 2000]  L. Sciavicco and B. Siciliano. Modeling and Control of
   Robot Manipulators, 2nd ed. Springer-Verlag, London. 2000.
 */
