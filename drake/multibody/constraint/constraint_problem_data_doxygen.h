/** @defgroup constraint_overview Constraints in Drake 

This documentation describes the types of constraints supported in Drake,
including specialized constraint types- namely point-based contact constraints-
which allow Drake's constraint solver to readily incorporate the Coulomb
friction model.

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
 - @ref constraint_softening
 - @ref constraint_Jacobians
 - @ref noninterpenetration_constraints
 - @ref frictional_constraints
 - @ref generic_bilateral
 - @ref generic_unilateral

 A prodigious number of variables will be referenced throughout the discussion
 on constraints. Variables common to both acceleration-level constraints and
 velocity-level constraints will be covered in @ref constraint_variable_defs. 
*/
 
/** @defgroup constraint_variable_defs Variable definitions
@ingroup constraint_overview
- b ∈ ℕ   The number of bilateral constraint equations.
- k ∈ ℕ   The number of edges in a polygonal approximation to a friction
           cone. Note that k = 2r.
- p ∈ ℕ   The number of non-interpenetration constraint equations
- q ∈ ℝⁿ' The generalized coordinate vector of the system. n' is at least
           as large as n.
- n ∈ ℕ   The dimension of the system generalized velocity / force.
- n' ∈ ℕ  The dimension of the system generalized coordinates.
- r ∈ ℕ   *Half* the number of edges in a polygonal approximation to a
          friction cone. Note that k = 2r.
- t ∈ ℝ   The system time variable (t ≥ 0).
- u ∈ ℕ   The number of "generic" (non-contact related) unilateral
          constraint equations.
- v ∈ ℝⁿ  The generalized velocity vector of the system, which is equivalent
          to the time derivative of the system quasi-coordinates.
- α ∈ ℝ   A non-negative scalar used to correct velocity-level constraint
          errors (i.e., "stabilize" the velocity constraints) via an error
          feedback process (Baumgarte Stabilization).
- β ∈ ℝ   A non-negative scalar used to correct position-level constraint
          errors via the same error feedback process (Baumgarte
          Stabilization) that uses α.
- γ ∈ ℝ   A non-negative scalar used to soften an otherwise perfectly
          "rigid" constraint.
*/

/** @defgroup constraint_types Constraint types
@ingroup constraint_overview

Constraints can be categorized as either bilateral or unilateral, which
roughly can be translated to equality (e.g., c(q) = 0) or inequality
constraints (e.g., c(q) ≥ 0). The former can be realized through the
latter using a pair of inequality constraints, c(q) ≥ 0 and -c(q) ≥ 0;
the constraint problem structure distinguishes the two types to maximize
computational efficiency in the solution algorithms. It is assumed throughout
this documentation that c() is a vector function. In general, the constraint
functions do not maintain consistent units: the units of the iᵗʰ dimension of
c() are not necessarily equivalent to the units of the jᵗʰ dimension of c().

Constraints may be posed at the position level:<pre>
c(t;q) → ℝᵐ
</pre>
at the velocity level:<pre>
c(t,q;v) → ℝᵐ
</pre>
or at the acceleration level:<pre>
c(t,q,v;v̇,λ) → ℝᵐ
</pre>
where λ ∈ ℝᵐ are *constraint-space forces*. Note the semicolon in these
definitions, which separates general constraint dependencies (q,v,t) from
variables that must be determined using the constraints (v̇,λ). *The three
constraint equations listed above can then
be categorized as having position-level unknowns, velocity-level unknowns,
or acceleration-level unknowns*, respectively.

<h4>Constraints with velocity-level unknowns</h4>
This document and class does not generally attempt (or need) to distinguish
between equations that are posable at the position level but are
differentiated once with respect to time (i.e., holonomic constraints):<pre>
d/dt c(t; q) = ċ(t, q; v)
</pre>
vs. equations that **must** be posed at at the velocity-level
(i.e., nonholonomic constraints):<pre>
c(t, q; v).
</pre>
Both cases yield a constraint with velocity-level unknowns usable for a
velocity-level constraint formulation. The only case where the distinction
is important in practice is in constraint stabilization: constraint errors
at the position level can be corrected only when the constraint is posed
at the position level (this property follows from the non-integrability of
nonholonomic constraints). See @ref constraint_stabilization.

Constraints with velocity-level unknowns can also include impulsive
constraint-space forces; the function definition would appear as c(t, q; v, λ) 
or ċ(t, q; v, λ). We do not use notation in this documentation to distinguish
between impulsive and non-impulsive constraint forces. 

<h4>Constraints with acceleration-level unknowns</h4>
A bilateral constraint equation with acceleration-level unknowns will take the
form:
<pre>
c̈(t,q,v;v̇,λ) = 0
</pre>
if c() is holonomic,<pre>
ċ(t,q,v;v̇,λ) = 0
</pre>
if c() is nonholonomic, or- generically:
<pre>
c(t,q,v;v̇,λ) = 0
</pre>
Similarly, unilateral constraints with acceleration-level variables may also be
differentiated with respect to time once, twice, or not at all.

<h4>Complementarity conditions</h4>
Each unilateral constraint comprises a triplet of equations. For example:<pre>
c(t,q,v;v̇,λ) ≥ 0
λ ≥ 0
c(t,q,v;v̇,λ)⋅λ = 0
</pre>
which we will typically write in the common shorthand notation:<pre>
0 ≤ c  ⊥  λ ≥ 0
</pre>
Interpreting this triplet of constraint equations, two conditions become
apparent: (1) when the constraint is inactive (c > 0), the constraint force
must be zero (λ = 0) and (2) the constraint force can only act in one
direction (λ ≥ 0). This triplet is known as a *complementarity constraint*.
*/

/** @defgroup constraint_stabilization Constraint stabilization
@ingroup constraint_overview
 
Both truncation and rounding errors can prevent constraints from being 
exactly satisfied. For example, consider the bilateral holonomic constraint
equation c(t,q) = 0. Even if
c(t₀,q(t₀)) = ċ(t₀,q(t₀),v(t₀)) = c̈(t₀,q(t₀),v(t₀),v̇(t₀)) = 0, c(t₁,q(t₁))
is unlikely to be zero for sufficiently large Δt = t₁ - t₀. Consequently,
we can modify holonomic unilateral constraints to:<pre>
0 ≤ c̈ + 2αċ + β²c  ⊥  λ ≥ 0
</pre>
and holonomic bilateral constraints to:<pre>
c̈ + 2αċ + β²c = 0
</pre>
for non-negative scalar α and real β (α and β can also represent diagonal
matrices for greater generality). α and β, which both have units of 1/sec
(i.e., the reciprocal of unit time) are described more fully in
[Baumgarte 1972].

We will temporarily consider the case c → ℝ, i.e., c() maps to a scalar rather
than a vector. The use of α and β above make correcting position-level (c) and
velocity-level (ċ) holonomic constraint errors analogous to the dynamic problem
of stabilizing a damped harmonic oscillator. Given that analogy, 2α is the
viscous damping coefficient and β² the stiffness coefficient. These two
coefficients make predicting the motion of the oscillator challenging to
interpret, so one typically converts them to undamped angular frequency (ω₀) 
and damping ratio (ζ) via the following equations:<pre>
ω₀² = m̂⁻¹β²
ζ² = m̂⁻¹α²/β²
</pre>
where m̂⁻¹ is the *inverse effective inertia* of the constraint and is determined
by GM⁻¹Gᵀ, where G ≡ ∂c/∂q̅, G ∈ ℝ¹ˣⁿ is the partial derivative of the constraint
function with respect to the quasi-coordinates (see @ref quasi_coordinates;
equivalently, G maps generalized velocities to the time derivative of the
constraints, i.e., ċ) and M is the generalized inertia matrix. Note that 
GM⁻¹Gᵀ is a scalar since c() maps to a scalar.

To eliminate constraint errors as quickly as possible, one will typically use
ζ=1, implying *critical damping*, and an undamped angular frequency that is
high enough to correct errors rapidly but low enough to avoid computational
stiffness. Picking that parameter is currently considered to be more art
than science (see [Ascher 1992]).
*/

/** @defgroup constraint_softening Constraint softening
@ingroup constraint_overview

It can be both numerically advantageous and a desirable modeling feature to
soften otherwise "rigid" constraints. For example, consider modifying a 
unilateral complementarity constraint to:<pre>
0 ≤ c(t,q,v;v̇,λ) + γλ  ⊥  λ ≥ 0
</pre>
where γ is a non-negative scalar; alternatively, it can represent a diagonal
matrix with the same number of rows/columns as the dimension of c and λ,
permitting different coefficients for each constraint equation.
With γλ > 0, it becomes easier to satisfy the constraint
c(t,q,v;v̇,λ) + γλ ≥ 0, though the resulting v̇ and λ will not quite
satisfy c ≥ 0: c will become slightly negative. As hinted above,
softening grants two benefits. First, the complementarity problems resulting
from the softened unilateral constraints are regularized, and any
complementarity problem becomes solvable given sufficient regularization
[Cottle 1992]; more softening results in greater regularization. 

<h4>Softening introduces compliance</h4>
Second (in concert with constraint stabilization and a particular discretization
of the constrained multibody dynamics equations), constraint
softening introduces compliant effects, e.g., at joint stops and
between contacting bodies; such effects are often desirable.

Unfortunately, not all constraints can be intuitively softened. While
[Lacoursiere 2007] convincingly argues for softening interpenetration
constraints between rigid bodies (and even provides an algorithmic mechanism
for doing so), users typically expect stiction and Coulomb friction constraints
to be maintained to high tolerances. Without softening *all* constraints, the
regularization benefits noted above disappear.

<h4>Softening at the velocity level</h4>
[Catto 2011] showed how the combination of constraint softening, constraint
stabilization, and a particular integration scheme results in numerically
stable spring-like constraints. For a one-dimensional particle with dynamics
defined by: 
<pre>
mẍ = f
</pre>
The "hard constraint" ẍ = 0 can be added, resulting in: 
<pre>
mẍ = f + λ
ẍ = 0
</pre>
This hard constraint can be softened and stabilized, resulting in:
<pre>
mẍ = f + λ
ẍ + 2αẋ + β²x + γλ = 0
</pre>
[Catto 2011] showed that the solution to this system yields the dynamics
of a harmonic oscillator by solving the following system of equations for
ẋ(t+h), x(t+h), and λ (thereby yielding an integration scheme).
<pre>
ẋ(t+h) = ẋ(t) + hf/m + hλ/m
ẋ(t+h) + x(t)ν/h + γλ = 0
x(t+h) = x(t) + hẋ(t+h)
</pre>
β and α can be selected to effect the desired undamping angular frequency
and damping ratio as described in @ref constraint_stabilization, and 
γ and ν are then determined using the formula:
<pre>
γ = 1/(2α + hβ²) 
ν = hβ²/(2α + hβ²)
</pre>
While Catto studied a mass-spring system, these results apply to general
multibody systems as well, as discussed in [Lacoursiere 2007]. The most
interesting part of this formulation is that γ acts as both a numerical
(regularization) parameter and as a physical modeling parameter. 

By using ConstraintSolver to help implement a time stepping scheme in Drake,
one would use α and β to correspondingly set gammaN to γ and kN to ν/h times the
signed distance (using, e.g., the point contact non-interpenetration constraint
as the motivating example).

<h4>Softening at the acceleration-level</h4>
Starting from the same stabilized and softened spring mass system:
<pre>
mẍ = f + λ + 2αẋ + β²x
ẍ + γλ = 0
</pre>
γ now becomes strictly a regularization parameter: larger values make
linear equations and linear complementarity problems easier to solve but yield
larger constraint errors to be stabilized. α and β are no longer be interpreted
as dynamic (stiffness and damping) parameters, and now are viewed as
constraint stabilization parameters.

<h4>Bilateral constraints</h4>
Note that Drake does not soften bilateral constraints.
*/

/** @defgroup constraint_Jacobians Constraint Jacobian matrices
@ingroup constraint_overview

Much of the problem data necessary to account for constraints in dynamical
systems refers to particular Jacobian matrices. If the time derivatives of the 
system's generalized coordinates are equal to the system's generalized
velocities (i.e., if q̇ = v), these Jacobian matrices
can be defined simply as the partial derivatives of the constraint equations
taken with respect to the partial derivatives of the generalized
coordinates (i.e., ∂c/∂q). 

The time derivative of the generalized coordinates need not equal the
generalized velocities, however,
which leads to a general, albeit harder to describe definition of the
Jacobian matrices as the partial derivatives of the constraint equations
taken with respect to the quasi-coordinates (see @ref quasi_coordinates);
using the notation within the citation for
quasi-coordinates means we write the Jacobian as ∂c/∂q̅ (quasi-coordinates
possess the property that ∂q̅/∂v = Iₙₓₙ, the n × n identity matrix).

Fortunately, for constraints defined strictly in the form c(q), the
Jacobians are described completely by the equation ċ = ∂c/∂q̅⋅v, where v are
the generalized velocities of the system. Since the problem data
specifically requires operators (see ConstraintAccelProblemData and
ConstraintVelProblemData) that compute (∂c/∂q̅⋅v), one can simply
evaluate ċ for a given v: no Jacobian need be formed explicitly.
*/

/** @defgroup noninterpenetration_constraints Noninterpenetration constraints
@ingroup constraint_overview

Consider two points pᵢ and pⱼ on rigid bodies i and j, respectively, and
assume that at a certain configuration of the two bodies, ᶜq, the two
points are coincident at a single location in space,
p(ᶜq). To constrain the motion of pᵢ and pⱼ to the contact
surface as the bodies move, one can introduce the constraint
c(q) ≡ n(q)ᵀ(pᵢ(q) - pⱼ(q)), where n(q) is the common surface normal
expressed in the world frame. c(q) is a unilateral constraint, meaning that
complementarity constraints are necessary (see @ref constraint_types):<pre>
0 ≤ c  ⊥  λ ≥ 0
</pre>

<h4>Differentiating c() with respect to time</h4>
As usual, c() must be differentiated (with respect to time) once to use the
contact constraint in velocity-level constraint formulation or twice to use
it in an acceleration-level formulation. 
Differentiating c(q) once with respect to
time yields:<pre>
ċ(q,v) ≡ nᵀ(ṗᵢ - ṗⱼ) + ṅᵀ(pᵢ - pⱼ);
</pre>
one more differentiation with respect to time yields:<pre>
c̈(q,v,v̇) ≡ nᵀ(p̈ᵢ - p̈ⱼ) + ṅᵀ(ṗᵢ - ṗⱼ) + n̈ᵀ(pᵢ - pⱼ).
</pre>

The non-negativity condition on the constraint force magnitudes (λ ≥ 0)
keeps the contact force along the contact normal compressive, as desired.

<h4>The Coulomb friction model</h4>
Incorporating the Coulomb friction model for contact between dry surfaces
introduces a new constraint equation:<pre>
μλ = ᶜλ 
</pre>
where ᶜλ is the force due to Coulomb friction, μ is the (dimensionless)
Coulomb coefficient of friction, and λ retains its previous definition.
The frictional force ᶜλ is applied against the direction of sliding between
two bodies. When there is no sliding, Coulomb friction does not apply; those
cases can be treated using special equations for stiction (as Drake's
constraint problem data does, see @ref frictional_constraints), by applying no
force at all, or by using a "regularized" Coulomb friction model (using e.g.,
the hyperbolic tangent function). 

Drake's constraint solver solves for ᶜλ using substitution: ᶜλ is substituted
with μλ in the multi-body dynamics equations. As a concrete example, consider
a system consisting for two bodies sliding at a single point of contact with
surface normal (n̂) and sliding direction q̂. If all other (non-contact related)
forces are captured by the generalized force f, the dynamics of that system
*could* be written
as:<pre>
Mv̇ = f + Nᵀλ - (Qᵀ)ᶜλ
</pre>
where M is the generalized inertia matrix. Instead, Drake uses the equivalent
equation:
<pre>
Mv̇ = f + (Nᵀ - μQᵀ)λ
</pre>
Where Nᵀ is the wrench on the bodies that results from applying a unit force
along n̂ to the bodies at the point of contact and Qᵀ is the wrench on the bodies
that results from applying a unit force along q̂. 

Note that *the Coulomb friction model is not applied in velocity-level
constraint equations* (see @ref frictional_constraints).
*/
 
/** @defgroup frictional_constraints Frictional constraints
@ingroup constraint_overview

<i>This section applies to both non-sliding contacts for constraints solved
at the acceleration-level and **all contacts** for constraints solved at the
velocity level.</i> 

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
(0) ⁰g ≡ μ⋅fᴺ - ||fˢ fᵗ|| ≥ 0
(1) ¹g ≡ ((p̈ᵢ - p̈ⱼ)ᵀbₛ) = 0
(2) ²g ≡ ((p̈ᵢ - p̈ⱼ)ᵀbₜ) = 0
</pre>
where μ is the coefficient of "static" friction, fᴺ is the magnitude of the
force applied along the contact normal, and fˢ and fᵗ are scalars
corresponding to the frictional forces applied along the basis vectors.
Since nonlinear equations are typically challenging to solve, ⁰g and ¹g
are often transformed to linear approximations:<pre>
(0')     g̅₀ ≡ μ⋅fᴺ - 1ᵀfᵇ
(1')     g̅₁ ≡ (p̈ᵢ - p̈ⱼ)ᵀb₁
...
(r')     g̅ᵣ ≡ (p̈ᵢ - p̈ⱼ)ᵀbᵣ
(r+1') g̅ᵣ₊₁ ≡ -(p̈ᵢ - p̈ⱼ)ᵀb₁
...
(k')     g̅k ≡ -(p̈ᵢ - p̈ⱼ)ᵀbᵣ
</pre>
where b₁,...,bᵣ ∈ ℝ³ (k = 2r) are a set of spanning vectors in the contact
tangent plane (the more vectors, the better the approximation to the
nonlinear friction cone), and fᵇ ≥ 0 (which will conveniently allow us
to pose these constraints within the linear complementarity problem
framework), where fᵇ ∈ ℝᵏ are non-negative scalars that represent the
frictional force along the spanning vectors and the negated spanning
vectors. Equations 0'-k' cannot generally be satisfied simultaneously:
maximizing fᵇ (i.e., fᵇ = μ⋅fᴺ) may be insufficient to keep the relative
tangential acceleration at the point of contact zero. Accordingly, we
transform Equations 0'-k' to the following:<pre>
(0*)     c₀ ≡ μ⋅fᴺ - 1ᵀfᵇ
(1*)     c₁ ≡ (p̈ᵢ - p̈ⱼ)ᵀb₁ - Λ
...
(r*)     cᵣ ≡ (p̈ᵢ - p̈ⱼ)ᵀbᵣ - Λ
(r+1*) cᵣ₊₁ ≡ -(p̈ᵢ - p̈ⱼ)ᵀb₁ - Λ
...
(k*)     cₖ ≡ -(p̈ᵢ - p̈ⱼ)ᵀbᵣ - Λ
</pre>
which lead to the following complementarity conditions:<pre>
0 ≤ c₀    ⊥      Λ ≥ 0
0 ≤ c₁    ⊥    fᵇ₁ ≥ 0
...
0 ≤ cᵣ    ⊥    fᵇᵣ ≥ 0
0 ≤ cᵣ₊₁  ⊥  fᵇᵣ₊₁ ≥ 0
0 ≤ cₖ    ⊥    fᵇₖ ≥ 0
</pre>
where Λ is roughly interpretable as the remaining tangential acceleration
at the contact after constraint forces have been applied.  From this
construction, the frictional force to be applied along direction
i will be equal to fᵇᵢ - fᵇᵣ₊ᵢ. 

<h4>Friction constraints at the velocity-level</h4>
Drake's velocity-level formulation treats both sliding and non-sliding contact
constraints identically: the treatment in the section above changes, and
the Coulomb friction constraints disappear. Velocity-level constraint
formulations apply at impacting states, and the aforementioned changes reflect
that contacts can instantaneously change from sliding to not sliding, and
vice versa, during impact. The constraints now act to maximize negative work in
the contact tangent plane [Anitescu 1997]. Reflecting this change at the
velocity level, Equations (0*)-(k*) are modified to: 
<pre>
(0⁺)     c₀ ≡ μ⋅fᴺ - 1ᵀfᵇ
(1⁺)     c₁ ≡ (ṗᵢ - ṗⱼ)ᵀb₁ - Λ
...
(r⁺)     cᵣ ≡ (ṗᵢ - ṗⱼ)ᵀbᵣ - Λ
(r+1⁺) cᵣ₊₁ ≡ -(ṗᵢ - ṗⱼ)ᵀb₁ - Λ
...
(k⁺)     cₖ ≡ -(ṗᵢ - ṗⱼ)ᵀbᵣ - Λ
</pre>
fᴺ and fᵇ now reflect impulsive forces, and, similarly, Λ now corresponds to
residual tangential velocity post-impact. The remainder of the discussion in
the previous section, apart from these modifications to the constraints, still
applies. 
*/

/** @defgroup generic_bilateral Generic bilateral constraints
@ingroup constraint_overview

The bilateral constraint functions supported includes, among others, holonomic
constraints, which are constraints posable as c(t, q). An example such holonomic
constraint function is the transmission (gearing) constraint:<pre>
qᵢ - rqⱼ = 0
</pre> where
c(q) ≡ qᵢ - rqⱼ
</pre>
and where `r` is the gear ratio (for simplicity, this equation does not
incorporate a constant angular offset between the rotational joints).
The first derivative of this function with respect to time:<pre>
ċ(q;v) = q̇ᵢ - rq̇ⱼ
</pre>
would allow this constraint to be used with a velocity-level constraint
formulation and can be read as the velocity at joint i (q̇ᵢ) must equal `r` times
the velocity at joint j (q̇ⱼ). The second derivative of c(q) with respect to
time,<pre>
c̈(q,v;v̇) = q̈ᵢ - rq̈ⱼ
</pre>
would allow this constraint to be used with an acceleration-level constraint
formulation. For this simple transmission constraint example, it will generally
be the case that q̇ᵢ = vᵢ and q̇ⱼ = vⱼ.
*/ 

/** @defgroup generic_unilateral Generic unilateral constraints
@ingroup constraint_overview

The unilateral constraint functions supported includes, among others,
holonomic constraints, which are constraints posable as c(t, q). An example
such unilateral holonomic constraint function is a joint range-of-motion
limit:<pre>
0 ≤ c(q)  ⊥  λᵢ ≥ 0
</pre> where
c(q) ≡ qᵢ.
</pre>
This limit range of motion limit requires joint qᵢ to be non-negative.
The force limit (λᵢ ≥ 0) requires the applied force to also be non-negative
(i.e., it acts against any force that would lead to make c(q)
negative). And, the complementarity constraint c(q)λᵢ = 0 implies that
force can only be applied when the joint is at the limit.

The first derivative of this function with respect to time:<pre>
ċ(q;v) = q̇ᵢ
</pre>
yields the complementarity condition:<pre>
0 ≤ ċ(q)  ⊥  λᵢ ≥ 0
</pre>
and would allow this constraint to be used with a velocity-level constraint
formulation. The second derivative of c(q) with respect to
time,<pre>
c̈(q,v;v̇) = q̈ᵢ
</pre>
would allow this constraint to be used with an acceleration-level constraint
formulation and yields the complementarity condition:<pre>
0 ≤ c̈(q)  ⊥  λᵢ ≥ 0
</pre>
For this simple range of motion constraint example, it will generally be
the case that q̇ᵢ = vᵢ.
*/ 

