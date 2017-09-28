#pragma once

#include <vector>

#include <Eigen/Core>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace constraint {

/// Structure for holding constraint data for computing forces due to
/// constraints and the resulting multibody accelerations.
///
/// The Newton-Euler equations (essentially F = ma) coupled with constraints
/// on the positional coordinates c(q) yields an Index-3 DAE
/// (see [Hairer 1996]), and generally makes initial value problems hard to
/// solve, computationally speaking; coupling the Newton-Euler equations with
/// the second time derivative of such constraint equations (i.e., c̈(q,v,v̇))
/// yields a far more manageable Index-1 DAE, again with regard to computation.
/// This structure stores problem data for computing dynamics under such
/// acceleration-level constraints.
///
/// <h3>Bilateral and unilateral constraints</h3>
/// Constraints can be categorized as either bilateral or unilateral, which
/// roughly can be translated to equality (e.g., c(q) = 0) or inequality
/// constraints (e.g., c(q) ≥ 0). The former can be realized through the
/// latter through a pair of inequality constraints, c(q) ≥ 0 and -c(q) ≥ 0;
/// the problem structure distinguishes the two types to maximize
/// computational efficiency in the solution algorithms. We assume hereafter
/// that c() are vector functions.
///
/// Constraints may be defined at the position level:<pre>
/// c(t,q)
/// </pre>
/// at the velocity level:<pre>
/// c(t,q;v)
/// </pre>
/// or at the acceleration level:<pre>
/// c(t,q,v;v̇,λ)
/// </pre>
/// where λ, which is the same dimension as c, is a vector of force magnitudes
/// used to enforce the constraint; note the semicolon, which separates general
/// constraint dependencies (t,q,v) from variables that must be determined using
/// the constraints (v̇,λ).
///
/// This class does not generally attempt (or need) to distinguish
/// between equations that are posable at the position level (i.e., holonomic
/// constraints) but are differentiated once with respect to time to
/// yield a velocity-level constraint vs. equations that *must* be formulated at
/// at the velocity-level (i.e., nonholonomic constraints). The section below
/// on constraint stabilization will provide the lone exception to this rule.
///
/// At the acceleration level, a bilateral constraint equation takes the form:
/// <pre>
/// c(t,q,v;v̇,λ) = 0
/// </pre>
/// Each unilateral constraint at the acceleration level
/// comprises a triplet of equations, for example:<pre>
/// c(t,q,v;v̇,λ) ≥ 0
/// λ ≥ 0
/// c(t,q,v;v̇,λ)⋅λ = 0
/// </pre>
/// which we will typically write in the common shorthand notation:<pre>
/// 0 ≤ c  ⊥  λ ≥ 0
/// </pre>
/// Interpreting this triplet of constraint equations, two conditions become
/// apparent: (1) when the constraint is inactive (c > 0), the constraint force
/// must be zero (λ = 0) and (2) the constraint force can only act in one
/// direction (λ ≥ 0). This triplet is known as a *complementarity constraint*.
///
/// <h3>Constraint softening</h3>
/// It can be both numerically advantageous and a desirable modeling feature to
/// soften otherwise "rigid" constraints. For example, consider modifying the
/// unilateral complementarity constraint above to:<pre>
/// 0 ≤ c(t,q,v;v̇,λ) + γλ  ⊥  λ ≥ 0
/// </pre>
/// where γ is a non-negative scalar; alternatively, it can represent a diagonal
/// matrix with the same number of rows/columns as the dimension of c and λ,
/// permitting different coefficients for each constraint equation.
/// With γλ > 0, it becomes easier to satisfy the constraint
/// c(t,q,v;v̇,λ) + γλ ≥ 0, though the resulting v̇ and λ will not quite
/// satisfy c = 0 (i.e., c will be slightly negative). As hinted above,
/// softening grants two benefits. First, the complementarity problems resulting
/// from the softened unilateral constraints are regularized, and any
/// complementarity problem becomes solvable given sufficient regularization
/// [Cottle 1992]. Second (in concert with constraint stabilization), constraint
/// softening introduces (coarse) compliant effects, e.g., at joint stops and
/// between contacting bodies; such effects are often desirable for effecting
/// robotic manipulation, for example; see [Catto 2004] for an accessible
/// description of soft constraints. Note that Drake does not soften bilateral
/// constraints.
///
/// <h3>Constraint stabilization</h3>
/// Truncation and rounding errors can prevent constraints from being satisfied.
/// For example, consider the bilateral constraint equation c(t,q) = 0. Even if
/// c(t₀,q(t₀)) = ċ(t₀,q(t₀),v(t₀)) = c̈(t₀,q(t₀),v(t₀),v̇(t₀)) = 0, c(t₁,q(t₁))
/// is unlikely to be zero for sufficiently large Δt = t₁ - t₀. Consequently,
/// we can modify unilateral constraints to:<pre>
/// 0 ≤ c̈ + 2αċ + β²c + γλ ⊥  λ ≥ 0
/// </pre>
/// and bilateral constraints to:<pre>
/// c̈ + 2αċ + β²c = 0
/// </pre>
/// for non-negative scalars α and β (like λ, α and β can also represent
/// diagonal matrices). α and β, which both have units of 1/sec (i.e., the
/// reciprocal of unit time) are described more fully in [Baumgarte 1972].
///
/// @ref accel_jacobians
/// <h3>Jacobian matrices</h3>
/// Much of the problem data in this class refers to particular Jacobian
/// matrices. If the time derivatives of the system's generalized coordinates
/// are equal to the system's generalized velocities, these Jacobian matrices
/// can be defined simply as the partial derivatives of the constraint equations
/// taken with respect to the partial derivatives of the generalized
/// coordinates (i.e., ∂c/∂q). The two kinds of coordinates need not be equal,
/// which leads to a general, albeit harder to describe definition of the
/// Jacobian matrices as the partial derivatives of the constraint equations
/// taken with respect to the quasi-coordinates (see @ref quasi_coordinates);
/// using the notation there for quasi-coordinates means we write the Jacobian
/// as ∂c/∂q̅ (quasi-coordinates possess the property that ∂q̅/∂v = Iₙₓₙ, i.e.,
/// the n × n identity matrix). Fortunately, for constraints defined strictly
/// in the form c(q), the Jacobians are described completely by the equation
/// ċ = ∂c/∂q̅⋅v, where v are the generalized velocities of the system. Since the
/// problem data specifically requires operators that compute (∂c/∂q̅⋅v), one
/// can simply evaluate ċ.
///
/// <h3>Definition of variables used within this documentation:</h3>
/// - b ∈ ℕ   The number of bilateral constraint equations.
/// - k ∈ ℕ   The number of edges in a polygonal approximation to a friction
///           cone. Note that k = 2r.
/// - p ∈ ℕ   The number of non-interpenetration constraint equations
/// - q ∈ ℝⁿ' The generalized coordinate vector of the system. n' is at least
///           as large as n.
/// - n ∈ ℕ   The dimension of the system generalized velocity / force.
/// - n' ∈ ℕ  The dimension of the system generalized coordinates.
/// - r ∈ ℕ   *Half* the number of edges in a polygonal approximation to a
///           friction cone. Note that k = 2r.
/// - s ∈ ℕ   The number of contacts at which sliding is occurring. Note
///           that p = s + y.
/// - t ∈ ℝ   The system time variable (t ≥ 0).
/// - u ∈ ℕ   The number of "generic" (non-contact related) unilateral
///           constraint equations.
/// - v ∈ ℝⁿ  The generalized velocity vector of the system, which is equivalent
///           to the time derivative of the system quasi-coordinates.
/// - y ∈ ℕ   The number of contacts at which sliding is not occurring. Note
///           that p = s + y.
/// - α ∈ ℝ   A non-negative scalar used to correct velocity-level constraint
///           errors (i.e., "stabilize" the position constraints) via an error
///           feedback process (Baumgarte Stabilization).
/// - β ∈ ℝ   A non-negative scalar used to correct position-level constraint
///           errors via the same error feedback process (Baumgarte
///           Stabilization) that uses α.
/// - γ ∈ ℝ   A non-negative scalar used to soften an otherwise perfectly
///           "rigid" constraint.
template <class T>
struct ConstraintAccelProblemData {
  /// Constructs acceleration problem data for a system with a @p gv_dim
  /// dimensional generalized velocity.
  explicit ConstraintAccelProblemData(int gv_dim) {
    // Set default for non-transpose operators- returns an empty vector.
    auto zero_fn = [](const VectorX<T>&) -> VectorX<T> {
      return VectorX<T>(0);
    };
    N_mult = zero_fn;
    F_mult = zero_fn;
    L_mult = zero_fn;
    G_mult = zero_fn;

    // Set default for transpose operators - returns the appropriately sized
    // zero vector.
    auto zero_gv_dim_fn = [gv_dim](const VectorX<T>&) -> VectorX<T> {
      return VectorX<T>::Zero(gv_dim);
    };
    N_minus_muQ_transpose_mult = zero_gv_dim_fn;
    F_transpose_mult = zero_gv_dim_fn;
    L_transpose_mult = zero_gv_dim_fn;
    G_transpose_mult = zero_gv_dim_fn;
  }

  /// The indices of the sliding contacts (those contacts at which there is
  /// non-zero relative velocity between bodies in the plane tangent to the
  /// point of contact), out of the set of all contact indices (0...m-1).
  /// This vector must be in sorted order.
  std::vector<int> sliding_contacts;

  /// The indices of the non-sliding contacts (those contacts at which there
  /// is zero relative velocity between bodies in the plane tangent to the
  /// point of contact), out of the set of all contact indices (0...m-1).
  /// This vector must be in sorted order.
  std::vector<int> non_sliding_contacts;

  /// The number of spanning vectors in the contact tangents (used to linearize
  /// the friction cone) at the p *non-sliding* contact points. For contact
  /// problems in two dimensions, each element of r will be one. For contact
  /// problems in three dimensions, a friction pyramid (for example), for a
  /// contact point i will have rᵢ = 2. [Anitescu 1997] define k such vectors
  /// and require that, for each vector w in the spanning set, -w also exists
  /// in the spanning set. The RigidContactAccelProblemData structure expects
  /// that the contact solving mechanism negates the spanning vectors so `r` =
  /// k/2 spanning vectors will correspond to a k-edge polygon friction cone
  /// approximation.
  std::vector<int> r;

  /// Coefficients of friction for the s = p - y sliding contacts (where `y` is
  /// the number of non-sliding contacts). The size of this vector should be
  /// equal to `sliding_contacts.size()`.
  VectorX<T> mu_sliding;

  /// Coefficients of friction for the y = p - s non-sliding contacts (where `s`
  /// is the number of sliding contacts). The size of this vector should be
  /// equal to `non_sliding_contacts.size()`.
  VectorX<T> mu_non_sliding;

  /// @name Data for bilateral constraints at the acceleration level
  /// Problem data for bilateral constraints of functions of system
  /// acceleration, where the constraint can be formulated as:<pre>
  /// 0 = G(q)⋅v̇ + kᴳ(t,q,v)
  /// </pre>
  /// which implies the constraint definition c(t,q,v;v̇) ≡ G(q)⋅v̇ + kᴳ(t,q,v).
  /// G is defined as the ℝᵇˣⁿ Jacobian matrix of the partial derivatives of c()
  /// taken with respect to the quasi-coordinates (see @ref quasi_coordinates).
  /// The class of constraint functions naturally includes
  /// holonomic constraints, which are constraints posable as c(t,q). Such
  /// holonomic constraints must be twice differentiated with respect to time to
  /// yield an acceleration-level formulation (i.e., c̈(t,q,v;v̇), for the
  /// aforementioned definition of c(t,q)). That differentiation yields
  /// c̈ = G⋅v̇ + Gdot⋅v + ∂c/∂t, which is consistent with the constraint class
  /// under the definition kᴳ(t,q,v) ≡ Gdot⋅v + ∂²c/∂t². An example such
  /// (holonomic) constraint function is the transmission (gearing) constraint
  /// below:<pre>
  /// 0 = v̇ᵢ - rv̇ⱼ
  /// </pre>
  /// which can be read as the acceleration at joint i (v̇ᵢ) must equal to `r`
  /// times the acceleration at joint j (v̇ⱼ); `r` is thus the gear ratio.
  /// In this example, the corresponding holonomic constraint function is
  /// c(q) ≡ qᵢ - rqⱼ, yielding c̈(q,v;v̇) = v̇ᵢ - rv̇ⱼ.
  ///
  /// Given these descriptions, the user needs to define operators for computing
  /// G⋅w (w ∈ ℝⁿ is an arbitrary vector) and Gᵀ⋅f (f ∈ ℝᵇ is an arbitrary
  /// vector). The user also needs to provide kᴳ ∈ ℝᵇ, which should be set to
  /// the vector Gdot⋅v + 2αċ + β²c.
  /// @{

  /// An operator that performs the multiplication G⋅v. The default operator
  /// returns an empty vector.
  std::function<VectorX<T>(const VectorX<T>&)> G_mult;

  /// An operator that performs the multiplication Gᵀ⋅f where f ∈ ℝᵇ are the
  /// magnitudes of the constraint forces. The default operator returns a
  /// zero vector of dimension equal to that of the generalized forces.
  std::function<VectorX<T>(const VectorX<T>&)> G_transpose_mult;

  /// This ℝᵇ vector is the vector kᴳ(t,q,v) defined above.
  VectorX<T> kG;
  /// @}

  /// @name Data for constraints on accelerations along the contact normal
  /// Problem data for constraining the acceleration of two bodies projected
  /// along the contact surface normal, for p point contacts.
  ///
  /// Consider two points pᵢ and pⱼ on rigid bodies i and j, respectively, and
  /// assume that at a certain configuration of the two bodies, ᶜq, the two
  /// points are coincident, i.e., contacting, at a single location in space,
  /// p(ᶜq). To constrain the motion of pᵢ and pⱼ to the contact
  /// surface as the bodies move, one can introduce the constraint
  /// c(q) ≡ n(q)ᵀ(pᵢ(q) - pⱼ(q)), where n(q) is the common surface normal
  /// expressed in the world frame. Differentiating c(q) once with respect to
  /// time yields ċ(q,v) ≡ nᵀ(ṗᵢ - ṗⱼ) + ṅᵀ(pᵢ - pⱼ); one
  /// more differentiation with respect to time yields
  /// c̈(q,v,v̇) ≡ nᵀ(p̈ᵢ - p̈ⱼ) + ṅᵀ(ṗᵢ - ṗⱼ) + n̈ᵀ(pᵢ - pⱼ). By collecting
  /// terms and using the to-be-defined Jacobian matrix N(q), we can introduce
  /// equivalent equations:<pre>
  /// ċ(q,v) ≡ N⋅v + ṅᵀ⋅(pᵢ - pⱼ)</pre>
  /// and:<pre>
  /// c̈(q,v,v̇) ≡ N⋅v̇ + Ndot⋅v + n̈ᵀ(pᵢ - pⱼ).
  /// </pre>
  ///
  /// The non-negativity condition on the constraint force magnitudes (λ ≥ 0)
  /// keeps the contact force along the contact normal compressive, as desired.
  /// With this background in mind, N is the ℝᵖˣⁿ Jacobian matrix that
  /// transforms generalized velocities (v ∈ ℝⁿ) into velocities projected along
  /// the contact normals at the p point contacts. The problem data also must
  /// consider Q ∈ ℝᵖˣⁿ, the Jacobian matrix that transforms generalized
  /// velocities (n is the dimension of generalized velocity) into velocities
  /// projected along the directions of sliding at the s *sliding* contact
  /// points (rows of Q that correspond to non-sliding contacts should be zero).
  ///
  /// Given these descriptions, the user needs to define operators for computing
  /// N⋅w (w ∈ ℝⁿ is an arbitrary vector) and (Nᵀ-μQᵀ)⋅f (f ∈ ℝᵖ is an arbitrary
  /// vector). The user also needs to provide γᴺ ∈ ℝᵖ, a vector of non-negative
  /// entries used to soften the non-interpenetration constraints, and kᴺ ∈ ℝᵖ,
  /// the vector Ndot⋅v + 2αċ(q,v) + β²c(q). There currently exist no guidelines
  /// for setting α, β, and γ to effect a particular damping ratio and
  /// oscillation frequency at the acceleration level.
  /// @{

  /// An operator that performs the multiplication N⋅v.
  /// The default operator returns an empty vector.
  std::function<VectorX<T>(const VectorX<T>&)> N_mult;

  /// An operator that performs the multiplication (Nᵀ - μQᵀ)⋅f, where μ is a
  /// diagonal matrix with nonzero entries corresponding to the coefficients of
  /// friction at the s sliding contact points, and (Nᵀ - μQᵀ) transforms forces
  /// (f ∈ ℝᵖ) applied along the contact normals at the p point contacts into
  /// generalized forces. The default operator returns a zero vector of
  /// dimension equal to that of the generalized forces.
  std::function<VectorX<T>(const VectorX<T>&)> N_minus_muQ_transpose_mult;

  /// This ℝᵖ vector is the vector kᴺ(t,q,v) defined above.
  VectorX<T> kN;

  /// This ℝᵖ vector represents the diagonal matrix γᴺ defined above.
  VectorX<T> gammaN;
  /// @}

  /// @name Data for non-sliding contact friction constraints
  /// Problem data for constraining the tangential acceleration of two bodies
  /// projected along the contact surface tangents, for p *non-sliding* point
  /// contacts.
  ///
  /// Non-sliding constraints can be dichotomized into two types: kinematic
  /// constraints on tangential motion and frictional force constraints.
  /// One of these constraint sets in 3D looks like:<pre>
  /// (0) ⁰g ≡ μ⋅fᴺ - ||fˢ fᵗ|| ≥ 0
  /// (1) ¹g ≡ ((pᵢ - pⱼ)ᵀbₛ)² + ((pᵢ - pⱼ)ᵀbₜ)² = 0
  /// </pre>
  /// where bₛ and bₜ are basis vectors in ℝ³ that span the contact tangent
  /// plane, pᵢ, pⱼ ∈ ℝ³ represent a point of contact between bodies i and j,
  /// μ is the coefficient of sticking friction, fᴺ is the magnitude of the
  /// force applied along the contact normal, and fˢ and fᵗ are scalars
  /// corresponding to the frictional forces applied along the basis vectors.
  /// Since nonlinear equations are typically challenging to solve, ⁰g and ¹g
  /// are often transformed to linear approximations:<pre>
  /// (0')     g̅₀ ≡ μ⋅fᴺ - 1ᵀfᵇ
  /// (1')     g̅₁ ≡ (pᵢ - pⱼ)ᵀb₁
  /// ...
  /// (r')     g̅ᵣ ≡ (pᵢ - pⱼ)ᵀbᵣ
  /// (r+1') g̅ᵣ₊₁ ≡ -(pᵢ - pⱼ)ᵀb₁
  /// ...
  /// (k')     g̅k ≡ -(pᵢ - pⱼ)ᵀbᵣ
  /// </pre>
  /// where b₁,...,bᵣ ∈ ℝ³ (k = 2r) are a set of spanning vectors in the contact
  /// tangent plane (the more vectors, the better the approximation to the
  /// nonlinear friction cone), and fᵇ ≥ 0 (which will conveniently allow us
  /// to pose these constraints within the linear complementarity problem
  /// framework), where fᵇ ∈ ℝᵏ are non-negative scalars that represent the
  /// frictional force along the spanning vectors and the negated spanning
  /// vectors. Equations 0'-k' cannot generally be satisfied simultaneously:
  /// maximizing fᵇ (i.e., fᵇ = μ⋅fᴺ) may be insufficient to keep the relative
  /// tangential movement at the point of contact zero. Accordingly, we
  /// transform Equations 0'-k' to the following:<pre>
  /// (0*)     c₀ ≡ μ⋅fᴺ - 1ᵀfᵇ
  /// (1*)     c₁ ≡ (pᵢ - pⱼ)ᵀb₁ - Λ
  /// ...
  /// (r*)     cᵣ ≡ (pᵢ - pⱼ)ᵀbᵣ - Λ
  /// (r+1*) cᵣ₊₁ ≡ -(pᵢ - pⱼ)ᵀb₁ - Λ
  /// ...
  /// (k*)     cₖ ≡ -(pᵢ - pⱼ)ᵀbᵣ - Λ
  /// </pre>
  /// which lead to the following complementarity conditions:<pre>
  /// 0 ≤ c₀    ⊥      Λ ≥ 0
  /// 0 ≤ c₁    ⊥    fᵇ₁ ≥ 0
  /// ...
  /// 0 ≤ cᵣ    ⊥    fᵇᵣ ≥ 0
  /// 0 ≤ cᵣ₊₁  ⊥  fᵇᵣ₊₁ ≥ 0
  /// 0 ≤ cₖ    ⊥    fᵇₖ ≥ 0
  /// </pre>
  /// where Λ is roughly interpretable as the remaining tangential acceleration
  /// at the contact after constraint forces have been applied.  From this
  /// construction, the frictional force to be applied along direction
  /// i will be equal to fᵇᵢ - fᵇᵣ₊ᵢ. Given the descriptions of c(.) and the
  /// Jacobian matrix F (defined in the following paragraph), the user needs to
  /// define operators for computing F⋅w (w ∈ ℝⁿ is an arbitrary vector) and
  /// Fᵀ⋅f (f ∈ ℝʸʳ is an arbitrary vector). The user also needs to provide
  /// γᶠ ∈ ℝʸʳ (a vector of non-negative entries used to relax the sticking
  /// constraints), γᴱ ∈ ℝʸ (a vector of non-negative entries used to relax the
  /// linearized Coulomb friction cone constraint, i.e., Equation 0* above),
  /// and kᶠ ∈ ℝʸʳ, the vector Fdot⋅v + 2αċ(q,v) + β²c(q) (where c(q) is the
  /// collection of Equations 1*-k*). Unlike with general constraint
  /// softening, γᶠ and γᴱ are useful primarily as regularization parameters for
  /// the numerical solution process: one typically selects these parameters to
  /// be as small as possible while still permitting the resulting
  /// complementarity problems to be solved reliably. The resulting stiction
  /// drift should be so small that no constraint stabilization (i.e., setting
  /// α = β = 0) should be sufficient for most applications.
  ///
  /// As noted above, the user must define operators based on the Jacobian
  /// matrix, F ∈ ℝʸʳˣⁿ, that transforms generalized velocities (v ∈ ℝⁿ) into
  /// velocities projected along the r vectors that span the contact tangents at
  /// the y *non-sliding* point contacts; the iᵗʰ set of r rows in F corresponds
  /// to the iᵗʰ non-sliding point contact. For contact problems in two
  /// dimensions, r would be one. For a friction pyramid in three dimensions, r
  /// would be two. While the definition of the dimension of the Jacobian matrix
  /// above indicates that every one of the y non-sliding contacts uses the same
  /// "r", the code imposes no such requirement.
  /// @{

  /// An operator that performs the multiplication F⋅v. The default operator
  /// returns an empty vector.
  std::function<VectorX<T>(const VectorX<T>&)> F_mult;

  /// An operator that performs the multiplication Fᵀ⋅f, where f ∈ ℝʸʳ
  /// corresponds to frictional force magnitudes. The default operator returns
  /// a zero vector of dimension equal to that of the generalized forces.
  std::function<VectorX<T>(const VectorX<T>&)> F_transpose_mult;

  /// This ℝʸʳ vector is the vector kᶠ(t,q,v) defined above.
  VectorX<T> kF;

  /// This ℝʸʳ vector represents the diagonal matrix γᶠ defined above.
  VectorX<T> gammaF;

  /// This ℝᵖ vector represents the diagonal matrix γᴱ defined above.
  VectorX<T> gammaE;
  /// @}

  /// @name Data for unilateral constraints at the acceleration level
  /// Problem data for unilateral constraints of functions of system
  /// acceleration, where the constraint can be formulated as:<pre>
  /// 0 ≤ L(q)⋅v̇ + kᴸ(t,q,v) + γᴸλ  ⊥  λ ≥ 0
  /// </pre>
  /// which means that the constraint c(t,q,v;v̇,λ) ≡ L(q)⋅v̇ + kᴸ(t,q,v) + γᴸλ
  /// is coupled to a force constraint (λ ≥ 0) and a complementarity constraint
  /// λ⋅(L⋅v̇ + kᴸ(t,q,v) + γᴸλ) = 0, meaning that the constraint can apply no
  /// force if it is inactive (i.e., if c(t,q,v;v̇,λ) is strictly greater than
  /// zero). L is defined as the ℝᵘˣⁿ Jacobian matrix of the partial derivatives
  /// of c() taken with respect to the quasi-coordinates (see
  /// @ref quasi_coordinates). As described in the section on
  /// constraint softening, the factor γᴸλ, where γᴸ ≥ 0 is a user-provided
  /// diagonal matrix, acts to "soften" the constraint: if γᴸ is nonzero, c will
  /// be satisfiable with a smaller λ.
  ///
  /// The class of constraint functions naturally includes holonomic
  /// constraints, which are constraints posable as c(t,q). Such holonomic
  /// constraints must be twice differentiated with respect to time to yield
  /// an acceleration-level formulation (i.e., c̈(t,q,v;v̇,λ), for the
  /// aforementioned definition of c(t,q)). That differentiation yields
  /// c̈ = L⋅v̇ + Ldot⋅v + ∂²c/∂t² (notice the absence of the softening term),
  /// which is consistent with the constraint class under the definition
  /// kᴸ(t,q,v) ≡ Ldot⋅v + ∂²c/∂t². An example such (holonomic) constraint
  /// function is a joint acceleration limit:<pre>
  /// 0 ≤ -v̇ⱼ  ⊥  λⱼ ≥ 0
  /// </pre>
  /// which can be read as the acceleration at joint j (v̇ⱼ) must be no larger
  /// than zero, the force must be applied to limit the acceleration at the
  /// joint, and the limiting force cannot be applied if the acceleration at the
  /// joint is not at the limit (i.e., v̇ⱼ < 0). In this example, a
  /// corresponding holonomic constraint function would be c(q) ≡ r - qⱼ (where
  /// r is the range of motion limit)  yielding ċ(q,v,v̇) = -v̇ⱼ. Solving for
  /// v̇(t₀) such that ċ(q(t₀), v(t₀), v̇(t₀)) = 0 will naturally allow one to
  /// determine (through integration) q(t₁) that satisfies c(q(t₁)) = 0 for
  /// t₁ > t₀ and t₁ ≈ t₀, assuming that c(q(t₀)) = 0 and ċ(q(t₀),v(t₀)) = 0.
  ///
  /// Given the description above, the user must define operators for computing
  /// L⋅w (w ∈ ℝⁿ is an arbitrary vector) and Lᵀ⋅f (f ∈ ℝᵘ is an arbitrary
  /// vector). The user also needs to provide γᴸ ∈ ℝᵘ, a vector of non-negative
  /// entries used to soften the unilateral constraints, and kᴸ ∈ ℝᵘ,
  /// the vector Ldot⋅v + αċ + β²c. There currently exist no guidelines
  /// for setting α, β, and γ to effect a particular damping ratio and
  /// oscillation frequency at the acceleration level.
  /// @{

  /// An operator that performs the multiplication L⋅v. The default operator
  /// returns an empty vector.
  std::function<VectorX<T>(const VectorX<T>&)> L_mult;

  /// An operator that performs the multiplication Lᵀ⋅f where f ∈ ℝˢ are the
  /// magnitudes of the constraint forces. The default operator returns a
  /// zero vector of dimension equal to that of the generalized forces.
  std::function<VectorX<T>(const VectorX<T>&)> L_transpose_mult;

  /// This ℝˢ vector is the vector kᴸ(t,q,v) defined above.
  VectorX<T> kL;

  /// This ℝˢ vector represents the diagonal matrix γᴸ defined above.
  VectorX<T> gammaL;
  /// @}

  /// The ℝⁿ vector tau, the generalized external force vector that
  /// comprises gravitational, centrifugal, Coriolis, actuator, etc. forces
  /// applied to the rigid body system at q. n is the dimension of the
  /// generalized force, which is also equal to the dimension of the
  /// generalized velocity.
  VectorX<T> tau;

  /// A function for solving the equation MX = B for matrix X, given input
  /// matrix B, where M is the generalized inertia matrix for the rigid body
  /// system.
  std::function<MatrixX<T>(const MatrixX<T>&)> solve_inertia;
};

/// Structure for holding constraint data for computing constraint forces
/// at the velocity-level (i.e., impact problems).
///
/// <h3>Bilateral and unilateral constraints</h3>
/// Constraints can be categorized as either bilateral or unilateral, which
/// roughly can be translated to equality (e.g., c(q) = 0) or inequality
/// constraints (e.g., c(q) ≥ 0). The former can be realized through the
/// latter through a pair of inequality constraints, c(q) ≥ 0 and -c(q) ≥ 0;
/// the problem structure distinguishes the two types to maximize
/// computational efficiency in the solution algorithms. We assume hereafter
/// that c() are vector functions.
///
/// Constraints may be defined at the position level:<pre>
/// c(t,q)
/// </pre>
/// at the velocity level:<pre>
/// c(t,q;v,λ)
/// </pre>
/// where λ, which is the same dimension as c, is a vector of impulsive force
/// magnitudes used to enforce the constraint; note the semicolon, which
/// separates general constraint dependencies (t,q) from variables that must be
/// determined using the constraints (v,λ).
///
/// This document and class does not generally attempt (or need) to distinguish
/// between equations that are posable at the position level (i.e., holonomic
/// constraints) but are differentiated once with respect to time to
/// yield a velocity-level constraint vs. equations that *must* be formulated at
/// at the velocity-level (i.e., nonholonomic constraints). The section below
/// on constraint stabilization will provide the lone exception to this rule.
///
/// At the velocity level, a bilateral constraint equation takes the form:
/// <pre>
/// c(t,q;v,λ) = 0
/// </pre>
/// Each unilateral constraint at the acceleration level
/// comprises a triplet of equations, for example:<pre>
/// c(t,q;v,λ) ≥ 0
/// λ ≥ 0
/// c(t,q;v,λ)⋅λ = 0
/// </pre>
/// which we will typically write in the common shorthand notation:<pre>
/// 0 ≤ c  ⊥  λ ≥ 0
/// </pre>
/// Interpreting this triplet of constraint equations, two conditions become
/// apparent: (1) when the constraint is inactive (c > 0), the constraint force
/// must be zero (λ = 0) and (2) the constraint force can only act in one
/// direction (λ ≥ 0). This triplet is known as a *complementarity constraint*.
///
/// <h3>Constraint softening</h3>
/// It can be both numerically advantageous and a desirable modeling feature to
/// soften otherwise "rigid" constraints. For example, consider modifying the
/// unilateral complementarity constraint above to:<pre>
/// 0 ≤ c(t,q;v,λ) + γλ  ⊥  λ ≥ 0
/// </pre>
/// where γ is a non-negative scalar; alternatively, it can represent a diagonal
/// matrix with the same number of rows/columns as the dimension of ċ and λ,
/// permitting different coefficients for each constraint equation.
/// With γλ > 0, it becomes easier to satisfy the constraint
/// c(t,q;v,λ) + γλ ≥ 0, though the resulting v and λ will not quite
/// satisfy c = 0 (i.e., c will be slightly negative). See the **Constraint
/// softening** section in ConstraintAccelProblemData for more details about
/// softening.
///
/// <h3>Constraint stabilization</h3>
/// Truncation and rounding errors can prevent constraints from being satisfied.
/// For example, consider the bilateral constraint equation c(t,q) = 0. Even if
/// c(t₀,q(t₀)) = ċ(t₀,q(t₀),v(t₀)) = 0, c(t₁,q(t₁))
/// is unlikely to be zero for sufficiently large Δt = t₁ - t₀. Consequently,
/// we can modify unilateral constraints to:<pre>
/// 0 ≤ ċ + ζc + γλ  ⊥  λ ≥ 0
/// </pre>
/// and bilateral constraints to:<pre>
/// ċ + ζc = 0
/// </pre>
/// for non-negative scalar ζ (like λ, ζ can also represent
/// a diagonal matrix). ζ has units of 1/sec (i.e., the
/// reciprocal of unit time).
///
/// <h3>Jacobian matrices</h3>
/// See the section **Jacobian matrices** in ConstraintAccelProblemData.
///
/// <h3>Definition of variables used within this documentation:</h3>
/// - b ∈ ℕ   The number of bilateral constraint equations.
/// - k ∈ ℕ   The number of edges in a polygonal approximation to a friction
///           cone. Note that k = 2r.
/// - p ∈ ℕ   The number of non-interpenetration constraint equations
/// - q ∈ ℝⁿ' The generalized coordinate vector of the system. n' is at least
///           as large as n.
/// - n ∈ ℕ   The dimension of the system generalized velocity / force.
/// - n' ∈ ℕ  The dimension of the system generalized coordinates.
/// - r ∈ ℕ   *Half* the number of edges in a polygonal approximation to a
///           friction cone. Note that k = 2r.
/// - t ∈ ℝ   The system time variable (t ≥ 0).
/// - u ∈ ℕ   The number of "generic" (non-contact related) unilateral
///           constraint equations.
/// - v ∈ ℝⁿ  The generalized velocity vector of the system, which is equivalent
///           to the time derivative of the system quasi-coordinates.
/// - ζ ∈ ℝ   A non-negative scalar used to correct position-level constraint
///           errors (i.e., "stabilize" the position constraints) via an error
///           feedback process (Baumgarte Stabilization).
/// - γ ∈ ℝ   A non-negative scalar used to soften an otherwise perfectly
///           "rigid" constraint.template <class T>
template <class T>
struct ConstraintVelProblemData {
  /// Constructs velocity problem data for a system with a `gv_dim` dimensional
  /// generalized velocity.
  explicit ConstraintVelProblemData(int gv_dim) {
    // Set default for non-transpose operators- returns an empty vector.
    auto zero_fn = [](const VectorX<T>&) -> VectorX<T> {
      return VectorX<T>(0);
    };
    N_mult = zero_fn;
    F_mult = zero_fn;
    L_mult = zero_fn;
    G_mult = zero_fn;

    // Set default for transpose operators - returns the appropriately sized
    // zero vector.
    auto zero_gv_dim_fn = [gv_dim](const VectorX<T>&) -> VectorX<T> {
      return VectorX<T>::Zero(gv_dim); };
    N_transpose_mult = zero_gv_dim_fn;
    F_transpose_mult = zero_gv_dim_fn;
    L_transpose_mult = zero_gv_dim_fn;
    G_transpose_mult = zero_gv_dim_fn;
  }

  /// The number of spanning vectors in the contact tangents (used to linearize
  /// the friction cone) at the p contact points. For contact
  /// problems in two dimensions, each element of r will be one. For contact
  /// problems in three dimensions, a friction pyramid (for example), for a
  /// contact point i will have rᵢ = 2. [Anitescu 1997] define k such vectors
  /// and require that, for each vector w in the spanning set, -w also exists
  /// in the spanning set. The RigidContactVelProblemData structure expects
  /// that the contact solving mechanism negates the spanning vectors so `r` =
  /// k/2 spanning vectors will correspond to a k-edge polygon friction cone
  /// approximation.
  std::vector<int> r;

  /// Coefficients of friction for the p contacts. This problem specification
  /// does not distinguish between static and dynamic friction coefficients.
  VectorX<T> mu;

  /// @name Data for bilateral constraints at the velocity level
  /// Problem data for bilateral constraints of functions of system
  /// velocity, where the constraint can be formulated as:<pre>
  /// 0 = G(q)⋅v + kᴳ(t,q)
  /// </pre>
  /// which implies the constraint definition c(t,q,v) ≡ G(q)⋅v + kᴳ(t,q). G
  /// is defined as the ℝᵇˣⁿ Jacobian matrix of the partial derivatives of c()
  /// taken with respect to the quasi-coordinates (see @ref accel_jacobians).
  /// The class of constraint functions naturally
  /// includes holonomic constraints, which are constraints posable as c(t,q).
  /// Such holonomic constraints must be differentiated with respect to time to
  /// yield a velocity-level formulation (i.e., ċ(t,q;v), for the
  /// aforementioned definition of c(t,q)). That differentiation yields
  /// ċ = G⋅v + ∂c/∂t, which is consistent with the constraint class under
  /// the definition kᴳ(t,q) ≡ ∂c/∂t. An example such holonomic constraint
  /// function is the transmission (gearing) constraint below:<pre>
  /// 0 = vᵢ - rvⱼ
  /// </pre>
  /// which can be read as the velocity at joint j (vⱼ) must equal to `r`
  /// times the velocity at joint i (vᵢ); `r` is thus the gear ratio.
  /// In this example, the corresponding holonomic constraint function is
  /// c(q) ≡ qᵢ -rqⱼ, yielding ċ(q, v) = vⱼ - rvⱼ.
  ///
  /// Given these descriptions, the user needs to define operators for computing
  /// G⋅w (w ∈ ℝⁿ is an arbitrary vector) and Gᵀ⋅f (f ∈ ℝᵇ is an arbitrary
  /// vector). The user also needs to provide kᴳ ∈ ℝᵇ, which should be set to
  /// the vector ζc.
  /// @{

  /// An operator that performs the multiplication G⋅v. The default operator
  /// returns an empty vector.
  std::function<VectorX<T>(const VectorX<T>&)> G_mult;

  /// An operator that performs the multiplication Gᵀ⋅f where f ∈ ℝᵇ are the
  /// magnitudes of the constraint forces. The default operator returns a
  /// zero vector of dimension equal to that of the generalized forces.
  std::function<VectorX<T>(const VectorX<T>&)> G_transpose_mult;

  /// This ℝᵇ vector is the vector kᴳ(t,q) defined above.
  VectorX<T> kG;
  /// @}

  /// @name Data for constraints on velocities along the contact normal
  /// Problem data for constraining the velocity of two bodies projected
  /// along the contact surface normal, for p point contacts.
  ///
  /// Consider two points pᵢ and pⱼ on rigid bodies i and j, respectively, and
  /// assume that at a certain configuration of the two bodies, ᶜq, the two
  /// points are coincident, i.e., contacting, at a single location in space,
  /// p(ᶜq). To constrain the motion of pᵢ and pⱼ to the contact
  /// surface as the bodies move, one can introduce the constraint
  /// c(q) ≡ n(q)ᵀ(pᵢ(q) - pⱼ(q)), where n(q) is the common surface normal
  /// expressed in the world frame. Differentiating c(q) once with
  /// respect to time yields ċ(q,v) ≡ nᵀ(ṗᵢ - ṗⱼ) + ṅᵀ(pᵢ - pⱼ).
  /// By collecting terms and using the to-be-defined Jacobian matrix N(q), we
  /// can introduce an equivalent equation:<pre>
  /// ċ(q,v) ≡ N⋅v + ṅᵀ⋅(pᵢ - pⱼ)</pre>
  /// </pre>
  ///
  /// The non-negativity condition on the constraint impulse magnitudes (λ ≥ 0)
  /// keeps the contact force along the contact normal compressive, as desired.
  /// With this background in mind, N is the ℝᵖˣⁿ Jacobian matrix that
  /// transforms generalized velocities (v ∈ ℝⁿ) into velocities projected along
  /// the contact normals at the p point contacts.
  ///
  /// Given this description, the user needs to define operators for computing
  /// N⋅w (w ∈ ℝⁿ is an arbitrary vector) and Nᵀ⋅f (f ∈ ℝᵖ is an arbitrary
  /// vector). The user also needs to provide γᴺ ∈ ℝᵖ, a vector of non-negative
  /// entries used to soften the non-interpenetration constraints, and kᴺ ∈ ℝᵖ,
  /// the vector ζc(q). Guidelines for setting ζ and γ to effect a particular
  /// damping ratio and oscillation frequency are described in [Catto 2004];
  /// the parameters are known as "ERP" and "CFM" in that context.
  /// @{

  /// An operator that performs the multiplication N⋅v. The default operator
  /// returns an empty vector.
  std::function<VectorX<T>(const VectorX<T>&)> N_mult;

  /// An operator that performs the multiplication Nᵀ⋅f, where f ∈ ℝᵖ are the
  /// the magnitudes of the impulsive forces applied along the contact normals
  /// at the p point contacts. The default operator returns a zero vector of
  /// dimension equal to that of the generalized velocities (which should be
  /// identical to the dimension of the generalized forces).
  std::function<VectorX<T>(const VectorX<T>&)> N_transpose_mult;

  /// This ℝᵖ vector is the vector kᴺ(t,q,v) defined above.
  VectorX<T> kN;

  /// This ℝᵖ vector represents the diagonal matrix γᴺ defined above.
  VectorX<T> gammaN;
  /// @}

  /// @name Data for contact friction constraints
  /// Problem data for constraining the tangential velocity of two bodies
  /// projected along the contact surface tangents, for p point contacts.
  ///
  /// These frictional constraints can be dichotomized into two types: kinematic
  /// constraints on tangential motion and frictional force constraints.
  /// One of these constraint sets in 3D looks like:<pre>
  /// (0) ⁰g ≡ μ⋅fᴺ - ||fˢ fᵗ|| ≥ 0
  /// (1) ¹g ≡ ((pᵢ - pⱼ)ᵀbₛ)² + ((pᵢ - pⱼ)ᵀbₜ)² = 0
  /// </pre>
  /// where bₛ and bₜ are basis vectors in ℝ³ that span the contact tangent
  /// plane, pᵢ, pⱼ ∈ ℝ³ represent a point of contact between bodies i and j,
  /// μ is the coefficient of sticking friction, fᴺ is the magnitude of the
  /// force applied along the contact normal, and fˢ and fᵗ are scalars
  /// corresponding to the frictional forces applied along the basis vectors.
  /// Since nonlinear equations are typically challenging to solve, ⁰g and ¹g
  /// are often transformed to linear approximations:<pre>
  /// (0')     g̅₀ ≡ μ⋅fᴺ - 1ᵀfᵇ
  /// (1')     g̅₁ ≡ (pᵢ - pⱼ)ᵀb₁
  /// ...
  /// (r')     g̅ᵣ ≡ (pᵢ - pⱼ)ᵀbᵣ
  /// (r+1') g̅ᵣ₊₁ ≡ -(pᵢ - pⱼ)ᵀb₁
  /// ...
  /// (k')     g̅k ≡ -(pᵢ - pⱼ)ᵀbᵣ
  /// </pre>
  /// where b₁,...,bᵣ ∈ ℝ³ (k = 2r) are a set of spanning vectors in the contact
  /// tangent plane (the more vectors, the better the approximation to the
  /// nonlinear friction cone), and fᵇ ≥ 0 (which will conveniently allow us
  /// to pose these constraints within the linear complementarity problem
  /// framework), where fᵇ ∈ ℝᵏ are non-negative scalars that represent the
  /// frictional force along the spanning vectors and the negated spanning
  /// vectors. Equations 0'-k' cannot generally be satisfied simultaneously:
  /// maximizing fᵇ (i.e., fᵇ = μ⋅fᴺ) may be insufficient to keep the relative
  /// tangential movement at the point of contact zero. Accordingly, we
  /// transform Equations 0'-k' to the following:<pre>
  /// (0*)     c₀ ≡ μ⋅fᴺ - 1ᵀfᵇ
  /// (1*)     c₁ ≡ (pᵢ - pⱼ)ᵀb₁ - Λ
  /// ...
  /// (r*)     cᵣ ≡ (pᵢ - pⱼ)ᵀbᵣ - Λ
  /// (r+1*) cᵣ₊₁ ≡ -(pᵢ - pⱼ)ᵀb₁ - Λ
  /// ...
  /// (k*)     cₖ ≡ -(pᵢ - pⱼ)ᵀbᵣ - Λ
  /// </pre>
  /// which lead to the following complementarity conditions:<pre>
  /// 0 ≤ c₀    ⊥      Λ ≥ 0
  /// 0 ≤ c₁    ⊥    fᵇ₁ ≥ 0
  /// ...
  /// 0 ≤ cᵣ    ⊥    fᵇᵣ ≥ 0
  /// 0 ≤ cᵣ₊₁  ⊥  fᵇᵣ₊₁ ≥ 0
  /// 0 ≤ cₖ    ⊥    fᵇₖ ≥ 0
  /// </pre>
  /// where Λ is roughly interpretable as the remaining tangential velocity
  /// at the contact after constraint impulses have been applied.  From this
  /// construction, the frictional impulse to be applied along direction
  /// i will be equal to fᵇᵢ - fᵇᵣ₊ᵢ. Given the descriptions of c(.) and the
  /// Jacobian matrix F (defined in the following paragraph), the user needs to
  /// define operators for computing F⋅w (w ∈ ℝⁿ is an arbitrary vector) and
  /// Fᵀ⋅f (f ∈ ℝᵖʳ is an arbitrary vector). The user also needs to provide
  /// γᶠ ∈ ℝʸʳ (a vector of non-negative entries used to relax the sticking
  /// constraints), γᴱ ∈ ℝᵖ (a vector of non-negative entries used to relax the
  /// linearized Coulomb friction cone constraint, i.e., Equation 0* above),
  /// and kᶠ ∈ ℝᵖʳ, the vector ζc(q) (where c(q) is the
  /// collection of Equations 1*-k*). Unlike with general constraint
  /// softening, γᶠ and γᴱ are useful primarily as regularization parameters for
  /// the numerical solution process: one typically selects these parameters to
  /// be as small as possible while still permitting the resulting
  /// complementarity problems to be solved reliably. The resulting stiction
  /// drift should be so small that no constraint stabilization (i.e., setting
  /// ζ = 0) should be sufficient for most applications.
  ///
  /// As noted above, the user must define operators based on the Jacobian
  /// matrix, F ∈ ℝᵖʳˣⁿ, that transforms generalized velocities (v ∈ ℝⁿ) into
  /// velocities projected along the r vectors that span the contact tangents at
  /// the p point contacts; the iᵗʰ set of r rows in F corresponds
  /// to the iᵗʰ point contact. For contact problems in two
  /// dimensions, r would be one. For a friction pyramid in three dimensions, r
  /// would be two. While the definition of the dimension of the Jacobian matrix
  /// above indicates that every one of the y non-sliding contacts uses the same
  /// "r", the code imposes no such requirement.
  /// @{

  /// An operator that performs the multiplication F⋅v. The default operator
  /// returns an empty vector.
  std::function<VectorX<T>(const VectorX<T>&)> F_mult;

  /// An operator that performs the multiplication Fᵀ⋅f, where f ∈ ℝᵖʳ
  /// corresponds to frictional impulsive force magnitudes. The default
  /// operator returns a zero vector of dimension equal to that of the
  /// generalized forces.
  std::function<VectorX<T>(const VectorX<T>&)> F_transpose_mult;

  /// This ℝʸʳ vector is the vector kᶠ(t,q,v) defined above.
  VectorX<T> kF;

  /// This ℝʸʳ vector represents the diagonal matrix γᶠ defined above.
  VectorX<T> gammaF;

  /// This ℝᵖ vector represents the diagonal matrix γᴱ defined above.
  VectorX<T> gammaE;

  /// @}

  /// @name Data for unilateral constraints at the velocity level
  /// Problem data for unilateral constraints of functions of system
  /// velocity, where the constraint can be formulated as:<pre>
  /// 0 ≤ L(q)⋅v + kᴸ(t,q) + γᴸλ  ⊥  λ ≥ 0
  /// </pre>
  /// which means that the constraint c(t,q;v,λ) ≡ L(q)⋅v + kᴸ(t,q) + γᴸλ
  /// is coupled to a force constraint (λ ≥ 0) and a complementarity constraint
  /// λ⋅(L⋅v + kᴸ(t,q) + γᴸλ) = 0, meaning that the constraint can apply no
  /// impulse if it is inactive (i.e., if c(t,q;v,λ) is strictly greater than
  /// zero). L is defined as the ℝᵘˣⁿ Jacobian matrix of the partial derivatives
  /// of c() taken with respect to the quasi-coordinates (see
  /// @ref accel_jacobians). As described in the section on
  /// constraint softening, the factor γᴸλ, where γᴸ ≥ 0 is a user-provided
  /// diagonal matrix, acts to "soften" the constraint: if γᴸ is nonzero, c will
  /// be satisfiable with a smaller λ.
  ///
  /// The class of constraint functions naturally includes holonomic
  /// constraints, which are constraints posable as c(t,q). Such holonomic
  /// constraints must be differentiated with respect to time to yield
  /// a velocity-level formulation (i.e., ċ(t,q;v,λ), for the
  /// aforementioned definition of c(t,q)). That differentiation yields
  /// ċ = L⋅v + ∂c/∂t (notice the absence of the softening term),
  /// which is consistent with the constraint class under the definition
  /// kᴸ(t,q) ≡ ∂c/∂t. An example such (holonomic) constraint
  /// function is a joint velocity limit:<pre>
  /// 0 ≤ -vⱼ  ⊥  λⱼ ≥ 0
  /// </pre>
  /// which can be read as the velocity at joint j (vⱼ) must be no larger
  /// than zero, the impulsive force must be applied to limit the velocity at
  /// the joint, and the limiting force cannot be applied if the velocity at the
  /// joint is not at the limit (i.e., vⱼ < 0). In this example, a
  /// corresponding holonomic constraint function would be c(q) ≡ r - qⱼ (where
  /// r is the range of motion limit), yielding ċ(q,v) = -vⱼ. Solving for
  /// v(t₀) such that ċ(q(t₀), v(t₀)) = 0 will naturally allow one to
  /// determine (through integration) q(t₁) that satisfies c(q(t₁)) = 0 for
  /// t₁ > t₀ and t₁ ≈ t₀, assuming that c(q(t₀)) = 0.
  ///
  /// Given the description above, the user must define operators for computing
  /// L⋅w (w ∈ ℝⁿ is an arbitrary vector) and Lᵀ⋅f (f ∈ ℝᵘ is an arbitrary
  /// vector). The user also needs to provide γᴸ ∈ ℝᵘ, a vector of non-negative
  /// entries used to soften the unilateral constraints, and kᴸ ∈ ℝᵘ,
  /// the vector ζc(q). Guidelines for setting ζ and γ to effect a particular
  /// damping ratio and oscillation frequency are described in [Catto 2004];
  /// the parameters are known as "ERP" and "CFM" in that context.
  /// @{

  /// An operator that performs the multiplication L⋅v. The default operator
  /// returns an empty vector.
  std::function<VectorX<T>(const VectorX<T>&)> L_mult;

  /// An operator that performs the multiplication Lᵀ⋅f where f ∈ ℝᵗ are the
  /// magnitudes of the impulsive constraint forces. The default operator
  /// returns a zero vector of dimension equal to that of the generalized
  /// forces.
  std::function<VectorX<T>(const VectorX<T>&)> L_transpose_mult;

  /// This ℝˢ vector is the vector kᴸ(t,q) defined above.
  VectorX<T> kL;

  /// This ℝˢ vector represents the diagonal matrix γᴸ defined above.
  VectorX<T> gammaL;
  /// @}

  /// The ℝⁿ vector v, the generalized velocity immediately before any impulsive
  /// forces (from impact) are applied.
  VectorX<T> v;

  /// A function for solving the equation MX = B for matrix X, given input
  /// matrix B, where M is the generalized inertia matrix for the rigid body
  /// system.
  std::function<MatrixX<T>(const MatrixX<T>&)> solve_inertia;
};

}  // namespace constraint
}  // namespace multibody
}  // namespace drake
