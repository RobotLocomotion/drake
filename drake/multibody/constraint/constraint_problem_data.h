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
/// that c() are vector equations.
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
/// constraint dependencies (q,v,t) from variables that must be determined using
/// the constraints (v̇,λ).
///
/// This document and class does not generally attempt (or need) to distinguish
/// between equations that are posable at the position level but are
/// differentiated once with respect to time (i.e., holonomic constraints) to
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
/// matrix with the same number of rows/columns as the dimension of c̈ and λ,
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
/// <h3>Jacobian matrices</h3>
/// Much of the problem data in this class refers to particular Jacobian
/// matrices. If the time derivatives of the system's generalized coordinates
/// are equal to the system's generalized velocities, these Jacobian matrices
/// can be defined simply as the partial derivatives of the constraint equations
/// taken with respect to the partial derivatives of the generalized
/// coordinates (i.e., ∂c/∂q). The two kinds of coordinates need not be equal,
/// which leads to a general, albeit harder to describe definition of the
/// Jacobian matrices as the partial derivatives of the constraint equations
/// taken with respect to the quasi-coordinates (see "Methods for weighting
/// state variable errors" in IntegratorBase); using the notation there for
/// quasi-coordinates means we write the Jacobian as ∂c/∂q̅ (quasi-coordinates
/// possess the property that ∂q̅/∂v = Iₙₓₙ (the n × n identity matrix).
/// Fortunately, for constraints defined strictly in the form c(q), the
/// Jacobians are described completely by the equation ċ = ∂c/∂q̅⋅v, where v are
/// the generalized velocities of the system. Since the problem data
/// specifically requires operators that compute (∂c/∂q̅⋅v), one can simply
/// evaluate ċ.
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
/// - α ∈ ℝ   A non-negative scalar used to correct position-level constraint
///           errors (i.e., "stabilize" the position constraints) via an error
///           feedback process (Baumgarte Stabilization).
/// - β ∈ ℝ   A non-negative scalar used to correct velocity-level constraint
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
  /// point of contact), out of the set of all contact indices (0...n-1).
  /// This vector must be in sorted order.
  std::vector<int> sliding_contacts;

  /// The indices of the non-sliding contacts (those contacts at which there
  /// is zero relative velocity between bodies in the plane tangent to the
  /// point of contact), out of the set of all contact indices (0...n-1).
  /// This vector must be in sorted order.
  std::vector<int> non_sliding_contacts;

  /// The number of spanning vectors in the contact tangents (used to linearize
  /// the friction cone) at the n *non-sliding* contact points. For contact
  /// problems in two dimensions, each element of r will be one. For contact
  /// problems in three dimensions, a friction pyramid (for example), for a
  /// contact point i will have rᵢ = 2. [Anitescu 1997] define k such vectors
  /// and require that, for each vector w in the spanning set, -w also exists
  /// in the spanning set. The RigidContactAccelProblemData structure expects
  /// that the contact solving mechanism negates the spanning vectors so `r` =
  /// k/2 spanning vectors will correspond to a k-edge polygon friction cone
  /// approximation.
  std::vector<int> r;

  /// Coefficients of friction for the s = n - y sliding contacts (where `y` is
  /// the number of non-sliding contacts). The size of this vector should be
  /// equal to `sliding_contacts.size()`.
  VectorX<T> mu_sliding;

  /// Coefficients of friction for the y = n - s non-sliding contacts (where `s`
  /// is the number of sliding contacts). The size of this vector should be
  /// equal to `non_sliding_contacts.size()`.
  VectorX<T> mu_non_sliding;

  /// @name Data for bilateral constraints at the acceleration level
  /// Problem data for bilateral constraints of functions of system
  /// acceleration, where the constraint can be formulated as:<pre>
  /// 0 = G(q)⋅v̇ + kᴳ(t,q,v)
  /// </pre>
  /// which implies the constraint definition c(t,q,v,v̇) ≡ G(q)⋅v̇ + kᴳ(t,q,v).
  /// G is defined as the ℝᵇˣᵐ Jacobian matrix that transforms generalized
  /// velocities (v ∈ ℝᵐ) into the time derivatives of b bilateral constraint
  /// functions. The class of constraint functions naturally includes holonomic
  /// constraints, which are constraints posable as g(t,q). Such holonomic
  /// constraints must be twice differentiated with respect to time to yield
  /// an acceleration-level formulation (i.e., g̈(t, q, v, v̇), for the
  /// aforementioned definition of g(t,q)). That differentiation yields
  /// g̈ = G⋅v̇ + dG/dt⋅v, which is consistent with the constraint class under
  /// the definition kᴳ(t,q,v) ≡ dG/dt⋅v. An example such (holonomic) constraint
  /// function is the transmission (gearing) constraint below:<pre>
  /// 0 = v̇ᵢ - rv̇ⱼ
  /// </pre>
  /// which can be read as the acceleration at joint i (v̇ᵢ) must equal to `r`
  /// times the acceleration at joint j (v̇ⱼ); `r` is thus the gear ratio.
  /// In this example, the corresponding holonomic constraint function is
  /// g(q) ≡ qᵢ - rqⱼ, yielding ̈g(q, v, v̇) = v̇ᵢ - rv̇ⱼ.
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
  /// along the contact surface normal, for n point contacts.
  /// These data center around two Jacobian matrices, N and Q. N is the ℝⁿˣᵐ
  /// Jacobian matrix that transforms generalized velocities (v ∈ ℝᵐ) into
  /// velocities projected along the contact normals at the n point contacts.
  /// Q ∈ ℝⁿˣᵐ is the Jacobian matrix that transforms generalized velocities
  /// (m is the dimension of generalized velocity) into velocities projected
  /// along the directions of sliding at the s *sliding* contact points (rows
  /// of Q that correspond to non-sliding contacts should be zero). Finally,
  /// the Jacobian matrix N allows formulating the non-interpenetration
  /// constraint (a constraint imposed at the velocity level) as:<pre>
  /// 0 ≤ N(q)⋅v̇ + kᴺ(t,q,v)  ⊥  fᶜ ≥ 0
  /// </pre>
  /// which means that the constraint c̈(q,v,v̇) ≡ N(q)⋅v̇ + kᴺ(t,q,v) is
  /// coupled to a force constraint (fᶜ ≥ 0) and a complementarity constraint
  /// fᶜ⋅(Nv̇ + kᴺ(t,q,v)) = 0, meaning that the constraint can apply no force
  /// if it is inactive (i.e., if c̈(q,v,v̇) is strictly greater than zero).
  /// Note that differentiating the original constraint ċ(t,q,v) ≡ Nv (i.e.,
  /// the constraint posed at the velocity level) once with
  /// respect to time, such that all constraints are imposed at the
  /// acceleration level, yields: <pre>
  /// c̈(t,q,v,v̇) = N(q) v̇ + dN/dt(q,v) v
  /// </pre>
  /// Thus, the constraint at the acceleration level can be realized by setting
  /// kᴺ(t,q,v) = dN/dt(q,v)⋅v. If there is pre-existing constraint error (e.g.,
  /// if N(q)⋅v < 0), the kᴺ term can be used to "stabilize" this error.
  /// For example, one could set `kᴺ(t,q,v) = dN/dt(q,v)⋅v + α⋅N(q)⋅v`, for
  /// α ≥ 0.
  /// </pre>
  /// @{

  /// An operator that performs the multiplication N⋅v.
  /// The default operator returns an empty vector.
  std::function<VectorX<T>(const VectorX<T>&)> N_mult;

  /// An operator that performs the multiplication (Nᵀ - μQᵀ)⋅f, where μ is a
  /// diagonal matrix with nonzero entries corresponding to the coefficients of
  /// friction at the s sliding contact points, and (Nᵀ - μQᵀ) transforms forces
  /// (f ∈ ℝⁿ) applied along the contact normals at the n point contacts into
  /// generalized forces. The default operator returns a zero vector of
  /// dimension equal to that of the generalized forces.
  std::function<VectorX<T>(const VectorX<T>&)> N_minus_muQ_transpose_mult;

  /// This ℝⁿ vector is the vector kᴺ(t,q,v) defined above.
  VectorX<T> kN;
  /// @}

  /// @name Data for non-sliding contact friction constraints
  /// Problem data for constraining the tangential acceleration of two bodies
  /// projected along the contact surface tangents, for n point contacts.
  /// These data center around the Jacobian matrix, F ∈ ℝʸʳˣᵐ, that
  /// transforms generalized velocities (v ∈ ℝᵐ) into velocities projected
  /// along the r vectors that span the contact tangents at the y *non-sliding*
  /// point contacts. For contact problems in two dimensions, r would be one.
  /// For a friction pyramid in three dimensions, r would be two. While the
  /// definition of the dimension of the Jacobian matrix above indicates that
  /// every one of the y non-sliding contacts uses the same "r", the code
  /// imposes no such requirement.
  ///
  /// Finally, the Jacobian matrix F allows formulating the non-sliding friction
  /// force constraints as:<pre>
  /// 0 ≤ F(q)⋅v̇ + kᶠ(t,q,v) + λe  ⊥  fᶜ ≥ 0
  /// </pre>
  /// which means that the constraint c̈(t,q,v,v̇) ≡ F(q)⋅v̇ + kᶠ(t,q,v) is
  /// coupled to a force constraint (fᶜ ≥ 0) and a complementarity constraint
  /// fᶜ⋅(Fv̇ + kᴺ(t,q,v) + λe) = 0: the constraint can apply no
  /// force if it is inactive (i.e., if c̈(t,q,v,v̇) is strictly greater than
  /// zero). The presence of the λe term is taken directly from [Anitescu 1997],
  /// where e is a vector of ones and zeros and λ corresponds roughly to the
  /// tangential acceleration at the contacts. The interested reader should
  /// refer to [Anitescu 1997] for a more thorough explanation of this
  /// constraint; the full constraint equation is presented only to elucidate
  /// the purpose of the kᶠ term. Analogously to the case of kᴺ, kᶠ should be
  /// set to dF/dt(q,v)⋅v; also analogously, kᶠ can be used to perform
  /// constraint stabilization.
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
  /// @}

  /// @name Data for unilateral constraints at the acceleration level
  /// Problem data for unilateral constraints of functions of system
  /// acceleration, where the constraint can be formulated as:<pre>
  /// 0 ≤ L(q)⋅v̇ + kᴸ(t,q,v)  ⊥  fᶜ ≥ 0
  /// </pre>
  /// which means that the constraint c(q,v,v̇) ≡ L(q)⋅v̇ + kᴸ(t,q,v) is coupled
  /// to a force constraint (fᶜ ≥ 0) and a complementarity constraint
  /// fᶜ⋅(L⋅v̇ + kᴸ(t,q,v)) = 0, meaning that the constraint can apply no force
  /// if it is inactive (i.e., if c(q,v,v̇) is strictly greater than zero). L
  /// is defined as the ℝˢˣᵐ Jacobian matrix that transforms generalized
  /// velocities (v ∈ ℝᵐ) into the time derivatives of s unilateral constraint
  /// functions. The class of constraint functions naturally includes holonomic
  /// constraints, which are constraints posable as g(t,q). Such holonomic
  /// constraints must be twice differentiated with respect to time to yield
  /// an acceleration-level formulation (i.e., g̈(q, v, v̇, t), for the
  /// aforementioned definition of g(t,q)). That differentiation yields
  /// g̈ = L⋅v̇ + dL/dt⋅v, which is consistent with the constraint class under
  /// the definition kᴸ(t,q,v) ≡ dL/dt⋅v. An example such holonomic constraint
  /// function is a joint acceleration limit:<pre>
  /// 0 ≤ -v̇ⱼ + r  ⊥  fᶜⱼ ≥ 0
  /// </pre>
  /// which can be read as the acceleration at joint j (v̇ⱼ) must be no larger
  /// than r, the force must be applied to limit the acceleration at the joint,
  /// and the limiting force cannot be applied if the acceleration at the
  /// joint is not at the limit (i.e., v̇ⱼ < r). In this example, the
  /// corresponding holonomic constraint function is g(t,q) ≡ -qⱼ + rt²,
  /// yielding ̈g(q, v, v̇) = -v̇ⱼ + r.
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
  /// @}

  /// The ℝᵐ vector tau, the generalized external force vector that
  /// comprises gravitational, centrifugal, Coriolis, actuator, etc. forces
  /// applied to the rigid body system at q. m is the dimension of the
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
template <class T>
struct ConstraintVelProblemData {
  /// Constructs velocity problem data for a system with a @p gv_dim dimensional
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
  /// the friction cone) at the n contact points. For contact
  /// problems in two dimensions, each element of r will be one. For contact
  /// problems in three dimensions, a friction pyramid (for example), for a
  /// contact point i will have rᵢ = 2. [Anitescu 1997] define k such vectors
  /// and require that, for each vector w in the spanning set, -w also exists
  /// in the spanning set. The RigidContactVelProblemData structure expects
  /// that the contact solving mechanism negates the spanning vectors so `r` =
  /// k/2 spanning vectors will correspond to a k-edge polygon friction cone
  /// approximation.
  std::vector<int> r;

  /// Coefficients of friction for the n contacts. This problem specification
  /// does not distinguish between static and dynamic friction coefficients.
  VectorX<T> mu;

  /// @name Data for bilateral constraints at the velocity level
  /// Problem data for bilateral constraints of functions of system
  /// velocity, where the constraint can be formulated as:<pre>
  /// 0 = G(q)⋅v + kᴳ(t,q)
  /// </pre>
  /// which implies the constraint definition c(t,q,v) ≡ G(q)⋅v + kᴳ(t,q). G
  /// is defined as the ℝᵇˣᵐ Jacobian matrix that transforms generalized
  /// velocities (v ∈ ℝᵐ) into the time derivatives of b bilateral constraint
  /// functions. The class of constraint functions naturally includes holonomic
  /// constraints, which are constraints posable as g(t,q). Such holonomic
  /// constraints must be differentiated with respect to time to yield
  /// a velocity-level formulation (i.e., ġ(t, q, v), for the
  /// aforementioned definition of g(t,q)). That differentiation yields
  /// ġ = G⋅v, which is consistent with the constraint class under
  /// the definition kᴳ(t,q) ≡ 0. An example such holonomic constraint
  /// function is the transmission (gearing) constraint below:<pre>
  /// 0 = vᵢ - rvⱼ
  /// </pre>
  /// which can be read as the velocity at joint i (vᵢ) must equal to `r`
  /// times the velocity at joint j (vⱼ); `r` is thus the gear ratio.
  /// In this example, the corresponding holonomic constraint function is
  /// g(q) ≡ qᵢ - rqⱼ, yielding ġ(q, v) = vᵢ - rvⱼ.
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
  /// along the contact surface normal, for n point contacts.
  /// These data center around the Jacobian matrix N, the ℝⁿˣᵐ
  /// Jacobian matrix that transforms generalized velocities (v ∈ ℝᵐ) into
  /// velocities projected along the contact normals at the n point contacts.
  /// Constraint error (φ < 0, where φ is the signed distance between two
  /// bodies) can be incorporated into the constraint solution process (and
  /// thereby reduced) through setting the `kN` term to something other than its
  /// nonzero value (typically `kN = αφ`, where `α ≥ 0`). The resulting
  /// constraint on the motion will be: <pre>
  /// 0 ≤ N(q) v + kᴺ(t,q)  ⊥  fᶜ ≥ 0
  /// </pre>
  /// which means that the constraint ċ(q,v) ≡ N(q)⋅v + kᴺ(t,q) is coupled
  /// to an impulsive force constraint (fᶜ ≥ 0) and a complementarity constraint
  /// fᶜ⋅(Nv + kᴺ(t,q)) = 0, meaning that the constraint can apply no force
  /// if it is inactive (i.e., if ċ(q,v) is strictly greater than zero).
  /// @{

  /// An operator that performs the multiplication N⋅v. The default operator
  /// returns an empty vector.
  std::function<VectorX<T>(const VectorX<T>&)> N_mult;

  /// An operator that performs the multiplication Nᵀ⋅f, where f ∈ ℝⁿ are the
  /// the magnitudes of the impulsive forces applied along the contact normals
  /// at the n point contacts. The default operator returns a zero vector of
  /// dimension equal to that of the generalized velocities (which should be
  /// identical to the dimension of the generalized forces).
  std::function<VectorX<T>(const VectorX<T>&)> N_transpose_mult;

  /// This ℝⁿ vector is the vector kᴺ(t,q,v) defined above.
  VectorX<T> kN;
  /// @}

  /// @name Data for constraints on contact friction
  /// Problem data for constraining the tangential velocity of two bodies
  /// projected along the contact surface tangents, for n point contacts.
  /// These data center around the Jacobian matrix, F ∈ ℝⁿʳˣᵐ, that
  /// transforms generalized velocities (v ∈ ℝᵐ) into velocities projected
  /// along the r vectors that span the contact tangents at the n
  /// point contacts. For contact problems in two dimensions, r would be one.
  /// For a friction pyramid in three dimensions, r would be two. While the
  /// definition of the dimension of the Jacobian matrix above indicates that
  /// every one of the n contacts uses the same "r", the code imposes no such
  /// requirement. Constraint error (F⋅v < 0) can be reduced through the
  /// constraint solution process by setting the `kF` term to something other
  /// than its default zero value. The resulting constraint on the motion will
  /// be:<pre>
  /// 0 ≤ F(q)⋅v + kᴺ(t,q) + eλ  ⊥  fᶜ ≥ 0
  /// </pre>
  /// which means that the constraint ċ(q,v) ≡ F(q)⋅v + kᶠ(t,q) + eλ is coupled
  /// to an impulsive force constraint (fᶜ ≥ 0) and a complementarity constraint
  /// fᶜ⋅(Fv + kᶠ(t,q) + eλ) = 0, meaning that the constraint can apply no force
  /// if it is inactive (i.e., if ċ(q,v) is strictly greater than zero).
  /// The presence of the λe term is taken directly from [Anitescu 1997],
  /// where e is a vector of ones and zeros and λ corresponds roughly to the
  /// tangential acceleration at the contacts. The interested reader should
  /// refer to [Anitescu 1997] for a more thorough explanation of this
  /// constraint; the full constraint equation is presented only to elucidate
  /// the purpose of the kᶠ term.
  /// @{

  /// An operator that performs the multiplication F⋅v. The default operator
  /// returns an empty vector.
  std::function<VectorX<T>(const VectorX<T>&)> F_mult;

  /// An operator that performs the multiplication Fᵀ⋅f, where f ∈ ℝⁿʳ
  /// corresponds to frictional impulsive force magnitudes. The default
  /// operator returns a zero vector of dimension equal to that of the
  /// generalized forces.
  std::function<VectorX<T>(const VectorX<T>&)> F_transpose_mult;

  /// This ℝʸʳ vector is the vector kᶠ(t,q,v) defined above.
  VectorX<T> kF;
  /// @}

  /// @name Data for unilateral constraints at the velocity level
  /// Problem data for unilateral constraints of functions of system
  /// velocity, where the constraint can be formulated as:<pre>
  /// 0 ≤ L(q)⋅v + kᴸ(t,q)  ⊥  fᶜ ≥ 0
  /// </pre>
  /// which means that the constraint ċ(q,v) ≡ L(q)⋅v + kᴸ(t,q) is coupled
  /// to an impulsive force constraint (fᶜ ≥ 0) and a complementarity constraint
  /// fᶜ⋅(L⋅v + kᴸ(t,q)) = 0, meaning that the constraint can apply no force
  /// if it is inactive (i.e., if ċ(q,v) is strictly greater than zero). L
  /// is defined as the ℝˢˣᵐ Jacobian matrix that transforms generalized
  /// velocities (v ∈ ℝᵐ) into the time derivatives of s unilateral constraint
  /// functions. The class of constraint functions naturally includes holonomic
  /// constraints, which are constraints posable as g(q, t). Such holonomic
  /// constraints must be differentiated with respect to time to yield
  /// a velocity-level formulation (i.e., ġ(q, v, t), for the aforementioned
  /// definition of g(q, t)). That differentiation yields ġ = L⋅v, which is
  /// consistent with the constraint class under the definition kᴸ(t,q) ≡ 0. An
  /// example such holonomic constraint function is a joint velocity limit:<pre>
  /// 0 ≤ -vⱼ + r  ⊥  fᶜⱼ ≥ 0
  /// </pre>
  /// which can be read as the velocity at joint j (vⱼ) must be no larger than
  /// r, the impulsive force must be applied to limit the acceleration at the
  /// joint, and the limiting force cannot be applied if the velocity at the
  /// joint is not at the limit (i.e., vⱼ < r). In this example, the constraint
  /// function is g(t,q) ≡ qⱼ + rt, yielding ġ(q, v) = -vⱼ + r.
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
  /// @}

  /// The ℝᵐ vector v, the generalized velocity immediately before any impulsive
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
