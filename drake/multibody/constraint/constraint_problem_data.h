#pragma once

#include <vector>

#include <Eigen/Core>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace constraint {

/// Structure for holding constraint data for computing constraint forces
/// at the acceleration-level.
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
  /// the friction cone) at the m *non-sliding* contact points. For contact
  /// problems in two dimensions, each element of r will be one. For contact
  /// problems in three dimensions, a friction pyramid (for example), for a
  /// contact point i will have rᵢ = 2. [Anitescu 1997] define k such vectors
  /// and require that, for each vector w in the spanning set, -w also exists
  /// in the spanning set. The RigidContactAccelProblemData structure expects
  /// that the contact solving mechanism negates the spanning vectors so `r` =
  /// k/2 spanning vectors will correspond to a k-edge polygon friction cone
  /// approximation.
  std::vector<int> r;

  /// Coefficients of friction for the s = m - y sliding contacts (where `y` is
  /// the number of non-sliding contacts). The size of this vector should be
  /// equal to `sliding_contacts.size()`.
  VectorX<T> mu_sliding;

  /// Coefficients of friction for the y = m - s non-sliding contacts (where `s`
  /// is the number of sliding contacts). The size of this vector should be
  /// equal to `non_sliding_contacts.size()`.
  VectorX<T> mu_non_sliding;

  /// @name Data for bilateral constraints at the acceleration level
  /// Problem data for bilateral constraints of functions of system
  /// acceleration, where the constraint can be formulated as:<pre>
  /// 0 = G(q)⋅v̇ + kᴳ(t,q,v)
  /// </pre>
  /// which implies the constraint definition c(t,q,v,v̇) ≡ G(q)⋅v̇ + kᴳ(t,q,v).
  /// G is defined as the ℝᵇˣⁿ Jacobian matrix that transforms generalized
  /// velocities (v ∈ ℝⁿ) into the time derivatives of b bilateral constraint
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
  /// along the contact surface normal, for m point contacts.
  ///
  /// Consider two rigid bodies i and j making contact at a single point, p(q),
  /// which is defined such that pᵢ(q(t)) = pⱼ(q(t)); in other words, a point
  /// defined on each rigid body (and expressed in the world frame) is defined
  /// such that the points coincide at time t. To limit the motion of the
  /// points to the contact surface as the bodies move, one can introduce the
  /// constraint c(q) ≡ n(q)ᵀ(pᵢ(q) - pⱼ(q)), where n(q) is the
  /// surface normal expressed in the world frame. Differentiating c(q) once
  /// with respect to time yields ċ(q,v) ≡ n(q)ᵀ(ṗᵢ(q,v) - ṗⱼ(q,v)) +
  /// ṅ(q,v)ᵀ(pᵢ(q) - pⱼ(q)); one more differentiation with respect to time
  /// yields c̈(q,v,v̇) ≡ n(q)ᵀ(p̈ᵢ(q) - p̈ⱼ(q)) + ṅ(q,v)ᵀ(ṗᵢ(q,v) - ṗⱼ(q,v)) +
  /// n̈(q,v,v̇)ᵀ(pᵢ(q) - pⱼ(q)). By collecting terms and using the
  /// to-be-defined Jacobian matrix N(q), we can introduce equivalent equations:
  /// ċ(q,v) ≡ N⋅v + ṅ(q,v)ᵀ(pᵢ(q) - pⱼ(q)) and
  /// c̈(q,v,v̇) ≡ N⋅v̇ + dN/dt⋅v + n̈(q,v,v̇)ᵀ(pᵢ(q) - pⱼ(q)).
  ///
  /// The Newton-Euler equations (essentially F = ma) coupled with the c(q)
  /// constraint above yields an Index-3 DAE (see [Hairer 1996]), generally
  /// makes the initial value problem hard to solve, computationally speaking;
  /// coupling the Newton-Euler equations with the constraint equation
  /// c̈(q,v,v̇) and the initial conditions c(q) = ċ(q,v) = 0 yields a far more
  /// manageable Index-1 DAE, again with regard to computation. Note that the
  /// assumption c(q) = 0, yielding pᵢ(q) - pⱼ(q) = 0, implies that
  /// ṅ(q,v)ᵀ(pᵢ(q) - pⱼ(q)) (in ċ(q,v)) and n̈(q,v,v̇)ᵀ(pᵢ(q) - pⱼ(q)) (in
  /// c̈(q,v,v̇)) are both zero.
  ///
  /// With this background in mind, N is the ℝᵐˣⁿ Jacobian matrix that
  /// transforms generalized velocities (v ∈ ℝⁿ) into velocities projected along
  /// the contact normals at the m point contacts. The problem data also must
  /// consider Q ∈ ℝᵐˣⁿ, the Jacobian matrix that transforms generalized
  /// velocities (n is the dimension of generalized velocity) into velocities
  /// projected along the directions of sliding at the s *sliding* contact
  /// points (rows of Q that correspond to non-sliding contacts should be zero).
  /// Finally, we introduce a new catch-all term k̅ᴺ(t,q,v) to reformulate
  /// c̈(q,v,v̇) and two other coupled vector constraint equations:<pre>
  /// 0 ≤ N(q)⋅v̇ + k̅ᴺ(t,q,v)  ⊥  fᴺ ≥ 0
  /// </pre>
  /// which means that c̈(q,v,v̇) is coupled to a force constraint that requires
  /// the force to be compressive (fᴺ ≥ 0) and a complementarity constraint
  /// fᴺ⋅(Nv̇ + k̅ᴺ(t,q,v)) = 0, meaning that the constraint can apply no
  /// force if it is inactive (i.e., if c̈(q,v,v̇) is strictly greater than
  /// zero).
  ///
  /// For reasons both numerical (hard constraints are computationally more
  /// challenging to solve to high precision) and qualitative (contact between
  /// bodies is never truly rigid [Lacoursiere 2007]), the problem data permits
  /// solving a a stabilized and softened version of c̈(q,v,v̇):<pre>
  /// c̈ₛ(q,v,v̇) ≡ N(q)⋅v̇ + kᴺ(t,q,v) + γᴺfᴺ
  /// </pre>
  /// where kᴺ(t,q,v) ≡ k̅(t,q,v) + αċ(q,v) + βc(q), for α ≥ 0 and β ≥ 0,
  /// combines k̅(t,q,v) with constraint stabilization terms, thereby allowing
  /// one to reduce ||ċ(q,v)|| and ||c(q)||. The factor γᴺfᴺ, where γᴺ ≥ 0 is a
  /// user-provided diagonal matrix, acts to "soften" the  constraint: if γᴺ is
  /// nonzero, less force will be required to satisfy c̈ₛ.
  /// </pre>
  /// @{

  /// An operator that performs the multiplication N⋅v.
  /// The default operator returns an empty vector.
  std::function<VectorX<T>(const VectorX<T>&)> N_mult;

  /// An operator that performs the multiplication (Nᵀ - μQᵀ)⋅f, where μ is a
  /// diagonal matrix with nonzero entries corresponding to the coefficients of
  /// friction at the s sliding contact points, and (Nᵀ - μQᵀ) transforms forces
  /// (f ∈ ℝᵐ) applied along the contact normals at the m point contacts into
  /// generalized forces. The default operator returns a zero vector of
  /// dimension equal to that of the generalized forces.
  std::function<VectorX<T>(const VectorX<T>&)> N_minus_muQ_transpose_mult;

  /// This ℝᵐ vector is the vector kᴺ(t,q,v) defined above.
  VectorX<T> kN;

  /// This ℝᵐ vector represents the diagonal matrix γᴺ defined above.
  VectorX<T> gammaN;
  /// @}

  /// @name Data for non-sliding contact friction constraints
  /// Problem data for constraining the tangential acceleration of two bodies
  /// projected along the contact surface tangents, for m point contacts.
  /// These data center around the Jacobian matrix, F ∈ ℝʸʳˣⁿ, that
  /// transforms generalized velocities (v ∈ ℝⁿ) into velocities projected
  /// along the r vectors that span the contact tangents at the y *non-sliding*
  /// point contacts. For contact problems in two dimensions, r would be one.
  /// For a friction pyramid in three dimensions, r would be two. While the
  /// definition of the dimension of the Jacobian matrix above indicates that
  /// every one of the y non-sliding contacts uses the same "r", the code
  /// imposes no such requirement.
  ///
  /// The Jacobian matrix F allows formulating the non-sliding friction
  /// force constraints as:<pre>
  /// 0 ≤ F(q)⋅v̇ + kᶠ(t,q,v) + E⋅Λ + γᶠfᶜ  ⊥  fᶜ ≥ 0
  /// 0 ≤ μ⋅fN - Eᵀ⋅fᶜ + γᴱΛ ⊥ Λ ≥ 0
  /// </pre>
  /// which means that the constraint
  /// c̈(t,q,v,v̇) ≡ F(q)⋅v̇ + kᶠ(t,q,v) + E⋅Λ + γᶠfᶜ is coupled to a force
  /// constraint (fᶜ ≥ 0) and a complementarity constraint
  /// fᶜ⋅(Fv̇ + kᴺ(t,q,v) + E⋅Λ + γᶠfᶜ) = 0: the constraint can apply no
  /// force if it is inactive (i.e., if c̈(t,q,v,v̇) is strictly greater than
  /// zero). Λ can roughly be interpreted as the remaining tangential
  /// acceleration at the non-sliding contacts after contact forces have been
  /// applied. E is a binary matrix with blocks of one-vector columns
  /// (refer to [Anitescu 1997]). Note that the second constraint, which
  /// represents a linearized friction cone constraint (i.e., it is a constraint
  /// on the unknown forces), is coupled to a "slack" variable (Λ).
  ///
  /// The factors γᶠfᶜ and γᴱΛ, where γᶠ ≥ 0 and γᴱ ≥ 0 are user-provided
  /// diagonal matrices, act to "soften" the constraints. If γᶠ is nonzero,
  /// c̈ will require less force to be satisfied; if γᴱ is nonzero,
  /// stiction will be treated as satisfied even when some non-zero tangential
  /// acceleration remains at the non-sliding contacts. The
  /// interested reader should refer to [Anitescu 1997] for a more thorough
  /// explanation of these constraints; the full constraint equations are
  /// presented only to elucidate the purpose of the kᶠ, γᶠ, and γᴱ terms.
  /// Analogously to the case of kᴺ, kᶠ should be set to dF/dt(q,v)⋅v; also
  /// analogously, kᶠ can be used to perform constraint stabilization.
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

  /// This ℝᵐ vector represents the diagonal matrix γᴱ defined above.
  VectorX<T> gammaE;
  /// @}

  /// @name Data for unilateral constraints at the acceleration level
  /// Problem data for unilateral constraints of functions of system
  /// acceleration, where the constraint can be formulated as:<pre>
  /// 0 ≤ L(q)⋅v̇ + kᴸ(t,q,v) + γᴸfᶜ  ⊥  fᶜ ≥ 0
  /// </pre>
  /// which means that the constraint c(q,v,v̇) ≡ L(q)⋅v̇ + kᴸ(t,q,v) + γᴸfᶜ
  /// is coupled to a force constraint (fᶜ ≥ 0) and a complementarity constraint
  /// fᶜ⋅(L⋅v̇ + kᴸ(t,q,v) + γᴸfᶜ) = 0, meaning that the constraint can apply no
  /// force if it is inactive (i.e., if c(q,v,v̇) is strictly greater than zero).
  /// L is defined as the ℝˢˣⁿ Jacobian matrix that transforms generalized
  /// velocities (v ∈ ℝⁿ) into the time derivatives of s unilateral constraint
  /// functions. The factor γᴸfᶜ, where γᴸ ≥ 0 is a user-provided diagonal
  /// matrix, acts to "soften" the constraint: if γᴸ is nonzero, c̈ will
  /// be satisfiable with a smaller fᶜ.
  ///
  /// The class of constraint functions naturally includes holonomic
  /// constraints, which are constraints posable as g(t,q). Such holonomic
  /// constraints must be twice differentiated with respect to time to yield
  /// an acceleration-level formulation (i.e., g̈(t, q, v, v̇), for the
  /// aforementioned definition of g(t,q)). That differentiation yields
  /// g̈ = L⋅v̇ + dL/dt⋅v (notice the absence of the softening term), which is
  /// consistent with the constraint class under the definition
  /// kᴸ(t,q,v) ≡ dL/dt⋅v. An example such (holonomic) constraint function is a
  /// joint acceleration limit:<pre>
  /// 0 ≤ -v̇ⱼ  ⊥  fᶜⱼ ≥ 0
  /// </pre>
  /// which can be read as the acceleration at joint j (v̇ⱼ) must be no larger
  /// than zero, the force must be applied to limit the acceleration at the
  /// joint, and the limiting force cannot be applied if the acceleration at the
  /// joint is not at the limit (i.e., v̇ⱼ < 0). In this example, a
  /// corresponding holonomic constraint function would be g(q) ≡ r - qⱼ (where
  /// r is the range of motion limit)  yielding ̈g(q, v, v̇) = -v̇ⱼ. Solving for
  /// v̇(t₀) such that ̈g(q(t₀), v(t₀), v̇(t₀)) = 0 will naturally allow one to
  /// determine (through integration) q(t₁) that satisfies g(q(t₁)) = 0 for
  /// t₁ > t₀ and t₁ ≈ t₀, assuming that g(q(t₀)) = 0 and ġ(q(t₀),v(t₀)) = 0.
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
  /// the friction cone) at the m contact points. For contact
  /// problems in two dimensions, each element of r will be one. For contact
  /// problems in three dimensions, a friction pyramid (for example), for a
  /// contact point i will have rᵢ = 2. [Anitescu 1997] define k such vectors
  /// and require that, for each vector w in the spanning set, -w also exists
  /// in the spanning set. The RigidContactVelProblemData structure expects
  /// that the contact solving mechanism negates the spanning vectors so `r` =
  /// k/2 spanning vectors will correspond to a k-edge polygon friction cone
  /// approximation.
  std::vector<int> r;

  /// Coefficients of friction for the m contacts. This problem specification
  /// does not distinguish between static and dynamic friction coefficients.
  VectorX<T> mu;

  /// @name Data for bilateral constraints at the velocity level
  /// Problem data for bilateral constraints of functions of system
  /// velocity, where the constraint can be formulated as:<pre>
  /// 0 = G(q)⋅v + kᴳ(t,q)
  /// </pre>
  /// which implies the constraint definition c(t,q,v) ≡ G(q)⋅v + kᴳ(t,q). G
  /// is defined as the ℝᵇˣⁿ Jacobian matrix that transforms generalized
  /// velocities (v ∈ ℝⁿ) into the time derivatives of b bilateral constraint
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
  /// which can be read as the velocity at joint j (vⱼ) must equal to `r`
  /// times the velocity at joint i (vᵢ); `r` is thus the gear ratio.
  /// In this example, the corresponding holonomic constraint function is
  /// g(q) ≡ qᵢ -rqⱼ, yielding ġ(q, v) = -vⱼ + - rvⱼ.
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
  /// along the contact surface normal, for m point contacts.
  /// These data center around the Jacobian matrix N, the ℝᵐˣⁿ
  /// Jacobian matrix that transforms generalized velocities (v ∈ ℝⁿ) into
  /// velocities projected along the contact normals at the m point contacts.
  /// Constraint error (φ < 0, where φ is the signed distance between two
  /// bodies) can be incorporated into the constraint solution process (and
  /// thereby reduced) through setting the `kN` term to something other than its
  /// nonzero value (typically `kN = αφ`, where `α ≥ 0`). The resulting
  /// constraint on the motion will be: <pre>
  /// 0 ≤ N(q) v + kᴺ(t,q) + γᴺfᶜ  ⊥  fᶜ ≥ 0
  /// </pre>
  /// which means that the constraint ċ(q,v) ≡ N(q)⋅v + kᴺ(t,q) + γᴺfᶜ is
  /// coupled to an impulsive force constraint (fᶜ ≥ 0) and a complementarity
  /// constraint fᶜ⋅(Nv + kᴺ(t,q) + γᴺfᶜ) = 0, meaning that the constraint can
  /// apply no force if it is inactive (i.e., if ċ(q,v) is strictly greater than
  /// zero). The factor γᴺfᶜ, where γᴺ ≥ 0 is a user-provided diagonal matrix,
  /// acts to  "soften" the constraint: if γᴺ is non-zero, less force will be
  /// required to satisfy c̈.
  /// @{

  /// An operator that performs the multiplication N⋅v. The default operator
  /// returns an empty vector.
  std::function<VectorX<T>(const VectorX<T>&)> N_mult;

  /// An operator that performs the multiplication Nᵀ⋅f, where f ∈ ℝᵐ are the
  /// the magnitudes of the impulsive forces applied along the contact normals
  /// at the m point contacts. The default operator returns a zero vector of
  /// dimension equal to that of the generalized velocities (which should be
  /// identical to the dimension of the generalized forces).
  std::function<VectorX<T>(const VectorX<T>&)> N_transpose_mult;

  /// This ℝᵐ vector is the vector kᴺ(t,q,v) defined above.
  VectorX<T> kN;

  /// This ℝᵐ vector represents the diagonal matrix γᴺ defined above.
  VectorX<T> gammaN;
  /// @}

  /// @name Data for constraints on contact friction
  /// Problem data for constraining the tangential velocity of two bodies
  /// projected along the contact surface tangents, for m point contacts.
  /// These data center around the Jacobian matrix, F ∈ ℝᵐʳˣⁿ, that
  /// transforms generalized velocities (v ∈ ℝⁿ) into velocities projected
  /// along the r vectors that span the contact tangents at the n
  /// point contacts. For contact problems in two dimensions, r would be one.
  /// For a friction pyramid in three dimensions, r would be two. While the
  /// definition of the dimension of the Jacobian matrix above indicates that
  /// every one of theunicontacts uses the same "r", the code imposes no such
  /// requirement. Constraint error (F⋅v < 0) can be reduced through the
  /// constraint solution process by setting the `kF` term to something other
  /// than its default zero value. The resulting constraint on the motion will
  /// be:<pre>
  /// 0 ≤ F(q)⋅v + kᶠ(t,q) + E⋅Λ + γᶠfᶜ ⊥  fᶜ ≥ 0
  /// 0 ≤ μ⋅fN - Eᵀ⋅fᶜ + γᴱΛ ⊥ Λ ≥ 0
  /// </pre>
  /// which means that the constraint ċ(q,v) ≡ F(q)⋅v + kᶠ(t,q) + E⋅Λ + γᶠfᶜ is
  /// coupled to an impulsive force constraint (fᶜ ≥ 0) and a complementarity
  /// constraint fᶜ⋅(Fv + kᶠ(t,q) + E⋅Λ + γᶠfᶜ) = 0, meaning that the constraint
  /// can apply no force if it is inactive (i.e., if ċ(q,v) is strictly greater
  /// than zero). λ can roughly be interpreted as the remaining tangential
  /// velocity at the non-sliding contacts after the impulsive forces are
  /// applied. E is a binary matrix with blocks of one-vector columns
  /// (refer to [Anitescu 1997]). Note that the second constraint, which
  /// represents a linearized friction cone constraint (i.e., it is a constraint
  /// on the unknown forces), is coupled to a "slack" variable (Λ).
  ///
  /// The factors γᶠfᶜ and γᴱΛ, where γᶠ ≥ 0 and γᴱ ≥ 0 are user-provided
  /// diagonal matrices, act to "soften" the constraints. If γᶠ is nonzero,
  /// c̈ will require less force to be satisfied; if γᴱ is nonzero,
  /// stiction will be treated as satisfied even when some non-zero tangential
  /// velocity remains. The interested reader should refer to [Anitescu 1997]
  /// for a more thorough explanation of these constraints; the full constraint
  /// equations are presented only to elucidate the purpose of the kᶠ, γᶠ, and
  /// γᴱ terms. Analogously to the case of kᴺ, kᶠ can be used to perform
  /// constraint stabilization.
  /// @{

  /// An operator that performs the multiplication F⋅v. The default operator
  /// returns an empty vector.
  std::function<VectorX<T>(const VectorX<T>&)> F_mult;

  /// An operator that performs the multiplication Fᵀ⋅f, where f ∈ ℝᵐʳ
  /// corresponds to frictional impulsive force magnitudes. The default
  /// operator returns a zero vector of dimension equal to that of the
  /// generalized forces.
  std::function<VectorX<T>(const VectorX<T>&)> F_transpose_mult;

  /// This ℝʸʳ vector is the vector kᶠ(t,q,v) defined above.
  VectorX<T> kF;

  /// This ℝʸʳ vector represents the diagonal matrix γᶠ defined above.
  VectorX<T> gammaF;

  /// This ℝᵐ vector represents the diagonal matrix γᴱ defined above.
  VectorX<T> gammaE;

  /// @}

  /// @name Data for unilateral constraints at the velocity level
  /// Problem data for unilateral constraints of functions of system
  /// velocity, where the constraint can be formulated as:<pre>
  /// 0 ≤ L(q)⋅v + kᴸ(t,q) + γᴸfᶜ  ⊥  fᶜ ≥ 0
  /// </pre>
  /// which means that the constraint ċ(q,v) ≡ L(q)⋅v + kᴸ(t,q) + γᴸfᶜ is
  /// coupled to an impulsive force constraint (fᶜ ≥ 0) and a complementarity
  /// constraint fᶜ⋅(L⋅v + kᴸ(t,q) + γᴸfᶜ) = 0, meaning that the constraint can
  /// apply no force if it is inactive (i.e., if ċ(q,v) is strictly greater than
  /// zero). L is defined as the ℝˢˣⁿ Jacobian matrix that transforms
  /// generalized velocities (v ∈ ℝⁿ) into the time derivatives of s unilateral
  /// constraint functions. The factor γᴸfᶜ, where γᴸ ≥ 0 is a user-provided
  /// diagonal matrix, acts to "soften" the constraint: if γᴸ is nonzero, c̈
  /// will be satisfiable with a smaller fᶜ.
  ///
  /// The class of constraint functions naturally includes holonomic
  /// constraints, which are constraints posable as g(t, q). Such holonomic
  /// constraints must be differentiated with respect to time to yield
  /// a velocity-level formulation (i.e., ġ(t, q, v), for the aforementioned
  /// definition of g(t,q)). That differentiation- without the inclusion of the
  /// softening term- yields ġ = L⋅v, which is consistent with the constraint
  /// class under the definition kᴸ(t,q) ≡ 0. An example such holonomic
  /// constraint function is a joint velocity limit:<pre>
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
