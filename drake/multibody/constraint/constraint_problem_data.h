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

    // Set default for transpose operators - returns the appropriately sized
    // zero vector.
    auto zero_gv_dim_fn = [gv_dim](const VectorX<T>&) -> VectorX<T> {
      return VectorX<T>::Zero(gv_dim);
    };
    N_minus_muQ_transpose_mult = zero_gv_dim_fn;
    F_transpose_mult = zero_gv_dim_fn;
    L_transpose_mult = zero_gv_dim_fn;
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
  /// constraints, which are constraints posable as g(q,t). Such holonomic
  /// constraints must be twice differentiated with respect to time to yield
  /// an acceleration-level formulation (i.e., g̈(q, v, v̇, t), for the
  /// aforementioned definition of g(q,t)). That differentiation yields
  /// g̈ = L⋅v̇ + dL/dt⋅v, which is consistent with the constraint class under
  /// the definition kᴸ(t,q,v) ≡ dL/dt⋅v. An example such holonomic constraint
  /// function is a joint acceleration limit:<pre>
  /// 0 ≤ -v̇ⱼ + r  ⊥  fᶜⱼ ≥ 0
  /// </pre>
  /// which can be read as the acceleration at joint j (v̇ⱼ) must be no larger
  /// than r, the force must be applied to limit the acceleration at the joint,
  /// and the limiting force cannot be applied if the acceleration at the
  /// joint is not at the limit (i.e., v̇ⱼ < r). In this example, the
  /// corresponding holonomic constraint function is g(q,t) ≡ qⱼ + rt²,
  /// yielding ̈g(q, v, v̇) = -v̇ⱼ + r.
  /// @{

  /// The number of limit constraints. Must equal `s`, i.e., the
  /// number of columns of L.
  int num_limit_constraints{0};

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

    // Set default for transpose operators - returns the appropriately sized
    // zero vector.
    auto zero_gv_dim_fn = [gv_dim](const VectorX<T>&) -> VectorX<T> {
      return VectorX<T>::Zero(gv_dim); };
    N_transpose_mult = zero_gv_dim_fn;
    F_transpose_mult = zero_gv_dim_fn;
    L_transpose_mult = zero_gv_dim_fn;
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
  /// function is g(q,t) ≡ qⱼ + rt, yielding ġ(q, v) = -vⱼ + r.
  /// @{

  /// The number of limit constraints. Must be equivalent to the dimension of
  /// the output of L_mult (defined below).
  int num_limit_constraints{0};

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
