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
/// <h3>Definition of variables specific to this class</h3>
/// (See @ref variable_definitions) for the more general set of definitions).
/// - s ∈ ℕ   The number of contacts at which sliding is occurring. Note
///           that p = s + y, where p is the number of points of contact.
/// - y ∈ ℕ   The number of contacts at which sliding is not occurring. Note
///           that p = s + y.
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
  /// point of contact), out of the set of all contact indices (0...p-1).
  /// This vector must be in sorted order.
  std::vector<int> sliding_contacts;

  /// The indices of the non-sliding contacts (those contacts at which there
  /// is zero relative velocity between bodies in the plane tangent to the
  /// point of contact), out of the set of all contact indices (0...p-1).
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
  /// Problem data for bilateral constraints at the acceleration level.
  /// The problem structure uses the constraint form:<pre>
  /// 0 = G(q)⋅v̇ + kᴳ(t,q,v)
  /// </pre>
  /// which implies the constraint definition c(t,q,v;v̇) ≡ G(q)⋅v̇ + kᴳ(t,q,v).
  /// G is defined as the ℝᵇˣⁿ Jacobian matrix of the partial derivatives of c()
  /// taken with respect to the quasi-coordinates (see @ref quasi_coordinates).
  /// For holonomic constraints posable as c(t,q), two differentiations with
  /// respect to time can be formulated as c̈ = G⋅v̇ + Gdot⋅v + ∂²c/∂t², which is
  /// consistent with our problem structure requirement by defining
  /// kᴳ(t,q,v) ≡ Gdot⋅v + ∂²c/∂t².
  ///
  /// Given these descriptions, the user needs to define operators for computing
  /// G⋅w (w ∈ ℝⁿ is an arbitrary vector) and Gᵀ⋅f (f ∈ ℝᵇ is an arbitrary
  /// vector). The user also needs to provide kᴳ ∈ ℝᵇ, which should be set to
  /// the vector Gdot⋅v + ∂²c/∂t² (without Baumgarte stabilization) or
  /// Gdot⋅v + ∂²c/∂t² + 2αċ + β²c (with Baumgarte stabilization); see
  /// @ref constraint_stabilization for further details.
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
  /// Starting from the equations in @ref noninterpenetration_constraints,
  /// then collecting terms and using the to-be-defined Jacobian matrix N(q),
  /// we now introduce equivalent equations:<pre>
  /// ċ(q,v) ≡ N⋅v + ṅᵀ⋅(pᵢ - pⱼ)</pre>
  /// and:<pre>
  /// c̈(q,v,v̇) ≡ N⋅v̇ + Ndot⋅v + n̈ᵀ(pᵢ - pⱼ).
  /// </pre>
  /// N is the ℝᵖˣⁿ Jacobian matrix that
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
  /// the vector Ndot⋅v (without Baumgarte Stabilization) or
  /// Ndot⋅v + 2αċ + β²c (with Baumgarte Stabilization, see
  /// @ref constraint_stabilization).
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
  /// Given the descriptions of c(.) (in @ref frictional_constraints) and the
  /// Jacobian matrix F (defined in the following paragraph), the user needs to
  /// define operators for computing F⋅w (w ∈ ℝⁿ is an arbitrary vector) and
  /// Fᵀ⋅f (f ∈ ℝʸʳ is an arbitrary vector). The user also needs to provide
  /// γᶠ ∈ ℝʸʳ (a vector of non-negative entries used to relax the sticking
  /// constraints), γᴱ ∈ ℝʸ (a vector of non-negative entries used to relax the
  /// linearized Coulomb friction cone constraint, i.e., Equation 0*),
  /// and kᶠ ∈ ℝʸʳ, the vector Fdot⋅v + 2αċ(q,v) + β²c(q) (where c(q) is the
  /// collection of Equations 1*-k*).
  ///
  /// Unlike with general constraint
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
  /// Problem data for unilateral constraints at the acceleration level.
  ///
  /// The problem structure uses the constraint form:<pre>
  /// 0 ≤ L(q)⋅v̇ + kᴸ(t,q,v) + γᴸλ
  /// </pre>
  /// which implies the constraint definition<pre>
  /// c(t,q,v;v̇) ≡ L(q)⋅v̇ + kᴸ(t,q,v)
  /// </pre>
  /// where L is defined as the ℝᵘˣⁿ Jacobian matrix of the partial derivatives
  /// of the constraint functions c() (see @ref generic_unilateral) taken with
  /// respect to the quasi-coordinates (see
  /// @ref quasi_coordinates). For holonomic constraints posable as c(t,q),
  /// two differentiations of c() with respect to time can be formulated as
  /// c̈ = L⋅v̇ + Ldot⋅v + ∂²c/∂t², which is consistent with our problem structure
  /// requirement by defining kᴸ(t,q,v) ≡ Ldot⋅v + ∂²c/∂t². Note that any
  /// constraint softening- which would be effected through positive γᴸ- is
  /// simply "mixed in" to the constraint; it does not emerge through any
  /// differentiations of the original holonomic constraint.
  ///
  /// Given these descriptions, the user needs to define operators for computing
  /// L⋅w (w ∈ ℝⁿ is an arbitrary vector) and Lᵀ⋅f (f ∈ ℝᵘ is an arbitrary
  /// vector). The user also needs to provide kᴸ ∈ ℝᵘ, which should be set to
  /// the vector Ldot⋅v + ∂²c/∂t² + 2αċ + β²c, and the vector γᴸ ≥ 0, which
  /// corresponds to a diagonal matrix that acts to "soften" the constraint
  /// (see @ref constraint_softening).
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
  /// Problem data for bilateral constraints at the velocity level.
  /// The problem structure uses the constraint form:<pre>
  /// 0 = G(q)⋅v + kᴳ(t,q)
  /// </pre>
  /// which implies the constraint definition c(t,q;v) ≡ G(q)⋅v + kᴳ(t,q).
  /// G is defined as the ℝᵇˣⁿ Jacobian matrix of the partial derivatives of c()
  /// taken with respect to the quasi-coordinates (see @ref quasi_coordinates).
  /// For holonomic constraints posable as c(t,q), differentiation with
  /// respect to time can be formulated as ċ = G⋅v + ∂c/∂t, which is
  /// consistent with our problem structure requirement by defining
  /// kᴳ(t,q) ≡ ∂c/∂t.
  ///
  /// Given these descriptions, the user needs to define operators for computing
  /// G⋅w (w ∈ ℝⁿ is an arbitrary vector) and Gᵀ⋅f (f ∈ ℝᵇ is an arbitrary
  /// vector). The user also needs to provide kᴳ ∈ ℝᵇ, which should be set to
  /// the vector ∂c/∂t (without Baumgarte stabilization) or
  /// ∂c/∂t + βc (with Baumgarte stabilization); see
  /// @ref constraint_stabilization for further details.
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
  /// Starting from the equations in @ref noninterpenetration_constraints,
  /// then collecting terms and using the to-be-defined Jacobian matrix N(q),
  /// we now introduce an equivalent equation:<pre>
  /// ċ(q,v) ≡ N⋅v + ṅᵀ⋅(pᵢ - pⱼ)
  /// </pre>
  /// N is the ℝᵖˣⁿ Jacobian matrix that
  /// transforms generalized velocities (v ∈ ℝⁿ) into velocities projected along
  /// the contact normals at the p point contacts.
  ///
  /// Given this description, the user needs to define operators for computing
  /// N⋅w (w ∈ ℝⁿ is an arbitrary vector) and Nᵀ⋅f (f ∈ ℝᵖ is an arbitrary
  /// vector). The user also needs to provide γᴺ ∈ ℝᵖ, a vector of non-negative
  /// entries used to soften the non-interpenetration constraints, and kᴺ ∈ ℝᵖ,
  /// the vector βc(q).
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
  /// Given the descriptions of c(.) (in @ref frictional_constraints) and the
  /// Jacobian matrix F (defined in the following paragraph), the user needs to
  /// define operators for computing F⋅w (w ∈ ℝⁿ is an arbitrary vector) and
  /// Fᵀ⋅f (f ∈ ℝʸʳ is an arbitrary vector). The user also needs to provide
  /// γᶠ ∈ ℝʸʳ (a vector of non-negative entries used to relax the sticking
  /// constraints), γᴱ ∈ ℝʸ (a vector of non-negative entries used to relax the
  /// linearized Coulomb friction cone constraint, i.e., Equation 0⁺ above),
  /// and kᶠ ∈ ℝʸʳ, the vector βc(q) (where c(q) is the
  /// collection of Equations 1⁺-k⁺).
  ///
  /// Unlike with general constraint
  /// softening, γᶠ and γᴱ are useful primarily as regularization parameters for
  /// the numerical solution process: one typically selects these parameters to
  /// be as small as possible while still permitting the resulting
  /// complementarity problems to be solved reliably. The resulting stiction
  /// drift should be so small that no constraint stabilization (i.e., setting
  /// β = 0) should be sufficient for most applications.
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
  /// Problem data for unilateral constraints at the velocity level.
  ///
  /// The problem structure uses the constraint form:<pre>
  /// 0 ≤ L(q)⋅v + kᴸ(t,q) + γᴸλ
  /// </pre>
  /// which implies the constraint definition<pre>
  /// c(t,q;v) ≡ L(q)⋅v + kᴸ(t,q)
  /// </pre>
  /// where L is defined as the ℝᵘˣⁿ Jacobian matrix of the partial derivatives
  /// of c() taken with respect to the quasi-coordinates (see
  /// @ref quasi_coordinates). For holonomic constraints posable as c(t,q),
  /// differentiation of c() with respect to time can be formulated as
  /// ċ = L⋅v + ∂c/∂t, which is consistent with our problem structure
  /// requirement by defining kᴸ(t,q) ≡ ∂c/∂t. Note that any
  /// constraint softening- which would be effected through positive γᴸ- is
  /// simply "mixed in" to the constraint; it does not emerge through
  /// differentiation of the original holonomic constraint.
  ///
  /// Given these descriptions, the user needs to define operators for computing
  /// L⋅w (w ∈ ℝⁿ is an arbitrary vector) and Lᵀ⋅f (f ∈ ℝᵘ is an arbitrary
  /// vector). The user also needs to provide kᴸ ∈ ℝᵘ, which should be set to
  /// the vector ∂c/∂t + βc, and the vector γᴸ ≥ 0, which
  /// corresponds to a diagonal matrix that acts to "soften" the constraint
  /// (see @ref constraint_softening).
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
