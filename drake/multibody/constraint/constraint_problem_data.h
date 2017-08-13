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

  /// An operator that performs the multiplication N⋅v, where N is the ℝⁿˣᵐ
  /// Jacobian matrix that transforms generalized velocities (v ∈ ℝᵐ) into
  /// velocities projected along the contact normals at the n point contacts.
  /// The default operator returns an empty vector.
  std::function<VectorX<T>(const VectorX<T>&)> N_mult;

  /// An operator that performs the multiplication (Nᵀ - μQᵀ)⋅f, where N is the
  /// ℝⁿˣᵐ Jacobian matrix defined immediately above, Q ∈ ℝⁿˣᵐ is the Jacobian
  /// matrix that transforms generalized velocities (m is the dimension of
  /// generalized velocity) into velocities projected along the directions of
  /// sliding at the s *sliding* contact points (rows of Q that correspond to
  /// non-sliding contacts should be zero), μ is a diagonal matrix with nonzero
  /// entries corresponding to the coefficients of friction
  /// at the s sliding contact points, and (Nᵀ - μQᵀ) transforms forces
  /// (f ∈ ℝⁿ) applied along the contact normals at the n point contacts into
  /// generalized forces. The default operator returns a zero vector of
  /// dimension equal to that of the generalized forces.
  std::function<VectorX<T>(const VectorX<T>&)> N_minus_muQ_transpose_mult;

  /// This ℝⁿ vector is the time derivative of the matrix N (defined above)
  /// times the generalized velocity (∈ ℝᵐ) of the rigid body system.
  VectorX<T> Ndot_x_v;

  /// An operator that performs the multiplication F⋅v, where F is the ℝʸʳˣᵐ
  /// Jacobian matrix that transforms generalized velocities (v ∈ ℝᵐ) into
  /// velocities projected along the r vectors that span the contact tangents at
  /// the y *non-sliding* point contacts. For contact problems in two
  /// dimensions, r would be one. For a friction pyramid in three dimensions, r
  /// would be two. While the definition of the dimension of the Jacobian matrix
  /// above indicates that every one of the y non-sliding contacts uses the
  /// same "r", the code imposes no such requirement. The default operator
  /// returns an empty vector.
  std::function<VectorX<T>(const VectorX<T>&)> F_mult;

  /// An operator that performs the multiplication Fᵀ⋅f, where F is the ℝʸʳˣᵐ
  /// Jacobian matrix defined immediately above and Fᵀ transforms forces
  /// (f ∈ ℝʸʳ) applied along the r vectors that span the contact tangents at
  /// the y *non-sliding* contact points into generalized forces. For contact
  /// problems in two dimensions, r would be one. For a friction pyramid in
  /// three dimensions, r would be two. While the definition of the dimension of
  /// the Jacobian matrix above indicates that every one of the y non-sliding
  /// contacts uses the same "r", the code imposes no such requirement. The
  /// default operator returns a zero vector of dimension equal to that of the
  /// generalized forces.
  std::function<VectorX<T>(const VectorX<T>&)> F_transpose_mult;

  /// This ℝʸʳ vector is the time derivative of the matrix F (defined above)
  /// times the generalized velocity (∈ ℝᵐ) of the rigid body system. As above,
  /// m is the dimension of the system generalized velocity.
  VectorX<T> Fdot_x_v;

  /// The number of limit constraints. Must be equivalent to the dimension of
  /// the output of L_mult (defined below).
  int num_limit_constraints{0};

  /// An operator that performs the multiplication L⋅v, where L is the ℝⁱˣᵐ
  /// Jacobian matrix that transforms generalized velocities (v ∈ ℝᵐ) into
  /// the time derivatives of unilateral constraint functions over the system
  /// state. A simple such constraint function is a joint velocity limit:<pre>
  /// 0 ≤ -vⱼ + k  ⊥  -fᶜⱼ ≥ 0
  /// </pre>
  /// which can be read as the joint velocity (vⱼ) must be no larger than k,
  /// the force must be applied to limit the joint velocity, and the limiting
  /// force cannot be applied if the joint velocity is not at the limit. The
  /// default operator returns an empty vector.
  std::function<VectorX<T>(const VectorX<T>&)> L_mult;

  /// An operator that performs the multiplication Lᵀ⋅f, where L is the ℝⁱˣᵐ
  /// Jacobian matrix defined immediately above and Lᵀ transforms constraint
  /// forces (f ∈ ℝⁿ) into generalized forces. The default operator returns a
  /// zero vector of dimension equal to that of the generalized forces.
  std::function<VectorX<T>(const VectorX<T>&)> L_transpose_mult;

  /// This ℝⁱ vector is the time derivative of the matrix L (defined above)
  /// times the generalized velocity (∈ ℝᵐ) of the rigid body system. As above,
  /// m is the dimension of the system generalized velocity;
  VectorX<T> Ldot_x_v;

  /// The ℝᵐ vector f, the generalized external force vector that
  /// comprises gravitational, centrifugal, Coriolis, actuator, etc. forces
  /// applied to the rigid body system at q. m is the dimension of the
  /// generalized force, which is also equal to the dimension of the
  /// generalized velocity.
  VectorX<T> f;

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

  /// An operator that performs the multiplication N⋅v, where N is the ℝⁿˣᵐ
  /// Jacobian matrix that transforms generalized velocities (v ∈ ℝᵐ) into
  /// velocities projected along the contact normals at the n point contacts.
  /// The default operator returns an empty vector.
  std::function<VectorX<T>(const VectorX<T>&)> N_mult;

  /// An operator that performs the multiplication Nᵀ⋅f, where N is the ℝⁿˣᵐ
  /// Jacobian matrix defined immediately above and Nᵀ transforms forces
  /// (f ∈ ℝⁿ) applied along the contact normals at the n point contacts into
  /// generalized forces. The default operator returns a zero vector of
  /// dimension equal to that of the generalized velocities (which should be
  /// identical to the dimension of the generalized forces).
  std::function<VectorX<T>(const VectorX<T>&)> N_transpose_mult;

  /// An operator that performs the multiplication F⋅v, where F is the ℝⁿʳˣᵐ
  /// Jacobian matrix that transforms generalized velocities (v ∈ ℝᵐ) into
  /// velocities projected along the r vectors that span the contact tangents at
  /// the n point contacts. For contact problems in two dimensions, r would be
  /// one. For a friction pyramid in three dimensions, r would be two. While the
  /// definition of the dimension of the Jacobian matrix above indicates that
  /// every one of the n contacts uses the same "r", the code imposes no such
  /// requirement. The default operator returns an empty vector.
  std::function<VectorX<T>(const VectorX<T>&)> F_mult;

  /// An operator that performs the multiplication Fᵀ⋅f, where F is the ℝⁿʳˣᵐ
  /// Jacobian matrix defined immediately above and Fᵀ transforms forces
  /// (f ∈ ℝⁿ) applied along the r vectors that span the contact tangents at the
  /// n contact points into generalized forces. For contact problems in two
  /// dimensions, r would be one. For a friction pyramid in three dimensions, r
  /// would be two. While the definition of the dimension of the Jacobian
  /// matrix above indicates that every one of the n contacts uses the same "r",
  /// the code imposes no such requirement. The default operator returns a zero
  /// vector of dimension equal to that of the generalized velocities (which
  /// should be identical to the dimension of the generalized forces).
  std::function<VectorX<T>(const VectorX<T>&)> F_transpose_mult;

  /// The number of limit constraints. Must be equivalent to the dimension of
  /// the output of L_mult (defined below).
  int num_limit_constraints{0};

  /// An operator that performs the multiplication L⋅v, where L is the ℝⁱˣᵐ
  /// Jacobian matrix that transforms generalized velocities (v ∈ ℝᵐ) into
  /// the time derivatives of unilateral constraint functions over the system
  /// state. A simple such constraint function is a joint velocity limit:<pre>
  /// 0 ≤ -vⱼ + k  ⊥  -fᶜⱼ ≥ 0
  /// </pre>
  /// which can be read as the joint velocity (vⱼ) must be no larger than k,
  /// the force must be applied to limit the joint velocity, and the limiting
  /// force cannot be applied if the joint velocity is not at the limit. The
  /// default operator returns an empty vector.
  std::function<VectorX<T>(const VectorX<T>&)> L_mult;

  /// An operator that performs the multiplication Lᵀ⋅f, where L is the ℝⁱˣᵐ
  /// Jacobian matrix defined immediately above and Lᵀ transforms constraint
  /// forces (f ∈ ℝⁿ) into generalized forces. The default operator returns a
  /// zero vector of dimension equal to that of the generalized velocities
  /// (which should be identical to the dimension of the generalized forces).
  std::function<VectorX<T>(const VectorX<T>&)> L_transpose_mult;

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
