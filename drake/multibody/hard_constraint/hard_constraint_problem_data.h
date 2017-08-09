#pragma once

#include <vector>

#include <Eigen/Core>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace hard_constraint {

/// Structure for holding hard constraint data for computing constraint forces
/// under the hard constraint model at the acceleration-level.
template <class T>
struct HardConstraintAccelProblemData {
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

  /// The ℝⁿˣᵐ Jacobian matrix that transforms generalized velocities (m is the
  /// dimension of generalized velocity) into velocities projected along the
  /// contact normals at the n contact points.
  MatrixX<T> N;

  /// This ℝⁿ vector is the time derivative of the matrix N (defined above)
  /// times the generalized velocity (∈ ℝᵐ) of the rigid body system.
  VectorX<T> Ndot_x_v;

  /// The ℝʸʳˣᵐ Jacobian matrix that transforms generalized velocities (m is the
  /// dimension of generalized velocity) into velocities projected along
  /// r vectors that span the contact tangents (used to linearize the friction
  /// cone) at the n *non-sliding* contact points. For contact problems in two
  /// dimensions, r will be one. For a friction pyramid in three dimensions, r
  /// would be two. While the definition of the dimension of the Jacobian matrix
  /// above indicates that every one of the y non-sliding contacts uses the
  /// same "r", the code imposes no such requirement.
  MatrixX<T> F;

  /// This ℝʸʳ vector is the time derivative of the matrix F (defined above)
  /// times the generalized velocity (∈ ℝᵐ) of the rigid body system. As above,
  /// m is the dimension of the system generalized velocity.
  VectorX<T> Fdot_x_v;

  /// The ℝⁱˣᵐ Jacobian matrix that transforms generalized velocities (m is the
  /// dimension of generalized velocity) into velocities along the i
  /// configuration limits.
  MatrixX<T> L;

  /// This ℝⁱ vector is the time derivative of the matrix L (defined above)
  /// times the generalized velocity (∈ ℝᵐ) of the rigid body system. As above,
  /// m is the dimension of the system generalized velocity;
  VectorX<T> Ldot_x_v;

  /// The ℝⁿˣᵐ matrix (N - μQ) that transforms generalized velocities (m is the
  /// dimension of generalized velocity) into velocities projected along the
  /// contact normals at the n contact points, where Q ∈ ℝⁿˣᵐ is the Jacobian
  /// matrix that transforms generalized velocities (m is the dimension of
  /// generalized velocity) into velocities projected along the directions of
  /// sliding at the s *sliding* contact points (rows of Q that correspond to
  /// non-sliding contacts are set to zero). The μQ factor indicates that
  /// any normal forces applied using this Jacobian will yield frictional
  /// effects for sliding contacts.
  MatrixX<T> N_minus_mu_Q;

  /// The ℝᵐ vector f, the generalized external force vector that
  /// comprises gravitational, centrifugal, Coriolis, actuator, etc. forces. m
  /// is the dimension of the generalized force, which is also equal to the
  /// domension of the generalized velocity.
  VectorX<T> f;

  /// A function for solving the equation MX = B for matrix X, given input
  /// matrix B, where M is the generalized inertia matrix for the rigid body
  /// system.
  std::function<MatrixX<T>(const MatrixX<T>&)> solve_inertia;
};

/// Structure for holding hard constraint data for computing constraint forces
/// under the hard constraint model at the velocity-level (i.e., impact
/// problems).
template <class T>
struct HardConstraintVelProblemData {
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

  /// The ℝⁿˣᵐ Jacobian matrix that transforms generalized velocities (m is the
  /// dimension of generalized velocity) into velocities projected along the
  /// contact normals at the n contact points.
  MatrixX<T> N;

  /// The ℝⁿʳˣᵐ Jacobian matrix that transforms generalized velocities (m is the
  /// dimension of generalized velocity) into velocities projected along
  /// r vectors that span the contact tangents (used to linearize the friction
  /// cone) at the n contact points. For contact problems in two
  /// dimensions, r will be one. For a friction pyramid in three dimensions, r
  /// would be two. While the definition of the dimension of the Jacobian matrix
  /// above indicates that every one of the n contacts uses the same "r", the
  /// code imposes no such requirement.
  MatrixX<T> F;

  /// The ℝⁱˣᵐ Jacobian matrix that transforms generalized velocities (m is the
  /// dimension of generalized velocity) into velocities along the i
  /// configuration limits.
  MatrixX<T> L;

  /// The ℝᵐ vector v, the generalized velocity immediately before any impulsive
  /// forces (from impact) are applied.
  VectorX<T> v;

  /// A function for solving the equation MX = B for matrix X, given input
  /// matrix B, where M is the generalized inertia matrix for the rigid body
  /// system.
  std::function<MatrixX<T>(const MatrixX<T>&)> solve_inertia;
};

}  // namespace hard_constraint
}  // namespace multibody
}  // namespace drake
