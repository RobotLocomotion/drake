#pragma once

#include <vector>

#include <Eigen/Core>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace rigid_contact {

/// Structure for holding rigid contact data for computing rigid contact
/// problems at the acceleration-level.
template <class T>
struct RigidContactAccelProblemData {
  /// The indices of the sliding contacts (those contacts at which there is
  /// non-zero relative velocity between bodies in the plane tangent to the
  /// point of contact), out of the set of all contact indices (0...n-1).
  std::vector<int> sliding_contacts;

  /// The indices of the non-sliding contacts (those contacts at which there
  /// is zero relative velocity between bodies in the plane tangent to the
  /// point of contact), out of the set of all contact indices (0...n-1).
  std::vector<int> non_sliding_contacts;

  /// The number of spanning vectors in the contact tangents (used to linearize
  /// the friction cone) at the n *non-sliding* contact points. For contact
  /// problems in two dimensions, each element of r will be one. For contact
  /// problems in three dimensions, a friction pyramid (for example), for a
  /// contact point i will have rᵢ = 2. The definition of r here corresponds
  /// to k/2 in [Anitescu 2003].
  std::vector<int> r;

  /// Coefficients of friction for the sliding contacts. The size of this vector
  /// should be equal to `sliding_contacts.size()`.
  VectorX<T> mu_sliding;

  /// Coefficients of friction for the non-sliding contacts. The size of this
  /// vector should be equal to `non_sliding_contacts.size()`.
  VectorX<T> mu_non_sliding;

  /// The ℝⁿˣᵐ Jacobian matrix that transforms generalized velocities (m is the
  /// dimension of generalized velocity) into velocities projected along the
  /// contact normals at the n contact points.
  MatrixX<T> N;

  /// This ℝⁿ vector is the time derivative of the matrix N (defined above)
  /// times the generalized velocity (∈ ℝᵐ) of the rigid body system.
  VectorX<T> Ndot_x_v;

  /// The ℝⁿʳˣᵐ Jacobian matrix that transforms generalized velocities (m is the
  /// dimension of generalized velocity) into velocities projected along the
  /// k vector that span the contact tangents (used to linearize the friction
  /// cone) at the n *non-sliding* contact points. For contact problems in two
  /// dimensions, r will be one. For a friction pyramid in three dimensions, r
  /// would be two. While the definition of the dimension of the Jacobian matrix
  /// above indicates that every contact uses the same "r", the code imposes no
  /// such requirement.
  MatrixX<T> F;

  /// This ℝⁿʳ vector is the time derivative of the matrix F (defined above)
  /// times the generalized velocity (∈ ℝᵐ) of the rigid body system.
  VectorX<T> Fdot_x_v;

  /// The ℝⁿˣᵐ matrix (N - μQ) that transforms generalized velocities (m is the
  /// dimension of generalized velocity) into velocities projected along the
  /// contact normals at the n contact points, where Q ∈ ℝⁿˣᵐ is the Jacobian
  /// matrix that transforms generalized velocities (m is the dimension of
  /// generalized velocity) into velocities projected along the directions of
  /// sliding at the n *sliding* contact points. The μQ factor indicates that
  /// any normal forces applied using this Jacobian will yield frictional
  /// effects for sliding contacts.
  MatrixX<T> N_minus_mu_Q;

  /// The ℝᵐ vector f, the generalized external force vector that
  /// comprises gravitational, centrifugal, Coriolis, actuator, etc. forces.
  VectorX<T> f;

  /// A function for solving the equation MX = B for matrix X, given input
  /// matrix B, where M is the generalized inertia matrix for the rigid body
  /// system.
  std::function<MatrixX<T>(const MatrixX<T>&)> solve_inertia;
};


}  // namespace rigid_contact
}  // namespace multibody
}  // namespace drake
