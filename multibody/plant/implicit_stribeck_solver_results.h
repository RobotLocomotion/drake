#pragma once

#include <vector>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {

/// This struct stores the results from a computation performed with
/// ImplicitStribeckSolver. See the ImplicitStribeckSolver class's documentation
/// for further details.
/// We denote with `nv` the size of the vector of generalized velocities and
/// with `nc` the number of contact points.
template <class T>
struct ImplicitStribeckSolverResults {
  /// Next time step vector of generalized velocities.
  VectorX<T> v_next;

  /// Vector storing the normal force (positive) at each contact point, of size
  /// `nc`.
  VectorX<T> fn;

  /// Vector of tangential velocities in the contact frame, of size `2⋅nc`.
  VectorX<T> ft;

  /// Vector of normal separation speeds, of size `nc`.
  VectorX<T> vn;

  /// Vector of tangential relative velocities, of size `2⋅nc`.
  VectorX<T> vt;

  /// Vector of generalized forces due to contact, of size `nv`.
  VectorX<T> tau_contact;
};

}  // namespace multibody
}  // namespace drake
