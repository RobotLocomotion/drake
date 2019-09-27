#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace internal {

/// This struct stores the results from a computation performed with
/// TamsiSolver. See the TamsiSolver class's documentation
/// for further details.
/// We denote `nv` the size of the vector of generalized velocities and `nc` the
/// number of contact points.
template <class T>
struct TamsiSolverResults {
  /// Vector of generalized velocities at the next time step.
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

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct ::drake::multibody::internal::TamsiSolverResults)
