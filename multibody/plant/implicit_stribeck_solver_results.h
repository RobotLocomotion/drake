#pragma once

#include <vector>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {

template <class T>
struct ImplicitStribeckSolverResults {
  /// Next time step generalized velocities.
  VectorX<T> v_next;

  VectorX<T> fn;
  VectorX<T> ft;

  VectorX<T> vn;
  VectorX<T> vt;

  VectorX<T> tau_contact;
};

}  // namespace multibody
}  // namespace drake
