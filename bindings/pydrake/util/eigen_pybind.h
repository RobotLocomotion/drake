#pragma once

#include <Eigen/Dense>
#include <pybind11/eigen.h>

namespace drake {
namespace pydrake {

/// Provides a mutable Ref<> for a pointer.
/// Meant to be used for decorating methods passed to `pybind11` (e.g. virtual
/// function dispatch).
// TODO(eric.cousineau): Ensure that all C++ mutator call sites use `EigenPtr`.
template <typename Derived>
auto ToEigenRef(Eigen::VectorBlock<Derived>* derived) {
  return Eigen::Ref<Derived>(*derived);
}

}  // namespace pydrake
}  // namespace drake
