#pragma once

#include "drake/common/drake_deprecated.h"

namespace drake {
namespace multibody {
namespace internal {

// Forward declaration.
template<typename T> class MultibodyTree;

}  // namespace internal

/// Public alias to internal `MultibodyTree`.
/// @warning This alias will soon be deprecated.
template <typename T>
using MultibodyTree
DRAKE_DEPRECATED(
    "This will soon be internal. Please use `MultibodyPlant` instead.")
    = internal::MultibodyTree<T>;

// // Forward declaration.
// template <typename T> class MultibodyPlant;

}  // namespace multibody
}  // namespace drake
