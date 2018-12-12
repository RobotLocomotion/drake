#pragma once

namespace drake {
namespace multibody {
namespace internal {

// Forward declaration.
template<typename T> class MultibodyTree;

}  // namespace internal

/// Public alias to internal `MultibodyTree`.
/// @warning This alias will soon be deprecated.
template <typename T>
using MultibodyTree = internal::MultibodyTree<T>;

}  // namespace multibody
}  // namespace drake
