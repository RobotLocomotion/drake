#pragma once

#include "drake/multibody/multibody_tree/body_node.h"

namespace drake {
namespace multibody {
namespace internal {

/// This class represents a BodyNode for nodes with zero degrees of freedom.
/// These include the world body and the WeldMobilzer.
/// Overrides in this class of general BodyNode methods conveniently reduce to
/// no-ops for body nodes with zero degrees of freedom. In addition, it also
/// solves the problem of instantiating a BodyNodeImpl with zero compile-time
/// sizes, which leads to Eigen expressions that assert at compile-time.
template <typename T>
class BodyNodeWelded : public BodyNode<T> {
 public:
  explicit BodyNodeWelded(const Body<T>& body) : BodyNode<T>(body, nullptr) {}
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
