#pragma once

#include "drake/multibody/tree/body_node.h"

namespace drake {
namespace multibody {
namespace internal {

// TODO(sherm1) Can we just get rid of this class and use a 0-dof BodyNodeImpl
//  instead?

// This class represents a BodyNode for the world body.
// Base class BodyNode methods are sufficient for this zero-dof node.
template <typename T>
class BodyNodeWorld : public BodyNode<T> {
 public:
  explicit BodyNodeWorld(const RigidBody<T>* body) :
      BodyNode<T>(nullptr, body, nullptr) {}
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
