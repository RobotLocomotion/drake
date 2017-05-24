#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body_node.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

/// This class represents a BodyNode for nodes with zero degrees of freedom.
/// These include the world body and the WeldMobilzer.
/// This class not only encapsulate the right abstraction of a node with zero
/// degrees of freedom but also solves the problem of instantiating a
/// BodyNodeImpl with zero compile-time sizes, which leads to Eigen expressions
/// that assert at compile-time.
/// Also, this class results in absolutely zero cost for WeldMobilizer's.
template <typename T>
class BodyNodeWelded : public BodyNode<T> {
 public:
  BodyNodeWelded(const Body<T>* body) : BodyNode<T>(body, nullptr) {}
};

}  // namespace multibody
}  // namespace drake
