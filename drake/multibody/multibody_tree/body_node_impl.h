#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body_node.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

template <typename T, int  num_positions, int num_velocities>
class BodyNodeImpl : public BodyNode<T> {
 public:
  // static constexpr int i = 42; discouraged.
  // See answer in: http://stackoverflow.com/questions/37259807/static-constexpr-int-vs-old-fashioned-enum-when-and-why
  enum : int {nq = num_positions, nv = num_velocities};

  BodyNodeImpl(const Body<T>* body, const Mobilizer<T>* mobilizer) :
      BodyNode<T>(body, mobilizer) {}

 protected:
  using BodyNode<T>::body_;
  using BodyNode<T>::mobilizer_;
};

}  // namespace multibody
}  // namespace drake
