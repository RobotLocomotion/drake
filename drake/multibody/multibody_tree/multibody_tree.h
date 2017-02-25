#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body.h"

namespace drake {
namespace multibody {

template <typename T>
class MultibodyTree {
 public:
  /// Creates a MultibodyTree containing only a *world* body (with unique BodyId
  /// equal to 0).
  MultibodyTree();

  /// Takes ownership of @p body and assigns a unique id to it.
  /// @note
  BodyIndex AddBody(std::unique_ptr<Body<T>> body);

  /// Returns the number of bodies in the MultibodyTree including including the
  /// *world* body. Therefore the minimum number of bodies in a MultibodyTree to
  /// which no other bodies have been added, is one.
  int get_num_bodies() const { return static_cast<int>(bodies_.size()); }

  /// Returns a constant reference to the *world* body.
  const Body<T>& get_world_body() const {
    return *bodies_[0];
  }

  const Body<T>& get_body(BodyIndex body_id) const {
    DRAKE_ASSERT(body_id.is_valid() && body_id < get_num_bodies());
    return *bodies_[body_id];
  }

  Body<T>* get_mutable_body(BodyIndex body_id) {
    return bodies_[body_id].get();
  }

 private:
  std::vector<std::unique_ptr<Body<T>>> bodies_;
};

}  // namespace multibody
}  // namespace drake
