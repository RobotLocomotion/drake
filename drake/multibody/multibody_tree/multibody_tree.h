#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/body.h"

namespace drake {
namespace multibody {

/// %MultibodyTree provides a representation for a physical system consisting of
/// a collection of interconnected rigid and deformable bodies. As such, it owns
/// and manages each of the elements that belong to this physical system.
/// Multibody dynamics elements include bodies, joints, force elements and
/// constraints.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class MultibodyTree {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyTree)

  /// Creates a MultibodyTree containing only a **world** body.
  MultibodyTree();

  /// Takes ownership of `body` and assigns a unique index to it. Once a body
  /// is added to a MultibodyTree it cannot be removed from it.
  ///
  /// @throws std::logic_error if users attempt to add a body to an already
  /// compiled multibody tree with MultibodyTree::Compile() or if `body` is a
  /// nullptr.
  ///
  /// @note This method is an implementation detail and users do not need to
  /// call it. The only allowed mechanism to create bodies is through their
  /// factory methods. For instance, see RigidBody::Create() to create a body
  /// and add it to a MultibodyTree.
  ///
  /// @returns The unique index of the body just added.
  BodyIndex AddBody(std::unique_ptr<Body<T>> body);

  /// Returns the number of bodies in the MultibodyTree including the *world*
  /// body. Therefore the minimum number of bodies in a MultibodyTree is one.
  int get_num_bodies() const { return static_cast<int>(owned_bodies_.size()); }

  /// Returns a constant reference to the *world* body.
  const Body<T>& get_world_body() const {
    return *owned_bodies_[world_index()];
  }

  /// Returns a constant reference to the body with unique index `body_index`.
  const Body<T>& get_body(BodyIndex body_index) const {
    DRAKE_ASSERT(body_index < get_num_bodies());
    return *owned_bodies_[body_index];
  }

  /// Returns a mutable reference to the body with unique index `body_index`.
  Body<T>& get_mutable_body(BodyIndex body_index) {
    DRAKE_ASSERT(body_index < get_num_bodies());
    return *owned_bodies_[body_index].get();
  }

  /// This method must be called after all elements in the tree (joints, bodies,
  /// force elements, constraints) were added and before any computations are
  /// performed.
  /// It essentially compiles all the necessary "topological information", i.e.
  /// how bodies, joints and, any other elements connect with each other, and
  /// performs all the required pre-processing to perform computations at a
  /// later stage.
  /// No more elements can be added to `this` MultibodyTree after calling this
  /// method. An excpetion is thrown if users attempt to add elements after
  /// this call was performed.
  void Compile();

 private:
  std::vector<std::unique_ptr<Body<T>>> owned_bodies_;
  // TODO(amcastro-tri): this flag will go within the topology class in a
  // future PR.
  bool topology_is_valid_{false};
};

}  // namespace multibody
}  // namespace drake
