#pragma once

#include <memory>
#include <utility>
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

  /// Takes ownership of `body`, assigns a unique index to it, and adds it to
  /// `this` %MultibodyTree. Returns a bare pointer to the body just added,
  /// which will remain valid for the lifetime of `this` %MultibodyTree.
  ///
  /// Example of usage:
  /// @code{.cpp}
  ///   MultibodyTree<T> model;
  ///   auto foo = model.AddBody(std::make_unique<RigidBody<T>>());
  /// @endcode
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
  /// @param[in] body A unique pointer to a body to add to `this`
  ///                 %MultibodyTree.
  /// @returns A bare pointer to the `body` just added, which will remain valid
  ///          for the lifetime of `this` MultibodyTree.
  ///
  /// @tparam BodyType The type of the specific sub-class of Body to add.
  template <class BodyType>
  BodyType* AddBody(std::unique_ptr<BodyType> body) {
    if (body == nullptr) {
      throw std::logic_error("Input body is an invalid nullptr.");
    }

    // If the topology is valid it means that this MultibodyTree was already
    // compiled. Thus throw an exception to alert users.
    if (topology_is_valid_) {
      throw std::logic_error(
          "Attempting to add a body to an already compiled MultibodyTree is "
          "not allowed. See MultibodyTree::Compile() for details.");
    }
    // TODO(amcastro-tri): This index will be returned by the
    // MultibodyTreeTopology class in a future PR.
    BodyIndex index(owned_bodies_.size());
    // MultibodyTree has access to these methods since it is a friend of
    // MultibodyTreeElement. Users of Body<T>, however, do not have access to
    // these methods.
    body->set_parent_tree(this);
    body->set_index(index);
    BodyType* raw_body_ptr = body.get();
    owned_bodies_.push_back(std::move(body));
    return raw_body_ptr;
  }

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
