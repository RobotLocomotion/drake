#pragma once

#include <memory>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

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

  /// @name Methods to add new multibody tree elements.
  ///
  /// These methods are an implementation detail and users do not need to
  /// call them. The only allowed mechanism to create MultibodyTreeElement
  /// objects is through their factory methods. For instance, see
  /// RigidBodyFrame::Create() to create a rigid body and add it to a
  /// MultibodyTree. However, these API's are made public so that specific
  /// factory methods can access them. The factory method for the creation of
  /// multibody tree elements allows for a very controlled construction that
  /// ensures that multibody tree elements have a properly intialized reference
  /// to their parent %MultibodyTree as they are added to it.
  ///
  /// Calling any of these methods invalidates the topology of this
  /// %MultibodyTree and therefore, the Compile() method must be called before
  /// invoking methods which require valid topology. See Compile() for details.
  /// @{
  // TODO(amcastro-tri): add at least one example of a method that requires a
  // valid topology in this documentation.
  // See this Reviewable comment: https://reviewable.io/reviews/robotlocomotion/drake/5583#-KgGqGisnX9uMuYDkHpx

  /// Takes ownership of `body`, and adds it to `this` %MultibodyTree. Returns a
  /// bare pointer to the body just added, which will remain valid for the
  /// lifetime of `this` %MultibodyTree.
  ///
  /// Example of usage:
  /// @code{.cpp}
  ///   MultibodyTree<T> model;
  ///   auto foo = model.AddBody(std::make_unique<RigidBody<T>>());
  /// @endcode
  /// where `auto` here resolves to `RigidBody<T>*`.
  ///
  /// @throws std::logic_error if `body` is a nullptr.
  ///
  /// @param[in] body A unique pointer to a body to add to `this`
  ///                 %MultibodyTree.
  /// @returns A bare pointer to the `body` just added, which will remain valid
  ///          for the lifetime of `this` MultibodyTree.
  ///
  /// @tparam BodyType The type of the specific sub-class of Body to add.
  template <class BodyType>
  const BodyType& AddBody(std::unique_ptr<BodyType> body) {
    static_assert(std::is_convertible<BodyType*, Body<T>*>::value,
                  "BodyType must be a sub-class of Body<T>.");
    if (topology_.is_valid) {
      throw std::logic_error("This MultibodyTree is compiled already. "
                             "Therefore adding more bodies is not allowed. "
                             "See documentation for Compile() for details.");
    }
    if (body == nullptr) {
      throw std::logic_error("Input body is an invalid nullptr.");
    }
    BodyIndex body_index(0);
    FrameIndex body_frame_index(0);
    std::tie(body_index, body_frame_index) = topology_.add_body();
    DRAKE_ASSERT(body_index == get_num_bodies());
    DRAKE_ASSERT(body_frame_index == get_num_frames());

    // TODO(amcastro-tri): consider not depending on setting this pointer at
    // all. Consider also removing MultibodyTreeElement altogether.
    body->set_parent_tree(this, body_index);
    Frame<T>* body_frame = body->get_mutable_body_frame();
    body_frame->set_parent_tree(this, body_frame_index);
    frames_.push_back(body_frame);
    BodyType* raw_body_ptr = body.get();
    owned_bodies_.push_back(std::move(body));
    return *raw_body_ptr;
  }

  template<template<typename Scalar> class BodyType, typename... Args>
  const BodyType<T>& AddBody(Args&&... args) {
    return AddBody(std::make_unique<BodyType<T>>(std::forward<Args>(args)...));
  }

  /// Takes ownership of `frame` and adds it to `this` %MultibodyTree. Returns a
  /// bare pointer to the frame just added, which will remain valid for the
  /// lifetime of `this` %MultibodyTree.
  ///
  /// Example of usage:
  /// @code
  ///   MultibodyTree<T> model;
  ///   auto foo = model.AddFrame(
  ///                std::make_unique<FixedOffsetFrame<T>>());
  /// @endcode
  /// where `auto` here resolves to `FixedOffsetFrame<T>*`.
  ///
  /// @throws std::logic_error if `frame` is a nullptr.
  ///
  /// @param[in] frame A unique pointer to a frame to be added to `this`
  ///                  %MultibodyTree.
  /// @returns A bare pointer to the frame just added, which will remain valid
  ///          for the lifetime of `this` MultibodyTree.
  ///
  /// @tparam FrameType The type of the specific sub-class of Frame to
  ///                   add.
  template <class FrameType>
  const FrameType& AddFrame(std::unique_ptr<FrameType> frame) {
    static_assert(std::is_convertible<FrameType*, Frame<T>*>::value,
                  "FrameType must be a sub-class of Frame<T>.");
    if (topology_.is_valid) {
      throw std::logic_error("This MultibodyTree is compiled already. "
                             "Therefore adding more frames is not allowed. "
                             "See documentation for Compile() for details.");
    }
    if (frame == nullptr) {
      throw std::logic_error("Input frame is an invalid nullptr.");
    }
    FrameIndex frame_index = topology_.add_frame(frame->get_body().get_index());
    DRAKE_ASSERT(frame_index == get_num_frames());
    // TODO(amcastro-tri): consider not depending on setting this pointer at
    // all. Consider also removing MultibodyTreeElement altogether.
    frame->set_parent_tree(this, frame_index);
    FrameType* raw_frame_ptr = frame.get();
    frames_.push_back(raw_frame_ptr);
    owned_frames_.push_back(std::move(frame));
    return *raw_frame_ptr;
  }

  template<template<typename Scalar> class FrameType, typename... Args>
  const FrameType<T>& AddFrame(Args&&... args) {
    return AddFrame(std::make_unique<FrameType<T>>(std::forward<Args>(args)...));
  }

  /// @}
  // Closes Doxygen section.

  /// Returns the number of Frame objects in the MultibodyTree.
  /// Frames include body frames associated with each of the bodies in
  /// the system including the _world_ body. Therefore the minimum number of
  /// frames in a MultibodyTree is one.
  int get_num_frames() const {
    return static_cast<int>(frames_.size());
  }

  /// Returns the number of bodies in the MultibodyTree including the *world*
  /// body. Therefore the minimum number of bodies in a MultibodyTree is one.
  int get_num_bodies() const { return static_cast<int>(owned_bodies_.size()); }

  /// Returns a constant reference to the *world* body.
  const Body<T>& get_world_body() const {
    return *owned_bodies_[world_index()];
  }

  /// Returns a constant reference to the body with unique index `body_index`.
  /// This method aborts in Debug builds when `body_index` does not correspond
  /// to a body in this multibody tree.
  const Body<T>& get_body(BodyIndex body_index) const {
    DRAKE_ASSERT(body_index < get_num_bodies());
    return *owned_bodies_[body_index];
  }

  /// Returns `true` if this %MultibodyTree was compiled with Compile() after
  /// all multibody elements were added, and `false` otherwise.
  /// The addition of new multibody elements invalidates the topology of the
  /// %MultibodyTree, while a call to Compile() validates the topology.
  /// @see Compile().
  bool topology_is_valid() const { return topology_.is_valid; }

  /// Returns the topology information for this multibody tree. Users should not
  /// need to call this method since MultibodyTreeTopology is an internal
  /// bookkeeping detail.
  const MultibodyTreeTopology& get_topology() const { return topology_; }

  /// This method must be called after all elements in the tree (joints, bodies,
  /// force elements, constraints) were added and before any computations are
  /// performed.
  /// It essentially compiles all the necessary "topological information", i.e.
  /// how bodies, joints and, any other elements connect with each other, and
  /// performs all the required pre-processing to perform computations at a
  /// later stage.
  ///
  /// If the compile stage is successful, the topology of this %MultibodyTree is
  /// validated, meaning that the topology is up-to-date after this call.
  /// The topology of a %MultibodyTree gets invalidated if more multibody
  /// elements are added and therefore the user needs to call this method in
  /// order to have a valid %MultibodyTree.
  ///
  /// @throws std::logic_error If users attempt to call this method on an
  ///         already compiled %MultibodyTree.
  void Compile();

 private:
  // TODO(amcastro-tri): In future PR's adding MBT computational methods, write
  // a method that verifies the state of the topology with a signature similar
  // to RoadGeometry::CheckInvariants().

  // Sets a flag indicate the topology is valid.
  void validate_topology() { topology_.validate(); }

  std::vector<std::unique_ptr<Body<T>>> owned_bodies_;
  std::vector<std::unique_ptr<Frame<T>>> owned_frames_;
  // List of all frames in the system ordered by their FrameIndex (which gets
  // assigned at Compile() time).
  std::vector<Frame<T>*> frames_;

  MultibodyTreeTopology topology_;
};

}  // namespace multibody
}  // namespace drake
