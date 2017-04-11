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
  /// To create a %MultibodyTree users will add multibody elements like bodies,
  /// joints, force elements, constraints, etc, using one of these methods.
  /// Once a user is done adding multibody elements, the Compile() method
  /// **must** be called before invoking any %MultibodyTree method.
  /// See Compile() for details.
  /// @{
  // TODO(amcastro-tri): add at least one example of a method that requires a
  // valid topology in this documentation.
  // See this Reviewable comment: https://reviewable.io/reviews/robotlocomotion/drake/5583#-KgGqGisnX9uMuYDkHpx

  /// Takes ownership of `body`, and adds it to `this` %MultibodyTree. Returns a
  /// constant reference to the body just added, which will remain valid for the
  /// lifetime of `this` %MultibodyTree.
  ///
  /// Example of usage:
  /// @code
  ///   MultibodyTree<T> model;
  ///   const auto& body =
  ///       model.AddBody(std::make_unique<RigidBody<T>>(Args...));
  /// @endcode
  /// where `const auto&` here resolves to `const RigidBody<T>&`.
  ///
  /// @throws std::logic_error if `body` is a nullptr.
  /// @throws std::logic_error if Compile() was already called on `this` tree.
  ///
  /// @param[in] body A unique pointer to a body to add to `this`
  ///                 %MultibodyTree.
  /// @returns A constant reference to the `body` just added, which will remain
  ///          valid for the lifetime of `this` MultibodyTree.
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

  /// Constructs a new body with type `BodyType` with the given `args`, and adds
  /// it to `this` %MultibodyTree, which retains ownership. The `BodyType` will
  /// be specialized on the scalar type T of this %MultibodyTree.
  ///
  /// Example of usage:
  /// @code
  ///   MultibodyTree<T> model;
  ///   // Notice RigidBody is a template an a scalar type.
  ///   const auto& body = model.AddBody<RigidBody>(Args...);
  /// @endcode
  /// where `const auto&` here resolves to `const RigidBody<T>&`.
  ///
  /// Note that for dependent names you must use the template keyword (say for
  /// instance you have a MultibodyTree<T> member within your custom class):
  ///
  /// @code
  ///   MultibodyTree<T> model;
  ///   auto body = model.template AddBody<RigidBody>(Args...);
  /// @endcode
  ///
  /// @throws std::logic_error if Compile() was already called on `this` tree.
  ///
  /// @param[in] args The arguments needed to construct a valid Body of type
  ///                 `BodyType`. `BodyType` must provide a public constructor
  ///                 that takes these arguments.
  /// @returns A constant reference to the body with type `BodyType` just
  ///          created, which will remain valid for the lifetime of `this`
  ///          MultibodyTree.
  ///
  /// @tparam BodyType A template for the type of Body to construct. The
  ///                  template will be specialized on the scalar type T of this
  ///                  %MultibodyTree.
  template<template<typename Scalar> class BodyType, typename... Args>
  const BodyType<T>& AddBody(Args&&... args) {
    return AddBody(std::make_unique<BodyType<T>>(std::forward<Args>(args)...));
  }

  /// Takes ownership of `frame`, and adds it to `this` %MultibodyTree. Returns
  /// a constant reference to the frame just added, which will remain valid for
  /// the lifetime of `this` %MultibodyTree.
  ///
  /// Example of usage:
  /// @code
  ///   MultibodyTree<T> model;
  ///   // ... Define body and X_BF ...
  ///   const auto& frame =
  ///       model.AddFrame(std::make_unique<FixedOffsetFrame<T>>(body, X_BF));
  /// @endcode
  /// where `const auto&` here resolves to `const FixedOffsetFrame<T>&`.
  ///
  /// @throws std::logic_error if `frame` is a nullptr.
  /// @throws std::logic_error if Compile() was already called on `this` tree.
  ///
  /// @param[in] frame A unique pointer to a frame to be added to `this`
  ///                  %MultibodyTree.
  /// @returns A constant reference to the frame just added, which will remain
  ///          valid for the lifetime of `this` MultibodyTree.
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

  /// Constructs a new frame with type `FrameType` with the given `args`, and
  /// adds it to `this` %MultibodyTree, which retains ownership. The `FrameType`
  /// will be specialized on the scalar type T of this %MultibodyTree.
  ///
  /// Example of usage:
  /// @code
  ///   MultibodyTree<T> model;
  ///   // ... Define body and X_BF ...
  ///   // Notice FixedOffsetFrame is a template an a scalar type.
  ///   const auto& frame = model.AddFrame<FixedOffsetFrame>(body, X_BF);
  /// @endcode
  /// where `const auto&` here resolves to `const RigidBody<T>&`.
  ///
  /// Note that for dependent names you must use the template keyword (say for
  /// instance you have a MultibodyTree<T> member within your custom class):
  ///
  /// @code
  ///   MultibodyTree<T> model;
  ///   // ... Define body and X_BF ...
  ///   const auto& frame = model.AddFrame<FixedOffsetFrame>(body, X_BF);
  /// @endcode
  ///
  /// @throws std::logic_error if Compile() was already called on `this` tree.
  ///
  /// @param[in] args The arguments needed to construct a valid Frame of type
  ///                 `FrameType`. `FrameType` must provide a public constructor
  ///                 that takes these arguments.
  /// @returns A constant reference to the frame with type `FrameType` just
  ///          created, which will remain valid for the lifetime of `this`
  ///          MultibodyTree.
  ///
  /// @tparam FrameType A template for the type of Frame to construct. The
  ///                   template will be specialized on the scalar type T of
  ///                   this %MultibodyTree.
  template<template<typename Scalar> class FrameType, typename... Args>
  const FrameType<T>& AddFrame(Args&&... args) {
    return AddFrame(
        std::make_unique<FrameType<T>>(std::forward<Args>(args)...));
  }

  /// @}
  // Closes Doxygen section.

  /// Returns the number of Frame objects in the MultibodyTree.
  /// Frames include body frames associated with each of the bodies in
  /// the %MultibodyTree including the _world_ body. Therefore the minimum
  /// number of frames in a %MultibodyTree is one.
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
  /// When a %MultibodyTree is instantiated, its topology remains invalid until
  /// Compile() is called, which validates the topology.
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
  /// No more multibody tree elements can be added after a call to Compile().
  ///
  /// @throws std::logic_error If users attempt to call this method on an
  ///         already compiled %MultibodyTree.
  // TODO(amcastro-tri): Consider making this method private and calling it
  // automatically when CreateDefaultContext() is called.
  void Compile();

 private:
  // TODO(amcastro-tri): In future PR's adding MBT computational methods, write
  // a method that verifies the state of the topology with a signature similar
  // to RoadGeometry::CheckInvariants().

  // Sets a flag indicate the topology is valid.
  void validate_topology() { topology_.validate(); }

  std::vector<std::unique_ptr<Body<T>>> owned_bodies_;
  std::vector<std::unique_ptr<Frame<T>>> owned_frames_;
  // List of all frames in the system ordered by their FrameIndex.
  // This vector contains a pointer to all bodies in owned_bodies_ as well as a
  // pointer to each BodyFrame, which are owned by their corresponding Body.
  std::vector<Frame<T>*> frames_;

  MultibodyTreeTopology topology_;
};

}  // namespace multibody
}  // namespace drake
