#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/multibody/multibody_tree/acceleration_kinematics_cache.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/body_node.h"
#include "drake/multibody/multibody_tree/force_element.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/joints/joint.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_forces.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/multibody/multibody_tree/position_kinematics_cache.h"
#include "drake/multibody/multibody_tree/velocity_kinematics_cache.h"
#include "drake/systems/framework/context.h"

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

  /// @name Methods to add new MultibodyTree elements.
  ///
  /// To create a %MultibodyTree users will add multibody elements like bodies,
  /// joints, force elements, constraints, etc, using one of these methods.
  /// Once a user is done adding multibody elements, the Finalize() method
  /// **must** be called before invoking any %MultibodyTree method.
  /// See Finalize() for details.
  /// @{
  // TODO(amcastro-tri): add at least one example of a method that requires a
  // valid topology in this documentation.
  // See this Reviewable comment:
  // https://reviewable.io/reviews/robotlocomotion/drake/5583#-KgGqGisnX9uMuYDkHpx

  /// Takes ownership of `body` and adds it to `this` %MultibodyTree. Returns a
  /// constant reference to the body just added, which will remain valid for the
  /// lifetime of `this` %MultibodyTree.
  ///
  /// Example of usage:
  /// @code
  ///   MultibodyTree<T> model;
  ///   // ... Code to define spatial_inertia, a SpatialInertia<T> object ...
  ///   const RigidBody<T>& body =
  ///       model.AddBody(std::make_unique<RigidBody<T>>(spatial_inertia));
  /// @endcode
  ///
  /// @throws std::logic_error if `body` is a nullptr.
  /// @throws std::logic_error if Finalize() was already called on `this` tree.
  ///
  /// @param[in] body A unique pointer to a body to add to `this`
  ///                 %MultibodyTree. The body class must be specialized on the
  ///                 same scalar type T as this %MultibodyTree.
  /// @returns A constant reference of type `BodyType` to the created body.
  ///          This reference which will remain valid for the lifetime of `this`
  ///          %MultibodyTree.
  ///
  /// @tparam BodyType The type of the specific sub-class of Body to add. The
  ///                  template needs to be specialized on the same scalar type
  ///                  T of this %MultibodyTree.
  template <template<typename Scalar> class BodyType>
  const BodyType<T>& AddBody(std::unique_ptr<BodyType<T>> body) {
    static_assert(std::is_convertible<BodyType<T>*, Body<T>*>::value,
                  "BodyType must be a sub-class of Body<T>.");
    if (topology_is_valid()) {
      throw std::logic_error("This MultibodyTree is finalized already. "
                             "Therefore adding more bodies is not allowed. "
                             "See documentation for Finalize() for details.");
    }
    if (body == nullptr) {
      throw std::logic_error("Input body is a nullptr.");
    }
    BodyIndex body_index(0);
    FrameIndex body_frame_index(0);
    std::tie(body_index, body_frame_index) = topology_.add_body();
    // These tests MUST be performed BEFORE frames_.push_back() and
    // owned_bodies_.push_back() below. Do not move them around!
    DRAKE_ASSERT(body_index == get_num_bodies());
    DRAKE_ASSERT(body_frame_index == get_num_frames());

    // TODO(amcastro-tri): consider not depending on setting this pointer at
    // all. Consider also removing MultibodyTreeElement altogether.
    body->set_parent_tree(this, body_index);
    // MultibodyTree can access selected private methods in Body through its
    // BodyAttorney.
    Frame<T>* body_frame =
        &internal::BodyAttorney<T>::get_mutable_body_frame(body.get());
    body_frame->set_parent_tree(this, body_frame_index);
    frames_.push_back(body_frame);
    BodyType<T>* raw_body_ptr = body.get();
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
  ///   // ... Code to define spatial_inertia, a SpatialInertia<T> object ...
  ///   // Notice RigidBody is a template on a scalar type.
  ///   const RigidBody<T>& body = model.AddBody<RigidBody>(spatial_inertia);
  /// @endcode
  ///
  /// Note that for dependent names you must use the template keyword (say for
  /// instance you have a MultibodyTree<T> member within your custom class):
  ///
  /// @code
  ///   MultibodyTree<T> model;
  ///   auto body = model.template AddBody<RigidBody>(Args...);
  /// @endcode
  ///
  /// @throws std::logic_error if Finalize() was already called on `this` tree.
  ///
  /// @param[in] args The arguments needed to construct a valid Body of type
  ///                 `BodyType`. `BodyType` must provide a public constructor
  ///                 that takes these arguments.
  /// @returns A constant reference of type `BodyType` to the created body.
  ///          This reference which will remain valid for the lifetime of `this`
  ///          %MultibodyTree.
  ///
  /// @tparam BodyType A template for the type of Body to construct. The
  ///                  template will be specialized on the scalar type T of this
  ///                  %MultibodyTree.
  template<template<typename Scalar> class BodyType, typename... Args>
  const BodyType<T>& AddBody(Args&&... args) {
    static_assert(std::is_convertible<BodyType<T>*, Body<T>*>::value,
                  "BodyType must be a sub-class of Body<T>.");
    return AddBody(std::make_unique<BodyType<T>>(std::forward<Args>(args)...));
  }

  /// Takes ownership of `frame` and adds it to `this` %MultibodyTree. Returns
  /// a constant reference to the frame just added, which will remain valid for
  /// the lifetime of `this` %MultibodyTree.
  ///
  /// Example of usage:
  /// @code
  ///   MultibodyTree<T> model;
  ///   // ... Define body and X_BF ...
  ///   const FixedOffsetFrame<T>& frame =
  ///       model.AddFrame(std::make_unique<FixedOffsetFrame<T>>(body, X_BF));
  /// @endcode
  ///
  /// @throws std::logic_error if `frame` is a nullptr.
  /// @throws std::logic_error if Finalize() was already called on `this` tree.
  ///
  /// @param[in] frame A unique pointer to a frame to be added to `this`
  ///                  %MultibodyTree. The frame class must be specialized on
  ///                  the same scalar type T as this %MultibodyTree.
  /// @returns A constant reference of type `FrameType` to the created frame.
  ///          This reference which will remain valid for the lifetime of `this`
  ///          %MultibodyTree.
  ///
  /// @tparam FrameType The type of the specific sub-class of Frame to add. The
  ///                   template needs to be specialized on the same scalar type
  ///                   T of this %MultibodyTree.
  template <template<typename Scalar> class FrameType>
  const FrameType<T>& AddFrame(std::unique_ptr<FrameType<T>> frame) {
    static_assert(std::is_convertible<FrameType<T>*, Frame<T>*>::value,
                  "FrameType must be a sub-class of Frame<T>.");
    if (topology_is_valid()) {
      throw std::logic_error("This MultibodyTree is finalized already. "
                             "Therefore adding more frames is not allowed. "
                             "See documentation for Finalize() for details.");
    }
    if (frame == nullptr) {
      throw std::logic_error("Input frame is a nullptr.");
    }
    FrameIndex frame_index = topology_.add_frame(frame->get_body().get_index());
    // This test MUST be performed BEFORE frames_.push_back() and
    // owned_frames_.push_back() below. Do not move it around!
    DRAKE_ASSERT(frame_index == get_num_frames());
    // TODO(amcastro-tri): consider not depending on setting this pointer at
    // all. Consider also removing MultibodyTreeElement altogether.
    frame->set_parent_tree(this, frame_index);
    FrameType<T>* raw_frame_ptr = frame.get();
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
  ///   const FixedOffsetFrame<T>& frame =
  ///       model.AddFrame<FixedOffsetFrame>(body, X_BF);
  /// @endcode
  ///
  /// Note that for dependent names you must use the template keyword (say for
  /// instance you have a MultibodyTree<T> member within your custom class):
  ///
  /// @code
  ///   MultibodyTree<T> model;
  ///   // ... Define body and X_BF ...
  ///   const auto& frame =
  ///       model.template AddFrame<FixedOffsetFrame>(body, X_BF);
  /// @endcode
  ///
  /// @throws std::logic_error if Finalize() was already called on `this` tree.
  ///
  /// @param[in] args The arguments needed to construct a valid Frame of type
  ///                 `FrameType`. `FrameType` must provide a public constructor
  ///                 that takes these arguments.
  /// @returns A constant reference of type `FrameType` to the created frame.
  ///          This reference which will remain valid for the lifetime of `this`
  ///          %MultibodyTree.
  ///
  /// @tparam FrameType A template for the type of Frame to construct. The
  ///                   template will be specialized on the scalar type T of
  ///                   this %MultibodyTree.
  template<template<typename Scalar> class FrameType, typename... Args>
  const FrameType<T>& AddFrame(Args&&... args) {
    static_assert(std::is_convertible<FrameType<T>*, Frame<T>*>::value,
                  "FrameType must be a sub-class of Frame<T>.");
    return AddFrame(
        std::make_unique<FrameType<T>>(std::forward<Args>(args)...));
  }

  /// Takes ownership of `mobilizer` and adds it to `this` %MultibodyTree.
  /// Returns a constant reference to the mobilizer just added, which will
  /// remain valid for the lifetime of `this` %MultibodyTree.
  ///
  /// Example of usage:
  /// @code
  ///   MultibodyTree<T> model;
  ///   // ... Code to define inboard and outboard frames by calling
  ///   // MultibodyTree::AddFrame() ...
  ///   const RevoluteMobilizer<T>& pin =
  ///     model.AddMobilizer(std::make_unique<RevoluteMobilizer<T>>(
  ///       inboard_frame, elbow_outboard_frame,
  ///       Vector3d::UnitZ() /*revolute axis*/));
  /// @endcode
  ///
  /// A %Mobilizer effectively connects the two bodies to which the inboard and
  /// outboard frames belong.
  ///
  /// @throws std::logic_error if `mobilizer` is a nullptr.
  /// @throws std::logic_error if Finalize() was already called on `this` tree.
  /// @throws a std::runtime_error if the new mobilizer attempts to connect a
  /// frame with itself.
  /// @throws std::runtime_error if attempting to connect two bodies with more
  /// than one mobilizer between them.
  ///
  /// @param[in] mobilizer A unique pointer to a mobilizer to add to `this`
  ///                      %MultibodyTree. The mobilizer class must be
  ///                      specialized on the same scalar type T as this
  ///                      %MultibodyTree. Notice this is a requirement of this
  ///                      method's signature and therefore an input mobilzer
  ///                      specialized on a different scalar type than that of
  ///                      this %MultibodyTree's T will fail to compile.
  /// @returns A constant reference of type `MobilizerType` to the created
  ///          mobilizer. This reference which will remain valid for the
  ///          lifetime of `this` %MultibodyTree.
  ///
  /// @tparam MobilizerType The type of the specific sub-class of Mobilizer to
  ///                       add. The template needs to be specialized on the
  ///                       same scalar type T of this %MultibodyTree.
  template <template<typename Scalar> class MobilizerType>
  const MobilizerType<T>& AddMobilizer(
      std::unique_ptr<MobilizerType<T>> mobilizer) {
    static_assert(std::is_convertible<MobilizerType<T>*, Mobilizer<T>*>::value,
                  "MobilizerType must be a sub-class of mobilizer<T>.");
    if (topology_is_valid()) {
      throw std::logic_error("This MultibodyTree is finalized already. "
                             "Therefore adding more mobilizers is not allowed. "
                             "See documentation for Finalize() for details.");
    }
    if (mobilizer == nullptr) {
      throw std::logic_error("Input mobilizer is a nullptr.");
    }
    // Verifies that the inboard/outboard frames provided by the user do belong
    // to this tree. This is a pathological case, but in theory nothing
    // (but this test) stops a user from adding frames to a tree1 and attempting
    // later to define mobilizers between those frames in a second tree2.
    mobilizer->get_inboard_frame().HasThisParentTreeOrThrow(this);
    mobilizer->get_outboard_frame().HasThisParentTreeOrThrow(this);
    const int num_positions = mobilizer->get_num_positions();
    const int num_velocities = mobilizer->get_num_velocities();
    MobilizerIndex mobilizer_index = topology_.add_mobilizer(
        mobilizer->get_inboard_frame().get_index(),
        mobilizer->get_outboard_frame().get_index(),
        num_positions, num_velocities);

    // This DRAKE_ASSERT MUST be performed BEFORE owned_mobilizers_.push_back()
    // below. Do not move it around!
    DRAKE_ASSERT(mobilizer_index == get_num_mobilizers());

    // TODO(amcastro-tri): consider not depending on setting this pointer at
    // all. Consider also removing MultibodyTreeElement altogether.
    mobilizer->set_parent_tree(this, mobilizer_index);

    MobilizerType<T>* raw_mobilizer_ptr = mobilizer.get();
    owned_mobilizers_.push_back(std::move(mobilizer));
    return *raw_mobilizer_ptr;
  }

  /// Constructs a new mobilizer with type `MobilizerType` with the given
  /// `args`, and adds it to `this` %MultibodyTree, which retains ownership.
  /// The `MobilizerType` will be specialized on the scalar type T of this
  /// %MultibodyTree.
  ///
  /// Example of usage:
  /// @code
  ///   MultibodyTree<T> model;
  ///   // ... Code to define inboard and outboard frames by calling
  ///   // MultibodyTree::AddFrame() ...
  ///   // Notice RevoluteMobilizer is a template an a scalar type.
  ///   const RevoluteMobilizer<T>& pin =
  ///     model.template AddMobilizer<RevoluteMobilizer>(
  ///       inboard_frame, outboard_frame,
  ///       Vector3d::UnitZ() /*revolute axis*/);
  /// @endcode
  ///
  /// Note that for dependent names _only_ you must use the template keyword
  /// (say for instance you have a MultibodyTree<T> member within your custom
  /// class).
  ///
  /// @throws std::logic_error if Finalize() was already called on `this` tree.
  /// @throws a std::runtime_error if the new mobilizer attempts to connect a
  /// frame with itself.
  /// @throws std::runtime_error if attempting to connect two bodies with more
  /// than one mobilizer between them.
  ///
  /// @param[in] args The arguments needed to construct a valid Mobilizer of
  ///                 type `MobilizerType`. `MobilizerType` must provide a
  ///                 public constructor that takes these arguments.
  /// @returns A constant reference of type `MobilizerType` to the created
  ///          mobilizer. This reference which will remain valid for the
  ///          lifetime of `this` %MultibodyTree.
  ///
  /// @tparam MobilizerType A template for the type of Mobilizer to construct.
  ///                       The template will be specialized on the scalar type
  ///                       T of `this` %MultibodyTree.
  template<template<typename Scalar> class MobilizerType, typename... Args>
  const MobilizerType<T>& AddMobilizer(Args&&... args) {
    static_assert(std::is_base_of<Mobilizer<T>, MobilizerType<T>>::value,
                  "MobilizerType must be a sub-class of Mobilizer<T>.");
    return AddMobilizer(
        std::make_unique<MobilizerType<T>>(std::forward<Args>(args)...));
  }

  /// Creates and adds to `this` %MultibodyTree (which retains ownership) a new
  /// `ForceElement` member with the specific type `ForceElementType`. The
  /// arguments to this method `args` are forwarded to `ForceElementType`'s
  /// constructor.
  ///
  /// The newly created `ForceElementType` object will be specialized on the
  /// scalar type T of this %MultibodyTree.
  template <template<typename Scalar> class ForceElementType>
  const ForceElementType<T>& AddForceElement(
      std::unique_ptr<ForceElementType<T>> force_element) {
    static_assert(
        std::is_convertible<ForceElementType<T>*, ForceElement<T>*>::value,
        "ForceElementType<T> must be a sub-class of ForceElement<T>.");
    if (topology_is_valid()) {
      throw std::logic_error(
          "This MultibodyTree is finalized already. Therefore adding more "
          "force elements is not allowed. "
          "See documentation for Finalize() for details.");
    }
    if (force_element == nullptr) {
      throw std::logic_error("Input force element is a nullptr.");
    }
    ForceElementIndex force_element_index = topology_.add_force_element();
    // This test MUST be performed BEFORE owned_force_elements_.push_back()
    // below. Do not move it around!
    DRAKE_ASSERT(force_element_index == get_num_force_elements());
    force_element->set_parent_tree(this, force_element_index);
    ForceElementType<T>* raw_force_element_ptr = force_element.get();
    owned_force_elements_.push_back(std::move(force_element));
    return *raw_force_element_ptr;
  }

  template<template<typename Scalar> class ForceElementType, typename... Args>
  const ForceElementType<T>& AddForceElement(Args&&... args) {
    static_assert(std::is_base_of<ForceElement<T>, ForceElementType<T>>::value,
                  "ForceElementType<T> must be a sub-class of "
                  "ForceElement<T>.");
    return AddForceElement(
        std::make_unique<ForceElementType<T>>(std::forward<Args>(args)...));
  }

  /// This method helps to create a Joint of type `JointType` between two
  /// bodies.
  /// The two bodies connected by this Joint object are referred to as the
  /// _parent_ and _child_ bodies. Although the terms _parent_ and _child_ are
  /// sometimes used synonymously to describe the relationship between inboard
  /// and outboard bodies in multibody models, this usage is wholly unrelated
  /// and implies nothing about the inboard-outboard relationship between the
  /// bodies.
  /// As explained in the Joint class's documentation, in Drake we define a
  /// frame F attached to the parent body P with pose `X_PF` and a frame M
  /// attached to the child body B with pose `X_BM`. This method helps creating
  /// a joint between two bodies with fixed poses `X_PF` and `X_BM`.
  /// Refer to the Joint class's documentation for more details.
  ///
  /// The arguments to this method `args` are forwarded to `JointType`'s
  /// constructor. The newly created `JointType` object will be specialized on
  /// the scalar type T of this %MultibodyTree.
  ///
  /// @param name
  ///   The name of the joint.
  /// @param[in] parent
  ///   The parent body connected by the new joint.
  /// @param[in] X_PF
  ///   The fixed pose of frame F attached to the parent body, measured in
  ///   the frame P of that body. `X_PF` is an optional parameter; empty curly
  ///   braces `{}` imply that frame F **is** the same body frame P. If instead
  ///   your intention is to make a frame F with pose `X_PF`, provide
  ///   `Isometry3<double>::Identity()` as your input.
  /// @param[in] child
  ///   The child body connected by the new joint.
  /// @param[in] X_BM
  ///   The fixed pose of frame M attached to the child body, measured in
  ///   the frame B of that body. `X_BM` is an optional parameter; empty curly
  ///   braces `{}` imply that frame M **is** the same body frame B. If instead
  ///   your intention is to make a frame F with pose `X_PF`, provide
  ///   `Isometry3<double>::Identity()` as your input.
  ///
  /// Example of usage:
  /// @code
  ///   MultibodyTree<T> model;
  ///   // ... Code to define a parent body P and a child body B.
  ///   const Body<double>& parent_body =
  ///     model.AddBody<RigidBody>(SpatialInertia<double>(...));
  ///   const Body<double>& child_body =
  ///     model.AddBody<RigidBody>(SpatialInertia<double>(...));
  ///   // Define the pose X_BM of a frame M rigidly atached to child body B.
  ///   const RevoluteJoint<double>& elbow =
  ///     model.AddJoint<RevoluteJoint>(
  ///       "Elbow",                /* joint name */
  ///       model.get_world_body(), /* parent body */
  ///       {},                     /* frame F IS the parent body frame P */
  ///       pendulum,               /* child body, the pendulum */
  ///       X_BM,                   /* pose of frame M in the body frame B */
  ///       Vector3d::UnitZ());     /* revolute axis in this case */
  /// @endcode
  ///
  /// @see The Joint class's documentation for further details on how a Joint
  /// is defined.
  template<template<typename> class JointType, typename... Args>
  const JointType<T>& AddJoint(
      const std::string& name,
      const Body<T>& parent, const optional<Isometry3<double>>& X_PF,
      const Body<T>& child, const optional<Isometry3<double>>& X_BM,
      Args&&... args) {
    static_assert(std::is_base_of<Joint<T>, JointType<T>>::value,
                  "JointType<T> must be a sub-class of Joint<T>.");

    const Frame<T>* frame_on_parent;
    if (X_PF) {
      frame_on_parent = &this->AddFrame<FixedOffsetFrame>(parent, *X_PF);
    } else {
      frame_on_parent = &parent.get_body_frame();
    }

    const Frame<T>* frame_on_child;
    if (X_BM) {
      frame_on_child = &this->AddFrame<FixedOffsetFrame>(child, *X_BM);
    } else {
      frame_on_child = &child.get_body_frame();
    }

    return AddJoint(
        std::make_unique<JointType<T>>(
            name,
            *frame_on_parent, *frame_on_child,
            std::forward<Args>(args)...));
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

  /// Returns the number of bodies in the %MultibodyTree including the *world*
  /// body. Therefore the minimum number of bodies in a MultibodyTree is one.
  int get_num_bodies() const { return static_cast<int>(owned_bodies_.size()); }

  /// Returns the number of joints added with AddJoint() to the %MultibodyTree.
  int get_num_joints() const { return static_cast<int>(owned_joints_.size()); }

  /// Returns the number of mobilizers in the %MultibodyTree. Since the world
  /// has no Mobilizer, the number of mobilizers equals the number of bodies
  /// minus one, i.e. get_num_mobilizers() returns get_num_bodies() - 1.
  // TODO(amcastro-tri): Consider adding a WorldMobilizer (0-dofs) for the world
  // body. This could be useful to query for reaction forces of the entire
  // model.
  int get_num_mobilizers() const {
    return static_cast<int>(owned_mobilizers_.size());
  }

  /// Returns the number of ForceElement objects in the MultibodyTree.
  int get_num_force_elements() const {
    return static_cast<int>(owned_force_elements_.size());
  }

  /// Returns the number of generalized positions of the model.
  int get_num_positions() const {
    return topology_.get_num_positions();
  }

  /// Returns the number of generalized velocities of the model.
  int get_num_velocities() const {
    return topology_.get_num_velocities();
  }

  /// Returns the total size of the state vector in the model.
  int get_num_states() const {
    return topology_.get_num_states();
  }

  /// Returns the height of the tree data structure of `this` %MultibodyTree.
  /// That is, the number of bodies in the longest kinematic path between the
  /// world and any other leaf body. For a model that only contains the _world_
  /// body, the height of the tree is one.
  /// Kinematic paths are created by Mobilizer objects connecting a chain of
  /// frames. Therefore, this method does not count kinematic cycles, which
  /// could only be considered in the model using constraints.
  int get_tree_height() const {
    return topology_.get_tree_height();
  }

  /// Returns a constant reference to the *world* body.
  const RigidBody<T>& get_world_body() const {
    // world_body_ is set in the constructor. So this assert is here only to
    // verify future constructors do not mess that up.
    DRAKE_ASSERT(world_body_ != nullptr);
    return *world_body_;
  }

  /// Returns a constant reference to the *world* frame.
  const BodyFrame<T>& get_world_frame() const {
    return owned_bodies_[world_index()]->get_body_frame();
  }

  /// Returns a constant reference to the body with unique index `body_index`.
  /// @throws std::runtime_error when `body_index` does not correspond to a body
  /// in this multibody tree.
  const Body<T>& get_body(BodyIndex body_index) const {
    DRAKE_THROW_UNLESS(body_index < get_num_bodies());
    return *owned_bodies_[body_index];
  }

  /// Returns a constant reference to the joint with unique index `joint_index`.
  /// @throws std::runtime_error when `joint_index` does not correspond to a
  /// joint in this multibody tree.
  const Joint<T>& get_joint(JointIndex joint_index) const {
    DRAKE_THROW_UNLESS(joint_index < get_num_joints());
    return *owned_joints_[joint_index];
  }

  /// Returns a constant reference to the frame with unique index `frame_index`.
  /// @throws std::runtime_error when `frame_index` does not correspond to a
  /// frame in `this` multibody tree.
  const Frame<T>& get_frame(FrameIndex frame_index) const {
    DRAKE_THROW_UNLESS(frame_index < get_num_frames());
    return *frames_[frame_index];
  }

  /// Returns a constant reference to the mobilizer with unique index
  /// `mobilizer_index`.
  /// @throws std::runtime_error when `mobilizer_index` does not correspond to a
  /// mobilizer in this multibody tree.
  const Mobilizer<T>& get_mobilizer(MobilizerIndex mobilizer_index) const {
    DRAKE_THROW_UNLESS(mobilizer_index < get_num_mobilizers());
    return *owned_mobilizers_[mobilizer_index];
  }

  /// Returns `true` if this %MultibodyTree was finalized with Finalize() after
  /// all multibody elements were added, and `false` otherwise.
  /// When a %MultibodyTree is instantiated, its topology remains invalid until
  /// Finalize() is called, which validates the topology.
  /// @see Finalize().
  bool topology_is_valid() const { return topology_.is_valid(); }

  /// Returns the topology information for this multibody tree. Users should not
  /// need to call this method since MultibodyTreeTopology is an internal
  /// bookkeeping detail. Used at Finalize() stage by multibody elements to
  /// retrieve a local copy of their topology.
  const MultibodyTreeTopology& get_topology() const { return topology_; }

  /// This method must be called after all elements in the tree (joints, bodies,
  /// force elements, constraints) were added and before any computations are
  /// performed.
  /// It essentially compiles all the necessary "topological information", i.e.
  /// how bodies, joints and, any other elements connect with each other, and
  /// performs all the required pre-processing to perform computations at a
  /// later stage.
  ///
  /// If the finalize stage is successful, the topology of this %MultibodyTree
  /// is validated, meaning that the topology is up-to-date after this call.
  /// No more multibody tree elements can be added after a call to Finalize().
  ///
  /// @throws std::logic_error If users attempt to call this method on an
  ///         already finalized %MultibodyTree.
  // TODO(amcastro-tri): Consider making this method private and calling it
  // automatically when CreateDefaultContext() is called.
  void Finalize();

  /// Allocates a new context for this %MultibodyTree uniquely identifying the
  /// state of the multibody system.
  ///
  /// @pre The method Finalize() must be called before attempting to create a
  /// context in order for the %MultibodyTree topology to be valid at the moment
  /// of allocation.
  ///
  /// @throws std::logic_error If users attempt to call this method on a
  ///         %MultibodyTree with an invalid topology.
  std::unique_ptr<systems::LeafContext<T>> CreateDefaultContext() const;

  /// Sets default values in the context. For mobilizers, this method sets them
  /// to their _zero_ configuration according to
  /// Mobilizer::set_zero_configuration().
  void SetDefaultContext(systems::Context<T>* context) const;

  /// Sets default values in the `state`. For mobilizers, this method sets them
  /// to their _zero_ configuration according to
  /// Mobilizer::set_zero_configuration().
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const;

  /// @name Computational methods
  /// These methods expose the computational capabilities of MultibodyTree to
  /// compute kinematics, forward and inverse dynamics, and Jacobian matrices,
  /// among others.
  /// These methods follow Drake's naming scheme for methods performing a
  /// computation and therefore are named `CalcXXX()`, where `XXX` corresponds
  /// to the quantity or object of interest to be computed. They all take a
  /// `systems::Context` as an input argument storing the state of the multibody
  /// system. A `std::bad_cast` exception is thrown if the passed context is not
  /// a MultibodyTreeContext.
  /// @{

  /// Computes into the position kinematics `pc` all the kinematic quantities
  /// that depend on the generalized positions only. These include:
  /// - For each body B, the pose `X_BF` of each of the frames F attached to
  ///   body B.
  /// - Pose `X_WB` of each body B in the model as measured and expressed in
  ///   the world frame W.
  /// - Across-mobilizer Jacobian matrices `H_FM` and `H_PB_W`.
  /// - Body specific quantities such as `com_W` and `M_Bo_W`.
  ///
  /// Aborts if `pc` is nullptr.
  void CalcPositionKinematicsCache(
      const systems::Context<T>& context,
      PositionKinematicsCache<T>* pc) const;

  /// Computes all the kinematic quantities that depend on the generalized
  /// velocities and stores them in the velocity kinematics cache `vc`.
  /// These include:
  /// - Spatial velocity `V_WB` for each body B in the model as measured and
  ///   expressed in the world frame W.
  /// - Spatial velocity `V_PB` for each body B in the model as measured and
  ///   expressed in the inboard (or parent) body frame P.
  ///
  /// @pre The position kinematics `pc` must have been previously updated with a
  /// call to CalcPositionKinematicsCache().
  ///
  /// Aborts if `vc` is nullptr.
  void CalcVelocityKinematicsCache(
      const systems::Context<T>& context,
      const PositionKinematicsCache<T>& pc,
      VelocityKinematicsCache<T>* vc) const;

  /// Computes all the kinematic quantities that depend on the generalized
  /// accelerations that is, the generalized velocities' time derivatives, and
  /// stores them in the acceleration kinematics cache `ac`.
  /// These include:
  /// - Spatial acceleration `A_WB` for each body B in the model as measured and
  ///   expressed in the world frame W.
  ///
  /// @param[in] context
  ///   The context containing the state of the %MultibodyTree model.
  /// @param[in] pc
  ///   A position kinematics cache object already updated to be in sync with
  ///   `context`.
  /// @param[in] vc
  ///   A velocity kinematics cache object already updated to be in sync with
  ///   `context`.
  /// @param[in] known_vdot
  ///   A vector with the generalized accelerations for the full %MultibodyTree
  ///   model.
  /// @param[out] ac
  ///   A pointer to a valid, non nullptr, acceleration kinematics cache. This
  ///   method aborts if `ac` is nullptr.
  ///
  /// @pre The position kinematics `pc` must have been previously updated with a
  /// call to CalcPositionKinematicsCache().
  /// @pre The velocity kinematics `vc` must have been previously updated with a
  /// call to CalcVelocityKinematicsCache().
  void CalcAccelerationKinematicsCache(
      const systems::Context<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc,
      const VectorX<T>& known_vdot,
      AccelerationKinematicsCache<T>* ac) const;

  /// Given the state of `this` %MultibodyTree in `context` and a known vector
  /// of generalized accelerations `known_vdot`, this method computes the
  /// spatial acceleration `A_WB` for each body as measured and expressed in the
  /// world frame W.
  ///
  /// @param[in] context
  ///   The context containing the state of the %MultibodyTree model.
  /// @param[in] pc
  ///   A position kinematics cache object already updated to be in sync with
  ///   `context`.
  /// @param[in] vc
  ///   A velocity kinematics cache object already updated to be in sync with
  ///   `context`.
  /// @param[in] known_vdot
  ///   A vector with the generalized accelerations for the full %MultibodyTree
  ///   model.
  /// @param[out] A_WB_array
  ///   A pointer to a valid, non nullptr, vector of spatial accelerations
  ///   containing the spatial acceleration `A_WB` for each body. It must be of
  ///   size equal to the number of bodies in the MultibodyTree. This method
  ///   will abort if the the pointer is null or if `A_WB_array` is not of size
  ///   `get_num_bodies()`. On output, entries will be ordered by BodyNodeIndex.
  ///   These accelerations can be read in the proper order with
  ///   Body::get_from_spatial_acceleration_array().
  ///
  /// @pre The position kinematics `pc` must have been previously updated with a
  /// call to CalcPositionKinematicsCache().
  /// @pre The velocity kinematics `vc` must have been previously updated with a
  /// call to CalcVelocityKinematicsCache().
  void CalcSpatialAccelerationsFromVdot(
      const systems::Context<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc,
      const VectorX<T>& known_vdot,
      std::vector<SpatialAcceleration<T>>* A_WB_array) const;

  /// Given the state of `this` %MultibodyTree in `context` and a known vector
  /// of generalized accelerations `vdot`, this method computes the
  /// set of generalized forces `tau` that would need to be applied at each
  /// Mobilizer in order to attain the specified generalized accelerations.
  /// Mathematically, this method computes: <pre>
  ///   tau = M(q)v̇ + C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W
  /// </pre>
  /// where `M(q)` is the %MultibodyTree mass matrix, `C(q, v)v` is the bias
  /// term containing Coriolis and gyroscopic effects and `tau_app` consists
  /// of a vector applied generalized forces. The last term is a summation over
  /// all bodies in the model where `Fapp_Bo_W` is an applied spatial force on
  /// body B at `Bo` which gets projected into the space of generalized forces
  /// with the geometric Jacobian `J_WB(q)` which maps generalized velocities
  /// into body B spatial velocity as `V_WB = J_WB(q)v`.
  /// This method does not compute explicit expressions for the mass matrix nor
  /// for the bias term, which would be of at least `O(n²)` complexity, but it
  /// implements an `O(n)` Newton-Euler recursive algorithm, where n is the
  /// number of bodies in the %MultibodyTree. The explicit formation of the
  /// mass matrix `M(q)` would require the calculation of `O(n²)` entries while
  /// explicitly forming the product `C(q, v) * v` could require up to `O(n³)`
  /// operations (see [Featherstone 1987, §4]), depending on the implementation.
  /// The recursive Newton-Euler algorithm is the most efficient currently known
  /// general method for solving inverse dynamics [Featherstone 2008].
  ///
  /// @param[in] context
  ///   The context containing the state of the %MultibodyTree model.
  /// @param[in] pc
  ///   A position kinematics cache object already updated to be in sync with
  ///   `context`.
  /// @param[in] vc
  ///   A velocity kinematics cache object already updated to be in sync with
  ///   `context`.
  /// @param[in] known_vdot
  ///   A vector with the known generalized accelerations `vdot` for the full
  ///   %MultibodyTree model. Use Mobilizer::get_accelerations_from_array() to
  ///   access entries into this array for a particular Mobilizer. You can use
  ///   the mutable version of this method to write into this array.
  /// @param[in] Fapplied_Bo_W_array
  ///   A vector containing the spatial force `Fapplied_Bo_W` applied on each
  ///   body at the body's frame origin `Bo` and expressed in the world frame W.
  ///   `Fapplied_Bo_W_array` can have zero size which means there are no
  ///   applied forces. To apply non-zero forces, `Fapplied_Bo_W_array` must be
  ///   of size equal to the number of bodies in `this` %MultibodyTree model.
  ///   This array must be ordered by BodyNodeIndex, which for a given body can
  ///   be retrieved with Body::get_node_index().
  ///   This method will abort if provided with an array that does not have a
  ///   size of either `get_num_bodies()` or zero.
  /// @param[in] tau_applied_array
  ///   An array of applied generalized forces for the entire model. For a
  ///   given mobilizer, entries in this array can be accessed using the method
  ///   Mobilizer::get_generalized_forces_from_array() while its mutable
  ///   counterpart, Mobilizer::get_mutable_generalized_forces_from_array(),
  ///   allows writing into this array.
  ///   `tau_applied_array` can have zero size, which means there are no applied
  ///   forces. To apply non-zero forces, `tau_applied_array` must be of size
  ///   equal to the number to the number of generalized velocities in the
  ///   model, see MultibodyTree::get_num_velocities().
  ///   This method will abort if provided with an array that does not have a
  ///   size of either MultibodyTree::get_num_velocities() or zero.
  /// @param[out] A_WB_array
  ///   A pointer to a valid, non nullptr, vector of spatial accelerations
  ///   containing the spatial acceleration `A_WB` for each body. It must be of
  ///   size equal to the number of bodies. This method will abort if the the
  ///   pointer is null or if `A_WB_array` is not of size `get_num_bodies()`.
  ///   On output, entries will be ordered by BodyNodeIndex.
  ///   To access the acceleration `A_WB` of given body B in this array, use the
  ///   index returned by Body::get_node_index().
  /// @param[out] F_BMo_W_array
  ///   A pointer to a valid, non nullptr, vector of spatial forces
  ///   containing, for each body B, the spatial force `F_BMo_W` corresponding
  ///   to its inboard mobilizer reaction forces on body B applied at the origin
  ///   `Mo` of the inboard mobilizer, expressed in the world frame W.
  ///   It must be of size equal to the number of bodies in the MultibodyTree.
  ///   This method will abort if the the pointer is null or if `F_BMo_W_array`
  ///   is not of size `get_num_bodies()`.
  ///   On output, entries will be ordered by BodyNodeIndex.
  ///   To access a mobilizer's reaction force on given body B in this array,
  ///   use the index returned by Body::get_node_index().
  /// @param[out] tau_array
  ///   On output this array will contain the generalized forces that must be
  ///   applied in order to achieve the desired generalized accelerations given
  ///   by the input argument `known_vdot`. It must not be nullptr and it
  ///   must be of size MultibodyTree::get_num_velocities(). Generalized forces
  ///   for each Mobilizer can be accessed with
  ///   Mobilizer::get_generalized_forces_from_array().
  ///
  /// @warning There is no mechanism to assert that either `A_WB_array` nor
  ///   `F_BMo_W_array` are ordered by BodyNodeIndex. You can use
  ///   Body::get_node_index() to obtain the node index for a given body.
  ///
  /// @note This method uses `F_BMo_W_array` and `tau_array` as the only local
  /// temporaries and therefore no additional dynamic memory allocation is
  /// performed.
  ///
  /// @warning `F_BMo_W_array` (`tau_array`) and `Fapplied_Bo_W_array`
  /// (`tau_applied_array`) can actually be the same
  /// array in order to reduce memory footprint and/or dynamic memory
  /// allocations. However the information in `Fapplied_Bo_W_array`
  /// (`tau_applied_array`) would be overwritten through `F_BMo_W_array`
  /// (`tau_array`). Make a copy if data must be preserved.
  ///
  /// @pre The position kinematics `pc` must have been previously updated with a
  /// call to CalcPositionKinematicsCache().
  /// @pre The velocity kinematics `vc` must have been previously updated with a
  /// call to CalcVelocityKinematicsCache().
  void CalcInverseDynamics(
      const systems::Context<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc,
      const VectorX<T>& known_vdot,
      const std::vector<SpatialForce<T>>& Fapplied_Bo_W_array,
      const Eigen::Ref<const VectorX<T>>& tau_applied_array,
      std::vector<SpatialAcceleration<T>>* A_WB_array,
      std::vector<SpatialForce<T>>* F_BMo_W_array,
      EigenPtr<VectorX<T>> tau_array) const;

  /// Computes the combined force contribution of ForceElement objects in the
  /// model. A ForceElement can apply forces as a spatial force per body or as
  /// generalized forces, depending on the ForceElement model. Therefore this
  /// method provides outputs for both spatial forces per body (with
  /// `F_Bo_W_array`) and generalized forces (with `tau_array`).
  /// ForceElement contributions are a function of the state and time only.
  /// The output from this method can immediately be used as input to
  /// CalcInverseDynamics() to include the effect of applied forces by force
  /// elements.
  ///
  /// @param[in] context
  ///   The context containing the state of the %MultibodyTree model.
  /// @param[in] pc
  ///   A position kinematics cache object already updated to be in sync with
  ///   `context`.
  /// @param[in] vc
  ///   A velocity kinematics cache object already updated to be in sync with
  ///   `context`.
  /// @param[out] forces
  ///   A pointer to a valid, non nullptr, multibody forces object. On output
  ///   `forces` will store the forces exerted by all the ForceElement
  ///   objects in the model. This method will abort if the `forces` pointer is
  ///   null or if the forces object is not compatible with `this`
  ///   %MultibodyTree, see MultibodyForces::CheckInvariants().
  ///
  /// @pre The position kinematics `pc` must have been previously updated with a
  /// call to CalcPositionKinematicsCache().
  /// @pre The velocity kinematics `vc` must have been previously updated with a
  /// call to CalcVelocityKinematicsCache().
  void CalcForceElementsContribution(
      const systems::Context<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc,
      MultibodyForces<T>* forces) const;

  /// Computes and returns the total potential energy stored in `this` multibody
  /// model for the configuration given by `context`.
  /// @param[in] context
  ///   The context containing the state of the %MultibodyTree model.
  /// @returns The total potential energy stored in `this` multibody model.
  T CalcPotentialEnergy(const systems::Context<T>& context) const;

  /// Computes and returns the power generated by conservative forces in the
  /// multibody model. This quantity is defined to be positive when the
  /// potential energy is decreasing. In other words, if `U(q)` is the potential
  /// energy as defined by CalcPotentialEnergy(), then the conservative power,
  /// `Pc`, is `Pc = -U̇(q)`.
  ///
  /// @see CalcPotentialEnergy()
  T CalcConservativePower(const systems::Context<T>& context) const;

  /// Performs the computation of the mass matrix `M(q)` of the model using
  /// inverse dynamics, where the generalized positions q are stored in
  /// `context`. See CalcInverseDynamics().
  ///
  /// @param[in] context
  ///   The context containing the state of the %MultibodyTree model.
  /// @param[out] H
  ///   A valid (non-null) pointer to a squared matrix in `ℛⁿˣⁿ` with n the
  ///   number of generalized velocities (get_num_velocities()) of the model.
  ///   This method aborts if H is nullptr or if it does not have the proper
  ///   size.
  ///
  /// The algorithm used to build `M(q)` consists in computing one column of
  /// `M(q)` at a time using inverse dynamics. The result from inverse dynamics,
  /// with no applied forces, is the vector of generalized forces: <pre>
  ///   tau = M(q)v̇ + C(q, v)v
  /// </pre>
  /// where q and v are the generalized positions and velocities, respectively.
  /// When `v = 0` the Coriolis and gyroscopic forces term `C(q, v)v` is zero.
  /// Therefore the `i-th` column of `M(q)` can be obtained performing inverse
  /// dynamics with an acceleration vector `v̇ = eᵢ`, with `eᵢ` the standard
  /// (or natural) basis of `ℛⁿ` with n the number of generalized velocities.
  /// We write this as: <pre>
  ///   H.ᵢ(q) = M(q) * e_i
  /// </pre>
  /// where `H.ᵢ(q)` (notice the dot for the rows index) denotes the `i-th`
  /// column in M(q).
  ///
  /// @warning This is an O(n²) algorithm. Avoid the explicit computation of the
  /// mass matrix whenever possible.
  void CalcMassMatrixViaInverseDynamics(
      const systems::Context<T>& context, EigenPtr<MatrixX<T>> H) const;

  /// Computes the bias term `C(q, v)v` containing Coriolis and gyroscopic
  /// effects of the multibody equations of motion: <pre>
  ///   M(q)v̇ + C(q, v)v = tau_app + ∑ J_WBᵀ(q) Fapp_Bo_W
  /// </pre>
  /// where `M(q)` is the multibody model's mass matrix and `tau_app` consists
  /// of a vector applied generalized forces. The last term is a summation over
  /// all bodies in the model where `Fapp_Bo_W` is an applied spatial force on
  /// body B at `Bo` which gets projected into the space of generalized forces
  /// with the geometric Jacobian `J_WB(q)` which maps generalized velocities
  /// into body B spatial velocity as `V_WB = J_WB(q)v`.
  ///
  /// @param[in] context
  ///   The context containing the state of the %MultibodyTree model. It stores
  ///   the generalized positions q and the generalized velocities v.
  /// @param[out] Cv
  ///   On output, `Cv` will contain the product `C(q, v)v`. It must be a valid
  ///   (non-null) pointer to a column vector in `ℛⁿ` with n the number of
  ///   generalized velocities (get_num_velocities()) of the model.
  ///   This method aborts if Cv is nullptr or if it does not have the
  ///   proper size.
  void CalcBiasTerm(
      const systems::Context<T>& context, EigenPtr<VectorX<T>> Cv) const;

  /// Computes the relative transform `X_AB(q)` from a frame B to a frame A, as
  /// a function of the generalized positions q of the model.
  /// That is, the position `p_AQ` of a point Q measured and expressed in
  /// frame A can be computed from the position `p_BQ` of this point measured
  /// and expressed in frame B using the transformation `p_AQ = X_AB⋅p_BQ`.
  ///
  /// @param[in] context
  ///   The context containing the state of the %MultibodyTree model. It stores
  ///   the generalized positions q of the model.
  /// @param[in] to_frame_A
  ///   The target frame A in the computed relative transform `X_AB`.
  /// @param[in] from_frame_B
  ///   The source frame B in the computed relative transform `X_AB`.
  /// @retval X_AB
  ///   The relative transform from frame B to frame A, such that
  ///   `p_AQ = X_AB⋅p_BQ`.
  Isometry3<T> CalcRelativeTransform(
      const systems::Context<T>& context,
      const Frame<T>& to_frame_A, const Frame<T>& from_frame_B) const;

  ///  Given the positions `p_BQi` for a set of points `Qi` measured and
  ///  expressed in a frame B, this method computes the positions `p_AQi(q)` of
  ///  each point `Qi` in the set as measured and expressed in another frame A,
  ///  as a function of the generalized positions q of the model.
  ///
  /// @param[in] context
  ///   The context containing the state of the %MultibodyTree model. It stores
  ///   the generalized positions q of the model.
  /// @param[in] from_frame_B
  ///   The frame B in which the positions `p_BQi` of a set of points `Qi` are
  ///   given.
  /// @param[in] p_BQi
  ///   The input positions of each point `Qi` in frame B. `p_BQi ∈ ℝ³ˣⁿᵖ` with
  ///   `np` the number of points in the set. Each column of `p_BQi` corresponds
  ///   to a vector in ℝ³ holding the position of one of the points in the set
  ///   as measured and expressed in frame B.
  /// @param[in] to_frame_A
  ///   The frame A in which it is desired to compute the positions `p_AQi` of
  ///   each point `Qi` in the set.
  /// @param[out] p_AQi
  ///   The output positions of each point `Qi` now computed as measured and
  ///   expressed in frame A. The output `p_AQi` **must** have the same size as
  ///   the input `p_BQi` or otherwise this method aborts. That is `p_AQi`
  ///   **must** be in `ℝ³ˣⁿᵖ`.
  ///
  /// @note Both `p_BQi` and `p_AQi` must have three rows. Otherwise this
  /// method will throw a std::runtime_error exception. This method also throws
  /// a std::runtime_error exception if `p_BQi` and `p_AQi` differ in the number
  /// of columns.
  void CalcPointsPositions(
      const systems::Context<T>& context,
      const Frame<T>& from_frame_B,
      const Eigen::Ref<const MatrixX<T>>& p_BQi,
      const Frame<T>& to_frame_A,
      EigenPtr<MatrixX<T>> p_AQi) const;

  /// @name Methods to compute multibody Jacobians.
  /// @{

  /// Given a set of points `Qi` with fixed position vectors `p_BQi` in a frame
  /// B, (that is, their time derivative `ᴮd/dt(p_BQi)` in frame B is zero),
  /// this method computes the geometric Jacobian `J_WQi` defined by:
  /// <pre>
  ///   J_WQi(q) = d(v_WQi(q, v))/dv
  /// </pre>
  /// where `p_WQi` is the position vector in the world frame for each point
  /// `Qi` in the input set, `v_WQi` is the translational velocity of point `Qi`
  /// in the world frame W and v is the vector of generalized velocities. Since
  /// the spatial velocity of each point `Qi` is linear in the generalized
  /// velocities, the geometric Jacobian `J_WQi` is a function of the
  /// generalized coordinates q only.
  ///
  /// @param[in] context
  ///   The context containing the state of the %MultibodyTree model. It stores
  ///   the generalized positions q.
  /// @param[in] frame_B
  ///   The positions `p_BQi` of each point in the input set are measured and
  ///   expressed in this frame B and are constant (fixed) in this frame.
  /// @param[in] p_BQi_set
  ///   A matrix with the fixed position of a set of points `Qi` measured and
  ///   expressed in `frame_B`.
  ///   Each column of this matrix contains the position vector `p_BQi` for a
  ///   point `Qi` measured and expressed in frame B. Therefore this input
  ///   matrix lives in ℝ³ˣⁿᵖ with `np` the number of points in the set.
  /// @param[out] p_WQi_set
  ///   The output positions of each point `Qi` now computed as measured and
  ///   expressed in frame W. These positions are computed in the process of
  ///   computing the geometric Jacobian `J_WQi` and therefore external storage
  ///   must be provided.
  ///   The output `p_WQi_set` **must** have the same size
  ///   as the input set `p_BQi_set` or otherwise this method throws a
  ///   std::runtime_error exception. That is `p_WQi_set` **must** be in
  ///   `ℝ³ˣⁿᵖ`.
  /// @param[out] J_WQi
  ///   The geometric Jacobian `J_WQi(q)`, function of the generalized positions
  ///   q only. This Jacobian relates the translational velocity `v_WQi` of
  ///   each point `Qi` in the input set by: <pre>
  ///     `v_WQi(q, v) = J_WQi(q)⋅v`
  ///   </pre>
  ///   so that `v_WQi` is a column vector of size `3⋅np` concatenating the
  ///   velocity of all points `Qi` in the same order they were given in the
  ///   input set. Therefore `J_WQi` is a matrix of size `3⋅np x nv`, with `nv`
  ///   the number of generalized velocities. On input, matrix `J_WQi` **must**
  ///   have size `3⋅np x nv` or this method throws a std::runtime_error
  ///   exception.
  void CalcPointsGeometricJacobianExpressedInWorld(
      const systems::Context<T>& context,
      const Frame<T>& frame_B, const Eigen::Ref<const MatrixX<T>>& p_BQi_set,
      EigenPtr<MatrixX<T>> p_WQi_set, EigenPtr<MatrixX<T>> J_WQi) const;

  /// @}
  // End of multibody Jacobian methods section.

  /// Transforms generalized velocities v to time derivatives `qdot` of the
  /// generalized positions vector `q` (stored in `context`). `v` and `qdot`
  /// are related linearly by `q̇ = N(q)⋅v`.
  /// Using the configuration `q` stored in the given `context` this method
  /// calculates `q̇ = N(q)⋅v`.
  ///
  /// @param[in] context
  ///   The context containing the state of the %MultibodyTree model.
  /// @param[in] v
  ///   A vector of of generalized velocities for `this` %MultibodyTree model.
  ///   This method aborts if v is not of size get_num_velocities().
  /// @param[out] qdot
  ///   A valid (non-null) pointer to a vector in `ℝⁿ` with n being the number
  ///   of generalized positions in `this` %MultibodyTree model,
  ///   given by `get_num_positions()`. This method aborts if `qdot` is nullptr
  ///   or if it is not of size get_num_positions().
  ///
  /// @see MapQDotToVelocity()
  /// @see Mobilizer::MapVelocityToQDot()
  void MapVelocityToQDot(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v,
      EigenPtr<VectorX<T>> qdot) const;

  /// Transforms the time derivative `qdot` of the generalized positions vector
  /// `q` (stored in `context`) to generalized velocities `v`. `v` and `q̇`
  /// are related linearly by `q̇ = N(q)⋅v`. Although `N(q)` is not
  /// necessarily square, its left pseudo-inverse `N⁺(q)` can be used to
  /// invert that relationship without residual error, provided that `qdot` is
  /// in the range space of `N(q)` (that is, if it *could* have been produced as
  /// `q̇ = N(q)⋅v` for some `v`).
  /// Using the configuration `q` stored in the given `context` this method
  /// calculates `v = N⁺(q)⋅q̇`.
  ///
  /// @param[in] context
  ///   The context containing the state of the %MultibodyTree model.
  /// @param[in] qdot
  ///   A vector containing the time derivatives of the generalized positions.
  ///   This method aborts if `qdot` is not of size get_num_positions().
  /// @param[out] v
  ///   A valid (non-null) pointer to a vector in `ℛⁿ` with n the number of
  ///   generalized velocities. This method aborts if v is nullptr or if it
  ///   is not of size get_num_velocities().
  ///
  /// @see MapVelocityToQDot()
  /// @see Mobilizer::MapQDotToVelocity()
  void MapQDotToVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      EigenPtr<VectorX<T>> v) const;

  /// @}
  // Closes "Computational methods" Doxygen section.

  /// @name Methods to retrieve multibody element variants
  ///
  /// Given two variants of the same %MultibodyTree, these methods map an
  /// element in one variant, to its corresponding element in the other variant.
  ///
  /// A concrete case is the call to ToAutoDiffXd() to obtain a
  /// %MultibodyTree variant templated on AutoDiffXd from a %MultibodyTree
  /// templated on `double`. Typically, a user holding a `Body<double>` (or any
  /// other multibody element in the original variant templated on `double`)
  /// would like to retrieve the corresponding `Body<AutoDiffXd>` variant from
  /// the new AutoDiffXd tree variant.
  ///
  /// Consider the following code example:
  /// @code
  ///   // The user creates a model.
  ///   MultibodyTree<double> model;
  ///   // User adds a body and keeps a reference to it.
  ///   const RigidBody<double>& body = model.AddBody<RigidBody>(...);
  ///   // User creates an AutoDiffXd variant. Variants on other scalar types
  ///   // can be created with a call to CloneToScalar().
  ///   std::unique_ptr<MultibodyTree<Tvariant>> variant_model =
  ///       model.ToAutoDiffXd();
  ///   // User retrieves the AutoDiffXd variant corresponding to the original
  ///   // body added above.
  ///   const RigidBody<AutoDiffXd>&
  ///       variant_body = variant_model.get_variant(body);
  /// @endcode
  ///
  /// MultibodyTree::get_variant() is templated on the multibody element
  /// type which is deduced from its only input argument. The returned element
  /// is templated on the scalar type T of the %MultibodyTree on which this
  /// method is invoked.
  /// @{

  /// SFINAE overload for Frame<T> elements.
  template <template <typename> class MultibodyElement, typename Scalar>
  std::enable_if_t<std::is_base_of<Frame<T>, MultibodyElement<T>>::value,
                   const MultibodyElement<T>&> get_variant(
      const MultibodyElement<Scalar>& element) const {
    return get_frame_variant(element);
  }

  /// SFINAE overload for Body<T> elements.
  template <template <typename> class MultibodyElement, typename Scalar>
  std::enable_if_t<std::is_base_of<Body<T>, MultibodyElement<T>>::value,
                   const MultibodyElement<T>&> get_variant(
      const MultibodyElement<Scalar>& element) const {
    return get_body_variant(element);
  }

  /// SFINAE overload for Mobilizer<T> elements.
  template <template <typename> class MultibodyElement, typename Scalar>
  std::enable_if_t<std::is_base_of<Mobilizer<T>, MultibodyElement<T>>::value,
                   const MultibodyElement<T>&> get_variant(
      const MultibodyElement<Scalar>& element) const {
    return get_mobilizer_variant(element);
  }

  /// SFINAE overload for Joint<T> elements.
  template <template <typename> class MultibodyElement, typename Scalar>
  std::enable_if_t<std::is_base_of<Joint<T>, MultibodyElement<T>>::value,
                   const MultibodyElement<T>&> get_variant(
      const MultibodyElement<Scalar>& element) const {
    return get_joint_variant(element);
  }
  /// @}

  /// Creates a deep copy of `this` %MultibodyTree templated on the same
  /// scalar type T as `this` tree.
  std::unique_ptr<MultibodyTree<T>> Clone() const {
    return CloneToScalar<T>();
  }

  /// Creates a deep copy of `this` %MultibodyTree templated on AutoDiffXd.
  std::unique_ptr<MultibodyTree<AutoDiffXd>> ToAutoDiffXd() const {
    return CloneToScalar<AutoDiffXd>();
  }

  /// Creates a deep copy of `this` %MultibodyTree templated on the scalar type
  /// `ToScalar`.
  /// The new deep copy is guaranteed to have exactly the same
  /// MultibodyTreeTopology as the original tree the method is called on.
  /// This method ensures the following cloning order:
  ///   - Body objects, and their corresponding BodyFrame objects.
  ///   - Frame objects.
  ///   - If a Frame is attached to another frame, its parent frame is
  ///     guaranteed to be created first.
  ///   - Mobilizer objects are created last and therefore clones of the
  ///     original Frame objects are guaranteed to already be part of the cloned
  ///     tree.
  ///
  /// Consider the following code example:
  /// @code
  ///   // The user creates a model.
  ///   MultibodyTree<double> model;
  ///   // User adds a body and keeps a reference to it.
  ///   const RigidBody<double>& body = model.AddBody<RigidBody>(...);
  ///   // User creates an AutoDiffXd variant, where ToScalar = AutoDiffXd.
  ///   std::unique_ptr<MultibodyTree<AutoDiffXd>> model_autodiff =
  ///       model.CloneToScalar<AutoDiffXd>();
  ///   // User retrieves the AutoDiffXd variant corresponding to the original
  ///   // body added above.
  ///   const RigidBody<AutoDiffXd>&
  ///       body_autodiff = model_autodiff.get_variant(body);
  /// @endcode
  ///
  /// MultibodyTree::get_variant() is templated on the multibody element
  /// type which is deduced from its only input argument. The returned element
  /// is templated on the scalar type T of the %MultibodyTree on which this
  /// method is invoked.
  /// In the example above, the user could have also invoked the method
  /// ToAutoDiffXd().
  ///
  /// @pre Finalize() must have already been called on this %MultibodyTree.
  template <typename ToScalar>
  std::unique_ptr<MultibodyTree<ToScalar>> CloneToScalar() const {
    if (!topology_is_valid()) {
      throw std::logic_error(
          "Attempting to clone a MultibodyTree with an invalid topology. "
          "MultibodyTree::Finalize() must be called before attempting to clone"
          " a MultibodyTree.");
    }
    auto tree_clone = std::make_unique<MultibodyTree<ToScalar>>();

    tree_clone->frames_.resize(get_num_frames());
    // Skipping the world body at body_index = 0.
    for (BodyIndex body_index(1); body_index < get_num_bodies(); ++body_index) {
      const Body<T>& body = get_body(body_index);
      tree_clone->CloneBodyAndAdd(body);
    }

    // Frames are cloned in their index order, that is, in the exact same order
    // they were added to the original tree. Since the Frame API enforces the
    // creation of the parent frame first, this traversal guarantees that parent
    // body frames are created before their child frames.
    for (const auto& frame : owned_frames_) {
      tree_clone->CloneFrameAndAdd(*frame);
    }

    for (const auto& mobilizer : owned_mobilizers_) {
      // This call assumes that tree_clone already contains all the cloned
      // frames.
      tree_clone->CloneMobilizerAndAdd(*mobilizer);
    }

    for (const auto& force_element : owned_force_elements_) {
      tree_clone->CloneForceElementAndAdd(*force_element);
    }

    // Since Joint<T> objects are implemented from basic element objects like
    // Body, Mobilizer, ForceElement and Constraint, they are cloned last so
    // that the clones of their dependencies are guaranteed to be available.
    // DO NOT change this order!!!
    for (const auto& joint : owned_joints_) {
      tree_clone->CloneJointAndAdd(*joint);
    }

    // We can safely make a deep copy here since the original multibody tree is
    // required to be finalized.
    tree_clone->topology_ = this->topology_;
    // All other internals templated on T are created with the following call to
    // FinalizeInternals().
    tree_clone->FinalizeInternals();
    return tree_clone;
  }

 private:
  // Make MultibodyTree templated on every other scalar type a friend of
  // MultibodyTree<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private methods from MultibodyTree<T>.
  template <typename> friend class MultibodyTree;

  template <template<typename Scalar> class JointType>
  const JointType<T>& AddJoint(
      std::unique_ptr<JointType<T>> joint) {
    static_assert(std::is_convertible<JointType<T>*, Joint<T>*>::value,
                  "JointType must be a sub-class of Joint<T>.");
    if (topology_is_valid()) {
      throw std::logic_error("This MultibodyTree is finalized already. "
                             "Therefore adding more joints is not allowed. "
                             "See documentation for Finalize() for details.");
    }
    if (joint == nullptr) {
      throw std::logic_error("Input joint is a nullptr.");
    }
    const JointIndex joint_index(owned_joints_.size());
    joint->set_parent_tree(this, joint_index);
    JointType<T>* raw_joint_ptr = joint.get();
    owned_joints_.push_back(std::move(joint));
    return *raw_joint_ptr;
  }

  // Finalizes the MultibodyTreeTopology of this tree.
  void FinalizeTopology();

  // At Finalize(), this method performs all other finalization that is not
  // topological (i.e. performed by FinalizeTopology()). This includes for
  // instance the creation of BodyNode objects.
  // This method will throw a std::logic_error if FinalizeTopology() was not
  // previously called on this tree.
  void FinalizeInternals();

  // Computes the cache entry associated with the geometric Jacobian H_PB_W for
  // each node.
  // The geometric Jacobian `H_PB_W` relates to the spatial velocity of B in P
  // by `V_PB_W = H_PB_W(q)⋅v_B`, where `v_B` corresponds to the generalized
  // velocities associated to body B. `H_PB_W` has size `6 x nm` with `nm` the
  // number of mobilities associated with body B.
  // `H_PB_W_cache` stores the Jacobian matrices for all nodes in the tree as a
  // vector of the columns of these matrices. Therefore `H_PB_W_cache` has as
  // many entries as number of generalized velocities in the tree.
  void CalcAcrossNodeGeometricJacobianExpressedInWorld(
      const systems::Context<T>& context,
      const PositionKinematicsCache<T>& pc,
      std::vector<Vector6<T>>* H_PB_W_cache) const;

  // Implementation for CalcMassMatrixViaInverseDynamics().
  // It assumes:
  //  - The position kinematics cache object is already updated to be in sync
  //    with `context`.
  //  - H is not nullptr.
  //  - H has storage for a square matrix of size get_num_velocities().
  void DoCalcMassMatrixViaInverseDynamics(
      const systems::Context<T>& context,
      const PositionKinematicsCache<T>& pc,
      EigenPtr<MatrixX<T>> H) const;

  // Implementation of CalcBiasTerm().
  // It assumes:
  //  - The position kinematics cache object is already updated to be in sync
  //    with `context`.
  //  - The velocity kinematics cache object is already updated to be in sync
  //    with `context`.
  //  - Cv is not nullptr.
  //  - Cv has storage for a vector of size get_num_velocities().
  void DoCalcBiasTerm(
      const systems::Context<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc,
      EigenPtr<VectorX<T>> Cv) const;

  // Implementation of CalcPotentialEnergy().
  // It is assumed that the position kinematics cache pc is in sync with
  // context.
  T DoCalcPotentialEnergy(const systems::Context<T>& context,
                          const PositionKinematicsCache<T>& pc) const;

  // Implementation of CalcConservativePower().
  // It is assumed that the position kinematics cache pc and the velocity
  // kinematics cache vc are in sync with context.
  T DoCalcConservativePower(const systems::Context<T>& context,
                            const PositionKinematicsCache<T>& pc,
                            const VelocityKinematicsCache<T>& vc) const;

  void CreateBodyNode(BodyNodeIndex body_node_index);

  // Helper method to create a clone of `frame` and add it to `this` tree.
  template <typename FromScalar>
  Frame<T>* CloneFrameAndAdd(const Frame<FromScalar>& frame) {
    FrameIndex frame_index = frame.get_index();

    auto frame_clone = frame.CloneToScalar(*this);
    frame_clone->set_parent_tree(this, frame_index);

    Frame<T>* raw_frame_clone_ptr = frame_clone.get();
    // The order in which frames are added into frames_ is important to keep the
    // topology invariant. Therefore we index new clones according to the
    // original frame_index.
    frames_[frame_index] = raw_frame_clone_ptr;
    // The order within owned_frames_ does not matter.
    owned_frames_.push_back(std::move(frame_clone));
    return raw_frame_clone_ptr;
  }

  // Helper method to create a clone of `body` and add it to `this` tree.
  // Because this method is only invoked in a controlled manner from within
  // CloneToScalar(), it is guaranteed that the cloned body in this variant's
  // `owned_bodies_` will occupy the same position as its corresponding Body
  // in the source variant `body`.
  template <typename FromScalar>
  Body<T>* CloneBodyAndAdd(const Body<FromScalar>& body) {
    const BodyIndex body_index = body.get_index();
    const FrameIndex body_frame_index = body.get_body_frame().get_index();

    auto body_clone = body.CloneToScalar(*this);
    body_clone->set_parent_tree(this, body_index);
    // MultibodyTree can access selected private methods in Body through its
    // BodyAttorney.
    Frame<T>* body_frame_clone =
        &internal::BodyAttorney<T>::get_mutable_body_frame(body_clone.get());
    body_frame_clone->set_parent_tree(this, body_frame_index);

    // The order in which frames are added into frames_ is important to keep the
    // topology invariant. Therefore we index new clones according to the
    // original body_frame_index.
    frames_[body_frame_index] = body_frame_clone;
    Body<T>* raw_body_clone_ptr = body_clone.get();
    // The order in which bodies are added into owned_bodies_ is important to
    // keep the topology invariant. Therefore this method is called from
    // MultibodyTree::CloneToScalar() within a loop by original body_index.
    DRAKE_DEMAND(static_cast<int>(owned_bodies_.size()) == body_index);
    owned_bodies_.push_back(std::move(body_clone));
    return raw_body_clone_ptr;
  }

  // Helper method to create a clone of `mobilizer` and add it to `this` tree.
  template <typename FromScalar>
  Mobilizer<T>* CloneMobilizerAndAdd(const Mobilizer<FromScalar>& mobilizer) {
    MobilizerIndex mobilizer_index = mobilizer.get_index();
    auto mobilizer_clone = mobilizer.CloneToScalar(*this);
    mobilizer_clone->set_parent_tree(this, mobilizer_index);
    Mobilizer<T>* raw_mobilizer_clone_ptr = mobilizer_clone.get();
    owned_mobilizers_.push_back(std::move(mobilizer_clone));
    return raw_mobilizer_clone_ptr;
  }

  // Helper method to create a clone of `force_element` and add it to `this`
  // tree.
  template <typename FromScalar>
  void CloneForceElementAndAdd(
      const ForceElement<FromScalar>& force_element) {
    ForceElementIndex force_element_index = force_element.get_index();
    auto force_element_clone = force_element.CloneToScalar(*this);
    force_element_clone->set_parent_tree(this, force_element_index);
    owned_force_elements_.push_back(std::move(force_element_clone));
  }

  // Helper method to create a clone of `joint` and add it to `this` tree.
  template <typename FromScalar>
  Joint<T>* CloneJointAndAdd(const Joint<FromScalar>& joint) {
    JointIndex joint_index = joint.get_index();
    auto joint_clone = joint.CloneToScalar(*this);
    joint_clone->set_parent_tree(this, joint_index);
    owned_joints_.push_back(std::move(joint_clone));
    return owned_joints_.back().get();
  }

  // Helper method to retrieve the corresponding Frame<T> variant to a Frame in
  // a MultibodyTree variant templated on Scalar.
  template <template <typename> class FrameType, typename Scalar>
  const FrameType<T>& get_frame_variant(const FrameType<Scalar>& frame) const {
    static_assert(std::is_base_of<Frame<T>, FrameType<T>>::value,
                  "FrameType<T> must be a sub-class of Frame<T>.");
    // TODO(amcastro-tri):
    //   DRAKE_DEMAND the parent tree of the variant is indeed a variant of this
    //   MultibodyTree. That will require the tree to have some sort of id.
    FrameIndex frame_index = frame.get_index();
    DRAKE_DEMAND(frame_index < get_num_frames());
    const FrameType<T>* frame_variant =
        dynamic_cast<const FrameType<T>*>(frames_[frame_index]);
    DRAKE_DEMAND(frame_variant != nullptr);
    return *frame_variant;
  }

  // Helper method to retrieve the corresponding Body<T> variant to a Body in a
  // MultibodyTree variant templated on Scalar.
  template <template <typename> class BodyType, typename Scalar>
  const BodyType<T>& get_body_variant(const BodyType<Scalar>& body) const {
    static_assert(std::is_base_of<Body<T>, BodyType<T>>::value,
                  "BodyType<T> must be a sub-class of Body<T>.");
    // TODO(amcastro-tri):
    //   DRAKE_DEMAND the parent tree of the variant is indeed a variant of this
    //   MultibodyTree. That will require the tree to have some sort of id.
    BodyIndex body_index = body.get_index();
    DRAKE_DEMAND(body_index < get_num_bodies());
    const BodyType<T>* body_variant =
        dynamic_cast<const BodyType<T>*>(
            owned_bodies_[body_index].get());
    DRAKE_DEMAND(body_variant != nullptr);
    return *body_variant;
  }

  // Helper method to retrieve the corresponding Mobilizer<T> variant to a Body
  // in a MultibodyTree variant templated on Scalar.
  template <template <typename> class MobilizerType, typename Scalar>
  const MobilizerType<T>& get_mobilizer_variant(
      const MobilizerType<Scalar>& mobilizer) const {
    static_assert(std::is_base_of<Mobilizer<T>, MobilizerType<T>>::value,
                  "MobilizerType<T> must be a sub-class of Mobilizer<T>.");
    // TODO(amcastro-tri):
    //   DRAKE_DEMAND the parent tree of the variant is indeed a variant of this
    //   MultibodyTree. That will require the tree to have some sort of id.
    MobilizerIndex mobilizer_index = mobilizer.get_index();
    DRAKE_DEMAND(mobilizer_index < get_num_mobilizers());
    const MobilizerType<T>* mobilizer_variant =
        dynamic_cast<const MobilizerType<T>*>(
            owned_mobilizers_[mobilizer_index].get());
    DRAKE_DEMAND(mobilizer_variant != nullptr);
    return *mobilizer_variant;
  }

  // Helper method to retrieve the corresponding Joint<T> variant to a Joint
  // in a MultibodyTree variant templated on Scalar.
  template <template <typename> class JointType, typename Scalar>
  const JointType<T>& get_joint_variant(const JointType<Scalar>& joint) const {
    static_assert(std::is_base_of<Joint<T>, JointType<T>>::value,
                  "JointType<T> must be a sub-class of Joint<T>.");
    // TODO(amcastro-tri):
    //   DRAKE_DEMAND the parent tree of the variant is indeed a variant of this
    //   MultibodyTree. That will require the tree to have some sort of id.
    JointIndex joint_index = joint.get_index();
    DRAKE_DEMAND(joint_index < get_num_joints());
    const JointType<T>* joint_variant =
        dynamic_cast<const JointType<T>*>(
            owned_joints_[joint_index].get());
    DRAKE_DEMAND(joint_variant != nullptr);
    return *joint_variant;
  }

  // TODO(amcastro-tri): In future PR's adding MBT computational methods, write
  // a method that verifies the state of the topology with a signature similar
  // to RoadGeometry::CheckHasRightSizeForModel().

  const RigidBody<T>* world_body_{nullptr};
  std::vector<std::unique_ptr<Body<T>>> owned_bodies_;
  std::vector<std::unique_ptr<Frame<T>>> owned_frames_;
  std::vector<std::unique_ptr<Mobilizer<T>>> owned_mobilizers_;
  std::vector<std::unique_ptr<ForceElement<T>>> owned_force_elements_;
  std::vector<std::unique_ptr<internal::BodyNode<T>>> body_nodes_;

  std::vector<std::unique_ptr<Joint<T>>> owned_joints_;

  // List of all frames in the system ordered by their FrameIndex.
  // This vector contains a pointer to all frames in owned_frames_ as well as a
  // pointer to each BodyFrame, which are owned by their corresponding Body.
  std::vector<const Frame<T>*> frames_;

  // Body node indexes ordered by level (a.k.a depth). Therefore for the
  // i-th level body_node_levels_[i] contains the list of all body node indexes
  // in that level.
  std::vector<std::vector<BodyNodeIndex>> body_node_levels_;

  MultibodyTreeTopology topology_;
};

}  // namespace multibody
}  // namespace drake
