#pragma once

#include <algorithm>
#include <iterator>
#include <memory>
#include <string>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/drake_optional.h"
#include "drake/multibody/multibody_tree/acceleration_kinematics_cache.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/body_node.h"
#include "drake/multibody/multibody_tree/force_element.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/joint_actuator.h"
#include "drake/multibody/multibody_tree/joints/joint.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/model_instance.h"
#include "drake/multibody/multibody_tree/multibody_forces.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/multibody/multibody_tree/position_kinematics_cache.h"
#include "drake/multibody/multibody_tree/quaternion_floating_mobilizer.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/multibody/multibody_tree/velocity_kinematics_cache.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

/// @cond
// Helper macro to throw an exception within methods that should not be called
// post-finalize.
#define DRAKE_MBT_THROW_IF_FINALIZED() ThrowIfFinalized(__func__)

// Helper macro to throw an exception within methods that should not be called
// pre-finalize.
#define DRAKE_MBT_THROW_IF_NOT_FINALIZED() ThrowIfNotFinalized(__func__)
/// @endcond

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
    DRAKE_DEMAND(body_index == num_bodies());
    DRAKE_DEMAND(body_frame_index == num_frames());
    DRAKE_DEMAND(body->model_instance().is_valid());

    // TODO(amcastro-tri): consider not depending on setting this pointer at
    // all. Consider also removing MultibodyTreeElement altogether.
    body->set_parent_tree(this, body_index);
    // MultibodyTree can access selected private methods in Body through its
    // BodyAttorney.
    // - Register body frame.
    Frame<T>* body_frame =
        &internal::BodyAttorney<T>::get_mutable_body_frame(body.get());
    body_frame->set_parent_tree(this, body_frame_index);
    DRAKE_ASSERT(body_frame->name() == body->name());
    frame_name_to_index_.insert({body_frame->name(), body_frame_index});
    frames_.push_back(body_frame);
    // - Register body.
    BodyType<T>* raw_body_ptr = body.get();
    body_name_to_index_.insert({body->name(), body->index()});
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

  /// Creates a rigid body with the provided name, model instance, and spatial
  /// inertia.  This method returns a constant reference to the body just added,
  /// which will remain valid for the lifetime of `this` %MultibodyTree.
  ///
  /// Example of usage:
  /// @code
  ///   MultibodyTree<T> model;
  ///   // ... Code to define spatial_inertia, a SpatialInertia<T> object ...
  ///   ModelInstanceIndex model_instance = model.AddModelInstance("instance");
  ///   const RigidBody<T>& body =
  ///     model.AddRigidBody("BodyName", model_instance, spatial_inertia);
  /// @endcode
  ///
  /// @param[in] name
  ///   A string that identifies the new body to be added to `this` model. A
  ///   std::runtime_error is thrown if a body named `name` already is part of
  ///   @p model_instance. See HasBodyNamed(), Body::name().
  /// @param[in] model_instance
  ///   A model instance index which this body is part of.
  /// @param[in] M_BBo_B
  ///   The SpatialInertia of the new rigid body to be added to `this` model,
  ///   computed about the body frame origin `Bo` and expressed in the body
  ///   frame B.
  /// @returns A constant reference to the new RigidBody just added, which will
  ///          remain valid for the lifetime of `this` %MultibodyTree.
  /// @throws std::logic_error if a body named `name` already exists in this
  ///         model instance.
  /// @throws std::logic_error if the model instance does not exist.
  const RigidBody<T>& AddRigidBody(
      const std::string& name, ModelInstanceIndex model_instance,
      const SpatialInertia<double>& M_BBo_B) {
    if (model_instance >= num_model_instances()) {
      throw std::logic_error("Invalid model instance specified.");
    }

    if (HasBodyNamed(name, model_instance)) {
      throw std::logic_error(
          "Model instance '" + instance_index_to_name_.at(model_instance) +
          "' already contains a body named '" + name + "'. " +
          "Body names must be unique within a given model.");
    }

    const RigidBody<T>& body =
        this->template AddBody<RigidBody>(name, model_instance, M_BBo_B);
    return body;
  }

  /// Creates a rigid body with the provided name, model instance, and spatial
  /// inertia.  The newly created body will be placed in the default model
  /// instance.  This method returns a constant reference to the body just
  /// added, which will remain valid for the lifetime of `this` %MultibodyTree.
  ///
  /// Example of usage:
  /// @code
  ///   MultibodyTree<T> model;
  ///   // ... Code to define spatial_inertia, a SpatialInertia<T> object ...
  ///   const RigidBody<T>& body =
  ///     model.AddRigidBody("BodyName", spatial_inertia);
  /// @endcode
  ///
  /// @param[in] name
  ///   A string that identifies the new body to be added to `this` model. A
  ///   std::runtime_error is thrown if a body named `name` already is part of
  ///   the model in the default model instance. See HasBodyNamed(),
  ///   Body::name().
  /// @param[in] M_BBo_B
  ///   The SpatialInertia of the new rigid body to be added to `this` model,
  ///   computed about the body frame origin `Bo` and expressed in the body
  ///   frame B.
  /// @returns A constant reference to the new RigidBody just added, which will
  ///          remain valid for the lifetime of `this` %MultibodyTree.
  /// @throws std::logic_error if a body named `name` already exists.
  /// @throws std::logic_error if additional model instances have been created
  ///                          beyond the world and default instances.
  const RigidBody<T>& AddRigidBody(
      const std::string& name, const SpatialInertia<double>& M_BBo_B) {
    if (num_model_instances() != 2) {
      throw std::logic_error(
          "This model has more model instances than the default.  Please "
          "call AddRigidBody with an explicit model instance.");
    }

    return AddRigidBody(name, default_model_instance(), M_BBo_B);
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
    FrameIndex frame_index = topology_.add_frame(frame->body().index());
    // This test MUST be performed BEFORE frames_.push_back() and
    // owned_frames_.push_back() below. Do not move it around!
    DRAKE_DEMAND(frame_index == num_frames());
    DRAKE_DEMAND(frame->model_instance().is_valid());

    // TODO(amcastro-tri): consider not depending on setting this pointer at
    // all. Consider also removing MultibodyTreeElement altogether.
    frame->set_parent_tree(this, frame_index);
    FrameType<T>* raw_frame_ptr = frame.get();
    frames_.push_back(raw_frame_ptr);
    frame_name_to_index_.insert(std::make_pair(frame->name(), frame_index));
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
  ///       inboard_frame, outboard_frame,
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
    mobilizer->inboard_frame().HasThisParentTreeOrThrow(this);
    mobilizer->outboard_frame().HasThisParentTreeOrThrow(this);
    const int num_positions = mobilizer->num_positions();
    const int num_velocities = mobilizer->num_velocities();
    MobilizerIndex mobilizer_index = topology_.add_mobilizer(
        mobilizer->inboard_frame().index(),
        mobilizer->outboard_frame().index(),
        num_positions, num_velocities);

    // This DRAKE_ASSERT MUST be performed BEFORE owned_mobilizers_.push_back()
    // below. Do not move it around!
    DRAKE_ASSERT(mobilizer_index == num_mobilizers());

    // TODO(sammy-tri) This effectively means that there's no way to
    // programmatically add mobilizers from outside of MultibodyTree
    // itself with multiple model instances.  I'm not convinced that
    // this is a problem.
    if (!mobilizer->model_instance().is_valid()) {
      mobilizer->set_model_instance(default_model_instance());
    }

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
    DRAKE_DEMAND(force_element_index == num_force_elements());
    DRAKE_DEMAND(force_element->model_instance().is_valid());
    force_element->set_parent_tree(this, force_element_index);

    ForceElementType<T>* raw_force_element_ptr = force_element.get();
    owned_force_elements_.push_back(std::move(force_element));
    return *raw_force_element_ptr;
  }

  /// Adds a new force element model of type `ForceElementType` to `this`
  /// %MultibodyTree.  The arguments to this method `args` are forwarded to
  /// `ForceElementType`'s constructor.
  /// @param[in] args
  ///   Zero or more parameters provided to the constructor of the new force
  ///   element. It must be the case that
  ///   `JointType<T>(args)` is a valid constructor.
  /// @tparam ForceElementType
  ///   The type of the ForceElement to add.
  ///   This method can only be called once for elements of type
  ///   UniformGravityFieldElement. That is, gravity can only be specified once
  ///   and std::runtime_error is thrown if the model already contains a gravity
  ///   field element.
  /// @returns A constant reference to the new ForceElement just added, of type
  ///   `ForceElementType<T>` specialized on the scalar type T of `this`
  ///   %MultibodyTree. It will remain valid for the lifetime of `this`
  ///   %MultibodyTree.
  /// @see The ForceElement class's documentation for further details on how a
  /// force element is defined.
  /// @throws std::exception if gravity was already added to the model.
  template<template<typename Scalar> class ForceElementType, typename... Args>
#ifdef DRAKE_DOXYGEN_CXX
  const ForceElementType<T>&
#else
  typename std::enable_if<!std::is_same<
      ForceElementType<T>,
      UniformGravityFieldElement<T>>::value, const ForceElementType<T>&>::type
#endif
  AddForceElement(Args&&... args) {
    static_assert(std::is_base_of<ForceElement<T>, ForceElementType<T>>::value,
        "ForceElementType<T> must be a sub-class of "
            "ForceElement<T>.");
    return AddForceElement(
        std::make_unique<ForceElementType<T>>(std::forward<Args>(args)...));
  }

  // SFINAE overload for ForceElementType = UniformGravityFieldElement.
  // This allow us to keep track of the gravity field parameters.
  template<template<typename Scalar> class ForceElementType, typename... Args>
  typename std::enable_if<std::is_same<
      ForceElementType<T>,
      UniformGravityFieldElement<T>>::value, const ForceElementType<T>&>::type
  AddForceElement(Args&&... args) {
    if (gravity_field_.has_value()) {
      throw std::runtime_error(
          "This model already contains a gravity field element. "
          "Only one gravity field element is allowed per model.");
    }
    // We save the force element so that we can grant users access to it for
    // gravity field specific queries.
    gravity_field_ = &AddForceElement(
        std::make_unique<ForceElementType<T>>(std::forward<Args>(args)...));
    return *gravity_field_.value();
  }

  /// This method adds a Joint of type `JointType` between the frames specified
  /// by the joint.
  ///
  /// @param[in] joint
  ///   Joint to be added.
  /// @tparam JointType
  ///   The type of the new joint to add, which must be a subclass of Joint<T>.
  /// @returns A const lvalue reference to the added joint.
  ///
  /// @see The Joint class's documentation for further details on how a Joint
  /// is defined, or the semi-emplace `AddJoint<>` overload below.
  template <template<typename Scalar> class JointType>
  const JointType<T>& AddJoint(
      std::unique_ptr<JointType<T>> joint) {
    static_assert(std::is_convertible<JointType<T>*, Joint<T>*>::value,
                  "JointType must be a sub-class of Joint<T>.");

    if (HasJointNamed(joint->name(), joint->model_instance())) {
      throw std::logic_error(
          "Model instance '" +
              instance_index_to_name_.at(joint->model_instance()) +
              "' already contains a joint named '" + joint->name() + "'. " +
              "Joint names must be unique within a given model.");
    }

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
  /// attached to the child body B with pose `X_BM`. This method helps create
  /// a joint between two bodies with fixed poses `X_PF` and `X_BM`.
  /// Refer to the Joint class's documentation for more details.
  ///
  /// The arguments to this method `args` are forwarded to `JointType`'s
  /// constructor. The newly created `JointType` object will be specialized on
  /// the scalar type T of this %MultibodyTree.
  ///
  /// @param[in] name
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
  /// @tparam JointType
  ///   The type of the new joint to add, which must be a subclass of Joint<T>.
  /// @returns A constant reference to the new joint just added, of type
  ///   `JointType<T>` specialized on the scalar type T of `this`
  ///   %MultibodyTree. It will remain valid for the lifetime of `this`
  ///   %MultibodyTree.
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
  ///       model.world_body(),     /* parent body */
  ///       {},                     /* frame F IS the parent body frame P */
  ///       pendulum,               /* child body, the pendulum */
  ///       X_BM,                   /* pose of frame M in the body frame B */
  ///       Vector3d::UnitZ());     /* revolute axis in this case */
  /// @endcode
  ///
  /// @throws if `this` model already contains a joint with the given `name`.
  /// See HasJointNamed(), Joint::name().
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
      frame_on_parent = &parent.body_frame();
    }

    const Frame<T>* frame_on_child;
    if (X_BM) {
      frame_on_child = &this->AddFrame<FixedOffsetFrame>(child, *X_BM);
    } else {
      frame_on_child = &child.body_frame();
    }

    const JointType<T>& joint = AddJoint(
        std::make_unique<JointType<T>>(
            name,
            *frame_on_parent, *frame_on_child,
            std::forward<Args>(args)...));
    joint_name_to_index_.insert(std::make_pair(name, joint.index()));
    return joint;
  }

  /// Creates and adds a JointActuator model for an actuator acting on a given
  /// `joint`.
  /// This method returns a constant reference to the actuator just added, which
  /// will remain valid for the lifetime of `this` %MultibodyTree.
  ///
  /// @param[in] name
  ///   A string that identifies the new actuator to be added to `this`
  ///   model. An exception is thrown if an actuator with the same name
  ///   already exists in the same model instance as @p joint. See
  ///   HasJointActuatorNamed().
  /// @param[in] joint
  ///   The Joint to be actuated by the new JointActuator.
  /// @returns A constant reference to the new JointActuator just added, which
  /// will remain valid for the lifetime of `this` %MultibodyTree.
  /// @throws if `this` model already contains a joint actuator with the given
  /// `name`. See HasJointActuatorNamed(), JointActuator::get_name().
  // TODO(amcastro-tri): consider adding sugar method to declare an actuated
  // joint with a single call. Maybe MBT::AddActuatedJoint() or the like.
  const JointActuator<T>& AddJointActuator(
      const std::string& name, const Joint<T>& joint) {
    if (HasJointActuatorNamed(name, joint.model_instance())) {
      throw std::logic_error(
          "Model instance '" +
          instance_index_to_name_.at(joint.model_instance()) +
          "' already contains a joint actuator named '" + name + "'. " +
          "Joint actuator names must be unique within a given model.");
    }

    if (topology_is_valid()) {
      throw std::logic_error("This MultibodyTree is finalized already. "
                             "Therefore adding more actuators is not allowed. "
                             "See documentation for Finalize() for details.");
    }

    const JointActuatorIndex actuator_index =
        topology_.add_joint_actuator(joint.num_velocities());
    owned_actuators_.push_back(std::make_unique<JointActuator<T>>(name, joint));
    JointActuator<T>* actuator = owned_actuators_.back().get();
    actuator->set_parent_tree(this, actuator_index);
    actuator_name_to_index_.insert(std::make_pair(name, actuator_index));
    return *actuator;
  }

  /// Creates a new model instance.  Returns the index for a new model
  /// instance (as there is no concrete object beyond the index).
  ///
  /// @param[in] name
  ///   A string that uniquely identifies the new instance to be added to `this`
  ///   model. An exception is thrown if an instance with the same name
  ///   already exists in the model. See HasModelInstanceNamed().
  /// @throws std::logic_error if Finalize() was already called on `this` tree.
  ModelInstanceIndex AddModelInstance(const std::string& name) {
    if (HasModelInstanceNamed(name)) {
      throw std::logic_error(
          "This model already contains a model instance named '" + name +
          "'. Model instance names must be unique within a given model.");
    }

    if (topology_is_valid()) {
      throw std::logic_error("This MultibodyTree is finalized already. "
                             "Therefore adding more model instances is not "
                             "allowed. See documentation for Finalize() for "
                             "details.");
    }
    const ModelInstanceIndex index(num_model_instances());
    instance_name_to_index_[name] = index;
    instance_index_to_name_[index] = name;
    return index;
  }

  /// @}
  // Closes Doxygen section "Methods to add new MultibodyTree elements."

  /// Returns the number of Frame objects in the MultibodyTree.
  /// Frames include body frames associated with each of the bodies in
  /// the %MultibodyTree including the _world_ body. Therefore the minimum
  /// number of frames in a %MultibodyTree is one.
  int num_frames() const {
    return static_cast<int>(frames_.size());
  }

  /// Returns the number of bodies in the %MultibodyTree including the *world*
  /// body. Therefore the minimum number of bodies in a MultibodyTree is one.
  int num_bodies() const { return static_cast<int>(owned_bodies_.size()); }

  /// Returns the number of joints added with AddJoint() to the %MultibodyTree.
  int num_joints() const { return static_cast<int>(owned_joints_.size()); }

  /// Returns the number of actuators in the model.
  /// @see AddJointActuator().
  int num_actuators() const {
    return static_cast<int>(owned_actuators_.size());
  }

  /// Returns the number of mobilizers in the %MultibodyTree. Since the world
  /// has no Mobilizer, the number of mobilizers equals the number of bodies
  /// minus one, i.e. num_mobilizers() returns num_bodies() - 1.
  // TODO(amcastro-tri): Consider adding a WorldMobilizer (0-dofs) for the world
  // body. This could be useful to query for reaction forces of the entire
  // model.
  int num_mobilizers() const {
    return static_cast<int>(owned_mobilizers_.size());
  }

  /// Returns the number of ForceElement objects in the MultibodyTree.
  int num_force_elements() const {
    return static_cast<int>(owned_force_elements_.size());
  }

  /// Returns the number of model instances in the MultibodyTree.
  int num_model_instances() const {
    return static_cast<int>(instance_name_to_index_.size());
  }

  /// Returns the number of generalized positions of the model.
  int num_positions() const {
    return topology_.num_positions();
  }

  /// Returns the number of generalized positions in a specific model instance.
  int num_positions(ModelInstanceIndex model_instance) const {
    DRAKE_MBT_THROW_IF_NOT_FINALIZED();
    return model_instances_.at(model_instance)->num_positions();
  }

  /// Returns the number of generalized velocities of the model.
  int num_velocities() const {
    return topology_.num_velocities();
  }

  /// Returns the number of generalized velocities in a specific model instance.
  int num_velocities(ModelInstanceIndex model_instance) const {
    DRAKE_MBT_THROW_IF_NOT_FINALIZED();
    return model_instances_.at(model_instance)->num_velocities();
  }

  /// Returns the total size of the state vector in the model.
  int num_states() const {
    return topology_.num_states();
  }

  /// Returns the total size of the state vector in a specific model instance.
  int num_states(ModelInstanceIndex model_instance) const {
    DRAKE_MBT_THROW_IF_NOT_FINALIZED();
    return model_instances_.at(model_instance)->num_positions() +
        model_instances_.at(model_instance)->num_velocities();
  }

  /// Returns the total number of Joint degrees of freedom actuated by the set
  /// of JointActuator elements added to `this` model.
  int num_actuated_dofs() const {
    return topology_.num_actuated_dofs();
  }

  /// Returns the total number of Joint degrees of freedom actuated by the set
  /// of JointActuator elements added to a specific model instance.
  int num_actuated_dofs(ModelInstanceIndex model_instance) const {
    DRAKE_MBT_THROW_IF_NOT_FINALIZED();
    return model_instances_.at(model_instance)->num_actuated_dofs();
  }

  /// Returns the height of the tree data structure of `this` %MultibodyTree.
  /// That is, the number of bodies in the longest kinematic path between the
  /// world and any other leaf body. For a model that only contains the _world_
  /// body, the height of the tree is one.
  /// Kinematic paths are created by Mobilizer objects connecting a chain of
  /// frames. Therefore, this method does not count kinematic cycles, which
  /// could only be considered in the model using constraints.
  int tree_height() const {
    return topology_.tree_height();
  }

  /// Returns a constant reference to the *world* body.
  const RigidBody<T>& world_body() const {
    // world_body_ is set in the constructor. So this assert is here only to
    // verify future constructors do not mess that up.
    DRAKE_ASSERT(world_body_ != nullptr);
    return *world_body_;
  }

  /// Returns a constant reference to the *world* frame.
  const BodyFrame<T>& world_frame() const {
    return owned_bodies_[world_index()]->body_frame();
  }

  /// Returns a constant reference to the body with unique index `body_index`.
  /// @throws if `body_index` does not correspond to a body in this multibody
  /// tree.
  const Body<T>& get_body(BodyIndex body_index) const {
    DRAKE_THROW_UNLESS(body_index < num_bodies());
    return *owned_bodies_[body_index];
  }

  /// Returns a constant reference to the joint with unique index `joint_index`.
  /// @throws std::runtime_error when `joint_index` does not correspond to a
  /// joint in this multibody tree.
  const Joint<T>& get_joint(JointIndex joint_index) const {
    DRAKE_THROW_UNLESS(joint_index < num_joints());
    return *owned_joints_[joint_index];
  }

  /// Returns a constant reference to the joint actuator with unique index
  /// `actuator_index`.
  /// @throws if `actuator_index` does not correspond to a joint actuator in
  /// this multibody tree.
  const JointActuator<T>& get_joint_actuator(
      JointActuatorIndex actuator_index) const {
    DRAKE_THROW_UNLESS(actuator_index < num_actuators());
    return *owned_actuators_[actuator_index];
  }

  /// Returns a constant reference to the frame with unique index `frame_index`.
  /// @throws if `frame_index` does not correspond to a frame in `this`
  /// multibody tree.
  const Frame<T>& get_frame(FrameIndex frame_index) const {
    DRAKE_THROW_UNLESS(frame_index < num_frames());
    return *frames_[frame_index];
  }

  /// Returns a constant reference to the mobilizer with unique index
  /// `mobilizer_index`.
  /// @throws std::runtime_error when `mobilizer_index` does not correspond to a
  /// mobilizer in this multibody tree.
  const Mobilizer<T>& get_mobilizer(MobilizerIndex mobilizer_index) const {
    DRAKE_THROW_UNLESS(mobilizer_index < num_mobilizers());
    return *owned_mobilizers_[mobilizer_index];
  }

  /// Returns the name of a model_instance.
  /// @throws std::logic_error when `model_instance` does not correspond to a
  /// model in this multibody tree.
  const std::string& GetModelInstanceName(
      ModelInstanceIndex model_instance) const {
    const auto it = instance_index_to_name_.find(model_instance);
    if (it == instance_index_to_name_.end()) {
      throw std::logic_error("There is no model instance id " +
                             std::to_string(model_instance) +
                             " in the model.");
    }
    return it->second;
  }

  /// @name Querying for multibody elements by name
  /// These methods allow a user to query whether a given multibody element is
  /// part of `this` model. These queries can be performed at any time during
  /// the lifetime of a %MultibodyTree model, i.e. there is no restriction on
  /// whether they must be called before or after Finalize(). That is, these
  /// queries can be performed while new multibody elements are being added to
  /// the model.
  /// @{

  /// @returns `true` if a body named `name` was added to the model.
  /// @see AddRigidBody().
  ///
  /// @throws std::logic_error if the body name occurs in multiple model
  /// instances.
  bool HasBodyNamed(const std::string& name) const {
    const int count = body_name_to_index_.count(name);
    if (count > 1) {
      throw std::logic_error(
          "Body " + name + " appears in multiple model instances.");
    }
    return count > 0;
  }

  /// @returns `true` if a body named `name` was added to @p model_instance.
  /// @see AddRigidBody().
  ///
  /// @throws if @p model_instance is not valid for this model.
  bool HasBodyNamed(const std::string& name,
                    ModelInstanceIndex model_instance) const {
    DRAKE_THROW_UNLESS(model_instance < instance_name_to_index_.size());
    // Search linearly on the assumption that we won't often have lots of
    // bodies with the same name in different model instances.  If this turns
    // out to be incorrect we can switch to a different data structure.
    // N.B. Please sync with `HasFrameNamed`, `HasJointNamed`, and
    // `HasJointActuatorNamed` if you change or remove this comment.
    const auto range = body_name_to_index_.equal_range(name);
    for (auto it = range.first; it != range.second; ++it) {
      if (get_body(it->second).model_instance() == model_instance) {
        return true;
      }
    }
    return false;
  }

  /// @returns `true` if a frame named `name` was added to the model.
  /// @see AddFrame().
  ///
  /// @throws std::logic_error if the frame name occurs in multiple model
  /// instances.
  bool HasFrameNamed(const std::string& name) const {
    const int count = frame_name_to_index_.count(name);
    if (count > 1) {
      throw std::logic_error(
          "Frame " + name + " appears in multiple model instances.");
    }
    return count > 0;
  }

  /// @returns `true` if a frame named `name` was added to @p model_instance.
  /// @see AddFrame().
  ///
  /// @throws if @p model_instance is not valid for this model.
  bool HasFrameNamed(const std::string& name,
                     ModelInstanceIndex model_instance) const {
    DRAKE_THROW_UNLESS(model_instance < instance_name_to_index_.size());
    // See notes in `HasBodyNamed`.
    const auto range = frame_name_to_index_.equal_range(name);
    for (auto it = range.first; it != range.second; ++it) {
      if (get_frame(it->second).body().model_instance() == model_instance) {
        return true;
      }
    }
    return false;
  }

  /// @returns `true` if a joint named `name` was added to the model.
  /// @see AddJoint().
  ///
  /// @throws std::logic_error if the joint name occurs in multiple model
  /// instances.
  bool HasJointNamed(const std::string& name) const {
    const int count = joint_name_to_index_.count(name);
    if (count > 1) {
      throw std::logic_error(
          "Joint " + name + " appears in multiple model instances.");
    }
    return count > 0;
  }

  /// @returns `true` if a joint named `name` was added to @p model_instance.
  /// @see AddJoint().
  ///
  /// @throws if @p model_instance is not valid for this model.
  bool HasJointNamed(const std::string& name,
                     ModelInstanceIndex model_instance) const {
    DRAKE_THROW_UNLESS(model_instance < instance_name_to_index_.size());
    // See notes in `HasBodyNamed`.
    const auto range = joint_name_to_index_.equal_range(name);
    for (auto it = range.first; it != range.second; ++it) {
      if (get_joint(it->second).model_instance() == model_instance) {
        return true;
      }
    }
    return false;
  }

  /// @returns `true` if a joint actuator named `name` was added to the model.
  /// @see AddJointActuator().
  ///
  /// @throws std::logic_error if the actuator name occurs in multiple model
  /// instances.
  bool HasJointActuatorNamed(const std::string& name) const {
    const int count = actuator_name_to_index_.count(name);
    if (count > 1) {
      throw std::logic_error(
          "Joint actuator " + name + " appears in multiple model instances.");
    }
    return count > 0;
  }

  /// @returns `true` if a joint actuator named `name` was added to
  /// @p model_instance.
  /// @see AddJointActuator().
  ///
  /// @throws if @p model_instance is not valid for this model.
  bool HasJointActuatorNamed(const std::string& name,
                             ModelInstanceIndex model_instance) const {
    DRAKE_THROW_UNLESS(model_instance < instance_name_to_index_.size());
    const auto range = actuator_name_to_index_.equal_range(name);
    // See notes in `HasBodyNamed`.
    for (auto it = range.first; it != range.second; ++it) {
      if (get_joint_actuator(it->second).model_instance() == model_instance) {
        return true;
      }
    }
    return false;
  }

  /// @returns `true` if a model instance named `name` was added to the model.
  /// @see AddModelInstance().
  bool HasModelInstanceNamed(const std::string& name) const {
    return instance_name_to_index_.find(name) != instance_name_to_index_.end();
  }
  /// @}

  /// @name Retrieving multibody elements by name
  /// These methods allow a user to retrieve a reference to a multibody element
  /// by its name. A std::logic_error is thrown if there is no element with the
  /// requested name.
  ///
  /// These queries can be performed at any time during the lifetime of a
  /// %MultibodyTree model, i.e. there is no restriction on whether they must
  /// be called before or after Finalize(). This implies that these queries can
  /// be performed while new multibody elements are being added to the model.
  ///
  /// If the named element is present in more than one model instance and a
  /// model instance is not explicitly specified, std::logic_error is thrown.
  ///
  /// @{

  /// Returns a constant reference to a body that is identified by the
  /// string `name` in `this` model.
  /// @throws std::logic_error if there is no body with the requested name.
  /// @throws std::logic_error if the body name occurs in multiple model
  /// instances.
  /// @see HasBodyNamed() to query if there exists a body in `this` model with a
  /// given specified name.
  const Body<T>& GetBodyByName(const std::string& name) const {
    return get_body(
        GetElementIndex<BodyIndex>(name, "Body", body_name_to_index_));
  }

  /// Returns a constant reference to the body that is uniquely identified
  /// by the string `name` in @p model_instance.
  /// @throws std::logic_error if there is no body with the requested name.
  /// @throws std::runtime_error if @p model_instance is not valid for this
  ///         model.
  /// @see HasBodyNamed() to query if there exists a body in `this` model with a
  /// given specified name.
  const Body<T>& GetBodyByName(
      const std::string& name, ModelInstanceIndex model_instance) const {
    DRAKE_THROW_UNLESS(model_instance < instance_name_to_index_.size());
    const auto range = body_name_to_index_.equal_range(name);
    for (auto it = range.first; it != range.second; ++it) {
      const Body<T>& body = get_body(it->second);
      if (body.model_instance() == model_instance) {
        return body;
      }
    }
    throw std::logic_error(
        "There is no body named '" + name + "' in model instance '" +
        instance_index_to_name_.at(model_instance) + "'.");
  }

  /// Returns a constant reference to a frame that is identified by the
  /// string `name` in `this` model.
  /// @throws std::logic_error if there is no frame with the requested name.
  /// @throws std::logic_error if the frame name occurs in multiple model
  /// instances.
  /// @see HasFrameNamed() to query if there exists a body in `this` model with
  /// a given specified name.
  const Frame<T>& GetFrameByName(const std::string& name) const {
    return get_frame(
        GetElementIndex<FrameIndex>(name, "Frame", frame_name_to_index_));
  }

  /// Returns a constant reference to the frame that is uniquely identified
  /// by the string `name` in @p model_instance.
  /// @throws std::logic_error if there is no frame with the requested name.
  /// @throws std::runtime_error if @p model_instance is not valid for this
  ///         model.
  /// @see HasFrameNamed() to query if there exists a frame in `this` model with
  /// a given specified name.
  const Frame<T>& GetFrameByName(
      const std::string& name, ModelInstanceIndex model_instance) const {
    DRAKE_THROW_UNLESS(model_instance < instance_name_to_index_.size());
    const auto range = frame_name_to_index_.equal_range(name);
    for (auto it = range.first; it != range.second; ++it) {
      const Frame<T>& frame = get_frame(it->second);
      if (frame.body().model_instance() == model_instance) {
        return frame;
      }
    }
    throw std::logic_error(
        "There is no frame named '" + name + "' in model instance '" +
        instance_index_to_name_.at(model_instance) + "'.");
  }

  /// Returns a constant reference to a rigid body that is identified
  /// by the string `name` in `this` model.
  /// @throws std::logic_error if there is no body with the requested name.
  /// @throws std::logic_error if the body name occurs in multiple model
  /// instances.
  /// @throws std::logic_error if the requested body is not a RigidBody.
  /// @see HasBodyNamed() to query if there exists a body in `this` model with a
  /// given specified name.
  const RigidBody<T>& GetRigidBodyByName(const std::string& name) const {
    const RigidBody<T>* body =
        dynamic_cast<const RigidBody<T>*>(&GetBodyByName(name));
    if (body == nullptr) {
      throw std::logic_error("Body '" + name + "' is not a RigidBody.");
    }
    return *body;
  }

  /// Returns a constant reference to the rigid body that is uniquely identified
  /// by the string `name` in @p model_instance.
  /// @throws std::logic_error if there is no body with the requested name.
  /// @throws std::logic_error if the requested body is not a RigidBody.
  /// @throws std::runtime_error if @p model_instance is not valid for this
  ///         model.
  /// @see HasBodyNamed() to query if there exists a body in `this` model with a
  /// given specified name.
  const RigidBody<T>& GetRigidBodyByName(
      const std::string& name, ModelInstanceIndex model_instance) const {
    DRAKE_THROW_UNLESS(model_instance < instance_name_to_index_.size());
    const RigidBody<T>* body =
        dynamic_cast<const RigidBody<T>*>(&GetBodyByName(name, model_instance));
    if (body == nullptr) {
      throw std::logic_error(
          "Body '" + name + "' in model instance '" +
          instance_index_to_name_.at(model_instance)  +"' is not a RigidBody.");
    }
    return *body;
  }

  /// Returns a constant reference to a joint that is identified
  /// by the string `name` in `this` model.
  /// @throws std::logic_error if there is no joint with the requested name.
  /// @throws std::logic_error if the joint name occurs in multiple model
  /// instances.
  /// @see HasJointNamed() to query if there exists a joint in `this` model with
  /// a given specified name.
  const Joint<T>& GetJointByName(const std::string& name) const {
    return get_joint(
        GetElementIndex<JointIndex>(name, "Joint", joint_name_to_index_));
  }

  /// Returns a constant reference to the joint that is uniquely identified
  /// by the string `name` in @p model_instance.
  /// @throws std::logic_error if there is no joint with the requested name.
  /// @throws std::runtime_error if @p model_instance is not valid for this
  ///         model.
  /// @see HasJointNamed() to query if there exists a joint in `this` model with
  /// a given specified name.
  const Joint<T>& GetJointByName(
      const std::string& name, ModelInstanceIndex model_instance) const {
    DRAKE_THROW_UNLESS(model_instance < instance_name_to_index_.size());
    const auto range = joint_name_to_index_.equal_range(name);
    for (auto it = range.first; it != range.second; ++it) {
      const Joint<T>& joint = get_joint(it->second);
      if (joint.model_instance() == model_instance) {
        return joint;
      }
    }
    throw std::logic_error(
        "There is no joint named '" + name + "' in model instance '" +
        instance_index_to_name_.at(model_instance) + "'.");
  }

  /// A templated version of GetJointByName() to return a constant reference of
  /// the specified type `JointType` in place of the base Joint class. See
  /// GetJointByName() for details.
  /// @tparam JointType The specific type of the Joint to be retrieved. It must
  /// be a subclass of Joint.
  /// @throws std::logic_error if the named joint is not of type `JointType` or
  /// if there is no Joint with that name.
  /// @throws std::logic_error if the joint name occurs in multiple model
  /// instances.
  /// @see HasJointNamed() to query if there exists a joint in `this` model with
  /// a given specified name.
  template <template<typename> class JointType>
  const JointType<T>& GetJointByName(const std::string& name) const {
    static_assert(std::is_base_of<Joint<T>, JointType<T>>::value,
                  "JointType<T> must be a sub-class of Joint<T>.");
    const JointType<T>* joint =
        dynamic_cast<const JointType<T>*>(&GetJointByName(name));
    if (joint == nullptr) {
      throw std::logic_error("Joint '" + name + "' is not of type '" +
          NiceTypeName::Get<JointType<T>>() + "' but of type '" +
          NiceTypeName::Get(GetJointByName(name)) + "'.");
    }
    return *joint;
  }

  /// A templated version of GetJointByName() to return a constant reference of
  /// the specified type `JointType` in place of the base Joint class. See
  /// GetJointByName() for details.
  /// @tparam JointType The specific type of the Joint to be retrieved. It must
  /// be a subclass of Joint.
  /// @throws std::logic_error if the named joint is not of type `JointType` or
  /// @throws std::runtime_error if @p model_instance is not valid for this
  ///         model.
  /// if there is no Joint with that name.
  /// @see HasJointNamed() to query if there exists a joint in `this` model with
  /// a given specified name.
  template <template<typename> class JointType>
  const JointType<T>& GetJointByName(
      const std::string& name, ModelInstanceIndex model_instance) const {
    static_assert(std::is_base_of<Joint<T>, JointType<T>>::value,
                  "JointType<T> must be a sub-class of Joint<T>.");
    const JointType<T>* joint =
        dynamic_cast<const JointType<T>*>(
            &GetJointByName(name, model_instance));
    if (joint == nullptr) {
      throw std::logic_error(
          "Joint '" + name + "' in model instance " +
          model_instances_.at(model_instance)->name() + "is not of type '" +
          NiceTypeName::Get<JointType<T>>() + "' but of type '" +
          NiceTypeName::Get(GetJointByName(name)) + "'.");
    }
    return *joint;
  }

  /// Returns a constant reference to an actuator that is identified
  /// by the string `name` in `this` model.
  /// @throws std::logic_error if there is no actuator with the requested name.
  /// @throws std::logic_error if the actuator name occurs in multiple model
  /// instances.
  /// @see HasJointActuatorNamed() to query if there exists an actuator in
  /// `this` model with a given specified name.
  const JointActuator<T>& GetJointActuatorByName(
      const std::string& name) const {
    return get_joint_actuator(
        GetElementIndex<JointActuatorIndex>(
            name, "Joint actuator", actuator_name_to_index_));
  }

  /// Returns a constant reference to the actuator that is uniquely identified
  /// by the string `name` in @p model_instance.
  /// @throws std::logic_error if there is no actuator with the requested name.
  /// @throws std::runtime_error if @p model_instance is not valid for this
  ///         model.
  /// @see HasJointActuatorNamed() to query if there exists an actuator in
  /// `this` model with a given specified name.
  const JointActuator<T>& GetJointActuatorByName(
      const std::string& name, ModelInstanceIndex model_instance) const {
    DRAKE_THROW_UNLESS(model_instance < instance_name_to_index_.size());
    const auto range = actuator_name_to_index_.equal_range(name);
    for (auto it = range.first; it != range.second; ++it) {
      const JointActuator<T>& actuator = get_joint_actuator(it->second);
      if (actuator.model_instance() == model_instance) {
        return actuator;
      }
    }
    throw std::logic_error(
        "There is no joint actuator named '" + name + "' in model instance '" +
        instance_index_to_name_.at(model_instance) + "'.");
  }

  /// Returns the index to the model instance that is uniquely identified
  /// by the string `name` in `this` model.
  /// @throws std::logic_error if there is no instance with the requested name.
  /// @see HasModelInstanceNamed() to query if there exists an instance in
  /// `this` model with a given specified name.
  ModelInstanceIndex GetModelInstanceByName(const std::string& name) const {
    const auto it = instance_name_to_index_.find(name);
    if (it == instance_name_to_index_.end()) {
      throw std::logic_error("There is no model instance named '" + name +
          "' in the model.");
    }
    return it->second;
  }
  /// @}

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

  /// @name Model instance accessors
  /// Many functions on %MultibodyTree expect vectors of tree state or
  /// joint actuator inputs which encompass the entire tree.  Methods
  /// in this section are convenience accessors for the portion of
  /// those vectors which apply to a single model instance only.
  /// @{

  /// Given the actuation values @p u_instance for all actuators in @p
  /// model_instance, this method sets the actuation vector u for the entire
  /// MultibodyTree model to which this actuator belongs to.
  /// @param[in] u_instance Actuation values for the actuators. It must be of
  ///   size equal to the number of degrees of freedom of all of the actuated
  ///   joints in @p model_instance.
  /// @param[out] u
  ///   The vector containing the actuation values for the entire MultibodyTree.
  void set_actuation_vector(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& u_instance,
      EigenPtr<VectorX<T>> u) const;

  /// Returns a vector of generalized positions for @p model_instance from a
  /// vector `q_array` of generalized positions for the entire MultibodyTree
  /// model.  This method aborts if `q_array` is not of size
  /// MultibodyTree::num_positions().
  VectorX<T> get_positions_from_array(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& q_array) const;

  /// Returns a vector of generalized velocities for @p model_instance from a
  /// vector `v_array` of generalized velocities for the entire MultibodyTree
  /// model.  This method aborts if the input array is not of size
  /// MultibodyTree::num_velocities().
  VectorX<T> get_velocities_from_array(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& v_array) const;

  /// @}
  // End of "Model instance accessors" section.

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
  /// @throws std::exception if called post-finalize.
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

  /// Returns a const Eigen vector containing the multibody state `x = [q; v]`
  /// of the model with q the vector of generalized positions and v the vector
  /// of generalized velocities.
  Eigen::VectorBlock<const VectorX<T>> get_multibody_state_vector(
      const systems::Context<T>& context) const;

  /// Returns a mutable Eigen vector containing the multibody state `x = [q; v]`
  /// of the model with q the vector of generalized positions and v the vector
  /// of generalized velocities.
  /// @throws std::exception if the `context` is nullptr or if it does not
  /// correspond to the context for a multibody model.
  Eigen::VectorBlock<VectorX<T>> get_mutable_multibody_state_vector(
      systems::Context<T>* context) const;

  /// Sets `context` to store the pose `X_WB` of a given `body` B in the world
  /// frame W.
  /// @note In general setting the pose and/or velocity of a body in the model
  /// would involve a complex inverse kinematics problem. This method allows us
  /// to simplify this process when we know the body is free in space.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::exception if called pre-finalize.
  void SetFreeBodyPoseOrThrow(
      const Body<T>& body, const Isometry3<T>& X_WB,
      systems::Context<T>* context) const;

  /// Sets `context` to store the spatial velocity `V_WB` of a given `body` B in
  /// the world frame W.
  /// @note In general setting the pose and/or velocity of a body in the model
  /// would involve a complex inverse kinematics problem. This method allows us
  /// to simplify this process when we know the body is free in space.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::exception if called pre-finalize.
  void SetFreeBodySpatialVelocityOrThrow(
      const Body<T>& body, const SpatialVelocity<T>& V_WB,
      systems::Context<T>* context) const;

  /// Sets `sate` to store the pose `X_WB` of a given `body` B in the world
  /// frame W, for a given `context` of `this` model.
  /// @note In general setting the pose and/or velocity of a body in the model
  /// would involve a complex inverse kinematics problem. This method allows us
  /// to simplify this process when we know the body is free in space.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::exception if called pre-finalize.
  void SetFreeBodyPoseOrThrow(
      const Body<T>& body, const Isometry3<T>& X_WB,
      const systems::Context<T>& context, systems::State<T>* state) const;

  /// Sets `state` to store the spatial velocity `V_WB` of a given `body` B in
  /// the world frame W, for a given `context` of `this` model.
  /// @note In general setting the pose and/or velocity of a body in the model
  /// would involve a complex inverse kinematics problem. This method allows us
  /// to simplify this process when we know the body is free in space.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::exception if called pre-finalize.
  void SetFreeBodySpatialVelocityOrThrow(
      const Body<T>& body, const SpatialVelocity<T>& V_WB,
      const systems::Context<T>& context, systems::State<T>* state) const;

  /// @name Kinematic computations
  /// Kinematics computations are concerned with the motion of bodies in the
  /// model without regard to their mass or the forces and moments that cause
  /// the motion. Methods in this category include the computation of poses and
  /// spatial velocities.
  /// @{

  /// Computes the world pose `X_WB(q)` of each body B in the model as a
  /// function of the generalized positions q stored in `context`.
  /// @param[in] context
  ///   The context containing the state of the model. It stores the generalized
  ///   positions q of the model.
  /// @param[out] X_WB
  ///   On output this vector will contain the pose of each body in the model
  ///   ordered by BodyIndex. The index of a body in the model can be obtained
  ///   with Body::index(). This method throws an exception if `X_WB` is
  ///   `nullptr`. Vector `X_WB` is resized when needed to have size
  ///   num_bodies().
  ///
  /// @throws if X_WB is nullptr.
  void CalcAllBodyPosesInWorld(
      const systems::Context<T>& context,
      std::vector<Isometry3<T>>* X_WB) const;

  /// Computes the spatial velocity `V_WB(q, v)` of each body B in the model,
  /// measured and expressed in the world frame W. The body spatial velocities
  /// are a function of the generalized positions q and generalized velocities
  /// v, both stored in `context`.
  /// @param[in] context
  ///   The context containing the state of the model. It stores the generalized
  ///   positions q and velocities v of the model.
  /// @param[out] V_WB
  ///   On output this vector will contain the spatial velocity of each body in
  ///   the model ordered by BodyIndex. The index of a body in the model can be
  ///   obtained with Body::index(). This method throws an exception if
  ///   `V_WB` is `nullptr`. Vector `V_WB` is resized when needed to have size
  ///   num_bodies().
  ///
  /// /// @throws if V_WB is nullptr.
  void CalcAllBodySpatialVelocitiesInWorld(
      const systems::Context<T>& context,
      std::vector<SpatialVelocity<T>>* V_WB) const;

  /// Computes the relative transform `X_AB(q)` from a frame B to a frame A, as
  /// a function of the generalized positions q of the model.
  /// That is, the position `p_AQ` of a point Q measured and expressed in
  /// frame A can be computed from the position `p_BQ` of this point measured
  /// and expressed in frame B using the transformation `p_AQ = X_AB⋅p_BQ`.
  ///
  /// @param[in] context
  ///   The context containing the state of the %MultibodyTree model. It stores
  ///   the generalized positions q of the model.
  /// @param[in] frame_A
  ///   The target frame A in the computed relative transform `X_AB`.
  /// @param[in] frame_B
  ///   The source frame B in the computed relative transform `X_AB`.
  /// @retval X_AB
  ///   The relative transform from frame B to frame A, such that
  ///   `p_AQ = X_AB⋅p_BQ`.
  Isometry3<T> CalcRelativeTransform(
      const systems::Context<T>& context,
      const Frame<T>& frame_A, const Frame<T>& frame_B) const;

  /// Given the positions `p_BQi` for a set of points `Qi` measured and
  /// expressed in a frame B, this method computes the positions `p_AQi(q)` of
  /// each point `Qi` in the set as measured and expressed in another frame A,
  /// as a function of the generalized positions q of the model.
  ///
  /// @param[in] context
  ///   The context containing the state of the %MultibodyTree model. It stores
  ///   the generalized positions q of the model.
  /// @param[in] frame_B
  ///   The frame B in which the positions `p_BQi` of a set of points `Qi` are
  ///   given.
  /// @param[in] p_BQi
  ///   The input positions of each point `Qi` in frame B. `p_BQi ∈ ℝ³ˣⁿᵖ` with
  ///   `np` the number of points in the set. Each column of `p_BQi` corresponds
  ///   to a vector in ℝ³ holding the position of one of the points in the set
  ///   as measured and expressed in frame B.
  /// @param[in] frame_A
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
      const Frame<T>& frame_B,
      const Eigen::Ref<const MatrixX<T>>& p_BQi,
      const Frame<T>& frame_A,
      EigenPtr<MatrixX<T>> p_AQi) const;

  /// Evaluate the pose `X_WB` of a body B in the world frame W.
  /// @param[in] context
  ///   The context storing the state of the %MultibodyTree model.
  /// @param[in] body_B
  ///   The body B for which the pose is requested.
  /// @retval X_WB
  ///   The pose of body frame B in the world frame W.
  /// @throws if Finalize() was not called on `this` model or if `body_B` does
  /// not belong to this model.
  const Isometry3<T>& EvalBodyPoseInWorld(
      const systems::Context<T>& context,
      const Body<T>& body_B) const;

  /// Evaluate the spatial velocity `V_WB` of a body B in the world frame W.
  /// @param[in] context
  ///   The context storing the state of the %MultibodyTree model.
  /// @param[in] body_B
  ///   The body B for which the spatial velocity is requested.
  /// @returns V_WB
  ///   The spatial velocity of body frame B in the world frame W.
  /// @throws if Finalize() was not called on `this` model or if `body_B` does
  /// not belong to this model.
  const SpatialVelocity<T>& EvalBodySpatialVelocityInWorld(
      const systems::Context<T>& context,
      const Body<T>& body_B) const;

  /// @}
  // End of "Kinematic computations" section.

  /// @name Methods to compute multibody Jacobians.
  /// @{

  /// Given a set of points `Qi` with fixed position vectors `p_BQi` in a frame
  /// B, (that is, their time derivative `ᴮd/dt(p_BQi)` in frame B is zero),
  /// this method computes the geometric Jacobian `Jv_WQi` defined by:
  /// <pre>
  ///   v_WQi(q, v) = Jv_WQi(q)⋅v
  /// </pre>
  /// where `p_WQi` is the position vector in the world frame for each point
  /// `Qi` in the input set, `v_WQi(q, v)` is the translational velocity of
  /// point `Qi` in the world frame W and q and v are the vectors of generalized
  /// position and velocity, respectively. Since the spatial velocity of each
  /// point `Qi` is linear in the generalized velocities, the geometric
  /// Jacobian `Jv_WQi` is a function of the generalized coordinates q only.
  ///
  /// @param[in] context
  ///   The context containing the state of the model. It stores the
  ///   generalized positions q.
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
  /// @param[out] Jv_WQi
  ///   The geometric Jacobian `Jv_WQi(q)`, function of the generalized
  ///   positions q only. This Jacobian relates the translational velocity
  ///   `v_WQi` of each point `Qi` in the input set by: <pre>
  ///     `v_WQi(q, v) = Jv_WQi(q)⋅v`
  ///   </pre>
  ///   so that `v_WQi` is a column vector of size `3⋅np` concatenating the
  ///   velocity of all points `Qi` in the same order they were given in the
  ///   input set. Therefore `J_WQi` is a matrix of size `3⋅np x nv`, with `nv`
  ///   the number of generalized velocities. On input, matrix `J_WQi` **must**
  ///   have size `3⋅np x nv` or this method throws a std::runtime_error
  ///   exception.
  ///
  /// @throws an exception if the output `p_WQi_set` is nullptr or does not have
  /// the same size as the input array `p_BQi_set`.
  /// @throws an exception if `Jv_WQi` is nullptr or if it does not have the
  /// appropriate size, see documentation for `Jv_WQi` for details.
  // TODO(amcastro-tri): provide the Jacobian-times-vector operation, since for
  // most applications it is all we need and it is more efficient to compute.
  void CalcPointsGeometricJacobianExpressedInWorld(
      const systems::Context<T>& context,
      const Frame<T>& frame_B, const Eigen::Ref<const MatrixX<T>>& p_BQi_set,
      EigenPtr<MatrixX<T>> p_WQi_set, EigenPtr<MatrixX<T>> Jv_WQi) const;

  /// This is a variant to compute the geometric Jacobian `Jv_WQi` for a set of
  /// points `Qi` moving with `frame_B`, given that we know the position `p_WQi`
  /// of each point in the set measured and expressed in the world frame W. The
  /// geometric Jacobian `Jv_WQi` is defined such that: <pre>
  ///   v_WQi(q, v) = Jv_WQi(q)⋅v
  /// </pre>
  /// where `v_WQi(q, v)` is the translational velocity of point `Qi` in the
  /// world frame W and q and v are the vectors of generalized position and
  /// velocity, respectively. Since the spatial velocity of each
  /// point `Qi` is linear in the generalized velocities, the geometric
  /// Jacobian `Jv_WQi` is a function of the generalized coordinates q only.
  ///
  /// @param[in] context
  ///   The context containing the state of the model. It stores the
  ///   generalized positions q.
  /// @param[in] frame_B
  ///   Points `Qi` in the set instantaneously move with this frame.
  /// @param[in] p_WQi_set
  ///   A matrix with the fixed position of a set of points `Qi` measured and
  ///   expressed in the world frame W.
  ///   Each column of this matrix contains the position vector `p_WQi` for a
  ///   point `Qi` measured and expressed in the world frame W. Therefore this
  ///   input matrix lives in ℝ³ˣⁿᵖ with `np` the number of points in the set.
  /// @param[out] Jv_WQi
  ///   The geometric Jacobian `Jv_WQi(q)`, function of the generalized
  ///   positions q only. This Jacobian relates the translational velocity
  ///   `v_WQi` of each point `Qi` in the input set by: <pre>
  ///     `v_WQi(q, v) = Jv_WQi(q)⋅v`
  ///   </pre>
  ///   so that `v_WQi` is a column vector of size `3⋅np` concatenating the
  ///   velocity of all points `Qi` in the same order they were given in the
  ///   input set. Therefore `J_WQi` is a matrix of size `3⋅np x nv`, with `nv`
  ///   the number of generalized velocities. On input, matrix `J_WQi` **must**
  ///   have size `3⋅np x nv` or this method throws a std::runtime_error
  ///   exception.
  ///
  /// @throws an exception if `Jv_WQi` is nullptr or if it does not have the
  /// appropriate size, see documentation for `Jv_WQi` for details.
  // TODO(amcastro-tri): provide the Jacobian-times-vector operation, since for
  // most applications it is all we need and it is more efficient to compute.
  void CalcPointsGeometricJacobianExpressedInWorld(
      const systems::Context<T>& context,
      const Frame<T>& frame_B, const Eigen::Ref<const MatrixX<T>>& p_WQi_set,
      EigenPtr<MatrixX<T>> Jv_WQi) const;

  /// Given a frame F with fixed position `p_BoFo_B` in a frame B, this method
  /// computes the geometric Jacobian `Jv_WF` defined by:
  /// <pre>
  ///   V_WF(q, v) = Jv_WF(q)⋅v
  /// </pre>
  /// where `V_WF(q, v)` is the spatial velocity of frame F measured and
  /// expressed in the world frame W and q and v are the vectors of generalized
  /// position and velocity, respectively. Since the spatial velocity of frame
  /// F is linear in the generalized velocities, the geometric Jacobian `Jv_WF`
  /// is a function of the generalized coordinates q only.
  ///
  /// @param[in] context
  ///   The context containing the state of the model. It stores the
  ///   generalized positions q.
  /// @param[in] frame_B
  ///   The position `p_BoFo_B` of frame F is measured and expressed in this
  ///   frame B.
  /// @param[in] p_BoFo_B
  ///   The (fixed) position of frame F as measured and expressed in frame B.
  /// @param[out] Jv_WF
  ///   The geometric Jacobian `Jv_WF(q)`, function of the generalized positions
  ///   q only. This Jacobian relates to the spatial velocity `V_WF` of frame F
  ///   by: <pre>
  ///     V_WF(q, v) = Jv_WF(q)⋅v
  ///   </pre>
  ///   Therefore `Jv_WF` is a matrix of size `6 x nv`, with `nv`
  ///   the number of generalized velocities. On input, matrix `Jv_WF` **must**
  ///   have size `6 x nv` or this method throws an exception. The top rows of
  ///   this matrix (which can be accessed with Jv_WF.topRows<3>()) is the
  ///   Jacobian `Hw_WF` related to the angular velocity of F in W by
  ///   `w_WF = Hw_WF⋅v`. The bottom rows of this matrix (which can be accessed
  ///   with Jv_WF.bottomRows<3>()) is the Jacobian `Hv_WF` related to the
  ///   translational velocity of the origin of frame F in W by
  ///   `v_WFo = Hw_WF⋅v`. This ordering is consistent with the internal storage
  ///   of the SpatialVector class. Therefore the following operations results
  ///   in a valid spatial velocity: <pre>
  ///     SpatialVelocity<double> Jv_WF_times_v(Jv_WF * v);
  ///   </pre>
  ///
  /// @throws if `J_WF` is nullptr or if it is not of size `6 x nv`.
  void CalcFrameGeometricJacobianExpressedInWorld(
      const systems::Context<T>& context,
      const Frame<T>& frame_B, const Eigen::Ref<const Vector3<T>>& p_BoFo_B,
      EigenPtr<MatrixX<T>> Jv_WF) const;

  /// @}
  // End of multibody Jacobian methods section.

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
  ///   `num_bodies()`. On output, entries will be ordered by BodyNodeIndex.
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
  /// set of generalized forces `tau` that would need to be applied in order to
  /// attain the specified generalized accelerations.
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
  ///   The context containing the state of the model.
  /// @param[in] known_vdot
  ///   A vector with the known generalized accelerations `vdot` for the full
  ///   %MultibodyTree model. Use the provided Joint APIs in order to access
  ///   entries into this array.
  /// @param[in] external_forces
  ///   A set of forces to be applied to the system either as body spatial
  ///   forces `Fapp_Bo_W` or generalized forces `tau_app`, see MultibodyForces
  ///   for details.
  ///
  /// @returns the vector of generalized forces that would need to be applied to
  /// the mechanical system in order to achieve the desired acceleration given
  /// by `known_vdot`.
  VectorX<T> CalcInverseDynamics(
      const systems::Context<T>& context,
      const VectorX<T>& known_vdot,
      const MultibodyForces<T>& external_forces) const;

  /// (Advanced) Given the state of `this` %MultibodyTree in `context` and a
  /// known vector of generalized accelerations `vdot`, this method computes the
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
  ///   be retrieved with Body::node_index().
  ///   This method will abort if provided with an array that does not have a
  ///   size of either `num_bodies()` or zero.
  /// @param[in] tau_applied_array
  ///   An array of applied generalized forces for the entire model. For a
  ///   given mobilizer, entries in this array can be accessed using the method
  ///   Mobilizer::get_generalized_forces_from_array() while its mutable
  ///   counterpart, Mobilizer::get_mutable_generalized_forces_from_array(),
  ///   allows writing into this array.
  ///   `tau_applied_array` can have zero size, which means there are no applied
  ///   forces. To apply non-zero forces, `tau_applied_array` must be of size
  ///   equal to the number to the number of generalized velocities in the
  ///   model, see MultibodyTree::num_velocities().
  ///   This method will abort if provided with an array that does not have a
  ///   size of either MultibodyTree::num_velocities() or zero.
  /// @param[out] A_WB_array
  ///   A pointer to a valid, non nullptr, vector of spatial accelerations
  ///   containing the spatial acceleration `A_WB` for each body. It must be of
  ///   size equal to the number of bodies. This method will abort if the the
  ///   pointer is null or if `A_WB_array` is not of size `num_bodies()`.
  ///   On output, entries will be ordered by BodyNodeIndex.
  ///   To access the acceleration `A_WB` of given body B in this array, use the
  ///   index returned by Body::node_index().
  /// @param[out] F_BMo_W_array
  ///   A pointer to a valid, non nullptr, vector of spatial forces
  ///   containing, for each body B, the spatial force `F_BMo_W` corresponding
  ///   to its inboard mobilizer reaction forces on body B applied at the origin
  ///   `Mo` of the inboard mobilizer, expressed in the world frame W.
  ///   It must be of size equal to the number of bodies in the MultibodyTree.
  ///   This method will abort if the the pointer is null or if `F_BMo_W_array`
  ///   is not of size `num_bodies()`.
  ///   On output, entries will be ordered by BodyNodeIndex.
  ///   To access a mobilizer's reaction force on given body B in this array,
  ///   use the index returned by Body::node_index().
  /// @param[out] tau_array
  ///   On output this array will contain the generalized forces that must be
  ///   applied in order to achieve the desired generalized accelerations given
  ///   by the input argument `known_vdot`. It must not be nullptr and it
  ///   must be of size MultibodyTree::num_velocities(). Generalized forces
  ///   for each Mobilizer can be accessed with
  ///   Mobilizer::get_generalized_forces_from_array().
  ///
  /// @warning There is no mechanism to assert that either `A_WB_array` nor
  ///   `F_BMo_W_array` are ordered by BodyNodeIndex. You can use
  ///   Body::node_index() to obtain the node index for a given body.
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
  ///   number of generalized velocities (num_velocities()) of the model.
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
  ///   generalized velocities (num_velocities()) of the model.
  ///   This method aborts if Cv is nullptr or if it does not have the
  ///   proper size.
  void CalcBiasTerm(
      const systems::Context<T>& context, EigenPtr<VectorX<T>> Cv) const;

  /// Computes the generalized forces `tau_g(q)` due to gravity as a function
  /// of the generalized positions `q` stored in the input `context`.
  /// The vector of generalized forces due to gravity `tau_g(q)` is defined such
  /// that it appears on the right hand side of the equations of motion together
  /// with any other generalized forces, like so:
  /// <pre>
  ///   Mv̇ + C(q, v)v = tau_g(q) + tau_app
  /// </pre>
  /// where `tau_app` includes any other generalized forces applied on the
  /// system.
  ///
  /// @param[in] context
  ///   The context storing the state of the multibody model.
  /// @returns tau_g
  ///   A vector containing the generalized forces due to gravity.
  ///   The generalized forces are consistent with the vector of
  ///   generalized velocities `v` for `this` MultibodyTree model so that
  ///   the inner product `v⋅tau_g` corresponds to the power applied by the
  ///   gravity forces on the mechanical system. That is, `v⋅tau_g > 0`
  ///   corresponds to potential energy going into the system, as either
  ///   mechanical kinetic energy, some other potential energy, or heat, and
  ///   therefore to a decrease of the gravitational potential energy.
  VectorX<T> CalcGravityGeneralizedForces(
      const systems::Context<T>& context) const;

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
  ///   This method aborts if v is not of size num_velocities().
  /// @param[out] qdot
  ///   A valid (non-null) pointer to a vector in `ℝⁿ` with n being the number
  ///   of generalized positions in `this` %MultibodyTree model,
  ///   given by `num_positions()`. This method aborts if `qdot` is nullptr
  ///   or if it is not of size num_positions().
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
  ///   This method aborts if `qdot` is not of size num_positions().
  /// @param[out] v
  ///   A valid (non-null) pointer to a vector in `ℛⁿ` with n the number of
  ///   generalized velocities. This method aborts if v is nullptr or if it
  ///   is not of size num_velocities().
  ///
  /// @see MapVelocityToQDot()
  /// @see Mobilizer::MapQDotToVelocity()
  void MapQDotToVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      EigenPtr<VectorX<T>> v) const;

  /// Computes all the quantities that are required in the final pass of the
  /// articulated body algorithm and stores them in the articulated body cache
  /// `abc`.
  ///
  /// These include:
  /// - Articulated body inertia `Pplus_PB_W`, which can be thought of as the
  ///   articulated body inertia of parent body P as though it were inertialess,
  ///   but taken about Bo and expressed in W.
  ///
  /// @param[in] context
  ///   The context containing the state of the %MultibodyTree model.
  /// @param[in] pc
  ///   A position kinematics cache object already updated to be in sync with
  ///   `context`.
  /// @param[out] abc
  ///   A pointer to a valid, non nullptr, articulated body cache. This method
  ///   throws an exception if `abc` is a nullptr.
  ///
  /// @pre The position kinematics `pc` must have been previously updated with a
  /// call to CalcPositionKinematicsCache() using the same `context`  .
  void CalcArticulatedBodyInertiaCache(
      const systems::Context<T>& context,
      const PositionKinematicsCache<T>& pc,
      ArticulatedBodyInertiaCache<T>* abc) const;

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

    tree_clone->frames_.resize(num_frames());
    // Skipping the world body at body_index = 0.
    for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
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

    for (const auto& actuator : owned_actuators_) {
      tree_clone->CloneActuatorAndAdd(*actuator);
    }

    // We can safely make a deep copy here since the original multibody tree is
    // required to be finalized.
    tree_clone->topology_ = this->topology_;
    tree_clone->body_name_to_index_ = this->body_name_to_index_;
    tree_clone->frame_name_to_index_ = this->frame_name_to_index_;
    tree_clone->joint_name_to_index_ = this->joint_name_to_index_;
    tree_clone->actuator_name_to_index_ = this->actuator_name_to_index_;
    tree_clone->instance_name_to_index_ = this->instance_name_to_index_;
    tree_clone->instance_index_to_name_ = this->instance_index_to_name_;

    // All other internals templated on T are created with the following call to
    // FinalizeInternals().
    tree_clone->FinalizeInternals();
    return tree_clone;
  }

  /// Evaluates position kinematics cached in context.
  /// @param context A MultibodyTreeContext on which to update position
  /// kinematics.
  /// @return Reference to the PositionKinematicsCache of context.
  const PositionKinematicsCache<T>& EvalPositionKinematics(
      const systems::Context<T>& context) const;

  /// Evaluates velocity kinematics cached in context.
  /// @param context A MultibodyTreeContext on which to update velocity
  /// kinematics.
  /// @return Reference to the VelocityKinematicsCache of context.
  const VelocityKinematicsCache<T>& EvalVelocityKinematics(
      const systems::Context<T>& context) const;

 private:
  // Make MultibodyTree templated on every other scalar type a friend of
  // MultibodyTree<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private methods from MultibodyTree<T>.
  template <typename> friend class MultibodyTree;

  // Friend class to facilitate testing.
  friend class MultibodyTreeTester;

  // Finalizes the MultibodyTreeTopology of this tree.
  void FinalizeTopology();

  // At Finalize(), this method performs all other finalization that is not
  // topological (i.e. performed by FinalizeTopology()). This includes for
  // instance the creation of BodyNode objects.
  // This method will throw a std::logic_error if FinalizeTopology() was not
  // previously called on this tree.
  void FinalizeInternals();

  // Helper method to add a QuaternionFreeMobilizer to all bodies that do not
  // have a mobilizer. The mobilizer is between each body and the world. To be
  // called at Finalize().
  // The world body is special in that it is the only body in the model with no
  // mobilizer, even after Finalize().
  void AddQuaternionFreeMobilizerToAllBodiesWithNoMobilizer();

  // Helper method to access the mobilizer of a free body.
  // If `body` is a free body in the model, this method will return the
  // QuaternionFloatingMobilizer for the body. If the body is not free but it
  // is connected to the model by a Joint, this method will throw a
  // std::exception.
  // The returned mobilizer provides a user-facing API to set the state for
  // this body including both pose and spatial velocity.
  // @note In general setting the pose and/or velocity of a body in the model
  // would involve a complex inverse kinematics problem. It is possible however
  // to do this directly for free bodies and the QuaternionFloatingMobilizer
  // user-facing API allows us to do exactly that.
  // @throws std::exception if `body` is not free in the model.
  // @throws std::exception if called pre-finalize.
  // @throws std::exception if called on the world body.
  const QuaternionFloatingMobilizer<T>& GetFreeBodyMobilizerOrThrow(
      const Body<T>& body) const;

  // Helper method for throwing an exception within public methods that should
  // not be called post-finalize. The invoking method should pass its name so
  // that the error message can include that detail.
  void ThrowIfFinalized(const char* source_method) const;

  // Helper method for throwing an exception within public methods that should
  // not be called pre-finalize. The invoking method should pass its name so
  // that the error message can include that detail.
  void ThrowIfNotFinalized(const char* source_method) const;

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
  //  - H has storage for a square matrix of size num_velocities().
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
  //  - Cv has storage for a vector of size num_velocities().
  void DoCalcBiasTerm(
      const systems::Context<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc,
      EigenPtr<VectorX<T>> Cv) const;

  // Helper method to apply forces due to damping at the joints.
  // MultibodyTree treats damping forces separately from other ForceElement
  // forces for a quick simple solution. This allows clients of MBT (namely MBP)
  // to implement their own customized (implicit) time stepping schemes.
  // TODO(amcastro-tri): Consider updating ForceElement to also compute a
  // Jacobian for general force models. That would allow us to implement
  // implicit schemes for any forces using a more general infrastructure rather
  // than having to deal with damping in a special way.
  void AddJointDampingForces(
      const systems::Context<T>& context, MultibodyForces<T>* forces) const;

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

  void CreateModelInstances();

  // Helper method to create a clone of `frame` and add it to `this` tree.
  template <typename FromScalar>
  Frame<T>* CloneFrameAndAdd(const Frame<FromScalar>& frame) {
    FrameIndex frame_index = frame.index();

    auto frame_clone = frame.CloneToScalar(*this);
    frame_clone->set_parent_tree(this, frame_index);
    frame_clone->set_model_instance(frame.model_instance());

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
    const BodyIndex body_index = body.index();
    const FrameIndex body_frame_index = body.body_frame().index();

    auto body_clone = body.CloneToScalar(*this);
    body_clone->set_parent_tree(this, body_index);
    body_clone->set_model_instance(body.model_instance());
    // MultibodyTree can access selected private methods in Body through its
    // BodyAttorney.
    Frame<T>* body_frame_clone =
        &internal::BodyAttorney<T>::get_mutable_body_frame(body_clone.get());
    body_frame_clone->set_parent_tree(this, body_frame_index);
    body_frame_clone->set_model_instance(body.model_instance());

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
    MobilizerIndex mobilizer_index = mobilizer.index();
    auto mobilizer_clone = mobilizer.CloneToScalar(*this);
    mobilizer_clone->set_parent_tree(this, mobilizer_index);
    mobilizer_clone->set_model_instance(mobilizer.model_instance());
    Mobilizer<T>* raw_mobilizer_clone_ptr = mobilizer_clone.get();
    owned_mobilizers_.push_back(std::move(mobilizer_clone));
    return raw_mobilizer_clone_ptr;
  }

  // Helper method to create a clone of `force_element` and add it to `this`
  // tree.
  template <typename FromScalar>
  void CloneForceElementAndAdd(
      const ForceElement<FromScalar>& force_element) {
    ForceElementIndex force_element_index = force_element.index();
    auto force_element_clone = force_element.CloneToScalar(*this);
    force_element_clone->set_parent_tree(this, force_element_index);
    force_element_clone->set_model_instance(force_element.model_instance());
    owned_force_elements_.push_back(std::move(force_element_clone));
  }

  // Helper method to create a clone of `joint` and add it to `this` tree.
  template <typename FromScalar>
  Joint<T>* CloneJointAndAdd(const Joint<FromScalar>& joint) {
    JointIndex joint_index = joint.index();
    auto joint_clone = joint.CloneToScalar(*this);
    joint_clone->set_parent_tree(this, joint_index);
    joint_clone->set_model_instance(joint.model_instance());
    owned_joints_.push_back(std::move(joint_clone));
    return owned_joints_.back().get();
  }

  // Helper method to create a clone of `actuator` (which is templated on
  // FromScalar) and add it to `this` tree (templated on T).
  template <typename FromScalar>
  void CloneActuatorAndAdd(
      const JointActuator<FromScalar>& actuator) {
    JointActuatorIndex actuator_index = actuator.index();
    std::unique_ptr<JointActuator<T>> actuator_clone =
        actuator.CloneToScalar(*this);
    actuator_clone->set_parent_tree(this, actuator_index);
    actuator_clone->set_model_instance(actuator.model_instance());
    owned_actuators_.push_back(std::move(actuator_clone));
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
    FrameIndex frame_index = frame.index();
    DRAKE_DEMAND(frame_index < num_frames());
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
    BodyIndex body_index = body.index();
    DRAKE_DEMAND(body_index < num_bodies());
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
    MobilizerIndex mobilizer_index = mobilizer.index();
    DRAKE_DEMAND(mobilizer_index < num_mobilizers());
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
    JointIndex joint_index = joint.index();
    DRAKE_DEMAND(joint_index < num_joints());
    const JointType<T>* joint_variant =
        dynamic_cast<const JointType<T>*>(
            owned_joints_[joint_index].get());
    DRAKE_DEMAND(joint_variant != nullptr);
    return *joint_variant;
  }

  // Helper function to find the element index for an element in the tree from
  // a multimap of name to index.  It finds the element from any model
  // instance and ensures only one element of that name exists.
  template<typename ElementIndex>
  static ElementIndex GetElementIndex(
      const std::string& name, const std::string& element_description,
      const std::unordered_multimap<std::string, ElementIndex>& name_to_index) {
    const auto range = name_to_index.equal_range(name);
    if (range.first == range.second) {
      std::string lower = element_description;
      std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
      throw std::logic_error("There is no " + lower + " named '" +
                             name + "' in the model.");
    } else if (std::next(range.first) != range.second) {
      throw std::logic_error(
          element_description + " "  + name +
          " appears in multiple model instances.");
    }
    return range.first->second;
  }

  // TODO(amcastro-tri): In future PR's adding MBT computational methods, write
  // a method that verifies the state of the topology with a signature similar
  // to RoadGeometry::CheckHasRightSizeForModel().

  const RigidBody<T>* world_body_{nullptr};
  std::vector<std::unique_ptr<Body<T>>> owned_bodies_;
  std::vector<std::unique_ptr<Frame<T>>> owned_frames_;
  std::vector<std::unique_ptr<Mobilizer<T>>> owned_mobilizers_;
  std::vector<std::unique_ptr<ForceElement<T>>> owned_force_elements_;
  std::vector<std::unique_ptr<JointActuator<T>>> owned_actuators_;
  std::vector<std::unique_ptr<internal::BodyNode<T>>> body_nodes_;
  std::vector<std::unique_ptr<internal::ModelInstance<T>>> model_instances_;

  std::vector<std::unique_ptr<Joint<T>>> owned_joints_;

  // List of all frames in the system ordered by their FrameIndex.
  // This vector contains a pointer to all frames in owned_frames_ as well as a
  // pointer to each BodyFrame, which are owned by their corresponding Body.
  std::vector<const Frame<T>*> frames_;

  // The gravity field force element.
  optional<const UniformGravityFieldElement<T>*> gravity_field_;

  // TODO(amcastro-tri): Consider moving these maps into MultibodyTreeTopology
  // since they are not templated on <T>.

  // The xxx_name_to_index_ structures are multimaps because
  // bodies/joints/actuators/etc may appear with the same name in different
  // model instances.  The index values are still unique across the entire
  // %MultibodyTree.

  // Map used to find body indexes by their body name.
  std::unordered_multimap<std::string, BodyIndex> body_name_to_index_;

  // Map used to find frame indexes by their frame name.
  std::unordered_multimap<std::string, FrameIndex> frame_name_to_index_;

  // Map used to find joint indexes by their joint name.
  std::unordered_multimap<std::string, JointIndex> joint_name_to_index_;

  // Map used to find actuator indexes by their actuator name.
  std::unordered_multimap<std::string,
                          JointActuatorIndex> actuator_name_to_index_;

  // Map used to find a model instance index by its model instance name.
  std::unordered_map<std::string, ModelInstanceIndex> instance_name_to_index_;

  // Map used to find a model instance name by its model instance index.
  std::unordered_map<ModelInstanceIndex, std::string> instance_index_to_name_;

  // Body node indexes ordered by level (a.k.a depth). Therefore for the
  // i-th level body_node_levels_[i] contains the list of all body node indexes
  // in that level.
  std::vector<std::vector<BodyNodeIndex>> body_node_levels_;

  MultibodyTreeTopology topology_;
};

/// @cond
// Undef macros defined at the top of the file. From the GSG:
// "Exporting macros from headers (i.e. defining them in a header without
// #undefing them before the end of the header) is extremely strongly
// discouraged."
// This will require us to re-define them in the .cc file.
#undef DRAKE_MBT_THROW_IF_FINALIZED
#undef DRAKE_MBT_THROW_IF_NOT_FINALIZED
/// @endcond

}  // namespace multibody
}  // namespace drake
