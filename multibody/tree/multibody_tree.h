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

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/random.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/acceleration_kinematics_cache.h"
#include "drake/multibody/tree/articulated_body_inertia_cache.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/multibody_tree_topology.h"
#include "drake/multibody/tree/position_kinematics_cache.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/velocity_kinematics_cache.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

template <typename T> class Body;
template <typename T> class BodyFrame;
template <typename T> class Frame;
template <typename T> class RigidBody;
template <typename T> class Joint;
template <typename T> class JointActuator;
template <typename T> class ForceElement;
template <typename T> class UniformGravityFieldElement;

/// Enumeration that indicates whether the Jacobian is partial differentiation
/// with respect to q̇ (time-derivatives of generalized positions) or
/// with respect to v (generalized velocities).
enum class JacobianWrtVariable {
  kQDot,  ///< J = ∂V/∂q̇
  kV      ///< J = ∂V/∂v
};

/// @cond
// Helper macro to throw an exception within methods that should not be called
// post-finalize.
#define DRAKE_MBT_THROW_IF_FINALIZED() ThrowIfFinalized(__func__)

// Helper macro to throw an exception within methods that should not be called
// pre-finalize.
#define DRAKE_MBT_THROW_IF_NOT_FINALIZED() ThrowIfNotFinalized(__func__)
/// @endcond

namespace internal {

template <typename T> class BodyNode;
template <typename T> class ModelInstance;
template <typename T> class Mobilizer;
template <typename T> class QuaternionFloatingMobilizer;

/// %MultibodyTree provides a representation for a physical system consisting of
/// a collection of interconnected rigid and deformable bodies. As such, it owns
/// and manages each of the elements that belong to this physical system.
/// Multibody dynamics elements include bodies, joints, force elements and
/// constraints.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
///
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
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
  const BodyType<T>& AddBody(std::unique_ptr<BodyType<T>> body);

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
  const BodyType<T>& AddBody(Args&&... args);

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
      const SpatialInertia<double>& M_BBo_B);

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
      const std::string& name, const SpatialInertia<double>& M_BBo_B);

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
  const FrameType<T>& AddFrame(std::unique_ptr<FrameType<T>> frame);

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
  const FrameType<T>& AddFrame(Args&&... args);

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
  /// @throws std::runtime_error if the new mobilizer attempts to connect a
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
      std::unique_ptr<MobilizerType<T>> mobilizer);

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
  /// @throws std::runtime_error if the new mobilizer attempts to connect a
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
  const MobilizerType<T>& AddMobilizer(Args&&... args);

  /// Creates and adds to `this` %MultibodyTree (which retains ownership) a new
  /// `ForceElement` member with the specific type `ForceElementType`. The
  /// arguments to this method `args` are forwarded to `ForceElementType`'s
  /// constructor.
  ///
  /// The newly created `ForceElementType` object will be specialized on the
  /// scalar type T of this %MultibodyTree.
  template <template<typename Scalar> class ForceElementType>
  const ForceElementType<T>& AddForceElement(
      std::unique_ptr<ForceElementType<T>> force_element);

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
  AddForceElement(Args&&... args);

  // SFINAE overload for ForceElementType = UniformGravityFieldElement.
  // This allow us to keep track of the gravity field parameters.
  // TODO(amcastro-tri): This specialization pattern leads to difficult to
  // mantain indirection layers between MBP/MBT and can cause difficult to find
  // bugs, see #11051. It is bad practice and should removed, see #11080.
  template<template<typename Scalar> class ForceElementType, typename... Args>
  typename std::enable_if<std::is_same<
      ForceElementType<T>,
      UniformGravityFieldElement<T>>::value, const ForceElementType<T>&>::type
  AddForceElement(Args&&... args);

  /// See MultibodyPlant documentation.
  template <template<typename Scalar> class JointType>
  const JointType<T>& AddJoint(
      std::unique_ptr<JointType<T>> joint);

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
  ///   `RigidTransform<double>::Identity()` as your input.
  /// @param[in] child
  ///   The child body connected by the new joint.
  /// @param[in] X_BM
  ///   The fixed pose of frame M attached to the child body, measured in
  ///   the frame B of that body. `X_BM` is an optional parameter; empty curly
  ///   braces `{}` imply that frame M **is** the same body frame B. If instead
  ///   your intention is to make a frame F with pose `X_PF`, provide
  ///   `RigidTransform<double>::Identity()` as your input.
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
  /// @throws std::exception if `this` model already contains a joint with the
  /// given `name`.
  /// See HasJointNamed(), Joint::name().
  ///
  /// @see The Joint class's documentation for further details on how a Joint
  /// is defined.
  template<template<typename> class JointType, typename... Args>
  const JointType<T>& AddJoint(
      const std::string& name,
      const Body<T>& parent, const optional<math::RigidTransform<double>>& X_PF,
      const Body<T>& child, const optional<math::RigidTransform<double>>& X_BM,
      Args&&... args);

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
  /// @throws std::exception if `this` model already contains a joint actuator
  /// with the given `name`. See HasJointActuatorNamed(),
  /// JointActuator::get_name().
  // TODO(amcastro-tri): consider adding sugar method to declare an actuated
  // joint with a single call. Maybe MBT::AddActuatedJoint() or the like.
  const JointActuator<T>& AddJointActuator(
      const std::string& name, const Joint<T>& joint);

  /// Creates a new model instance.  Returns the index for a new model
  /// instance (as there is no concrete object beyond the index).
  ///
  /// @param[in] name
  ///   A string that uniquely identifies the new instance to be added to `this`
  ///   model. An exception is thrown if an instance with the same name
  ///   already exists in the model. See HasModelInstanceNamed().
  /// @throws std::logic_error if Finalize() was already called on `this` tree.
  ModelInstanceIndex AddModelInstance(const std::string& name);

  /// @}
  // Closes Doxygen section "Methods to add new MultibodyTree elements."

  /// See MultibodyPlant method.
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

  /// See MultibodyPlant method.
  int num_mobilizers() const {
    return static_cast<int>(owned_mobilizers_.size());
  }

  /// See MultibodyPlant method.
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

  /// See MultibodyPlant method.
  int num_actuated_dofs() const {
    return topology_.num_actuated_dofs();
  }

  /// See MultibodyPlant method.
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

  /// See MultibodyPlant method.
  const Body<T>& get_body(BodyIndex body_index) const {
    DRAKE_THROW_UNLESS(body_index < num_bodies());
    return *owned_bodies_[body_index];
  }

  /// See MultibodyPlant method.
  const Joint<T>& get_joint(JointIndex joint_index) const {
    DRAKE_THROW_UNLESS(joint_index < num_joints());
    return *owned_joints_[joint_index];
  }

  /// See MultibodyPlant method.
  Joint<T>& get_mutable_joint(JointIndex joint_index) {
    DRAKE_THROW_UNLESS(joint_index < num_joints());
    return *owned_joints_[joint_index];
  }

  /// See MultibodyPlant method.
  const JointActuator<T>& get_joint_actuator(
      JointActuatorIndex actuator_index) const {
    DRAKE_THROW_UNLESS(actuator_index < num_actuators());
    return *owned_actuators_[actuator_index];
  }

  /// See MultibodyPlant method.
  const Frame<T>& get_frame(FrameIndex frame_index) const {
    DRAKE_THROW_UNLESS(frame_index < num_frames());
    return *frames_[frame_index];
  }

  /// See MultibodyPlant method.
  const Mobilizer<T>& get_mobilizer(MobilizerIndex mobilizer_index) const {
    DRAKE_THROW_UNLESS(mobilizer_index < num_mobilizers());
    return *owned_mobilizers_[mobilizer_index];
  }

  /// See MultibodyPlant method.
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
  /// @throws std::exception if @p model_instance is not valid for this model.
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

  /// See MultibodyPlant method.
  bool HasFrameNamed(const std::string& name) const {
    const int count = frame_name_to_index_.count(name);
    if (count > 1) {
      throw std::logic_error(
          "Frame " + name + " appears in multiple model instances.");
    }
    return count > 0;
  }

  /// See MultibodyPlant method.
  bool HasFrameNamed(const std::string& name,
                     ModelInstanceIndex model_instance) const {
    DRAKE_THROW_UNLESS(model_instance < instance_name_to_index_.size());
    // See notes in `HasBodyNamed`.
    const auto range = frame_name_to_index_.equal_range(name);
    for (auto it = range.first; it != range.second; ++it) {
      if (get_frame(it->second).model_instance() == model_instance) {
        return true;
      }
    }
    return false;
  }

  /// See MultibodyPlant method.
  bool HasJointNamed(const std::string& name) const {
    const int count = joint_name_to_index_.count(name);
    if (count > 1) {
      throw std::logic_error(
          "Joint " + name + " appears in multiple model instances.");
    }
    return count > 0;
  }

  /// See MultibodyPlant method.
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

  /// See MultibodyPlant method.
  bool HasJointActuatorNamed(const std::string& name) const {
    const int count = actuator_name_to_index_.count(name);
    if (count > 1) {
      throw std::logic_error(
          "Joint actuator " + name + " appears in multiple model instances.");
    }
    return count > 0;
  }

  /// See MultibodyPlant method.
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

  /// See MultibodyMethod.
  bool HasModelInstanceNamed(const std::string& name) const {
    return instance_name_to_index_.find(name) != instance_name_to_index_.end();
  }
  /// @}

  /// See MultibodyPlant method.
  const Body<T>& GetBodyByName(const std::string& name) const {
    return get_body(
        GetElementIndex<BodyIndex>(name, "Body", body_name_to_index_));
  }

  /// See MultibodyPlant method.
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

  /// Returns a list of body indices associated with `model_instance`.
  std::vector<BodyIndex> GetBodyIndices(ModelInstanceIndex model_instance)
  const {
    DRAKE_THROW_UNLESS(model_instance < instance_name_to_index_.size());
    std::vector<BodyIndex> indices;
    for (auto& body : owned_bodies_) {
      if (body->model_instance() == model_instance) {
        indices.emplace_back(body->index());
      }
    }
    return indices;
  }

  /// Returns a list of joint indices associated with `model_instance`.
  std::vector<JointIndex> GetJointIndices(ModelInstanceIndex model_instance)
  const {
    DRAKE_THROW_UNLESS(model_instance < instance_name_to_index_.size());
    std::vector<JointIndex> indices;
    for (auto& joint : owned_joints_) {
      if (joint->model_instance() == model_instance) {
        indices.emplace_back(joint->index());
      }
    }
    return indices;
  }

  /// See MultibodyPlant method.
  const Frame<T>& GetFrameByName(const std::string& name) const {
    return get_frame(
        GetElementIndex<FrameIndex>(name, "Frame", frame_name_to_index_));
  }

  /// See MultibodyPlant method.
  const Frame<T>& GetFrameByName(
      const std::string& name, ModelInstanceIndex model_instance) const {
    DRAKE_THROW_UNLESS(model_instance < instance_name_to_index_.size());
    const auto range = frame_name_to_index_.equal_range(name);
    for (auto it = range.first; it != range.second; ++it) {
      const Frame<T>& frame = get_frame(it->second);
      if (frame.model_instance() == model_instance) {
        return frame;
      }
    }
    throw std::logic_error(
        "There is no frame named '" + name + "' in model instance '" +
            instance_index_to_name_.at(model_instance) + "'.");
  }

  /// See MultibodyPlant method.
  const RigidBody<T>& GetRigidBodyByName(const std::string& name) const {
    const RigidBody<T>* body =
        dynamic_cast<const RigidBody<T>*>(&GetBodyByName(name));
    if (body == nullptr) {
      throw std::logic_error("Body '" + name + "' is not a RigidBody.");
    }
    return *body;
  }

  /// See MultibodyPlant method.
  const RigidBody<T>& GetRigidBodyByName(
      const std::string& name, ModelInstanceIndex model_instance) const {
    DRAKE_THROW_UNLESS(model_instance < instance_name_to_index_.size());
    const RigidBody<T>* body =
        dynamic_cast<const RigidBody<T>*>(&GetBodyByName(name, model_instance));
    if (body == nullptr) {
      throw std::logic_error("Body '" + name + "' in model instance '" +
                             instance_index_to_name_.at(model_instance) +
                             "' is not a RigidBody.");
    }
    return *body;
  }

  /// See MultibodyPlant method.
  template <template <typename> class JointType = Joint>
  const JointType<T>& GetJointByName(
      const std::string& name,
      optional<ModelInstanceIndex> model_instance = nullopt) const {
    static_assert(std::is_base_of<Joint<T>, JointType<T>>::value,
                  "JointType<T> must be a sub-class of Joint<T>.");

    const Joint<T>* joint = nullptr;
    if (model_instance) {
      DRAKE_THROW_UNLESS(*model_instance < instance_name_to_index_.size());
      const auto range = joint_name_to_index_.equal_range(name);
      for (auto it = range.first; it != range.second; ++it) {
        const Joint<T>& this_joint = get_joint(it->second);
        if (this_joint.model_instance() == *model_instance) {
          joint = &this_joint;
        }
      }
      if (joint == nullptr) {
        throw std::logic_error(
            "There is no joint named '" + name + "' in model instance '" +
            instance_index_to_name_.at(*model_instance) + "'.");
      }
    } else {
      joint = &get_joint(
          GetElementIndex<JointIndex>(name, "Joint", joint_name_to_index_));
    }

    const JointType<T>* typed_joint = dynamic_cast<const JointType<T>*>(joint);
    if (typed_joint == nullptr) {
      throw std::logic_error(
          "Joint '" + name + "' in model instance " +
          instance_index_to_name_.at(*model_instance) + " is not of type '" +
          NiceTypeName::Get<JointType<T>>() + "' but of type '" +
          NiceTypeName::Get(*joint) + "'.");
    }
    return *typed_joint;
  }

  /// See MultibodyPlant method.
  template <template <typename> class JointType = Joint>
  JointType<T>& GetMutableJointByName(
      const std::string& name,
      optional<ModelInstanceIndex> model_instance = nullopt) {
    const JointType<T>& const_joint =
        GetJointByName<JointType>(name, model_instance);

    // Note: Using the const method to implement this non-const one
    // relies on the fact (true today) that no lower-level MultibodyTree code
    // needs to know we're obtaining mutable access here. For example,
    // this wouldn't work if a stored computation needed to be invalidated.
    return const_cast<JointType<T>&>(const_joint);
  }

  /// See MultibodyPlant method.
  const JointActuator<T>& GetJointActuatorByName(
      const std::string& name) const {
    return get_joint_actuator(
        GetElementIndex<JointActuatorIndex>(
            name, "Joint actuator", actuator_name_to_index_));
  }

  /// See MultibodyPlant method.
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

  /// See MultibodyPlant method.
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

  /// See MultibodyPlant method.
  void SetActuationInArray(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& u_instance,
      EigenPtr<VectorX<T>> u) const;

  /// See MultibodyPlant method.
  VectorX<T> GetPositionsFromArray(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& q) const;

  /// See MultibodyPlant method.
  void SetPositionsInArray(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& q_instance,
      EigenPtr<VectorX<T>> q) const;

  /// See MultibodyPlant method.
  VectorX<T> GetVelocitiesFromArray(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& v) const;

  /// Sets the vector of generalized velocities for `model_instance` in
  /// `v` using `v_instance`, leaving all other elements in the array
  /// untouched. This method throws an exception if `v` is not of size
  /// MultibodyTree::num_velocities() or `v_instance` is not of size
  /// `MultibodyTree::num_positions(model_instance)`.
  void SetVelocitiesInArray(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& v_instance,
      EigenPtr<VectorX<T>> v) const;

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

  /// (Advanced) Allocates a new context for this %MultibodyTree uniquely
  /// identifying the state of the multibody system.
  ///
  /// @throws std::runtime_error if this is not owned by a MultibodyPlant /
  /// MultibodyTreeSystem.
  std::unique_ptr<systems::LeafContext<T>> CreateDefaultContext() const {
    if (tree_system_ == nullptr) {
      throw std::runtime_error(
          "MultibodyTree::CreateDefaultContext(): can only be called from a "
          "MultibodyTree that is owned by a MultibodyPlant / "
          "MultibodyTreeSystem");
    }
    return dynamic_pointer_cast<systems::LeafContext<T>>(
        tree_system_->CreateDefaultContext());
  }

  /// See MultibodyPlant method.
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const;

  /// See MultibodyPlant method.
  void SetRandomState(const systems::Context<T>& context,
                      systems::State<T>* state,
                      RandomGenerator* generator) const;

  /// Returns a const Eigen vector reference containing the vector
  /// `[q; v]` of the model with `q` the vector of generalized positions and
  /// `v` the vector of generalized velocities.
  /// @note This method returns a reference to existing data, exhibits constant
  ///       i.e., O(1) time complexity, and runs very quickly.
  /// @throws std::exception if the `context` does not correspond to the context
  /// for a multibody model.
  Eigen::VectorBlock<const VectorX<T>> GetPositionsAndVelocities(
      const systems::Context<T>& context) const;

  /// Returns a Eigen vector containing the multibody state `x = [q; v]`
  /// of the model with `q` the vector of generalized positions and `v` the
  /// vector of generalized velocities for model instance `model_instance`.
  /// @throws std::exception if the `context` does not correspond to the context
  /// for a multibody model or `model_instance` is invalid.
  /// @note returns a dense vector of dimension `q.size() + v.size()` associated
  ///          with `model_instance` in O(`q.size()`) time.
  VectorX<T> GetPositionsAndVelocities(
      const systems::Context<T>& context,
      ModelInstanceIndex model_instance) const;

  /// Returns a mutable Eigen vector containing the vector `[q; v]`
  /// of the model with `q` the vector of generalized positions and `v` the
  /// vector of generalized velocities.
  /// @throws std::exception if the `context` is nullptr or if it does not
  /// correspond to the context for a multibody model.
  /// @note This method returns a reference to existing data, exhibits constant
  ///       i.e., O(1) time complexity, and runs very quickly.
  /// @pre `state` must be the systems::State<T> owned by the `context`.
  Eigen::VectorBlock<VectorX<T>> GetMutablePositionsAndVelocities(
  const systems::Context<T>& context, systems::State<T>* state) const;

  /// See GetMutablePositionsAndVelocities(context, state) above.
  Eigen::VectorBlock<VectorX<T>> GetMutablePositionsAndVelocities(
  systems::Context<T>* context) const {
    return GetMutablePositionsAndVelocities(*context,
                                            &context->get_mutable_state());
  }

  /// Sets `context` to store the vector `[q; v]`
  /// with `q` the vector of generalized positions and `v` the vector
  /// of generalized velocities for model instance `model_instance`.
  /// @throws std::exception if the `context` does not correspond to the context
  /// for a multibody model, `context` is nullptr, `model_instance` is invalid,
  /// or `instance_state.size()` does not equal `num_positions(model_instance)`
  /// + `num_velocities(model_instance)`.
  void SetPositionsAndVelocities(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& instance_state,
      systems::Context<T>* context) const;

  /// See MultibodyPlant::GetFreeBodyPose.
  math::RigidTransform<T> GetFreeBodyPoseOrThrow(
      const systems::Context<T>& context, const Body<T>& body) const;

  /// See MultibodyPlant::SetFreeBodyPose.
  void SetFreeBodyPoseOrThrow(
      const Body<T>& body, const math::RigidTransform<T>& X_WB,
      systems::Context<T>* context) const;

  /// See MultibodyPlant::SetFreeBodySpatialVelocity.
  void SetFreeBodySpatialVelocityOrThrow(
      const Body<T>& body, const SpatialVelocity<T>& V_WB,
      systems::Context<T>* context) const;

  /// See MultibodyPlant::SetFreeBodyPose.
  void SetFreeBodyPoseOrThrow(
      const Body<T>& body, const math::RigidTransform<T>& X_WB,
      const systems::Context<T>& context, systems::State<T>* state) const;

  /// See MultibodyPlant::SetFreeBodySpatialVelocity.
  void SetFreeBodySpatialVelocityOrThrow(
      const Body<T>& body, const SpatialVelocity<T>& V_WB,
      const systems::Context<T>& context, systems::State<T>* state) const;

  /// See MultibodyPlant::SetFreeBodyRandomPositionDistribution.
  void SetFreeBodyRandomPositionDistributionOrThrow(
      const Body<T>& body,
      const Vector3<symbolic::Expression>& position);

  /// See MultibodyPlant::SetFreeBodyRandomRotationDistribution.
  void SetFreeBodyRandomRotationDistributionOrThrow(
      const Body<T>& body,
      const Eigen::Quaternion<symbolic::Expression>& rotation);

  /// @name Kinematic computations
  /// Kinematics computations are concerned with the motion of bodies in the
  /// model without regard to their mass or the forces and moments that cause
  /// the motion. Methods in this category include the computation of poses and
  /// spatial velocities.
  /// @{

  /// See MultibodyPlant method.
  void CalcAllBodyPosesInWorld(
      const systems::Context<T>& context,
      std::vector<math::RigidTransform<T>>* X_WB) const;

  /// See MultibodyPlant method.
  void CalcAllBodySpatialVelocitiesInWorld(
      const systems::Context<T>& context,
      std::vector<SpatialVelocity<T>>* V_WB) const;

  /// See MultibodyPlant method.
  math::RigidTransform<T> CalcRelativeTransform(
      const systems::Context<T>& context,
      const Frame<T>& frame_A, const Frame<T>& frame_B) const;

  /// See MultibodyPlant method.
  void CalcPointsPositions(
      const systems::Context<T>& context,
      const Frame<T>& frame_B,
      const Eigen::Ref<const MatrixX<T>>& p_BQi,
      const Frame<T>& frame_A,
      EigenPtr<MatrixX<T>> p_AQi) const;

  /// See MultibodyPlant method.
  const math::RigidTransform<T>& EvalBodyPoseInWorld(
      const systems::Context<T>& context,
      const Body<T>& body_B) const;

  /// See MultibodyPlantMethod.
  const SpatialVelocity<T>& EvalBodySpatialVelocityInWorld(
      const systems::Context<T>& context,
      const Body<T>& body_B) const;

  /// @}
  // End of "Kinematic computations" section.

  /// @name Methods to compute multibody Jacobians.
  /// @{

  /// See MultibodyPlant method.
  void CalcPointsGeometricJacobianExpressedInWorld(
      const systems::Context<T>& context,
      const Frame<T>& frame_F, const Eigen::Ref<const MatrixX<T>>& p_FP_list,
      EigenPtr<MatrixX<T>> p_WP_list, EigenPtr<MatrixX<T>> Jv_WFp) const;

  /// See MultibodyPlant method.
  void CalcPointsGeometricJacobianExpressedInWorld(
      const systems::Context<T>& context,
      const Frame<T>& frame_F, const Eigen::Ref<const MatrixX<T>>& p_WP_list,
      EigenPtr<MatrixX<T>> Jv_WFp) const;

  /// See MultibodyPlant method.
  VectorX<T> CalcBiasForPointsGeometricJacobianExpressedInWorld(
      const systems::Context<T>& context,
      const Frame<T>& frame_F,
      const Eigen::Ref<const MatrixX<T>>& p_FP_list) const;

  /// See MultibodyPlant method.
  void CalcPointsAnalyticalJacobianExpressedInWorld(
      const systems::Context<T>& context,
      const Frame<T>& frame_F, const Eigen::Ref<const MatrixX<T>>& p_FP_list,
      EigenPtr<MatrixX<T>> p_WP_list, EigenPtr<MatrixX<T>> Jq_WFp) const;

  /// See MultibodyPlant method.
  void CalcFrameGeometricJacobianExpressedInWorld(
      const systems::Context<T>& context,
      const Frame<T>& frame_F, const Eigen::Ref<const Vector3<T>>& p_FP,
      EigenPtr<MatrixX<T>> Jv_WFp) const;

  /// See MultibodyPlant method.
  void CalcRelativeFrameGeometricJacobian(
      const systems::Context<T>& context,
      const Frame<T>& frame_B, const Eigen::Ref<const Vector3<T>>& p_BP,
      const Frame<T>& frame_A, const Frame<T>& frame_E,
      EigenPtr<MatrixX<T>> Jv_ABp_E) const;

  /// See MultibodyPlant method.
  Vector6<T> CalcBiasForFrameGeometricJacobianExpressedInWorld(
      const systems::Context<T>& context,
      const Frame<T>& frame_F, const Eigen::Ref<const Vector3<T>>& p_FP) const;

  /// See MultibodyPlant method.
  void CalcJacobianSpatialVelocity(
      const systems::Context<T>& context,
      JacobianWrtVariable with_respect_to,
      const Frame<T>& frame_B, const Eigen::Ref<const Vector3<T>>& p_BP,
      const Frame<T>& frame_A, const Frame<T>& frame_E,
      EigenPtr<MatrixX<T>> Jw_ABp_E) const;

  /// See MultibodyPlant method.
  void CalcJacobianAngularVelocity(const systems::Context<T>& context,
                                   JacobianWrtVariable with_respect_to,
                                   const Frame<T>& frame_B,
                                   const Frame<T>& frame_A,
                                   const Frame<T>& frame_E,
                                   EigenPtr<MatrixX<T>> Js_w_AB_E) const;

  /// See MultibodyPlant method.
  void CalcJacobianTranslationalVelocity(
      const systems::Context<T>& context,
      JacobianWrtVariable with_respect_to, const Frame<T>& frame_B,
      const Eigen::Ref<const Vector3<T>>& p_BoBp_B, const Frame<T>& frame_A,
      const Frame<T>& frame_E, EigenPtr<MatrixX<T>> Js_v_ABp_E) const;

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
  /// system.
  /// @{

  /// Computes into the position kinematics `pc` all the kinematic quantities
  /// that depend on the generalized positions only. These include:
  ///
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

  /// Computes the spatial inertia M_Bo_W(q) for each body B in the model about
  /// its frame origin Bo and expressed in the world frame W.
  /// @param[in] context
  ///   The context storing the state of the model.
  /// @param[out] M_B_W_cache
  ///   For each body in the model, entry Body::node_index() in M_B_W_cache
  ///   contains the updated spatial inertia `M_B_W(q)` for that body. On input
  ///   it must be a valid pointer to a vector of size num_bodies().
  /// @throws std::exception if M_B_W_cache is nullptr or if its size is not
  /// num_bodies().
  void CalcSpatialInertiaInWorldCache(
      const systems::Context<T>& context,
      std::vector<SpatialInertia<T>>* M_B_W_cache) const;

  /// Computes the bias term `b_Bo_W(q, v)` for each body in the model.
  /// For a body B, this is the bias term `b_Bo_W` in the equation
  /// `F_BBo_W = M_Bo_W * A_WB + b_Bo_W`, where `M_Bo_W` is the spatial inertia
  /// about B's origin Bo, `A_WB` is the spatial acceleration of B in W and
  /// `F_BBo_W` is the spatial force applied on B about Bo, expressed in W.
  /// @param[in] context
  ///   The context storing the state of the model.
  /// @param[out] b_Bo_W_cache
  ///   For each body in the model, entry Body::node_index() in b_Bo_W_cache
  ///   contains the updated bias term `b_Bo_W(q, v)` for that body. On input it
  ///   must be a valid pointer to a vector of size num_bodies().
  /// @throws std::exception if b_Bo_W_cache is nullptr or if its size is not
  /// num_bodies().
  void CalcDynamicBiasCache(const systems::Context<T>& context,
                            std::vector<SpatialForce<T>>* b_Bo_W_cache) const;

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

  /// See MultibodyPlant method.
  /// @warning The output parameter `A_WB_array` is indexed by BodyNodeIndex,
  /// while MultibodyPlant's method returns accelerations indexed by BodyIndex.
  void CalcSpatialAccelerationsFromVdot(
      const systems::Context<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc,
      const VectorX<T>& known_vdot,
      std::vector<SpatialAcceleration<T>>* A_WB_array) const;

  /// See MultibodyPlant method.
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
      const systems::Context<T>& context, const VectorX<T>& known_vdot,
      const std::vector<SpatialForce<T>>& Fapplied_Bo_W_array,
      const Eigen::Ref<const VectorX<T>>& tau_applied_array,
      std::vector<SpatialAcceleration<T>>* A_WB_array,
      std::vector<SpatialForce<T>>* F_BMo_W_array,
      EigenPtr<VectorX<T>> tau_array) const;

  /// See MultibodyPlant method.
  void CalcForceElementsContribution(
      const systems::Context<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc,
      MultibodyForces<T>* forces) const;

  /// See MultibodyPlant method.
  T CalcPotentialEnergy(const systems::Context<T>& context) const;

  /// See MultibodyPlant method.
  T CalcConservativePower(const systems::Context<T>& context) const;

  /// See MultibodyPlant method.
  void CalcMassMatrixViaInverseDynamics(
      const systems::Context<T>& context, EigenPtr<MatrixX<T>> H) const;

  /// See MultibodyPlant method.
  void CalcBiasTerm(
      const systems::Context<T>& context, EigenPtr<VectorX<T>> Cv) const;

  /// See MultibodyPlant method.
  VectorX<T> CalcGravityGeneralizedForces(
      const systems::Context<T>& context) const;

  /// See MultibodyPlant method.
  void MapVelocityToQDot(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v,
      EigenPtr<VectorX<T>> qdot) const;

  /// See MultibodyPlant method.
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

  /// See MultibodyPlant method.
  MatrixX<double> MakeStateSelectorMatrix(
      const std::vector<JointIndex>& user_to_joint_index_map) const;

  /// Alternative signature to build a state selector matrix from a std::vector
  /// of joint names.
  /// See MakeStateSelectorMatrixFromJointNames(const std::vector<JointIndex>&)
  /// for details.
  /// `selected_joints` must not contain any duplicates.
  ///
  /// A user specifies the preferred order in the selected states vector xₛ via
  /// `selected_joints`. The selected state is built such that selected
  /// positions are followed by selected velocities, as in `xₛ = [qₛ, vₛ]`.
  /// The positions in qₛ are a concatenation of the positions for each joint
  /// in the order they appear in `selected_joints`. That is, the positions for
  /// `selected_joints[0]` are first, followed by the positions for
  /// `selected_joints[1]`, etc. Similarly for the selected velocities vₛ.
  ///
  /// @throws std::logic_error if there are any duplicates in `selected_joints`.
  /// @throws std::logic_error if there is no joint in the model with a name
  /// specified in `selected_joints`.
  MatrixX<double> MakeStateSelectorMatrixFromJointNames(
      const std::vector<std::string>& selected_joints) const;

  /// See MultibodyPlant method.
  MatrixX<double> MakeActuatorSelectorMatrix(
      const std::vector<JointActuatorIndex>& user_to_actuator_index_map) const;

  /// See MultibodyPlant method.
  MatrixX<double> MakeActuatorSelectorMatrix(
      const std::vector<JointIndex>& user_to_joint_index_map) const;

  /// See MultibodyPlant method.
  VectorX<double> GetPositionLowerLimits() const;

  /// See MultibodyPlant method.
  VectorX<double> GetPositionUpperLimits() const;

  /// See MultibodyPlant method.
  VectorX<double> GetVelocityLowerLimits() const;

  /// See MultibodyPlant method.
  VectorX<double> GetVelocityUpperLimits() const;

  /// See MultibodyPlant method.
  VectorX<double> GetAccelerationLowerLimits() const;

  /// See MultibodyPlant method.
  VectorX<double> GetAccelerationUpperLimits() const;

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

  /// SFINAE overload for Mobilizer<T> elements.
  template <template <typename> class MultibodyElement, typename Scalar>
  std::enable_if_t<std::is_base_of<Mobilizer<T>, MultibodyElement<T>>::value,
                   MultibodyElement<T>&> get_mutable_variant(
      const MultibodyElement<Scalar>& element) {
    return get_mutable_mobilizer_variant(element);
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
  ///
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
  /// @param context A Context whose position kinematics cache will be
  ///                updated and returned.
  /// @return Reference to the PositionKinematicsCache of context.
  const PositionKinematicsCache<T>& EvalPositionKinematics(
      const systems::Context<T>& context) const {
    DRAKE_ASSERT(tree_system_ != nullptr);
    return tree_system_->EvalPositionKinematics(context);
  }

  /// Evaluates velocity kinematics cached in context. This will also
  /// force position kinematics to be updated if it hasn't already.
  /// @param context A Context whose velocity kinematics cache will be
  ///                updated and returned.
  /// @return Reference to the VelocityKinematicsCache of context.
  const VelocityKinematicsCache<T>& EvalVelocityKinematics(
      const systems::Context<T>& context) const {
    DRAKE_ASSERT(tree_system_ != nullptr);
    return tree_system_->EvalVelocityKinematics(context);
  }

  /// @name                 State access methods
  /// These methods use information in the MultibodyTree to determine how to
  /// locate the tree's state variables in a given Context or State.
  //@{

  /// Returns true if we are using discrete state for positions and velocities;
  /// otherwise we're using continuous state.
  bool is_state_discrete() const {
    DRAKE_ASSERT(tree_system_ != nullptr);
    return tree_system_->is_discrete();
  }

  /// Returns a const reference to the kinematic state vector stored in the
  /// given Context as an `Eigen::VectorBlock<const VectorX<T>>`. This will
  /// consist only of q and v partitions in that order.
  Eigen::VectorBlock<const VectorX<T>> get_state_vector(
      const systems::Context<T>& context) const;

  /// This is a mutable-Context version of `get_state_vector()`.
  /// @note Invalidates all q- or v-dependent cache entries.
  Eigen::VectorBlock<VectorX<T>> get_mutable_state_vector(
      systems::Context<T>* context) const;

  /// This is a mutable-State version of `get_state_vector()`.
  /// @note This does not cause cache invalidation.
  Eigen::VectorBlock<VectorX<T>> get_mutable_state_vector(
      systems::State<T>* state) const;

  /// Returns a const reference to the position state vector q stored in the
  /// given Context as an `Eigen::VectorBlock<const VectorX<T>>`.
  Eigen::VectorBlock<const VectorX<T>> get_positions(
      const systems::Context<T>& context) const;

  /// This is a mutable-Context version of `get_positions()`.
  /// @note Invalidates all q-dependent cache entries. (May also invalidate
  ///       v-dependent cache entries.)
  Eigen::VectorBlock<VectorX<T>> get_mutable_positions(
      systems::Context<T>* context) const;

  /// This is a mutable-State version of `get_positions()`.
  /// @note This does not cause cache invalidation.
  Eigen::VectorBlock<VectorX<T>> get_mutable_positions(
      systems::State<T>* state) const;

  /// Returns a const reference to the velcoity state vector v stored in the
  /// given Context as an `Eigen::VectorBlock<const VectorX<T>>`.
  Eigen::VectorBlock<const VectorX<T>> get_velocities(
      const systems::Context<T>& context) const;

  /// This is a mutable-Context version of `get_velocities()`.
  /// @note Invalidates all v-dependent cache entries. (May also invalidate
  ///       q-dependent cache entries.)
  Eigen::VectorBlock<VectorX<T>> get_mutable_velocities(
      systems::Context<T>* context) const;

  /// This is a mutable-State version of `get_velocities()`.
  /// @note This does not cause cache invalidation.
  Eigen::VectorBlock<VectorX<T>> get_mutable_velocities(
      systems::State<T>* state) const;

  /// Returns a const fixed-size Eigen::VectorBlock of `count` elements
  /// referencing a segment in the state vector with its first element
  /// at `start`.
  template <int count>
  Eigen::VectorBlock<const VectorX<T>, count> get_state_segment(
      const systems::Context<T>& context, int start) const {
    // (Comments here apply to similar methods below too.)
    // We know that context is a LeafContext and therefore the
    // continuous state vector must be a BasicVector.
    // TODO(amcastro-tri): make use of VectorBase::get_contiguous_vector() once
    // PR #6049 gets merged.
    Eigen::VectorBlock<const VectorX<T>> x = get_state_vector(context);
    // xc.nestedExpression() resolves to "VectorX<T>&" since the continuous
    // state is a BasicVector.
    // If we do return xc.segment() directly, we would instead get a
    // Block<Block<VectorX>>, which is very different from Block<VectorX>.
    return x.nestedExpression().template segment<count>(start);
  }

  /// This is a mutable-Context version of `get_state_segment<count>(start)`.
  /// @note Invalidates all q- or v-dependent cache entries.
  template <int count>
  Eigen::VectorBlock<VectorX<T>, count> get_mutable_state_segment(
      systems::Context<T>* context, int start) const {
    Eigen::VectorBlock<VectorX<T>> x = get_mutable_state_vector(context);
    return x.nestedExpression().template segment<count>(start);
  }

  /// This is a mutable-State version of `get_state_segment<count>(start)`.
  /// @note This does not cause cache invalidation.
  template <int count>
  Eigen::VectorBlock<VectorX<T>, count> get_mutable_state_segment(
      systems::State<T>* state, int start) const {
    Eigen::VectorBlock<VectorX<T>> x = get_mutable_state_vector(&*state);
    return x.nestedExpression().template segment<count>(start);
  }

  /// Returns a const fixed-size Eigen::VectorBlock of `count` elements
  /// referencing a segment in the Context's state vector with its first element
  /// at `start`.
  Eigen::VectorBlock<const VectorX<T>> get_state_segment(
      const systems::Context<T>& context, int start, int count) const {
    Eigen::VectorBlock<const VectorX<T>> x = get_state_vector(context);
    return x.nestedExpression().segment(start, count);
  }

  /// This is a mutable-Context version of `get_state_segment(start, count)`.
  /// @note Invalidates all q- or v-dependent cache entries.
  Eigen::VectorBlock<VectorX<T>> get_mutable_state_segment(
      systems::Context<T>* context, int start, int count) const {
    Eigen::VectorBlock<VectorX<T>> x = get_mutable_state_vector(context);
    return x.nestedExpression().segment(start, count);
  }

  /// This is a mutable-State version of `get_state_segment(start, count)`.
  /// @note This does not cause cache invalidation.
  Eigen::VectorBlock<VectorX<T>> get_mutable_state_segment(
      systems::State<T>* state, int start, int count) const {
    Eigen::VectorBlock<VectorX<T>> x = get_mutable_state_vector(&*state);
    return x.nestedExpression().segment(start, count);
  }
  //@}

  /// (Internal use only) Informs the MultibodyTree how to access its resources
  /// within a Context.
  void set_tree_system(MultibodyTreeSystem<T>* tree_system) {
    DRAKE_DEMAND(tree_system != nullptr && tree_system_ == nullptr);
    tree_system_ = tree_system;
  }

  /// (Internal) Computes the cache entry associated with the geometric Jacobian
  /// H_PB_W for each node.
  /// The geometric Jacobian `H_PB_W` relates to the spatial velocity of B in P
  /// by `V_PB_W = H_PB_W(q)⋅v_B`, where `v_B` corresponds to the generalized
  /// velocities associated to body B. `H_PB_W` has size `6 x nm` with `nm` the
  /// number of mobilities associated with body B.
  /// `H_PB_W_cache` stores the Jacobian matrices for all nodes in the tree as a
  /// vector of the columns of these matrices. Therefore `H_PB_W_cache` has as
  /// many entries as number of generalized velocities in the tree.
  // TODO(amcastro-tri): Rework this method as per issue #10155.
  void CalcAcrossNodeGeometricJacobianExpressedInWorld(
      const systems::Context<T>& context,
      const PositionKinematicsCache<T>& pc,
      std::vector<Vector6<T>>* H_PB_W_cache) const;

 private:
  // Make MultibodyTree templated on every other scalar type a friend of
  // MultibodyTree<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private methods from MultibodyTree<T>.
  template <typename> friend class MultibodyTree;

  // Friend class to facilitate testing.
  friend class MultibodyTreeTester;

  // Helpers for getting the full qv discrete state once we know we are using
  // discrete state.
  Eigen::VectorBlock<const VectorX<T>> get_discrete_state_vector(
      const systems::Context<T>& context) const;

  // Invalidates q- or v-dependent cache entries.
  Eigen::VectorBlock<VectorX<T>> get_mutable_discrete_state_vector(
      systems::Context<T>* context) const;

  // Does no invalidation.
  Eigen::VectorBlock<VectorX<T>> get_mutable_discrete_state_vector(
      systems::State<T>* state) const;

  Eigen::VectorBlock<VectorX<T>> extract_qv_from_continuous(
      systems::VectorBase<T>* continuous_qvz) const;

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

  // Evaluates the cache entry stored in context with the spatial inertias
  // M_Bo_W(q) for each body in the system. These will be updated as needed.
  const std::vector<SpatialInertia<T>>& EvalSpatialInertiaInWorldCache(
      const systems::Context<T>& context) const {
    DRAKE_ASSERT(tree_system_ != nullptr);
    return tree_system_->EvalSpatialInertiaInWorldCache(context);
  }

  // Evaluates the cache entry stored in context with the bias term b_Bo_W(q, v)
  // for each body. These will be updated as needed.
  const std::vector<SpatialForce<T>>& EvalDynamicBiasCache(
      const systems::Context<T>& context) const {
    DRAKE_ASSERT(tree_system_ != nullptr);
    return tree_system_->EvalDynamicBiasCache(context);
  }

  // Given the state of this model in `context` and a known vector
  // of generalized accelerations `known_vdot`, this method computes the
  // spatial acceleration `A_WB` for each body as measured and expressed in the
  // world frame W.
  //
  // Iff `ignore_velocities = true` velocity values stored in `context` are
  // ignored and are assumed to be zero. Therefore, Velocity kinematics and
  // velocity dependent terms that become zero (such as bias terms) are not
  // computed to avoid unnecessary work.
  void CalcSpatialAccelerationsFromVdot(
      const systems::Context<T>& context, const VectorX<T>& known_vdot,
      bool ignore_velocities,
      std::vector<SpatialAcceleration<T>>* A_WB_array) const;

  // Given the state stored in `context` and a
  // known vector of generalized accelerations `vdot`, this method computes the
  // set of generalized forces `tau_id` that would need to be applied at each
  // Mobilizer in order to attain the specified generalized accelerations.
  // Mathematically, this method computes: <pre>
  //   tau_id = M(q)v̇ + C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W
  // </pre>
  // where `M(q)` is the mass matrix, `C(q, v)v` is the bias
  // term containing Coriolis and gyroscopic effects and `tau_app` consists
  // of a vector applied generalized forces.
  //
  // iff `ignore_velocities = true` velocity values stored in `context` are
  // ignored and are assumed to be zero. Therefore, C(q, v)v = 0 and it is not
  // computed to avoid unnecessary work.
  void CalcInverseDynamics(
      const systems::Context<T>& context, const VectorX<T>& known_vdot,
      const std::vector<SpatialForce<T>>& Fapplied_Bo_W_array,
      const Eigen::Ref<const VectorX<T>>& tau_applied_array,
      bool ignore_velocities, std::vector<SpatialAcceleration<T>>* A_WB_array,
      std::vector<SpatialForce<T>>* F_BMo_W_array,
      EigenPtr<VectorX<T>> tau_array) const;

  // Helper method to compute the rotational part of the frame Jacobian Jr_WFq
  // and the translational part of the frame Jacobian Jt_WFq for a list of
  // points Q which instantaneously move with frame F that is, the position
  // of these points Q is fixed in frame F.
  // Jacobians Jr_WFq and Jt_WFq are defined such that the angular velocity
  // w_WFq and the translational velocity v_WFq of frame F shifted (see
  // SpatialVelocity::Shift() for a description of the shift operation) to a
  // frame Fq with origin at a point Q are given by:
  //   w_WFq = Jr_WFq⋅v
  //   v_WFq = Jt_WFq⋅v
  // when computed in terms of generalized velocities
  // (with_respect_to = JacobianWrtVariable::kV) or by:
  //   w_WFq = Jr_WFq⋅q̇
  //   v_WFq = Jt_WFq⋅q̇
  // when computed in terms of the time derivatives of the generalized
  // positions (with_respect_to = JacobianWrtVariable::kQDot).
  //
  // This method provides the option to specify whether angular and/or
  // translational terms need to be computed, however the caller must at least
  // request one of them.
  // If Jr_WFq is nullptr, then angular terms are not computed.
  // If Jt_WFq is nullptr, then translational terms are not computed.
  //
  //
  //               Format of the Jacobian matrix Jr_WFq
  //
  // Notice that, the angular velocity of frame F shifted to a frame Fq with
  // origin at a point Q is the same as that of frame F, for any point Q.
  // That is, w_WFq = w_WF for any point Q. With this in mind, Jr_WFq is
  // defined so that:
  //   w_WFq = w_WF = Jr_WFq⋅v  if with_respect_to = JacobianWrtVariable::kV
  //   w_WFq = w_WF = Jr_WFq⋅q̇  if with_respect_to = JacobianWrtVariable::kQDot
  // and therefore Jr_WFq is a matrix with 3 rows and
  // nv columns if with_respect_to = JacobianWrtVariable::kV or
  // nq columns if with_respect_to = JacobianWrtVariable::kQDot, where nv and nq
  // are the number of generalized velocities and positions, respectively.
  // If not nullptr on input, matrix Jr_WFq **must** have the documented size
  // or this method throws a std::runtime_error exception.
  //
  //               Format of the Jacobian matrix Jt_WFq
  //
  // We stack the translational velocity of each point Q into a column vector
  // v_WFq = [v_WFq1; v_WFq2; ...] of size 3⋅np, with np the number of
  // points in the input list. Then the translational velocities Jacobian is
  // defined such that:
  //   v_WFq = Jt_WFq⋅v, if with_respect_to = JacobianWrtVariable::kV
  //   v_WFq = Jt_WFq⋅q̇, if with_respect_to = JacobianWrtVariable::kQDot
  //
  // Therefore Jt_WFq is a matrix with 3⋅np rows and
  // nv columns if with_respect_to = JacobianWrtVariable::kV or
  // nq columns if with_respect_to = JacobianWrtVariable::kQDot
  // If not nullptr on input, matrix Jt_WFq **must** have the required size or
  // this method throws a std::runtime_error exception.
  //
  // This helper throws std::runtime_error when:
  // - The number of rows in p_WQ_list does not equal three. That is, p_WQ_list
  //   must be a matrix with each column being a 3D vector for each point Q.
  // - Jr_WFq and Jt_WFq are both nullptr (caller must request at least one
  //   Jacobian).
  // - The number of columns of Jr_WFq and/or Jt_WFq does not equal
  //   num_velocities() if with_respect_to = JacobianWrtVariable::kV or
  //   num_positions()  if with_respect_to = JacobianWrtVariable::kQDot.
  // - The number of rows of Jr_WFq does not equal 3.
  // - The number of rows of Jt_WFq does not equal 3⋅np.
  void CalcFrameJacobianExpressedInWorld(
      const systems::Context<T>& context,
      const Frame<T>& frame_F,
      const Eigen::Ref<const MatrixX<T>>& p_WQ_list,
      JacobianWrtVariable with_respect_to,
      EigenPtr<MatrixX<T>> Jr_WFq, EigenPtr<MatrixX<T>> Jt_WFq) const;

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
  Frame<T>* CloneFrameAndAdd(const Frame<FromScalar>& frame);

  // Helper method to create a clone of `body` and add it to `this` tree.
  // Because this method is only invoked in a controlled manner from within
  // CloneToScalar(), it is guaranteed that the cloned body in this variant's
  // `owned_bodies_` will occupy the same position as its corresponding Body
  // in the source variant `body`.
  template <typename FromScalar>
  Body<T>* CloneBodyAndAdd(const Body<FromScalar>& body);

  // Helper method to create a clone of `mobilizer` and add it to `this` tree.
  template <typename FromScalar>
  Mobilizer<T>* CloneMobilizerAndAdd(const Mobilizer<FromScalar>& mobilizer);

  // Helper method to create a clone of `force_element` and add it to `this`
  // tree.
  template <typename FromScalar>
  void CloneForceElementAndAdd(
      const ForceElement<FromScalar>& force_element);

  // Helper method to create a clone of `joint` and add it to `this` tree.
  template <typename FromScalar>
  Joint<T>* CloneJointAndAdd(const Joint<FromScalar>& joint);

  // Helper method to create a clone of `actuator` (which is templated on
  // FromScalar) and add it to `this` tree (templated on T).
  template <typename FromScalar>
  void CloneActuatorAndAdd(
      const JointActuator<FromScalar>& actuator);

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
        dynamic_cast<const BodyType<T>*>(owned_bodies_[body_index].get());
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

  // TODO(russt): Add mutable accessors for other variants as needed.
  template <template <typename> class MobilizerType, typename Scalar>
  MobilizerType<T>& get_mutable_mobilizer_variant(
      const MobilizerType<Scalar>& mobilizer) {
    static_assert(std::is_base_of<Mobilizer<T>, MobilizerType<T>>::value,
                  "MobilizerType<T> must be a sub-class of Mobilizer<T>.");
    // TODO(amcastro-tri):
    //   DRAKE_DEMAND the parent tree of the variant is indeed a variant of this
    //   MultibodyTree. That will require the tree to have some sort of id.
    MobilizerIndex mobilizer_index = mobilizer.index();
    DRAKE_DEMAND(mobilizer_index < num_mobilizers());
    MobilizerType<T>* mobilizer_variant = dynamic_cast<MobilizerType<T>*>(
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
        dynamic_cast<const JointType<T>*>(owned_joints_[joint_index].get());
    DRAKE_DEMAND(joint_variant != nullptr);
    return *joint_variant;
  }

  // Helper function to find the element index for an element in the tree from
  // a multimap of name to index.  It finds the element from any model
  // instance and ensures only one element of that name exists.
  template <typename ElementIndex>
  static ElementIndex GetElementIndex(
      const std::string& name, const std::string& element_description,
      const std::unordered_multimap<std::string, ElementIndex>& name_to_index) {
    const auto range = name_to_index.equal_range(name);
    if (range.first == range.second) {
      std::string lower = element_description;
      std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
      throw std::logic_error("There is no " + lower + " named '" + name +
                             "' in the model.");
    } else if (std::next(range.first) != range.second) {
      throw std::logic_error(element_description + " " + name +
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

  const MultibodyTreeSystem<T>* tree_system_{};
};

}  // namespace internal

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

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::MultibodyTree)
