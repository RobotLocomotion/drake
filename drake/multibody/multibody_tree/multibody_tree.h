#pragma once

#include <memory>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/body_node.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
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

  /// Returns the number of mobilizers in the %MultibodyTree. Since the world
  /// has no Mobilizer, the number of mobilizers equals the number of bodies
  /// minus one, i.e. get_num_mobilizers() returns get_num_bodies() - 1.
  // TODO(amcastro-tri): Consider adding a WorldMobilizer (0-dofs) for the world
  // body. This could be useful to query for reaction forces of the entire
  // model.
  int get_num_mobilizers() const {
    return static_cast<int>(owned_mobilizers_.size());
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
  const Body<T>& get_world_body() const {
    return *owned_bodies_[world_index()];
  }

  /// Returns a constant reference to the *world* frame.
  const BodyFrame<T>& get_world_frame() const {
    return owned_bodies_[world_index()]->get_body_frame();
  }

  /// Returns a constant reference to the body with unique index `body_index`.
  /// This method aborts in Debug builds when `body_index` does not correspond
  /// to a body in this multibody tree.
  const Body<T>& get_body(BodyIndex body_index) const {
    DRAKE_ASSERT(body_index < get_num_bodies());
    return *owned_bodies_[body_index];
  }

  /// Returns a constant reference to the body with unique index `body_index`.
  /// This method aborts in Debug builds when `body_index` does not correspond
  /// to a body in this multibody tree.
  const Frame<T>& get_frame(FrameIndex frame_index) const {
    DRAKE_ASSERT(frame_index < get_num_frames());
    return *frames_[frame_index];
  }

  /// Returns a constant reference to the mobilizer with unique index
  /// `mobilizer_index`. This method aborts in Debug builds when
  /// `mobilizer_index` does not correspond to a mobilizer in this multibody
  /// tree.
  const Mobilizer<T>& get_mobilizer(MobilizerIndex mobilizer_index) const {
    DRAKE_ASSERT(mobilizer_index < get_num_mobilizers());
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
  // TODO(amcastro-tri): Split this method into implementations to be used by
  // System::AllocateContext() so that MultibodyPlant() can make use of it
  // within the system's infrastructure. This will require at least the
  // introduction of system's methods to:
  //  - Create a context different from LeafContext, in this case MBTContext.
  //  - Create or request cache entries.
  std::unique_ptr<systems::Context<T>> CreateDefaultContext() const;

  /// Sets default values in the context. For mobilizers, this method sets them
  /// to their _zero_ configuration according to
  /// Mobilizer::set_zero_configuration().
  void SetDefaults(systems::Context<T>* context) const;

  /// Computes into the position kinematics `pc` all the kinematic quantities
  /// that depend on the generalized positions only. These include:
  /// - For each body B, the pose `X_BF` of each of the frames F attached to
  ///   body B.
  /// - Pose `X_WB` of each body B in the model as measured and expressed in
  ///   the world frame W.
  /// - Across-mobilizer Jacobian matrices `H_FM` and `H_PB_W`.
  /// - Body specific quantities such as `com_W` and `M_Bo_W`.
  ///
  /// @throws std::bad_cast if `context` is not a `MultibodyTreeContext`.
  /// @throws std::runtime_error if `pc` is the nullptr.
  void CalcPositionKinematicsCache(
      const systems::Context<T>& context,
      PositionKinematicsCache<T>* pc) const;

  /// @name Methods to retrieve multibody element variants.
  ///
  /// Given the user already has possession of a %MultibodyTree templated on a
  /// scalar type T, a variant is a version of that %MultibodyTree templated on
  /// a different scalar type `Tvariant`.
  /// A typical use case is the call to ToAutoDiffXd() to obtain a
  /// %MultibodyTree templated on AutoDiffXd from a %MultibodyTree templated on
  /// `double`.
  /// If a user has a given multibody tree, templated on a scalar type T,
  /// and created a variant tree templated on another scalar
  /// type `Tvariant`, the methods in this section provide a mechanism to
  /// retrieve multibody elements from the variant tree that correspond to
  /// elements in the original tree templated on T.
  /// Considere the following code example:
  /// @code
  ///   // The user creates a model.
  ///   MultibodyTree<T> model;
  ///   // User adds a body and keeps a reference to it.
  ///   const RigidBody<T>& body = model.AddBody<RigidBody>(...);
  ///   // User creates a variant. ToAutoDiffXd() is available for
  ///   // Tvariant = AutoDiffXd.
  ///   std::unique_ptr<MultibodyTree<Tvariant>> variant_model =
  ///       CloneToScalar<Tvariant>();
  ///   // User retrieves the variant corresponding to the body added above.
  ///   const RigidBody<AutoDiffXd>&
  ///       variant_body = variant_model.retrieve_variant(body);
  /// @endcode
  ///
  /// MultibodyTree::retrieve_variant() is templated on the multibody element
  /// type which is deduced from its only input argument. The returned element
  /// is templated on the scalar type T of the %MultibodyTree on which this
  /// method is invoked.
  /// @{

  /// SFINAE overload for Body<T> elements.
  template <template <typename> class MultibodyElement, typename Scalar>
  std::enable_if_t<std::is_base_of<Body<T>, MultibodyElement<T>>::value,
                   const MultibodyElement<T>&> retrieve_variant(
      const MultibodyElement<Scalar>& element) {
    return retrieve_body_variant(element);
  }

  /// SFINAE overload for Mobilizer<T> elements.
  template <template <typename> class MultibodyElement, typename Scalar>
  std::enable_if_t<std::is_base_of<Mobilizer<T>, MultibodyElement<T>>::value,
                   const MultibodyElement<T>&> retrieve_variant(
      const MultibodyElement<Scalar>& element) {
    return retrieve_mobilizer_variant(element);
  }
  /// @}

  /// Creates an exact clone of `this` %MultibodyTree templated on the same
  /// scalar type T as `this` tree.
  std::unique_ptr<MultibodyTree<T>> Clone() const {
    return CloneToScalar<T>();
  }

  /// Creates a deep copy of `this` %MultibodyTree templated on the same
  /// scalar type T as `this` tree.
  std::unique_ptr<MultibodyTree<AutoDiffXd>> ToAutoDiffXd() const {
    return CloneToScalar<AutoDiffXd>();
  }

  /// Creates a deep copy of `this` %MultibodyTree templated on the scalar type
  /// `ToScalar`.
  /// The new deep copy is guaranteed to have exactly the same
  /// MultibodyTreeTopology as the original tree the method is called on.
  template <typename ToScalar>
  std::unique_ptr<MultibodyTree<ToScalar>> CloneToScalar() const {
    auto tree_clone = std::make_unique<MultibodyTree<ToScalar>>();

    tree_clone->frames_.resize(get_num_frames());
    for (const auto& body : owned_bodies_) {
      // Skip cloning the world since it is automatically created by
      // MultibodyTree's constructor.
      if (body->get_index() == world_index()) continue;
      tree_clone->CloneBodyAndAdd(*body);
    }

    for (const auto& frame : owned_frames_) {
      tree_clone->CloneFrameAndAdd(*frame);
    }

    for (const auto& mobilizer : owned_mobilizers_) {
      // This call assumes that tree_clone already contains all the cloned
      // frames.
      tree_clone->CloneMobilizerAndAdd(*mobilizer);
    }

    tree_clone->topology_ = this->topology_;
    tree_clone->FinalizeInternals();
    return tree_clone;
  }

 private:
  // Make MultibodyTree templated on any other scalar type a friend of
  // MultibodyTree<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private methods from MultibodyTree<T>.
  template <typename> friend class MultibodyTree;

  // Finalizes the MultibodyTreeTopology of this tree.
  void FinalizeTopology();
  // At Finalize(), this method performs all other finalization that is not
  // topological (i.e. performed by FinalizeTopology()). This includes for
  // instance the creation of BodyNode objects.
  void FinalizeInternals();

  void CreateBodyNode(BodyNodeIndex body_node_index);

  // Helper method to create a clone of `frame` and add it to `this` tree.
  template <typename FromScalar>
  Frame<T>* CloneFrameAndAdd(const Frame<FromScalar>& frame) {
    FrameIndex frame_index = frame.get_index();

    auto frame_clone = frame.CloneToScalar(*this);
    frame_clone->set_parent_tree(this, frame_index);

    Frame<T>* raw_frame_clone_ptr = frame_clone.get();
    frames_[frame_index] = raw_frame_clone_ptr;
    owned_frames_.push_back(std::move(frame_clone));
    return raw_frame_clone_ptr;
  }

  // Helper method to create a clone of `body` and add it to `this` tree.
  template <typename FromScalar>
  Body<T>* CloneBodyAndAdd(const Body<FromScalar>& body) {
    BodyIndex body_index = body.get_index();
    FrameIndex body_frame_index = body.get_body_frame().get_index();

    auto body_clone = body.CloneToScalar(*this);
    body_clone->set_parent_tree(this, body_index);
    // MultibodyTree can access selected private methods in Body through its
    // BodyAttorney.
    Frame<T>* body_frame_clone =
        &internal::BodyAttorney<T>::get_mutable_body_frame(body_clone.get());
    body_frame_clone->set_parent_tree(this, body_frame_index);
    frames_[body_frame_index] = body_frame_clone;
    Body<T>* raw_body_clone_ptr = body_clone.get();
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

  // Helper method to retrieve the corresponding Body<T> variant to a Body in a
  // MultibodyTree variant templated on Scalar.
  template <template <typename> class BodyType, typename Scalar>
  const BodyType<T>& retrieve_body_variant(const BodyType<Scalar>& body) {
    static_assert(std::is_convertible<BodyType<T>*, Body<T>*>::value,
                  "BodyType must be a sub-class of Body<T>.");
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
  const MobilizerType<T>& retrieve_mobilizer_variant(
      const MobilizerType<Scalar>& mobilizer) {
    static_assert(std::is_convertible<MobilizerType<T>*, Mobilizer<T>*>::value,
                  "MobilizerType must be a sub-class of Mobilizer<T>.");
    MobilizerIndex mobilizer_index = mobilizer.get_index();
    DRAKE_DEMAND(mobilizer_index < get_num_mobilizers());
    const MobilizerType<T>* mobilizer_variant =
        dynamic_cast<const MobilizerType<T>*>(
            owned_mobilizers_[mobilizer_index].get());
    DRAKE_DEMAND(mobilizer_variant != nullptr);
    return *mobilizer_variant;
  }

  // TODO(amcastro-tri): In future PR's adding MBT computational methods, write
  // a method that verifies the state of the topology with a signature similar
  // to RoadGeometry::CheckInvariants().

  std::vector<std::unique_ptr<Body<T>>> owned_bodies_;
  std::vector<std::unique_ptr<Frame<T>>> owned_frames_;
  std::vector<std::unique_ptr<Mobilizer<T>>> owned_mobilizers_;
  std::vector<std::unique_ptr<internal::BodyNode<T>>> body_nodes_;

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
