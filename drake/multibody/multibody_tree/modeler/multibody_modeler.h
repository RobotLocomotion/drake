#pragma once

#include <memory>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/modeler/modeler_ids.h"
//#include "drake/multibody/multibody_tree/modeler/link.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/systems/framework/context.h"
#include "drake/multibody/multibody_tree/revolute_mobilizer.h"

namespace drake {
namespace multibody {

template <typename T>
class Link {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Link)

  Link();

  void set_parent_modeler(const MultibodyModeler<T>*, LinkId id);

 private:
  std::string name_;
};

template <typename T>
class RigidLink : public Link<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidLink)

  RigidLink();

 private:
  const Body<T>* body_;
};

template <typename T>
class Joint {
 public:
  Joint(
      const Link<T>& link1, const Isometry3<double> X_L1J1,
      const Link<T>& link2, const Isometry3<double> X_L2J2) {}

  void set_parent_modeler(const MultibodyModeler<T>*, JointId id);

 private:
  const MultibodyModeler<T>* parent_modeler_;
  JointId id_;
};

template <typename T>
class RevoluteJoint : public Joint<T> {
 public:
  RevoluteJoint(
      const Link<T>& link1, const Link<T>& link2,
      const Vector3<double>& axis_F) :
      Joint(link1, link2), axis_F_(axis_F) {

}

  /// @retval axis_F The rotation axis as a unit vector expressed in the inboard
  ///                frame F.
  const Vector3<double>& get_revolute_axis() const {
    return axis_F_;
  }

  /// Gets the rotation angle of `this` mobilizer from `context`. See class
  /// documentation for sign convention.
  /// @throws std::logic_error if `context` is not a valid
  /// MultibodyTreeContext.
  /// @param[in] context The context of the MultibodyTree this mobilizer
  ///                    belongs to.
  /// @returns The angle coordinate of `this` mobilizer in the `context`.
  const T& get_angle(const systems::Context<T>& context) const {
    GetMobilizerOrThrow().get_angle(context);
  }

  /// Sets the `context` so that the generalized coordinate corresponding to the
  /// rotation angle of `this` mobilizer equals `angle`.
  /// @throws std::logic_error if `context` is not a valid
  /// MultibodyTreeContext.
  /// @param[in] context The context of the MultibodyTree this mobilizer
  ///                    belongs to.
  /// @param[in] angle The desired angle in radians.
  /// @returns a constant reference to `this` mobilizer.
  const RevoluteMobilizer<T>& set_angle(
      systems::Context<T>* context, const T& angle) const;

 private:
  const RevoluteMobilizer<T>& GetMobilizerOrThrow() const {
    if(mobilizer_ == nullptr) {
      throw std::logic_error(
          "You must finalize your model before attempting this request");
    }
    return *mobilizer_;
  }

  const Vector3<double> axis_F_;
  const RevoluteMobilizer<T>* mobilizer_{nullptr};
};

/// %MultibodyModeler provides a representation for a physical system consisting of
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
class MultibodyModeler {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyModeler)

  /// Creates a MultibodyModeler containing only a **world** body.
  MultibodyModeler();

  /// Constructs a new body with type `BodyType` with the given `args`, and adds
  /// it to `this` %MultibodyModeler, which retains ownership. The `BodyType` will
  /// be specialized on the scalar type T of this %MultibodyModeler.
  ///
  /// Example of usage:
  /// @code
  ///   MultibodyModeler<T> model;
  ///   // ... Code to define spatial_inertia, a SpatialInertia<T> object ...
  ///   // Notice RigidBody is a template on a scalar type.
  ///   const RigidBody<T>& body = model.AddBody<RigidBody>(spatial_inertia);
  /// @endcode
  ///
  /// Note that for dependent names you must use the template keyword (say for
  /// instance you have a MultibodyModeler<T> member within your custom class):
  ///
  /// @code
  ///   MultibodyModeler<T> model;
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
  ///          %MultibodyModeler.
  ///
  /// @tparam BodyType A template for the type of Body to construct. The
  ///                  template will be specialized on the scalar type T of this
  ///                  %MultibodyModeler.
  template<template<typename Scalar> class LinkType, typename... Args>
  const LinkType<T>& AddLink(Args&&... args) {
    static_assert(std::is_convertible<LinkType<T>*, Link<T>*>::value,
                  "LinkType must be a sub-class of Link<T>.");
    return AddLink(std::make_unique<LinkType<T>>(std::forward<Args>(args)...));
  }

#if 0
  /// Takes ownership of `mobilizer` and adds it to `this` %MultibodyModeler.
  /// Returns a constant reference to the mobilizer just added, which will
  /// remain valid for the lifetime of `this` %MultibodyModeler.
  ///
  /// Example of usage:
  /// @code
  ///   MultibodyModeler<T> model;
  ///   // ... Code to define inboard and outboard frames by calling
  ///   // MultibodyModeler::AddFrame() ...
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
  ///                      %MultibodyModeler. The mobilizer class must be
  ///                      specialized on the same scalar type T as this
  ///                      %MultibodyModeler. Notice this is a requirement of this
  ///                      method's signature and therefore an input mobilzer
  ///                      specialized on a different scalar type than that of
  ///                      this %MultibodyModeler's T will fail to compile.
  /// @returns A constant reference of type `MobilizerType` to the created
  ///          mobilizer. This reference which will remain valid for the
  ///          lifetime of `this` %MultibodyModeler.
  ///
  /// @tparam MobilizerType The type of the specific sub-class of Mobilizer to
  ///                       add. The template needs to be specialized on the
  ///                       same scalar type T of this %MultibodyModeler.
  template <template<typename Scalar> class MobilizerType>
  const MobilizerType<T>& AddMobilizer(
      std::unique_ptr<MobilizerType<T>> mobilizer) {
    static_assert(std::is_convertible<MobilizerType<T>*, Mobilizer<T>*>::value,
                  "MobilizerType must be a sub-class of mobilizer<T>.");
    if (topology_is_valid()) {
      throw std::logic_error("This MultibodyModeler is finalized already. "
                             "Therefore adding more bodies is not allowed. "
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
    // all. Consider also removing MultibodyModelerElement altogether.
    mobilizer->set_parent_tree(this, mobilizer_index);

    MobilizerType<T>* raw_mobilizer_ptr = mobilizer.get();
    owned_mobilizers_.push_back(std::move(mobilizer));
    return *raw_mobilizer_ptr;
  }

  /// Constructs a new mobilizer with type `MobilizerType` with the given
  /// `args`, and adds it to `this` %MultibodyModeler, which retains ownership.
  /// The `MobilizerType` will be specialized on the scalar type T of this
  /// %MultibodyModeler.
  ///
  /// Example of usage:
  /// @code
  ///   MultibodyModeler<T> model;
  ///   // ... Code to define inboard and outboard frames by calling
  ///   // MultibodyModeler::AddFrame() ...
  ///   // Notice RevoluteMobilizer is a template an a scalar type.
  ///   const RevoluteMobilizer<T>& pin =
  ///     model.template AddMobilizer<RevoluteMobilizer>(
  ///       inboard_frame, outboard_frame,
  ///       Vector3d::UnitZ() /*revolute axis*/);
  /// @endcode
  ///
  /// Note that for dependent names _only_ you must use the template keyword
  /// (say for instance you have a MultibodyModeler<T> member within your custom
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
  ///          lifetime of `this` %MultibodyModeler.
  ///
  /// @tparam MobilizerType A template for the type of Mobilizer to construct.
  ///                       The template will be specialized on the scalar type
  ///                       T of `this` %MultibodyModeler.
  template<template<typename Scalar> class MobilizerType, typename... Args>
  const MobilizerType<T>& AddMobilizer(Args&&... args) {
    static_assert(std::is_base_of<Mobilizer<T>, MobilizerType<T>>::value,
                  "MobilizerType must be a sub-class of Mobilizer<T>.");
    return AddMobilizer(
        std::make_unique<MobilizerType<T>>(std::forward<Args>(args)...));
  }
#endif
  /// @}
  // Closes Doxygen section.

  int get_num_links() const {
    return static_cast<int>(owned_links_.size());
  }

  int get_num_joints() const;

  /// Returns the number of generalized positions of the model.
  int get_num_positions() const {
    if (multibody_tree_)
      return multibody_tree_->get_num_positions();
    else
      return 0;
  }

  /// Returns the number of generalized velocities of the model.
  int get_num_velocities() const {
    if (multibody_tree_)
      return multibody_tree_->get_num_velocities();
    else
      return 0;
  }

  bool is_finalized() const;

  /// This method must be called after all elements in the tree (joints, bodies,
  /// force elements, constraints) were added and before any computations are
  /// performed.
  /// It essentially compiles all the necessary "topological information", i.e.
  /// how bodies, joints and, any other elements connect with each other, and
  /// performs all the required pre-processing to perform computations at a
  /// later stage.
  ///
  /// If the finalize stage is successful, the topology of this %MultibodyModeler
  /// is validated, meaning that the topology is up-to-date after this call.
  /// No more multibody tree elements can be added after a call to Finalize().
  ///
  /// @throws std::logic_error If users attempt to call this method on an
  ///         already finalized %MultibodyModeler.
  // TODO(amcastro-tri): Consider making this method private and calling it
  // automatically when CreateDefaultContext() is called.
  void Finalize();

  /// Allocates a new context for this %MultibodyModeler uniquely identifying the
  /// state of the multibody system.
  ///
  /// @pre The method Finalize() must be called before attempting to create a
  /// context in order for the %MultibodyModeler topology to be valid at the moment
  /// of allocation.
  ///
  /// @throws std::logic_error If users attempt to call this method on a
  ///         %MultibodyModeler with an invalid topology.
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
  void SetZeroConfiguration(systems::Context<T>* context) const;

 private:
  template <template<typename Scalar> class LinkType>
  const LinkType<T>& AddLink(std::unique_ptr<LinkType<T>> link) {
    static_assert(std::is_convertible<LinkType<T>*, Link<T>*>::value,
                  "LinkType must be a sub-class of Link<T>.");
    is_finalized_ = false;

    if (link == nullptr) {
      throw std::logic_error("Input link is a nullptr.");
    }

    LinkId link_id = LinkId::get_new_id();
    link->set_parent_modeler(this, link_id);
    const LinkType* raw_link_ptr = link.get();
    owned_links_[link_id] = std::move(link);
    return *raw_link_ptr;
  }

  // The modeler owns the underlying MultibodyTree. The tree model can be reset
  // if the modeler changes and needs to be re-finalized.
  std::unique_ptr<MultibodyTree<T>> multibody_tree_;

  // List of links owned by the modeler accessed by their id.
  std::unordered_map<LinkId, std::unique_ptr<Link<T>>> owned_links_;

  // Set of Body objects in the MultibodyTree forming each Link model.
  // Each Link could be modeled by multiple "ghost" bodies.
  std::unordered_map<LinkId, std::unordered_set<BodyIndex>>
      link_id_to_body_index_map_;

  // This *implicitly* maps the index of each body in the MultibodyTree engine
  // to its corresponding unique link identifier in the modeler.
  // It assumes that the index in this vector *is* the index in the underlying
  // MultibodyTree.
  // The following invariants should always be true:
  //   1. body_index_to_link_id_map_.size() ==
  //      multibody_tree_->get_num_bodies().
  std::vector<LinkId> body_index_to_link_id_map_;

  bool is_finalized_{false};
};

}  // namespace multibody
}  // namespace drake
