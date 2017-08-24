#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/multibody/multibody_tree/modeler/modeler_ids.h"
//#include "drake/multibody/multibody_tree/modeler/link.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/systems/framework/context.h"
#include "drake/multibody/multibody_tree/revolute_mobilizer.h"

namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class MultibodyModeler;

template <typename T>
class Link {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Link)

  Link(const std::string& name) : name_(name) {}

  LinkId get_id() const {
    return id_;
  }

  /// @cond
  // For internal use only.
  void set_parent_modeler(const MultibodyModeler<T>* parent_modeler, LinkId id)
  {
    id_ = id;
    parent_modeler_ = parent_modeler;
  }
  /// @endcond

 private:
  std::string name_;
  const MultibodyModeler<T>* parent_modeler_{nullptr};
  LinkId id_;
};

template <typename T>
class RigidLink : public Link<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidLink)

  /// Constructs a %RigidLink with the given default SpatialInertia.
  /// @param[in] M_LLo_L
  ///   Spatial inertia of `this` link L about its frame's L origin `Lo` and
  ///   expressed in the link frame L.
  /// @note See @ref multibody_spatial_inertia for details on the monogram
  /// notation used for spatial inertia quantities.
  RigidLink(const std::string& name,
            const SpatialInertia<double>& M_LLo_L) :
      Link<T>(name), M_Lo_default_(M_LLo_L) {}

 private:
  const Body<T>* body_;
  // Spatial inertia about the link frame origin Lo, expressed in L.
  SpatialInertia<double> M_Lo_default_;
};

template <typename T>
class Joint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Joint)

  Joint(const Link<T>& parent, const Isometry3<double> X_PJp,
        const Link<T>& child, const Isometry3<double> X_CJc) :
      parent_(parent), child_(child), X_PJp_(X_PJp), X_CJc_(X_CJc) {}

  /// Constructor for the common case when joint frame attached on the child
  /// link **is** the link's frame.
  Joint(const Link<T>& parent, const Isometry3<double> X_PJp,
        const Link<T>& child) :
      parent_(parent), child_(child), X_PJp_(X_PJp) {}

  /// @cond
  // For internal use only.
  void set_parent_modeler(const MultibodyModeler<T>*, JointId id);
  /// @endcond

 private:
  const MultibodyModeler<T>* parent_modeler_;
  JointId id_;
  const Link<T>& parent_;
  const Link<T>& child_;
  // The pose of the joint frame J1 in link frame L1 is optional, meaning that
  // frame J1 IS frame L1.
  optional<Isometry3<double>> X_PJp_;
  // The pose of the joint frame J2 in link frame L2 is optional, meaning that
  // frame J2 IS frame L2.
  optional<Isometry3<double>> X_CJc_;
};

template <typename T>
class RevoluteJoint : public Joint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RevoluteJoint)

  template<typename Scalar>
  using Context = systems::Context<Scalar>;

  RevoluteJoint(
      const Link<T>& parent, const Isometry3<double> X_PJp,
      const Link<T>& child, const Isometry3<double> X_CJc,
      const Vector3<double>& axis_Jp) :
      Joint<T>(parent, X_PJp, child, X_CJc), axis_Jp_(axis_Jp) {}

  /// Constructor for the common case when joint frame attached on the child
  /// link **is** the link's frame.
  RevoluteJoint(
      const Link<T>& parent, const Isometry3<double> X_PJp,
      const Link<T>& child, const Vector3<double>& axis_Jp) :
      Joint<T>(parent, X_PJp, child), axis_Jp_(axis_Jp) {}

  /// Returns the axis of revolution of `this` joint as a unit vector expressed
  /// in the frame `Jp` attached on the parent body P.
  const Vector3<double>& get_revolute_axis() const {
    return axis_Jp_;
  }

  /// Gets the rotation angle of `this` mobilizer from `context`. See class
  /// documentation for sign convention.
  /// @throws std::logic_error if the parent MultibodyModeler of `this` joint
  /// was not finalized, @see MultibodyModeler::Finalize().
  /// @param[in] context The context of the MultibodyTree this mobilizer
  ///                    belongs to.
  /// @returns The angle coordinate of `this` mobilizer in the `context`.
  const T& get_angle(const Context<T>& context) const {
    // TODO(amcastro-tri): expand this capability for the case when the joint
    // is implemented as a constraint.
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
      Context<T>* context, const T& angle) const {
    // TODO(amcastro-tri): expand this capability for the case when the joint
    // is implemented as a constraint.
    GetMobilizerOrThrow().set_angle(context, angle);
  }

 private:
  const RevoluteMobilizer<T>& GetMobilizerOrThrow() const {
    if(mobilizer_ == nullptr) {
      throw std::logic_error(
          "You must finalize your model before attempting this request");
    }
    return *mobilizer_;
  }

  const Vector3<double> axis_Jp_;
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
  MultibodyModeler() {
    // Adds a "world" body to MultibodyTree having a NaN SpatialInertia.
    world_id_ = AddLink<RigidLink>("World", SpatialInertia<double>()).get_id();
  }

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


  template<template<typename Scalar> class JointType, typename... Args>
  const JointType<T>& AddJoint(Args&&... args) {
    static_assert(std::is_convertible<JointType<T>*, Joint<T>*>::value,
                  "JointType must be a sub-class of Joint<T>.");
    return AddJoint(std::make_unique<JointType<T>>(
        std::forward<Args>(args)...));
  }
  /// @}
  // Closes Doxygen section.

  int get_num_links() const {
    return static_cast<int>(owned_links_.size());
  }

  const Link<T>& get_world_link() const {
    return *owned_links_.at(world_id_);
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
    const LinkType<T>* raw_link_ptr = link.get();
    owned_links_[link_id] = std::move(link);
    return *raw_link_ptr;
  }

  template <template<typename Scalar> class JointType>
  const JointType<T>& AddJoint(std::unique_ptr<JointType<T>> joint) {
    static_assert(std::is_convertible<JointType<T>*, Joint<T>*>::value,
                  "JointType must be a sub-class of Joint<T>.");
    is_finalized_ = false;

    if (joint == nullptr) {
      throw std::logic_error("Input joint is a nullptr.");
    }

    JointId joint_id = JointId::get_new_id();
    joint->set_parent_modeler(this, joint_id);
    const JointType<T>* raw_joint_ptr = joint.get();
    owned_joints_[joint_id] = std::move(joint);
    return *raw_joint_ptr;
  }

  // The modeler owns the underlying MultibodyTree. The tree model can be reset
  // if the modeler changes and needs to be re-finalized.
  std::unique_ptr<MultibodyTree<T>> multibody_tree_;

  // List of links owned by the modeler accessed by their id.
  std::unordered_map<LinkId, std::unique_ptr<Link<T>>> owned_links_;
  LinkId world_id_;

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

  // List of joints owned by the modeler accessed by their id.
  std::unordered_map<JointId, std::unique_ptr<Joint<T>>> owned_joints_;

  bool is_finalized_{false};
};

}  // namespace multibody
}  // namespace drake
