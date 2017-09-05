#pragma once

#include <limits>
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
#include "drake/multibody/multibody_tree/fixed_offset_frame.h"
#include "drake/multibody/multibody_tree/revolute_mobilizer.h"
#include "drake/multibody/multibody_tree/rigid_body.h"

namespace drake {
namespace multibody {

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;


// Forward declarations.
template<typename T> class MultibodyModeler;

template <typename T>
class Link {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Link)

  Link(const std::string& name) : name_(name) {}

  virtual ~Link() {}

  const std::string& get_name() const { return name_; }

  LinkId get_id() const {
    return id_;
  }

  virtual std::unique_ptr<Body<T>> MakeBody() const = 0;

  /// @cond
  // For internal use only.
  void set_parent_modeler(const MultibodyModeler<T>* parent_modeler, LinkId id)
  {
    id_ = id;
    parent_modeler_ = parent_modeler;
  }
  /// @endcond

 private:
  const MultibodyModeler<T>* parent_modeler_{nullptr};
  LinkId id_;
  std::string name_;
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

  std::unique_ptr<Body<T>> MakeBody() const final {
    return std::make_unique<RigidBody<T>>(M_Lo_default_);
  }

 private:
  const Body<T>* body_;
  // Spatial inertia about the link frame origin Lo, expressed in L.
  SpatialInertia<double> M_Lo_default_;
};

template <typename T>
class Joint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Joint)

  Joint(const std::string& name,
        const Link<T>& parent, const Isometry3<double> X_PJp,
        const Link<T>& child, const Isometry3<double> X_CJc) :
      name_(name), parent_(parent), child_(child) {
    if (!X_PJp.matrix().isIdentity(std::numeric_limits<double>::epsilon())) {
      X_PJp_ = X_PJp;
    }
    if (!X_CJc.matrix().isIdentity(std::numeric_limits<double>::epsilon())) {
      X_CJc_ = X_CJc;
    }
  }

  virtual ~Joint() {}

  const std::string& get_name() const { return name_; }

  JointId get_id() const {
    return id_;
  }

  const Link<T>& get_parent_link() const { return parent_; }
  const Link<T>& get_child_link() const { return child_; }

  optional<Isometry3<double>> get_X_PJp() const { return X_PJp_; }

  optional<Isometry3<double>> get_X_CJc() const { return X_CJc_; }

  /// @cond
  // For internal use only.
  void set_parent_modeler(const MultibodyModeler<T>* parent_modeler, JointId id)
  {
    id_ = id;
    parent_modeler_ = parent_modeler;
  }

  void SetImplementation(const Mobilizer<T>* impl) {
    DoSetImplementation(impl);
  }
  /// @endcond

  virtual std::unique_ptr<Mobilizer<T>> MakeMobilizer(
      const Frame<T>& inboard_frame, const Frame<T>& outboard_frame) const = 0;

 private:
  virtual void DoSetImplementation(const Mobilizer<T>* impl) = 0;

  const MultibodyModeler<T>* parent_modeler_;
  JointId id_;

  std::string name_;
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
      const std::string& name,
      const Link<T>& parent, const Isometry3<double> X_PJp,
      const Link<T>& child, const Isometry3<double> X_CJc,
      const Vector3<double>& axis_Jp) :
      Joint<T>(name, parent, X_PJp, child, X_CJc), axis_Jp_(axis_Jp) {}

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
    return GetMobilizerOrThrow().get_angle(context);
  }

  const T& get_angular_rate(const Context<T>& context) const {
    // TODO(amcastro-tri): expand this capability for the case when the joint
    // is implemented as a constraint.
    return GetMobilizerOrThrow().get_angular_rate(context);
  }

  const RevoluteJoint<T>& set_angular_rate(
      Context<T>* context, const T& angle) const {
    // TODO(amcastro-tri): expand this capability for the case when the joint
    // is implemented as a constraint.
    GetMobilizerOrThrow().set_angular_rate(context, angle);
    return *this;
  }

  /// Sets the `context` so that the generalized coordinate corresponding to the
  /// rotation angle of `this` mobilizer equals `angle`.
  /// @throws std::logic_error if `context` is not a valid
  /// MultibodyTreeContext.
  /// @param[in] context The context of the MultibodyTree this mobilizer
  ///                    belongs to.
  /// @param[in] angle The desired angle in radians.
  /// @returns a constant reference to `this` mobilizer.
  const RevoluteJoint<T>& set_angle(
      Context<T>* context, const T& angle) const {
    // TODO(amcastro-tri): expand this capability for the case when the joint
    // is implemented as a constraint.
    GetMobilizerOrThrow().set_angle(context, angle);
    return *this;
  }

  virtual std::unique_ptr<Mobilizer<T>> MakeMobilizer(
      const Frame<T>& inboard_frame, const Frame<T>& outboard_frame) const final
  {
    return std::make_unique<RevoluteMobilizer<T>>(
        inboard_frame, outboard_frame, axis_Jp_);
  }

 private:
  const RevoluteMobilizer<T>& GetMobilizerOrThrow() const {
    if(mobilizer_ == nullptr) {
      throw std::logic_error(
          "You must finalize your model before attempting this request");
    }
    return *mobilizer_;
  }

  void DoSetImplementation(const Mobilizer<T>* impl) final {
    mobilizer_ = dynamic_cast<const RevoluteMobilizer<T>*>(impl);
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

  template<typename Scalar>
  using Context = systems::Context<Scalar>;

  /// Creates a MultibodyModeler containing only a **world** body.
  MultibodyModeler() {
    // Adds a "world" body to MultibodyTree having a NaN SpatialInertia.
    model_state_.world_id =
        AddLink<RigidLink>("World", SpatialInertia<double>()).get_id();
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
    return static_cast<int>(model_state_.owned_links.size());
  }

  const Link<T>& get_world_link() const {
    return *model_state_.owned_links.at(model_state_.world_id);
  }

  int get_num_joints() const;

  /// Returns the number of generalized positions of the model.
  int get_num_positions() const {
    if (model_state_.multibody_tree)
      return model_state_.multibody_tree->get_num_positions();
    else
      return 0;
  }

  /// Returns the number of generalized velocities of the model.
  int get_num_velocities() const {
    if (model_state_.multibody_tree)
      return model_state_.multibody_tree->get_num_velocities();
    else
      return 0;
  }

  /// Returns the total size of the state vector in the model.
  int get_num_states() const {
    if (model_state_.multibody_tree)
      return model_state_.multibody_tree->get_num_states();
    else
      return 0;
  }

  bool is_finalized() const { return model_state_.is_finalized; }

  void set_finalized(bool is_finalized = true) const {
    model_state_.is_finalized = is_finalized;
  }

  const MultibodyTree<T>& get_multibody_tree_model() const {
    FinalizeOnlyIfNeeded();
    return *model_state_.multibody_tree;
  }

  MultibodyTree<T>& get_mutable_multibody_tree_model() const {
    // Finalize? or remove completely?
    return *model_state_.multibody_tree;
  }

  /// Retrieve the Body<T> associated with a Link<T>. It assumes that there is
  /// only one body associated with the given link.
  const Body<T>& get_link_body(LinkId link_id) const;

  /// Retrieve the Body<T> associated with a Link<T>. It assumes that there is
  /// only one body associated with the given link.
  const Body<T>& get_link_body(const Link<T>& link) const {
    return get_link_body(link.get_id());
  }

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
  std::unique_ptr<systems::LeafContext<T>> CreateDefaultContext() const {
    FinalizeOnlyIfNeeded();
    return model_state_.multibody_tree->CreateDefaultContext();
  }

  /// Sets default values in the context. For mobilizers, this method sets them
  /// to their _zero_ configuration according to
  /// Mobilizer::set_zero_configuration().
  void SetZeroConfiguration(systems::Context<T>* context) const;

  void CalcMassMatrixViaInverseDynamics(
      const Context<T>& context, Eigen::Ref<MatrixX<T>> H) const;

  void CalcBiasTerm(
      const Context<T>& context, Eigen::Ref<VectorX<T>> C) const;

  void Finalize() const {
    // Discard previous model and create a new one.
    MakeMultibodyTreeModel();
    set_finalized(true);
  }

 private:
  template <template<typename Scalar> class LinkType>
  const LinkType<T>& AddLink(std::unique_ptr<LinkType<T>> link) {
    static_assert(std::is_convertible<LinkType<T>*, Link<T>*>::value,
                  "LinkType must be a sub-class of Link<T>.");
    set_finalized(false);

    if (link == nullptr) {
      throw std::logic_error("Input link is a nullptr.");
    }

    LinkId link_id = LinkId::get_new_id();
    link->set_parent_modeler(this, link_id);
    const LinkType<T>* raw_link_ptr = link.get();
    model_state_.owned_links[link_id] = std::move(link);
    return *raw_link_ptr;
  }

  template <template<typename Scalar> class JointType>
  const JointType<T>& AddJoint(std::unique_ptr<JointType<T>> joint) {
    static_assert(std::is_convertible<JointType<T>*, Joint<T>*>::value,
                  "JointType must be a sub-class of Joint<T>.");
    set_finalized(false);

    if (joint == nullptr) {
      throw std::logic_error("Input joint is a nullptr.");
    }

    JointId joint_id = JointId::get_new_id();
    joint->set_parent_modeler(this, joint_id);
    const JointType<T>* raw_joint_ptr = joint.get();
    model_state_.owned_joints[joint_id] = std::move(joint);
    return *raw_joint_ptr;
  }

  // Finalizes the MultibodyTree model but only if the modeler is not finalized.
  void FinalizeOnlyIfNeeded() const {
    // Only finalize if not in finalized state.
    if(!is_finalized()) Finalize();
  }

  void MakeMultibodyTreeModel() const;

  struct ModelState {
    // The modeler owns the underlying MultibodyTree. The tree model can be reset
    // if the modeler changes and needs to be re-finalized.
    std::unique_ptr<MultibodyTree<T>> multibody_tree;
    bool is_finalized{false};

    // Id of the "world" link.
    LinkId world_id;

    // List of links owned by the modeler accessed by their id.
    std::unordered_map<LinkId, std::unique_ptr<Link<T>>> owned_links;

    // List of joints owned by the modeler accessed by their id.
    std::unordered_map<JointId, std::unique_ptr<Joint<T>>> owned_joints;

    // Set of Body objects in the MultibodyTree forming each Link model.
    // Each Link could be modeled by multiple "ghost" bodies.
    std::unordered_map<LinkId, std::unordered_set<BodyIndex>>
        link_id_to_body_index_map;

    // This *implicitly* maps the index of each body in the MultibodyTree engine
    // to its corresponding unique link identifier in the modeler.
    // It assumes that the index in this vector *is* the index in the underlying
    // MultibodyTree.
    // The following invariants should always be true:
    //   1. body_index_to_link_id_map_.size() ==
    //      multibody_tree_->get_num_bodies().
    std::vector<LinkId> body_index_to_link_id_map;
  };
  // The ModelState member is made mutable so we can call it within the
  // const Finalize() method.
  // Another options would be to place this "finalization state" within the
  // Modeler's Context.
  mutable ModelState model_state_;
};

}  // namespace multibody
}  // namespace drake
