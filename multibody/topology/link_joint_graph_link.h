#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_GRAPH_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/graph.h".
#endif

#include <string>
#include <utility>
#include <vector>

namespace drake {
namespace multibody {
// TODO(sherm1) Promote from internal once API has stabilized: issue #11307.
namespace internal {

/** Represents a %Link in the LinkJointGraph. This includes Links provided via
user input and also those added during forest building as Shadow links created
when we cut a user %Link in order to break a kinematic loop. Links may be
modeled individually or can be combined into Composite Links comprising groups
of Links that were connected by weld joints. */
class LinkJointGraph::Link {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Link)

  /** Returns this %Link's unique index in the graph. */
  BodyIndex index() const { return index_; }

  /** Returns this %Link's model instance. */
  ModelInstanceIndex model_instance() const { return model_instance_; }

  /** Returns this %Link's name, unique within model_instance(). */
  const std::string& name() const { return name_; }

  /** Returns indexes of all the Joints that connect to this %Link. This is
  the union of joints_as_parent() and joints_as_child(). */
  const std::vector<JointIndex>& joints() const { return joints_; }

  /** Returns indexes of all the Joints that connect to this %Link in which
  this is the parent %Link. */
  const std::vector<JointIndex>& joints_as_parent() const {
    return joints_as_parent_;
  }

  /** Returns indexes of all the joints that connect to this %Link in which
  this is the child %Link. */
  const std::vector<JointIndex>& joints_as_child() const {
    return joints_as_child_;
  }

  /** Returns `true` only if this is the World %Link. Static Links and Links
  in the World Composite are not included; see is_anchored() if you want to
  include everything that is fixed with respect to World. */
  bool is_world() const { return index_ == BodyIndex(0); }

  /** After modeling, returns `true` if this %Link is fixed with respect to
  World. That includes World itself, static Links, and any Link that is part
  of the World Composite (that is, it is directly or indirectly welded to
  World). */
  bool is_anchored() const {
    return is_world() || is_static() ||
           (composite().is_valid() && composite() == LinkCompositeIndex(0));
  }

  /** Returns `true` if this %Link was added with LinkFlags::kStatic. */
  bool is_static() const {
    return static_cast<bool>(flags_ & LinkFlags::kStatic);
  }

  /** Returns `true` if this %Link was added with LinkFlags::kMustBeBaseBody. */
  bool must_be_base_body() const {
    return static_cast<bool>(flags_ & LinkFlags::kMustBeBaseBody);
  }

  /** Returns `true` if this %Link was added with
  LinkFlags::kTreatAsMassless. */
  bool treat_as_massless() const {
    return static_cast<bool>(flags_ & LinkFlags::kTreatAsMassless);
  }

  /** Returns `true` if this is a shadow Link added by BuildForest(). */
  bool is_shadow() const {
    return static_cast<bool>(flags_ & LinkFlags::kShadow);
  }

  /** If this %Link is a shadow, returns the primary %Link it shadows. If
  not a shadow then it is its own primary %Link so returns index(). */
  BodyIndex primary_link() const { return primary_link_; }

  /** If this is a primary %Link (not a shadow) returns the number of Shadow
  Links that were added due to loop breaking. */
  int num_shadows() const { return ssize(shadow_links_); }

  /** Returns the index of the mobilized body (Mobod) that mobilizes this %Link.
  If this %Link is part of a LinkComposite, this is the Mobod that mobilizes the
  LinkComposite as a whole via the composite's active %Link. If you ask
  this Mobod what Joint it represents, it will report the Joint that was used
  to mobilize the LinkComposite; that won't necessarily be a Joint connected to
  this %Link. See inboard_joint_index() to find the Joint that connected this
  %Link to its LinkComposite. */
  MobodIndex mobod_index() const { return mobod_; }

  /** Returns the Joint that was used to associate this %Link with its
  mobilized body. If this %Link is part of a LinkComposite, returns the Joint
  that connects this %Link to the Composite, not necessarily the Joint that is
  modeled by the Mobod returned by mobod_index(). */
  JointIndex inboard_joint_index() const { return joint_; }

  /** Returns the index of the Composite this %Link is part of, if any.
  Otherwise returns an invalid index. */
  LinkCompositeIndex composite() const { return link_composite_index_; }

 private:
  friend class LinkJointGraph;
  friend class LinkJointGraphTester;

  Link(BodyIndex index, std::string name, ModelInstanceIndex model_instance,
       LinkFlags flags);

  // (For testing) If `to_set` is LinkFlags::kDefault sets the flags to
  // Default. Otherwise or's in the given flags to the current set. Returns
  // the updated value of this %Link's flags.
  LinkFlags set_flags(LinkFlags to_set) {
    return flags_ = (to_set == LinkFlags::kDefault ? LinkFlags::kDefault
                                                   : flags_ | to_set);
  }

  void renumber_mobod_indexes(const std::vector<MobodIndex>& old_to_new) {
    if (mobod_.is_valid()) mobod_ = old_to_new[mobod_];
  }

  // Notes that this Link is connected by `joint`.
  void add_joint_as_parent(JointIndex joint) {
    joints_as_parent_.push_back(joint);
    joints_.push_back(joint);
  }

  void add_joint_as_child(JointIndex joint) {
    joints_as_child_.push_back(joint);
    joints_.push_back(joint);
  }

  void clear_model(int num_user_joints) {
    mobod_ = {};
    joint_ = {};
    primary_link_ = {};
    shadow_links_.clear();
    link_composite_index_ = {};

    auto remove_model_joints =
        [num_user_joints](std::vector<JointIndex>& joints) {
          while (!joints.empty() && joints.back() >= num_user_joints)
            joints.pop_back();
        };

    remove_model_joints(joints_as_parent_);
    remove_model_joints(joints_as_child_);
    remove_model_joints(joints_);
  }

  BodyIndex index_;
  std::string name_;
  ModelInstanceIndex model_instance_;
  LinkFlags flags_{LinkFlags::kDefault};

  // Members below here may contain as-modeled information that has to be
  // removed when the SpanningForest is cleared or rebuilt. The joint
  // lists always have the as-built extra joints at the end.

  // Lists of joints connecting to this Link, in order of arrival.
  std::vector<JointIndex> joints_as_parent_;
  std::vector<JointIndex> joints_as_child_;
  std::vector<JointIndex> joints_;  // All joints whether as parent or child.

  MobodIndex mobod_;  // Which Mobod mobilizes this Link?
  JointIndex joint_;  // Which Joint connected us to the Mobod?

  BodyIndex primary_link_;  // Same as index_ if this is a primary link.
  std::vector<BodyIndex> shadow_links_;

  LinkCompositeIndex link_composite_index_;  // Invalid if not in composite.
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
