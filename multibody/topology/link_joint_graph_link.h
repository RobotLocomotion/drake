#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_GRAPH_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/graph.h".
#endif

#include <optional>
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
modeled individually or can be combined into LinkComposites comprising groups
of Links that were connected by weld joints. */
class LinkJointGraph::Link {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Link);

  /** Returns this %Link's unique index in the graph. This is persistent after
  the %Link has been allocated. */
  BodyIndex index() const { return index_; }

  /** Returns the current value of this %Link's ordinal (position in the
  links() vector). This can change as %Links are removed. */
  LinkOrdinal ordinal() const { return ordinal_; }

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

  /** Returns indexes of all the LoopConstraints that connect to this %Link. */
  const std::vector<LoopConstraintIndex>& loop_constraints() const {
    return loop_constraints_;
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
    return is_world() || is_static_flag_set() ||
           (composite() == LinkCompositeIndex(0));
  }

  /** Returns `true` if this %Link was added with LinkFlags::kStatic. */
  bool is_static_flag_set() const {
    return static_cast<bool>(flags_ & LinkFlags::kStatic);
  }

  /** Returns `true` if this %Link was added with LinkFlags::kMustBeBaseBody. */
  bool must_be_base_body() const {
    return static_cast<bool>(flags_ & LinkFlags::kMustBeBaseBody);
  }

  /** Returns `true` if this %Link was added with LinkFlags::kMassless.
  However, this %Link may still be _effectively_ massful if it is welded
  into a massful composite. See
  LinkJointGraph::link_and_its_composite_are_massless() for the full story. */
  bool is_massless() const {
    return static_cast<bool>(flags_ & LinkFlags::kMassless);
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

  /** Returns the index of the LinkComposite this %Link is part of, if any.
  World is always in LinkComposite 0; any other link is in a Composite only if
  it is connected by a weld joint to another link. */
  std::optional<LinkCompositeIndex> composite() const {
    return link_composite_index_;
  }

 private:
  friend class LinkJointGraph;
  friend class LinkJointGraphTester;

  Link(BodyIndex index, LinkOrdinal ordinal, std::string name,
       ModelInstanceIndex model_instance, LinkFlags flags);

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

  // Notes that this Link is connected by the given joint.
  void add_joint_as_parent(JointIndex joint_index) {
    joints_as_parent_.push_back(joint_index);
    joints_.push_back(joint_index);
  }

  void add_joint_as_child(JointIndex joint_index) {
    joints_as_child_.push_back(joint_index);
    joints_.push_back(joint_index);
  }

  void add_loop_constraint(LoopConstraintIndex constraint_index) {
    loop_constraints_.push_back(constraint_index);
  }

  // This is used when a Joint is removed.
  void RemoveJointReferences(JointIndex joint_index) {
    std::erase(joints_as_parent_, joint_index);
    std::erase(joints_as_child_, joint_index);
    std::erase(joints_, joint_index);
  }

  // Removes any as-modeled information added to this user link during forest
  // building. Forgets any connections with ephemeral links, joints, and
  // constraints. Preserves only the as-constructed information: index, name,
  // model instance, flags, and primary_link (= index for a user link).
  // @pre this is a user link, not a shadow.
  void ClearModel(JointIndex max_user_joint_index);

  BodyIndex index_;      // persistent
  LinkOrdinal ordinal_;  // can change
  std::string name_;
  ModelInstanceIndex model_instance_;
  LinkFlags flags_{LinkFlags::kDefault};
  BodyIndex primary_link_;  // Same as index_ unless this is a shadow link.

  // Members below here may contain as-modeled information that has to be
  // removed when the SpanningForest is cleared or rebuilt. The joint
  // lists always have the as-built extra joints at the end.

  // Lists of joints connecting to this Link, in order of arrival.
  std::vector<JointIndex> joints_as_parent_;
  std::vector<JointIndex> joints_as_child_;
  std::vector<JointIndex> joints_;  // All joints whether as parent or child.

  std::vector<LoopConstraintIndex> loop_constraints_;

  MobodIndex mobod_;  // Mobod that mobilizes this Link.
  JointIndex joint_;  // Joint that connects us to the Mobod (invalid if World).

  std::vector<BodyIndex> shadow_links_;

  // World is always in a composite; other links are in a composite only
  // if they are welded to another link.
  std::optional<LinkCompositeIndex> link_composite_index_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
