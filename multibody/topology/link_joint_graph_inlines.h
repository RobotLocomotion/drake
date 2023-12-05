#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_GRAPH_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/graph.h".
#endif

namespace drake {
namespace multibody {
// TODO(sherm1) Promote from internal once API has stabilized: issue #11307.
namespace internal {

// LinkJointGraph definitions deferred until Link defined.

inline auto LinkJointGraph::links(LinkIndex link_index) const -> const Link& {
  return links().at(link_index);
}

inline auto LinkJointGraph::mutable_link(LinkIndex link_index) -> Link& {
  return data_.links[link_index];
}

inline MobodIndex LinkJointGraph::link_to_mobod(LinkIndex index) const {
  return links(index).mobod_;
}

inline void LinkJointGraph::set_primary_mobod_for_link(
    LinkIndex link_index, MobodIndex primary_mobod_index,
    JointIndex primary_joint_index) {
  Link& link = data_.links[link_index];
  DRAKE_DEMAND(!link.mobod_.is_valid());
  link.mobod_ = primary_mobod_index;
  link.joint_ = primary_joint_index;
}

inline void LinkJointGraph::change_link_flags(LinkIndex link_index,
                                              LinkFlags flags) {
  ClearForest();
  mutable_link(link_index).set_flags(flags);
}

inline bool LinkJointGraph::must_treat_as_massless(LinkIndex link_index) const {
  const Link& link = links(link_index);
  // TODO(sherm1) If part of a Composite then this is only massless if the
  //  entire Composite is composed of massless Links.
  return link.treat_as_massless();
}

// LinkJointGraph definitions deferred until Joint defined.

inline auto LinkJointGraph::joints(JointIndex joint_index) const
    -> const Joint& {
  return joints().at(joint_index);
}

inline auto LinkJointGraph::mutable_joint(JointIndex joint_index) -> Joint& {
  return data_.joints[joint_index];
}

inline void LinkJointGraph::set_mobod_for_joint(JointIndex joint_index,
                                                MobodIndex mobod_index) {
  Joint& joint = mutable_joint(joint_index);
  DRAKE_DEMAND(joint.how_modeled_.index() == 0);  // I.e., empty.
  joint.how_modeled_ = mobod_index;
}

inline void LinkJointGraph::ignore_loop_joint(JointIndex joint_index) {
  Joint& joint = mutable_joint(joint_index);
  DRAKE_DEMAND(joint.how_modeled_.index() == 0);  // I.e., empty.
  joint.how_modeled_ = Joint::IgnoredLoopJoint();
}

inline void LinkJointGraph::change_joint_flags(JointIndex joint_index,
                                               JointFlags flags) {
  ClearForest();
  mutable_joint(joint_index).set_flags(flags);
}

// LinkJointGraph definitions deferred until Constraint defined.

inline auto LinkJointGraph::constraints(ConstraintIndex constraint_index) const
    -> const Constraint& {
  return constraints().at(constraint_index);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
