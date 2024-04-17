#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_GRAPH_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/graph.h".
#endif

namespace drake {
namespace multibody {
// TODO(sherm1) Promote from internal once API has stabilized: issue #11307.
namespace internal {

// LinkJointGraph definitions deferred until Link defined.

inline auto LinkJointGraph::links(BodyIndex link_index) const -> const Link& {
  return links().at(link_index);
}

inline auto LinkJointGraph::mutable_link(BodyIndex link_index) -> Link& {
  return data_.links.at(link_index);
}

inline MobodIndex LinkJointGraph::link_to_mobod(BodyIndex index) const {
  return links(index).mobod_;
}

inline void LinkJointGraph::set_primary_mobod_for_link(
    BodyIndex link_index, MobodIndex primary_mobod_index,
    JointIndex primary_joint_index) {
  Link& link = mutable_link(link_index);
  DRAKE_DEMAND(!link.mobod_.is_valid());
  link.mobod_ = primary_mobod_index;
  link.joint_ = primary_joint_index;
}

// LinkJointGraph definitions deferred until Joint defined.

inline auto LinkJointGraph::joints(JointIndex joint_index) const
    -> const Joint& {
  return joints().at(joint_index);
}

inline auto LinkJointGraph::mutable_joint(JointIndex joint_index) -> Joint& {
  return data_.joints.at(joint_index);
}

inline void LinkJointGraph::set_mobod_for_joint(JointIndex joint_index,
                                                MobodIndex mobod_index) {
  Joint& joint = mutable_joint(joint_index);
  DRAKE_DEMAND(joint.how_modeled_.index() == 0);  // I.e., empty.
  joint.how_modeled_ = mobod_index;
}

// LinkJointGraph definitions deferred until LoopConstraint defined.

inline auto LinkJointGraph::loop_constraints(
    LoopConstraintIndex loop_constraint_index) const -> const LoopConstraint& {
  return loop_constraints().at(loop_constraint_index);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
