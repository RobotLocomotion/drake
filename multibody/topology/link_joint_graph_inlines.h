#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_GRAPH_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/graph.h".
#endif

#include <optional>

namespace drake {
namespace multibody {
// TODO(sherm1) Promote from internal once API has stabilized: issue #11307.
namespace internal {

// LinkJointGraph definitions deferred until Link defined.

inline const LinkJointGraph::Link& LinkJointGraph::links(
    LinkOrdinal link_ordinal) const {
  DRAKE_ASSERT(link_ordinal < ssize(links()));
  return data_.links[link_ordinal];
}

inline int LinkJointGraph::num_links() const {
  return std::ssize(links());
}

inline const LinkJointGraph::Link& LinkJointGraph::link_by_index(
    LinkIndex link_index) const {
  const std::optional<LinkOrdinal>& ordinal =
      data_.link_index_to_ordinal.at(link_index);
  if (!ordinal.has_value()) ThrowLinkWasRemoved(__func__, link_index);
  DRAKE_ASSERT(ordinal < ssize(links()));
  return links(*ordinal);
}

inline LinkJointGraph::Link& LinkJointGraph::mutable_link(
    LinkOrdinal link_ordinal) {
  DRAKE_ASSERT(link_ordinal < ssize(links()));
  return data_.links[link_ordinal];
}

inline MobodIndex LinkJointGraph::link_to_mobod(LinkIndex index) const {
  return link_by_index(index).mobod_;
}

inline void LinkJointGraph::set_primary_mobod_for_link(
    LinkOrdinal link_ordinal, MobodIndex primary_mobod_index,
    JointIndex primary_joint_index) {
  Link& link = mutable_link(link_ordinal);
  DRAKE_ASSERT(!link.mobod_.is_valid());
  link.mobod_ = primary_mobod_index;
  link.joint_ = primary_joint_index;
}

inline bool LinkJointGraph::link_and_its_assembly_are_massless(
    LinkOrdinal link_ordinal) const {
  const Link& link = links(link_ordinal);
  if (!link.is_massless()) return false;

  return link.welded_links_assembly().has_value()
             ? welded_links_assemblies(*link.welded_links_assembly())
                   .is_massless()
             : true;
}

// LinkJointGraph definitions deferred until Joint defined.

inline const LinkJointGraph::Joint& LinkJointGraph::joints(
    JointOrdinal joint_ordinal) const {
  DRAKE_ASSERT(joint_ordinal < ssize(joints()));
  return data_.joints[joint_ordinal];
}

inline int LinkJointGraph::num_joints() const {
  return std::ssize(joints());
}

inline const LinkJointGraph::Joint& LinkJointGraph::joint_by_index(
    JointIndex joint_index) const {
  const std::optional<JointOrdinal>& ordinal =
      data_.joint_index_to_ordinal.at(joint_index);
  if (!ordinal.has_value()) ThrowJointWasRemoved(__func__, joint_index);
  DRAKE_ASSERT(ordinal < ssize(joints()));
  return joints(*ordinal);
}

inline LinkJointGraph::Joint& LinkJointGraph::mutable_joint(
    JointOrdinal joint_ordinal) {
  DRAKE_ASSERT(joint_ordinal < ssize(joints()));
  return data_.joints[joint_ordinal];
}

inline void LinkJointGraph::set_mobod_for_joint(JointOrdinal joint_ordinal,
                                                MobodIndex mobod_index) {
  Joint& joint = mutable_joint(joint_ordinal);
  DRAKE_ASSERT(joint.how_modeled_.index() == 0);  // I.e., empty.
  joint.how_modeled_ = mobod_index;
}

// LinkJointGraph definitions deferred until LoopConstraint defined.

inline const LinkJointGraph::LoopConstraint& LinkJointGraph::loop_constraints(
    LoopConstraintIndex loop_constraint_index) const {
  return loop_constraints().at(loop_constraint_index);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
