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

// LinkJointGraph definitions deferred until Joint defined.

inline auto LinkJointGraph::joints(JointIndex joint_index) const
    -> const Joint& {
  return joints().at(joint_index);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
