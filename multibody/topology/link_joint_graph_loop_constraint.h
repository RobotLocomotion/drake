#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_GRAPH_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/graph.h".
#endif

#include <string>

namespace drake {
namespace multibody {
// TODO(sherm1) Promote from internal once API has stabilized: issue #11307.
namespace internal {

/* A weld constraint used to close a topological loop (cycle) in the input
graph after modeling as a forest. The parent/child ordering sets the sign
convention for the constraint multipliers. Added welds between a primary %Link
and one of its shadow Links always make the primary %Link the parent. */
class LinkJointGraph::LoopConstraint {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LoopConstraint);

  LoopConstraint(LoopConstraintIndex index, std::string name,
                 ModelInstanceIndex model_instance,
                 LinkIndex primary_link_index, LinkIndex shadow_link_index)
      : index_(index),
        name_(name),
        model_instance_(model_instance),
        primary_link_index_(primary_link_index),
        shadow_link_index_(shadow_link_index) {}

  LoopConstraintIndex index() const { return index_; }

  ModelInstanceIndex model_instance() const { return model_instance_; }

  const std::string& name() const { return name_; }

  LinkIndex primary_link() const { return primary_link_index_; }
  LinkIndex shadow_link() const { return shadow_link_index_; }

 private:
  LoopConstraintIndex index_;
  std::string name_;
  ModelInstanceIndex model_instance_;
  LinkIndex primary_link_index_;
  LinkIndex shadow_link_index_;

  // TODO(sherm1) Record the ephemeral MultibodyPlant Constraint here, e.g.:
  //  MultibodyConstraintId plant_constraint_id_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
