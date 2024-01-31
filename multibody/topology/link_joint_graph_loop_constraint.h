#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_GRAPH_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/graph.h".
#endif

#include <string>

namespace drake {
namespace multibody {
// TODO(sherm1) Promote from internal once API has stabilized: issue #11307.
namespace internal {

/** A weld constraint used to close a topological loop (cycle) in the input
graph after modeling as a forest. The
parent/child ordering sets the sign convention for the constraint
multipliers. Added welds between a primary %Link and one of its shadow Links
always make the primary %Link the parent. A constraint used to model a Joint
preserves parent/child order. */
class LinkJointGraph::LoopConstraint {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LoopConstraint)

  LoopConstraint(LoopConstraintIndex index, std::string name,
                 ModelInstanceIndex model_instance, BodyIndex parent_link_index,
                 BodyIndex child_link_index)
      : index_(index),
        name_(name),
        model_instance_(model_instance),
        parent_link_index_(parent_link_index),
        child_link_index_(child_link_index) {}

  LoopConstraintIndex index() const { return index_; }

  ModelInstanceIndex model_instance() const { return model_instance_; }

  const std::string& name() const { return name_; }

  BodyIndex parent_link() const { return parent_link_index_; }
  BodyIndex child_link() const { return child_link_index_; }

 private:
  LoopConstraintIndex index_;
  std::string name_;
  ModelInstanceIndex model_instance_;
  BodyIndex parent_link_index_;
  BodyIndex child_link_index_;

  // TODO(sherm1) Record the assigned MultibodyPlant Constraint here.
  MultibodyConstraintId plant_constraint_id;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
