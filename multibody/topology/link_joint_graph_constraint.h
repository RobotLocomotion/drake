#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_GRAPH_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/graph.h".
#endif

#include <string>

namespace drake {
namespace multibody {
// TODO(sherm1) Promote from internal once API has stabilized: issue #11307.
namespace internal {

/** A constraint that restricts the relative motion of two Links. The
parent/child distinction sets the sign convention for the constraint
multipliers. Added welds between a primary %Link and one of its shadow Links
always make the primary %Link the parent. A constraint used to model a Joint
preserves parent/child order. */
class LinkJointGraph::Constraint {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Constraint)

  Constraint(ConstraintIndex index, std::string name,
             ModelInstanceIndex model_instance, LinkIndex parent_link_index,
             LinkIndex child_link_index)
      : index_(index),
        name_(name),
        model_instance_(model_instance),
        parent_link_index_(parent_link_index),
        child_link_index_(child_link_index) {}

  ModelInstanceIndex model_instance() const { return model_instance_; }

  const std::string& name() const { return name_; }

  LinkIndex parent_link() const { return parent_link_index_; }
  LinkIndex child_link() const { return child_link_index_; }

  ConstraintIndex index() const { return index_; }

 private:
  ConstraintIndex index_;
  std::string name_;
  ModelInstanceIndex model_instance_;
  JointTypeIndex type_index_;
  LinkIndex parent_link_index_;
  LinkIndex child_link_index_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
