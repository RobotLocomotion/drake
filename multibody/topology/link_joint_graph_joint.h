#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_GRAPH_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/graph.h".
#endif

#include <string>
#include <variant>
#include <vector>

namespace drake {
namespace multibody {
// TODO(sherm1) Promote from internal once API has stabilized: issue #11307.
namespace internal {

class LinkJointGraph::Joint {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Joint)

  Joint(JointIndex index, const std::string& name,
        ModelInstanceIndex model_instance, JointTypeIndex joint_type_index,
        BodyIndex parent_link_index, BodyIndex child_link_index,
        JointFlags flags)
      : index_(index),
        name_(name),
        model_instance_(model_instance),
        flags_(flags),
        type_index_(joint_type_index),
        parent_link_index_(parent_link_index),
        child_link_index_(child_link_index) {}

  ModelInstanceIndex model_instance() const { return model_instance_; }

  const std::string& name() const { return name_; }

  BodyIndex parent_link() const { return parent_link_index_; }
  BodyIndex child_link() const { return child_link_index_; }

  bool is_weld() const { return type_index() == weld_joint_type_index(); }

  JointTypeIndex type_index() const { return type_index_; }

  JointIndex index() const { return index_; }

  bool connects(BodyIndex link) const {
    return link == parent_link() || link == child_link();
  }

  bool connects(BodyIndex link1, BodyIndex link2) const {
    return (parent_link() == link1 && child_link() == link2) ||
           (parent_link() == link2 && child_link() == link1);
  }

  // Any moving (articulated) Joint must be modeled (i.e. it must be represented
  // by some Mobod). Welds typically won't be modeled though unless specifically
  // requested, which might happen if a user thinks reaction forces at this
  // Joint are particularly important and should be calculated.
  bool must_be_modeled() const {
    return static_cast<bool>(flags_ & JointFlags::MustBeModeled);
  }

  MobodIndex mobod_index() const {
    if (!std::holds_alternative<MobodIndex>(how_modeled_)) return MobodIndex();
    return std::get<MobodIndex>(how_modeled_);
  }

  bool has_been_processed() const {
    return !std::holds_alternative<std::monostate>(how_modeled_);
  }

  // Given one of the Links connected by this Joint, return the other one.
  BodyIndex other_link_index(BodyIndex link_index) const {
    DRAKE_DEMAND((parent_link() == link_index) ^ (child_link() == link_index));
    return parent_link() == link_index ? child_link() : parent_link();
  }

  // (For testing) If `to_set` is JointFlags::Default sets the flags to
  // Default. Otherwise or's in the given flags to the current set. Returns
  // the updated value of this Joint's flags.
  JointFlags set_flags(JointFlags to_set) {
    return flags_ = (to_set == JointFlags::Default ? JointFlags::Default
                                                   : flags_ | to_set);
  }

 private:
  friend class LinkJointGraph;

  void clear_model() { how_modeled_ = std::monostate{}; }

  void renumber_mobod_indexes(const std::vector<MobodIndex>& old_to_new) {
    if (std::holds_alternative<MobodIndex>(how_modeled_))
      how_modeled_ = old_to_new[std::get<MobodIndex>(how_modeled_)];
  }

  struct IgnoredLoopJoint {};

  JointIndex index_;
  std::string name_;
  ModelInstanceIndex model_instance_;
  JointFlags flags_{JointFlags::Default};

  JointTypeIndex type_index_;
  BodyIndex parent_link_index_;
  BodyIndex child_link_index_;

  // Below here is the as-built information; must be flushed when the Forest is
  // cleared or rebuilt.

  // Meaning of the variants:
  // - monostate: not yet processed
  // - MobodIndex: modeled directly by a mobilizer
  // - ConstraintIndex: modeled as a constraint (TBD)
  // - LinkCompositeIndex: not modeled because this is a weld interior to
  //     the indicated composite and we are combining so that one Mobod serves
  //     the whole composite.
  // - IgnoredLoopJoint: not modeled because we intentionally ignored the Joint
  //     (used with the IgnoreLoopJoints modeling option)
  std::variant<std::monostate, MobodIndex, LinkCompositeIndex, IgnoredLoopJoint>
      how_modeled_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
