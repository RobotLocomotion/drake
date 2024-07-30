#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_GRAPH_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/graph.h".
#endif

#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace drake {
namespace multibody {
// TODO(sherm1) Promote from internal once API has stabilized: issue #11307.
namespace internal {

class LinkJointGraph::Joint {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Joint);

  /** Returns this %Joint's unique index in the graph. This is persistent after
  the %Joint has been allocated. */
  JointIndex index() const { return index_; }

  /** Returns the current value of this %Joint's ordinal (position in the
  joints() vector). This can change as %Joints are removed. */
  JointOrdinal ordinal() const { return ordinal_; }

  /** Returns this %Joint's model instance. */
  ModelInstanceIndex model_instance() const { return model_instance_; }

  /** Returns this %Joint's name, unique within model_instance(). */
  const std::string& name() const { return name_; }

  /** Returns the index of this %Joint's parent Link. */
  BodyIndex parent_link_index() const { return parent_link_index_; }

  /** Returns the index of this %Joint's child Link. */
  BodyIndex child_link_index() const { return child_link_index_; }

  /** Returns `true` if this is a Weld %Joint. */
  bool is_weld() const { return traits_index() == weld_joint_traits_index(); }

  /** Returns the index of this %Joint's traits. */
  JointTraitsIndex traits_index() const { return traits_index_; }

  /** Returns `true` if either the parent or child Link of this %Joint is
  the specified `link`. */
  bool connects(BodyIndex link) const {
    return link == parent_link_index() || link == child_link_index();
  }

  /** Returns `true` if this %Joint connects the two given Links. That is, if
  one of these is the parent Link and the other is the child Link, in either
  order. */
  bool connects(BodyIndex link1, BodyIndex link2) const {
    return (parent_link_index() == link1 && child_link_index() == link2) ||
           (parent_link_index() == link2 && child_link_index() == link1);
  }

  // TODO(sherm1) Per Joe M an unchecked version of this could just do
  //  return link_index ^ (parent_link_index ^ child_link_index);
  //  with the second term precalculated. Consider that if performance warrants.

  /** Given one of the Links connected by this %Joint, returns the other one.
  @pre `link_index` is either the parent or child */
  BodyIndex other_link_index(BodyIndex link_index) const {
    DRAKE_DEMAND((parent_link_index() == link_index) ||
                 (child_link_index() == link_index));
    return parent_link_index() == link_index ? child_link_index()
                                             : parent_link_index();
  }

  /** Returns `true` if this %Joint was added with
  JointFlags::kMustBeModeled. */
  bool must_be_modeled() const {
    return static_cast<bool>(flags_ & JointFlags::kMustBeModeled);
  }

  /** Returns the index of the Mobod whose inboard mobilizer models this
  %Joint, if any. If this %Joint is unmodeled then the returned index is
  invalid. */
  MobodIndex mobod_index() const {
    if (!std::holds_alternative<MobodIndex>(how_modeled_)) return MobodIndex();
    return std::get<MobodIndex>(how_modeled_);
  }

  /** (Internal use only) During construction of the forest, this is used
  to check whether this %Joint has already been modeled. */
  bool has_been_processed() const {
    return !std::holds_alternative<std::monostate>(how_modeled_);
  }

 private:
  friend class LinkJointGraph;
  friend class LinkJointGraphTester;

  Joint(JointIndex index, JointOrdinal ordinal, std::string name,
        ModelInstanceIndex model_instance, JointTraitsIndex joint_traits_index,
        BodyIndex parent_link_index, BodyIndex child_link_index,
        JointFlags flags);

  void ClearModel() { how_modeled_ = std::monostate{}; }

  // (For testing) If `to_set` is JointFlags::kDefault sets the flags to
  // kDefault. Otherwise or's in the given flags to the current set. Returns
  // the updated value of this %Joint's flags.
  JointFlags set_flags(JointFlags to_set) {
    return flags_ = (to_set == JointFlags::kDefault ? JointFlags::kDefault
                                                    : flags_ | to_set);
  }

  // Only joints that are modeled with Mobods need renumbering. This gets
  // called on all joints but does nothing unless there is a misnumbered
  // Mobod lurking beneath.
  void renumber_mobod_indexes(const std::vector<MobodIndex>& old_to_new) {
    if (std::holds_alternative<MobodIndex>(how_modeled_))
      how_modeled_ = old_to_new[std::get<MobodIndex>(how_modeled_)];
  }

  JointIndex index_;      // persistent
  JointOrdinal ordinal_;  // can change
  std::string name_;
  ModelInstanceIndex model_instance_;
  JointFlags flags_{JointFlags::kDefault};

  JointTraitsIndex traits_index_;
  BodyIndex parent_link_index_;
  BodyIndex child_link_index_;

  // Below here is the as-built information; must be flushed when the forest is
  // cleared or rebuilt.

  // Meaning of the variants:
  // - monostate: not yet processed
  // - MobodIndex: modeled directly by a mobilizer
  // - LinkCompositeIndex: not modeled because this is a weld interior to
  //     the indicated composite and we are combining so that one Mobod serves
  //     the whole composite.
  std::variant<std::monostate, MobodIndex, LinkCompositeIndex> how_modeled_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
