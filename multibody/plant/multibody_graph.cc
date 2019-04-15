#include "drake/multibody/plant/multibody_graph.h"

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace multibody {
namespace internal {

MultibodyGraph::MultibodyGraph() { RegisterJointType("weld"); }

LinkIndex MultibodyGraph::AddLink(const std::string& link_name,
                                  ModelInstanceIndex model_instance) {
  DRAKE_DEMAND(model_instance.is_valid());

  // Reject the usage of the "world" model instance for any other links but the
  // world.
  if (num_links() > 0 && model_instance == world_model_instance()) {
    const std::string msg = fmt::format(
        "AddLink(): Model instance index = {} is reserved for the world body. "
        " link_index = 0, named '{}'",
        world_model_instance(), world_link_name());
    throw std::runtime_error(msg);
  }

  // Reject duplicate link name.
  if (HasLinkNamed(link_name, model_instance)) {
    throw std::runtime_error("AddLink(): Duplicate link name '" + link_name +
                             "'");
  }
  // next available
  const LinkIndex link_index(num_links());
  // provide fast name lookup
  link_name_to_index_.insert({link_name, link_index});

  // Can't use emplace_back below because the constructor is private.
  links_.push_back(Link(link_name, model_instance));

  return link_index;
}

bool MultibodyGraph::HasLinkNamed(const std::string& name,
                                  ModelInstanceIndex model_instance) const {
  DRAKE_DEMAND(model_instance.is_valid());

  // Search linearly on the assumption that we won't often have lots of
  // bodies with the same name in different model instances.  If this turns
  // out to be incorrect we can switch to a different data structure.
  const auto range = link_name_to_index_.equal_range(name);
  for (auto it = range.first; it != range.second; ++it) {
    if (get_link(it->second).model_instance() == model_instance) {
      return true;
    }
  }
  return false;
}

bool MultibodyGraph::HasJointNamed(const std::string& name,
                                   ModelInstanceIndex model_instance) const {
  DRAKE_DEMAND(model_instance.is_valid());

  // Search linearly on the assumption that we won't often have lots of
  // bodies with the same name in different model instances.  If this turns
  // out to be incorrect we can switch to a different data structure.
  const auto range = joint_name_to_index_.equal_range(name);
  for (auto it = range.first; it != range.second; ++it) {
    if (get_joint(it->second).model_instance() == model_instance) {
      return true;
    }
  }
  return false;
}

const std::string& MultibodyGraph::world_link_name() const {
  if (links_.empty())
    throw std::logic_error(
        "get_world_link_name(): you can't call this until you have called "
        "AddLink() at least once -- the first link is World.");
  return links_[0].name();
}

ModelInstanceIndex MultibodyGraph::world_model_instance() const {
  if (links_.empty())
    throw std::logic_error(
        "world_model_instance(): you can't call this until you have called "
        "AddLink() at least once -- the first link is World.");
  return links_[0].model_instance();
}

JointIndex MultibodyGraph::AddJoint(const std::string& name,
                                    ModelInstanceIndex model_instance,
                                    const std::string& type,
                                    LinkIndex parent_link_index,
                                    LinkIndex child_link_index) {
  DRAKE_DEMAND(model_instance.is_valid());

  // Reject duplicate joint name.
  if (HasJointNamed(name, model_instance)) {
    throw std::runtime_error("AddJoint(): Duplicate joint name '" + name +
                             "'.");
  }

  const JointTypeIndex type_index = GetJointTypeIndex(type);
  if (!type_index.is_valid()) {
    throw std::runtime_error("AddJoint(): Unrecognized type '" + type +
                             "' for joint '" + name + "'.");
  }

  // Verify we are connecting links within the graph.
  if (!(parent_link_index.is_valid() && parent_link_index < num_links())) {
    throw std::runtime_error("AddJoint(): parent link index for joint '" +
                             name + "' is invalid.");
  }
  if (!(child_link_index.is_valid() && child_link_index < num_links())) {
    throw std::runtime_error("AddJoint(): child link index for joint '" + name +
                             "' is invalid.");
  }

  // next available index.
  const JointIndex joint_index(num_joints());
  // provide fast name lookup.
  joint_name_to_index_.insert({name, joint_index});

  // Can't use emplace_back below because the constructor is private.
  joints_.push_back(Joint(name, model_instance, type_index, parent_link_index,
                          child_link_index));

  // Connect the graph.
  get_mutable_link(parent_link_index).add_joint_as_parent(joint_index);
  get_mutable_link(child_link_index).add_joint_as_child(joint_index);

  return joint_index;
}

int MultibodyGraph::num_joint_types() const {
  return static_cast<int>(joint_type_name_to_index_.size());
}

JointTypeIndex MultibodyGraph::RegisterJointType(
    const std::string& joint_type_name) {
  // Reject duplicate type name.
  const auto it = joint_type_name_to_index_.find(joint_type_name);
  if (it != joint_type_name_to_index_.end())
    throw std::runtime_error(fmt::format(
        "RegisterJointType(): Duplicate joint type: '{}'.", joint_type_name));
  const JointTypeIndex joint_type_index(num_joint_types());
  joint_type_name_to_index_[joint_type_name] = joint_type_index;
  return joint_type_index;
}

JointTypeIndex MultibodyGraph::GetJointTypeIndex(
    const std::string& joint_type_name) const {
  const auto it = joint_type_name_to_index_.find(joint_type_name);
  return it == joint_type_name_to_index_.end() ? JointTypeIndex() : it->second;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
