#include "drake/multibody/plant/multibody_graph.h"

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace multibody {
namespace internal {

MultibodyGraph::MultibodyGraph() { 
  RegisterJointType(weld_type_name());
  // Verify invariant promised to users in the documentation.
  DRAKE_DEMAND(joint_type_name_to_index_[weld_type_name()] ==
               JointTypeIndex(0));
}

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
  links_.push_back(Link(link_index, link_name, model_instance));

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
  get_mutable_link(parent_link_index).add_joint(joint_index);
  get_mutable_link(child_link_index).add_joint(joint_index);

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

bool MultibodyGraph::IsJointTypeRegistered(
    const std::string& joint_type_name) const {
  const auto it = joint_type_name_to_index_.find(joint_type_name);
  return it != joint_type_name_to_index_.end() ? true : false;
}

JointTypeIndex MultibodyGraph::GetJointTypeIndex(
    const std::string& joint_type_name) const {
  const auto it = joint_type_name_to_index_.find(joint_type_name);
  return it == joint_type_name_to_index_.end() ? JointTypeIndex() : it->second;
}

std::vector<std::set<LinkIndex>> MultibodyGraph::FindIslandsOfWeldedLinks()
    const {
  std::vector<bool> visited(num_links(), false);
  std::vector<std::set<LinkIndex>> islands;  // islands?

  // Reserve the maximum possible of welded bodies (that is, when each body
  // forms its own welded body) in advance in order to avoid reallocation in
  // welded_bodies which would cause the invalidation of references as we
  // recursively fill it in.
  islands.reserve(num_links());

  // The first link visited is the "world" (link_index = 0), and therefore
  // islands[0] corresponds to the islands of all links welded to the world.
  for (const auto& link : links_) {
    if (!visited[link.index()]) {
      // If `link` was not visited yet, we crate an island for it.
      islands.push_back(std::set<LinkIndex>{link.index()});

      // We build the island to which `link` belongs by recursively traversing
      // the sub-graph it belongs to.
      std::set<LinkIndex>& link_island = islands.back();

      // Thus far `link` form its own island. Find if other bodies belong to
      // this island by recursively traversing the sub-graph of welded joints
      // connected to `link`.
      FindIslandsOfWeldedLinksRecurse(link, &link_island, &islands, &visited);
    }
  }
  return islands;
}

// Recursive helper method for CreateListOfWeldedBodies().
// This method scans the children of body with parent_index. If a child is
// welded to body with parent_index, it gets added to the parent's body welded
// body, parent_welded_body. Otherwise a new welded body is created for the
// child body and gets added to the list of all welded bodies, welded_bodies.
void MultibodyGraph::FindIslandsOfWeldedLinksRecurse(
    const Link& parent_link, std::set<LinkIndex>* parent_island,
    std::vector<std::set<LinkIndex>>* islands,
    std::vector<bool>* visited) const {
  // Mark parent_link as visited in order to detect loops.
  visited->at(parent_link.index()) = true;

  // Scan each sibling link.
  for (JointIndex joint_index : parent_link.joints()) {
    const Joint& joint = get_joint(joint_index);
    const LinkIndex sibling_index = joint.parent_link() == parent_link.index()
                                        ? joint.child_link()
                                        : joint.parent_link();

    // If already visited continue with the next joint.
    if (visited->at(sibling_index)) continue;

    const Link& sibling = get_link(sibling_index);
    visited->at(sibling_index) = true;    
    if (joint.type_index() == weld_type_index()) {
      // Welded to parent_link, add it to parent_island.
      parent_island->insert(sibling_index);
      FindIslandsOfWeldedLinksRecurse(sibling, parent_island, islands, visited);
    } else {
      // Disconnected (non-welded) from parent_island. Create its own new island
      // and continue the recursion from "sibling" with its new island
      // "sibling_island".
      islands->push_back(std::set<LinkIndex>{sibling_index});
      std::set<LinkIndex>& sibling_island = islands->back();
      FindIslandsOfWeldedLinksRecurse(sibling, &sibling_island, islands,
                                      visited);
    }
  }
}

std::set<LinkIndex> MultibodyGraph::FindLinksWeldedTo(
    LinkIndex link_index) const {
  DRAKE_DEMAND(link_index.is_valid() && link_index < num_links());

  const std::vector<std::set<LinkIndex>> islands = FindIslandsOfWeldedLinks();

  // Find subgraph that contains this link_index.
  auto predicate = [link_index](auto& island) {
    return island.count(link_index) > 0;
  };
  auto island_iter = std::find_if(islands.begin(), islands.end(), predicate);

  // If link_index is a valid index to a link in this graph, then it MUST belong
  // to one of the islands. We verify this explicitly.
  DRAKE_DEMAND(island_iter != islands.end());

  // Copy indexes into a vector.
  return *island_iter;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
