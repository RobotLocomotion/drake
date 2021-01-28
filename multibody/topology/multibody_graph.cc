#include "drake/multibody/topology/multibody_graph.h"

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

BodyIndex MultibodyGraph::AddBody(const std::string& body_name,
                                  ModelInstanceIndex model_instance) {
  DRAKE_DEMAND(model_instance.is_valid());

  // Reject the usage of the "world" model instance for any other bodies but the
  // world.
  if (num_bodies() > 0 && model_instance == world_model_instance()) {
    const std::string msg = fmt::format(
        "AddBody(): Model instance index = {} is reserved for the world body. "
        " body_index = 0, named '{}'",
        world_model_instance(), world_body_name());
    throw std::runtime_error(msg);
  }

  // Reject duplicate body name.
  if (HasBodyNamed(body_name, model_instance)) {
    throw std::runtime_error("AddBody(): Duplicate body name '" + body_name +
                             "'");
  }
  // next available
  const BodyIndex body_index(num_bodies());
  // provide fast name lookup
  body_name_to_index_.insert({body_name, body_index});

  // Can't use emplace_back below because the constructor is private.
  bodies_.push_back(Body(body_index, body_name, model_instance));

  return body_index;
}

bool MultibodyGraph::HasBodyNamed(const std::string& name,
                                  ModelInstanceIndex model_instance) const {
  DRAKE_DEMAND(model_instance.is_valid());

  // Search linearly on the assumption that we won't often have lots of
  // bodies with the same name in different model instances.  If this turns
  // out to be incorrect we can switch to a different data structure.
  const auto range = body_name_to_index_.equal_range(name);
  for (auto it = range.first; it != range.second; ++it) {
    if (get_body(it->second).model_instance() == model_instance) {
      return true;
    }
  }
  return false;
}

bool MultibodyGraph::HasJointNamed(const std::string& name,
                                   ModelInstanceIndex model_instance) const {
  DRAKE_DEMAND(model_instance.is_valid());

  // Search linearly on the assumption that we won't often have lots of
  // joints with the same name in different model instances.  If this turns
  // out to be incorrect we can switch to a different data structure.
  const auto range = joint_name_to_index_.equal_range(name);
  for (auto it = range.first; it != range.second; ++it) {
    if (get_joint(it->second).model_instance() == model_instance) {
      return true;
    }
  }
  return false;
}

const std::string& MultibodyGraph::world_body_name() const {
  if (bodies_.empty())
    throw std::runtime_error(
        "get_world_body_name(): you can't call this until you have called "
        "AddBody() at least once -- the first body is World.");
  return bodies_[0].name();
}

const MultibodyGraph::Body& MultibodyGraph::world_body() const {
  if (bodies_.empty())
    throw std::runtime_error(
        "world_body(): you can't call this until you have called "
        "AddBody() at least once -- the first body is 'world'.");
  return bodies_[0];
}

JointIndex MultibodyGraph::AddJoint(const std::string& name,
                                    ModelInstanceIndex model_instance,
                                    const std::string& type,
                                    BodyIndex parent_body_index,
                                    BodyIndex child_body_index) {
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

  // Verify we are connecting bodies within the graph.
  if (!(parent_body_index.is_valid() && parent_body_index < num_bodies())) {
    throw std::runtime_error("AddJoint(): parent body index for joint '" +
                             name + "' is invalid.");
  }
  if (!(child_body_index.is_valid() && child_body_index < num_bodies())) {
    throw std::runtime_error("AddJoint(): child body index for joint '" + name +
                             "' is invalid.");
  }

  // next available index.
  const JointIndex joint_index(num_joints());
  auto [map_iter, inserted] = bodies_to_joint_.insert(
      {{parent_body_index, child_body_index}, joint_index});
  if (!inserted) {
    auto existing_joint_index = map_iter->second;
    const auto& existing_joint = get_joint(existing_joint_index);
    const auto& existing_parent = get_body(existing_joint.parent_body());
    const auto& existing_child = get_body(existing_joint.child_body());
    const auto& new_parent = get_body(parent_body_index);
    const auto& new_child = get_body(child_body_index);
    throw std::runtime_error(
        "This MultibodyGraph already has a joint '" + existing_joint.name() +
        "' connecting '" + existing_parent.name() +
        "' to '" + existing_child.name() +
        "'. Therefore adding joint '" + name +
        "' connecting '" + new_parent.name() + "' to '" + new_child.name() +
        "' is not allowed.");
  }

  // provide fast name lookup.
  joint_name_to_index_.insert({name, joint_index});

  // Can't use emplace_back below because the constructor is private.
  joints_.push_back(Joint(name, model_instance, type_index, parent_body_index,
                          child_body_index));

  // Connect the graph.
  get_mutable_body(parent_body_index).add_joint(joint_index);
  get_mutable_body(child_body_index).add_joint(joint_index);

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
  return it != joint_type_name_to_index_.end();
}

JointTypeIndex MultibodyGraph::GetJointTypeIndex(
    const std::string& joint_type_name) const {
  const auto it = joint_type_name_to_index_.find(joint_type_name);
  return it == joint_type_name_to_index_.end() ? JointTypeIndex() : it->second;
}

std::vector<std::set<BodyIndex>> MultibodyGraph::FindSubgraphsOfWeldedBodies()
    const {
  std::vector<bool> visited(num_bodies(), false);
  std::vector<std::set<BodyIndex>> subgraphs;

  // Reserve the maximum possible number of subgraphs (that is, when each body
  // forms its own subgraph) in advance in order to avoid reallocation in the
  // std::vector "subgraphs" which would cause the invalidation of references as
  // we recursively fill it in.
  subgraphs.reserve(num_bodies());

  // The first body visited is the "world" (body_index = 0), and therefore
  // subgraphs[0] corresponds to the subgraphs of all bodies welded to the
  // world.
  for (const auto& body : bodies_) {
    if (!visited[body.index()]) {
      // If `body` was not visited yet, we create an subgraph for it.
      subgraphs.push_back(std::set<BodyIndex>{body.index()});

      // We build the subgraph to which `body` belongs by recursively traversing
      // the sub-graph it belongs to.
      std::set<BodyIndex>& body_subgraph = subgraphs.back();

      // Thus far `body` forms its own subgraph. Find if other bodies belong to
      // this subgraph by recursively traversing the sub-graph of welded joints
      // connected to `body`.
      FindSubgraphsOfWeldedBodiesRecurse(body, &body_subgraph, &subgraphs,
                                         &visited);
    }
  }
  return subgraphs;
}

void MultibodyGraph::FindSubgraphsOfWeldedBodiesRecurse(
    const Body& parent_body, std::set<BodyIndex>* parent_subgraph,
    std::vector<std::set<BodyIndex>>* subgraphs,
    std::vector<bool>* visited) const {
  // Mark parent_body as visited in order to detect loops.
  visited->at(parent_body.index()) = true;

  // Scan each sibling body.
  for (JointIndex joint_index : parent_body.joints()) {
    const Joint& joint = get_joint(joint_index);
    const BodyIndex sibling_index = joint.parent_body() == parent_body.index()
                                        ? joint.child_body()
                                        : joint.parent_body();

    // If already visited continue with the next joint.
    if (visited->at(sibling_index)) continue;

    const Body& sibling = get_body(sibling_index);
    if (joint.type_index() == weld_type_index()) {
      // Welded to parent_body, add it to parent_subgraph.
      parent_subgraph->insert(sibling_index);
      FindSubgraphsOfWeldedBodiesRecurse(sibling, parent_subgraph, subgraphs,
                                         visited);
    } else {
      // Disconnected (non-welded) from parent_subgraph. Create its own new
      // subgraph and continue the recursion from "sibling" with its new
      // subgraph "sibling_subgraph".
      subgraphs->push_back(std::set<BodyIndex>{sibling_index});
      std::set<BodyIndex>& sibling_subgraph = subgraphs->back();
      FindSubgraphsOfWeldedBodiesRecurse(sibling, &sibling_subgraph, subgraphs,
                                         visited);
    }
  }
}

std::set<BodyIndex> MultibodyGraph::FindBodiesWeldedTo(
    BodyIndex body_index) const {
  DRAKE_THROW_UNLESS(body_index.is_valid() && body_index < num_bodies());

  // TODO(amcastro-tri): Notice that "subgraphs" will get compute with every
  // call to FindBodiesWeldedTo(). Consider storing this for subsequent calls if
  // it becomes a performance bottleneck.
  const std::vector<std::set<BodyIndex>> subgraphs =
      FindSubgraphsOfWeldedBodies();

  // Find subgraph that contains this body_index.
  // TODO(amcastro-tri): Consider storing within Body the subgraph it belongs to
  // if performance becomes an issue.
  auto predicate = [body_index](auto& subgraph) {
    return subgraph.count(body_index) > 0;
  };
  auto subgraph_iter =
      std::find_if(subgraphs.begin(), subgraphs.end(), predicate);

  // If body_index is a valid index to a body in this graph, then it MUST belong
  // to one of the subgraphs. We verify this explicitly.
  DRAKE_DEMAND(subgraph_iter != subgraphs.end());

  return *subgraph_iter;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
