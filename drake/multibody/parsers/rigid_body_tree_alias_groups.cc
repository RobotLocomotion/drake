#include "drake/multibody/parsers/rigid_body_tree_alias_groups.h"

namespace drake {
namespace parsers {

template <typename T>
constexpr char RigidBodyTreeAliasGroups<T>::kBodyGroupsKeyword[];
template <typename T>
constexpr char RigidBodyTreeAliasGroups<T>::kJointGroupsKeyword[];

namespace {
template <typename Type>
void InsertOrMergeVector(const std::string key, const std::vector<Type>& vec,
    std::map<std::string, std::vector<Type>>* map) {
  DRAKE_DEMAND(map);
  typename std::map<std::string, std::vector<Type>>::iterator it =
      map->find(key);
  if (it == map->end()) {
    map->emplace(key, vec);
  } else {
    it->second.insert(it->second.end(), vec.begin(), vec.end());
  }
}

// Return a std::vector of a YAML::Node. This function trys to cast the node
// as a std::vector<Type> or as a single Type. If both cast fails, it throws
// exceptions.
template <typename Type>
std::vector<Type> ParseYAMLNodeAsVector(const YAML::Node& node) {
  std::vector<Type> values;

  if (node.IsNull())
    return values;

  // Try cast as a vector of strings
  try {
    values = node.as<std::vector<Type>>();
  } catch (std::runtime_error e) {
    // If not, try cast as a single string
    try {
      values.push_back(node.as<Type>());
    } catch (std::runtime_error e1) {
      // Throws if cannot be parsed.
      throw e1;
    }
  }

  return values;
}
}  // namespace

template <typename T>
void RigidBodyTreeAliasGroups<T>::AddBodyGroup(
    const std::string& group_name, const std::vector<std::string>& body_names) {
  std::vector<const RigidBody<T>*> bodies;
  bodies.reserve(body_names.size());
  for (const std::string& name : body_names) {
    bodies.push_back(tree_->FindBody(name));
  }

  InsertOrMergeVector(group_name, bodies, &body_groups_);
}

template <typename T>
void RigidBodyTreeAliasGroups<T>::AddJointGroup(
    const std::string& group_name,
    const std::vector<std::string>& joint_names) {
  std::vector<const RigidBody<T>*> bodies;
  std::vector<const DrakeJoint*> joints;
  bodies.reserve(joint_names.size());
  joints.reserve(joint_names.size());
  int q_size = 0;
  int v_size = 0;
  for (const std::string& name : joint_names) {
    bodies.push_back(tree_->FindChildBodyOfJoint(name));
    joints.push_back(&(bodies.back()->getJoint()));

    q_size += joints.back()->get_num_positions();
    v_size += joints.back()->get_num_velocities();
  }

  InsertOrMergeVector(group_name, joints, &joint_groups_);

  std::vector<int> q_indices, v_indices;
  q_indices.reserve(q_size);
  v_indices.reserve(v_size);

  for (const RigidBody<T>* body : bodies) {
    int q_start = body->get_position_start_index();
    int q_end = q_start + body->getJoint().get_num_positions();
    for (int i = q_start; i < q_end; ++i) q_indices.push_back(i);

    int v_start = body->get_velocity_start_index();
    int v_end = v_start + body->getJoint().get_num_velocities();
    for (int i = v_start; i < v_end; ++i) v_indices.push_back(i);
  }

  InsertOrMergeVector(group_name, q_indices, &position_groups_);
  InsertOrMergeVector(group_name, v_indices, &velocity_groups_);
}

template <typename T>
void RigidBodyTreeAliasGroups<T>::LoadFromYAMLFile(
    const YAML::Node& config) {
  // Parse body groups.
  YAML::Node body_groups = config[kBodyGroupsKeyword];
  for (auto group_it = body_groups.begin(); group_it != body_groups.end();
       ++group_it) {
    std::string group_name = group_it->first.as<std::string>();
    std::vector<std::string> body_names =
        ParseYAMLNodeAsVector<std::string>(group_it->second);

    AddBodyGroup(group_name, body_names);
  }

  // Parse joint groups
  YAML::Node joint_groups = config[kJointGroupsKeyword];
  for (auto group_it = joint_groups.begin(); group_it != joint_groups.end();
       ++group_it) {
    std::string group_name = group_it->first.as<std::string>();
    std::vector<std::string> joint_names =
        ParseYAMLNodeAsVector<std::string>(group_it->second);

    AddJointGroup(group_name, joint_names);
  }
}

template class RigidBodyTreeAliasGroups<double>;

}  // namespace parsers
}  // namespace drake
