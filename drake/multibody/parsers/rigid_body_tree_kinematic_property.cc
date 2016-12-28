#include "drake/multibody/parsers/rigid_body_tree_kinematic_property.h"

#include <string>
#include <vector>

namespace drake {
namespace parsers {

template <typename T>
const std::string RigidBodyTreeKinematicProperty<T>::kBodyGroupKeyWord = "body_groups";

template <typename T>
const std::string RigidBodyTreeKinematicProperty<T>::kJointGroupKeyWord = "joint_groups";

template <typename T>
void RigidBodyTreeKinematicProperty<T>::AddBodyGroup(
    const std::string& group_name,
    const std::vector<std::string>& body_names) {
  std::vector<const RigidBody<T>*> bodies;
  bodies.reserve(body_names.size());
  for (const std::string& name : body_names) {
    bodies.push_back(robot_->FindBody(name));
  }
  body_groups_[group_name] = bodies;
}

template <typename T>
void RigidBodyTreeKinematicProperty<T>::AddJointGroup(
    const std::string& group_name,
    const std::vector<std::string>& joint_names) {

  std::vector<const RigidBody<T>*> bodies;
  std::vector<const DrakeJoint*> joints;
  bodies.reserve(joint_names.size());
  joints.reserve(joint_names.size());
  int q_size = 0;
  int v_size = 0;
  for (const std::string& name : joint_names) {
    bodies.push_back(robot_->FindChildBodyOfJoint(name));
    joints.push_back(&(bodies.back()->getJoint()));

    q_size += bodies.back()->getJoint().get_num_positions();
    v_size += bodies.back()->getJoint().get_num_velocities();
  }
  joint_groups_[group_name] = joints;

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

  generalized_position_groups_[group_name] = q_indices;
  generalized_velocity_groups_[group_name] = v_indices;
}

template <typename T>
void RigidBodyTreeKinematicProperty<T>::LoadFromYAMLFile(const YAML::Node& config) {

  // Parse body groups.
  YAML::Node body_groups = config[kBodyGroupKeyWord];
  for (auto group_it = body_groups.begin(); group_it != body_groups.end();
       ++group_it) {
    std::string group_name = group_it->first.as<std::string>();
    std::vector<std::string> body_names =
        group_it->second.as<std::vector<std::string>>();

    AddBodyGroup(group_name, body_names);
  }

  // Parse joint groups
  YAML::Node joint_groups = config[kJointGroupKeyWord];
  for (auto group_it = joint_groups.begin(); group_it != joint_groups.end();
       ++group_it) {
    std::string group_name = group_it->first.as<std::string>();
    std::vector<std::string> joint_names =
        group_it->second.as<std::vector<std::string>>();

    AddJointGroup(group_name, joint_names);
  }
}

template class RigidBodyTreeKinematicProperty<double>;

}  // namespace parsers
}  // namespace drake
