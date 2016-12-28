#pragma once

#include <string>

#include "drake/multibody/rigid_body_tree.h"

#include "yaml-cpp/yaml.h"

namespace drake {
namespace parsers {

template <typename T> class RigidBodyTreeKinematicProperty {
 public:
  static const std::string kBodyGroupKeyWord;
  static const std::string kJointGroupKeyWord;

  RigidBodyTreeKinematicProperty(const RigidBodyTree<T>& robot)
    : robot_(&robot) {}

  void LoadFromYAMLFile(const YAML::Node& config);

  const std::vector<const RigidBody<T>*>& get_body_group(
      const std::string& group_name) const { return body_groups_.at(group_name); }

  const std::vector<const DrakeJoint*>& get_joint_group(
      const std::string& group_name) const { return joint_groups_.at(group_name); }

  const std::vector<int> get_generalized_position_group(
      const std::string& group_name) const {
    return generalized_position_groups_.at(group_name);
  }

  const std::vector<int> get_generalized_velocity_group(
      const std::string& group_name) const {
    return generalized_velocity_groups_.at(group_name);
  }

  bool has_body_group(const std::string& group_name) const {
    return body_groups_.find(group_name) != body_groups_.end();
  }

  bool has_joint_group(const std::string& group_name) const {
    return joint_groups_.find(group_name) != joint_groups_.end();
  }

  bool has_generalized_position_group(const std::string& group_name) const {
    return generalized_position_groups_.find(group_name) != generalized_position_groups_.end();
  }

  bool has_generalized_velocity_group(const std::string& group_name) const {
    return generalized_velocity_groups_.find(group_name) != generalized_velocity_groups_.end();
  }

  const std::map<std::string, std::vector<const RigidBody<T>*>>&
      get_body_groups() const { return body_groups_; }

  const std::map<std::string, std::vector<const DrakeJoint*>>&
      get_joint_groups() const { return joint_groups_; }

  const std::map<std::string, std::vector<int>>
      get_generalized_position_groups() const { return generalized_position_groups_; }

  const std::map<std::string, std::vector<int>>
      get_generalized_velocity_groups() const { return generalized_velocity_groups_; }

 private:
  const RigidBodyTree<T>* robot_;

  /**
   * Parse @p robot's body groups specified by @p metadata, which is loaded from
   * a YAML file.
   *
   * The config file is assumed to be formatted as:
   * <pre>
   * ...
   * body_groups:
   *   group_name1:
   *     [body_name1, body_name2, ...]
   *   group_name2:
   *     [body_name1, body_name3, ...]
   *   ...
   * ...
   * </pre>
   * This function is looking for a top level keyword kBodyGroupKeyWord,
   * which contains individual groups consists of a vector of body names.
   * The result is stored in @p robot, and can be accessed by calling
   * RigidBodyTree::get_body_group.
   *
   * @param metadata, YAML node representing the config file.
   * @param robot, Pointer to the RigidBodyTree.
   */
  void AddBodyGroup(const std::string& group_name,
                    const std::vector<std::string>& body_names);

  /**
   * Parse @p robot's joint groups specified by @p metadata, which is loaded from
   * a YAML file.
   *
   * The config file is assumed to be formatted as:
   * <pre>
   * ...
   * joint_groups:
   *   group_name1:
   *     [joint_name1, joint_name2, ...]
   *   group_name2:
   *     [joint_name1, joint_name3, ...]
   *   ...
   * ...
   * </pre>
   * This function is looking for a top level keyword kJointGroupKeyWord
   * which contains individual groups consists of a vector of joint names.
   * The result is stored in @p robot, and can be accessed by calling
   * RigidBodyTree::get_position_group and RigidBodyTree::get_velocity_group.
   *
   * @param metadata, YAML node representing the config file.
   * @param robot, Pointer to the RigidBodyTree.
   */
  void AddJointGroup(const std::string& group_name,
                    const std::vector<std::string>& joint_names);

  std::map<std::string, std::vector<const RigidBody<T>*>> body_groups_;
  std::map<std::string, std::vector<const DrakeJoint*>> joint_groups_;

  std::map<std::string, std::vector<int>> generalized_position_groups_;
  std::map<std::string, std::vector<int>> generalized_velocity_groups_;
};

}  // namespace parsers
}  // namespace drake
