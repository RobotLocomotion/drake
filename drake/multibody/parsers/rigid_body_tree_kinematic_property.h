#pragma once

#include <map>
#include <string>
#include <vector>

#include "drake/multibody/rigid_body_tree.h"
#include "yaml-cpp/yaml.h"

namespace drake {
namespace parsers {

template <typename T>
class RigidBodyTreeKinematicProperty {
 public:
  static constexpr char kBodyGroupKeyWord[] = "body_groups";
  static constexpr char kJointGroupKeyWord[] = "joint_groups";

  /**
   * Constructor for RigidBodyTreeKinematicProperty.
   * @param tree, Reference for the RigidBodyTree.
   * A pointer is stored internally, so `tree` needs to outlive this object.
   */
  explicit RigidBodyTreeKinematicProperty(const RigidBodyTree<T>& tree)
      : tree_(&tree) {}

  /**
   * Parse joint groups and body groups from the config file.
   * This function looks for top level key word kBodyGroupKeyWord and
   * kJointGroupKeyWord in `config`.
   * An example config file looks like:
   * <pre>
   * ...
   * body_groups:
   *   group_name1:
   *     [body_name1, body_name2, ...]
   *   group_name2:
   *     [body_name1, body_name3, ...]
   *   ...
   * ...
   * joint_groups:
   *   group_name1:
   *     [joint_name1, joint_name2, ...]
   *   group_name2:
   *     [joint_name1, joint_name3, ...]
   *   ...
   * ...
   * </pre>
   * body_namei and joint_namei need to match some link's name and joint's
   * name in the RigidBodyTree tree_.
   * @param config, YAML node specifying joint and body groups.
   *
   * @throws std::logic_error if cannot find body_namei or joint_namei in
   * tree_.
   */
  void LoadFromYAMLFile(const YAML::Node& config);

  bool has_body_group(const std::string& group_name) const {
    return body_groups_.find(group_name) != body_groups_.end();
  }

  bool has_joint_group(const std::string& group_name) const {
    return joint_groups_.find(group_name) != joint_groups_.end();
  }

  bool has_generalized_position_group(const std::string& group_name) const {
    return generalized_position_groups_.find(group_name) !=
           generalized_position_groups_.end();
  }

  bool has_generalized_velocity_group(const std::string& group_name) const {
    return generalized_velocity_groups_.find(group_name) !=
           generalized_velocity_groups_.end();
  }

  /**
   * Returns the body group identified by `group_name`.
   *
   * @throws std::out_of_range if `group_name` is not found.
   */
  const std::vector<const RigidBody<T>*>& get_body_group(
      const std::string& group_name) const {
    return body_groups_.at(group_name);
  }

  /**
   * Returns the joint group identified by `group_name`.
   *
   * @throws std::out_of_range if `group_name` is not found.
   */
  const std::vector<const DrakeJoint*>& get_joint_group(
      const std::string& group_name) const {
    return joint_groups_.at(group_name);
  }

  /**
   * Returns the generalized position indices associated with the joint group
   * identified by `group_name`.
   *
   * @throws std::out_of_range if `group_name` is not found.
   */
  const std::vector<int>& get_generalized_position_group(
      const std::string& group_name) const {
    return generalized_position_groups_.at(group_name);
  }

  /**
   * Returns the generalized velocity indices associated with the joint group
   * identified by `group_name`.
   *
   * @throws std::out_of_range if `group_name` is not found.
   */
  const std::vector<int>& get_generalized_velocity_group(
      const std::string& group_name) const {
    return generalized_velocity_groups_.at(group_name);
  }

  const std::map<std::string, std::vector<const RigidBody<T>*>>&
  get_body_groups() const {
    return body_groups_;
  }

  const std::map<std::string, std::vector<const DrakeJoint*>>&
  get_joint_groups() const {
    return joint_groups_;
  }

  const std::map<std::string, std::vector<int>>&
  get_generalized_position_groups() const {
    return generalized_position_groups_;
  }

  const std::map<std::string, std::vector<int>>&
  get_generalized_velocity_groups() const {
    return generalized_velocity_groups_;
  }

  const RigidBodyTree<T>& get_tree() const { return *tree_; }

 private:
  const RigidBodyTree<T>* tree_;

  void AddBodyGroup(const std::string& group_name,
                    const std::vector<std::string>& body_names);

  void AddJointGroup(const std::string& group_name,
                     const std::vector<std::string>& joint_names);

  std::map<std::string, std::vector<const RigidBody<T>*>> body_groups_;
  std::map<std::string, std::vector<const DrakeJoint*>> joint_groups_;

  std::map<std::string, std::vector<int>> generalized_position_groups_;
  std::map<std::string, std::vector<int>> generalized_velocity_groups_;
};

}  // namespace parsers
}  // namespace drake
