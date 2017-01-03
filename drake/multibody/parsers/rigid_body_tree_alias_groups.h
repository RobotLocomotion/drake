#pragma once

#include <map>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace parsers {

/**
 * This class provides a way to create aliases to groups of RigidBody or
 * DrakeJoint objects. The creation of these groups can be done either
 * programmatically or via a YAML file.
 * For example, suppose we have a RigidBodyTree with 6 links named
 * [link0 ~ link5], and 6 joints [base, joint0 ~ joint4].
 * We can "rename" `link0` to be `robot_base` by creating a body group named
 * `robot_base` with one entry `link0`, and accessing the first element in this
 * body group. Alias names for joints can also be similarly created.
 * Furthermore, the corresponding indices to the generalized position and
 * velocity are computed and accessible through an instance of this class.
 *
 * The elements within each body group or joint group are not guaranteed to be
 * unique. Body groups and joint groups are independent, so a body group can
 * have the same name of a joint group. However, all body groups have unique
 * group names, and so do all joint groups. When adding new members to an
 * existing group, the new members will be appended to the existing group.
 */
template <typename T>
class RigidBodyTreeAliasGroups {
 public:
  static constexpr char kBodyGroupsKeyword[] = "body_groups";
  static constexpr char kJointGroupsKeyword[] = "joint_groups";

  /**
   * Constructor for RigidBodyTreeAliasGroups.
   * @param tree Reference to the RigidBodyTree.
   * A pointer is stored internally, so `tree` needs to outlive this object.
   */
  explicit RigidBodyTreeAliasGroups(const RigidBodyTree<T>& tree)
      : tree_(&tree) {}

  /**
   * Parses body groups and joint groups from a config file.
   * This function looks for optional top level keywords kBodyGroupsKeyword and
   * kJointGroupsKeyword in `config`.
   * An example config file looks like:
   * <pre>
   * ...
   * body_groups:
   *   body_group_name1:
   *     [body_name1, body_name2, ...]
   *   body_group_name2:
   *     [body_name1, body_name3, ...]
   *   body_group_name3:
   *
   *   body_group_name4:
   *     body_name4
   *   ...
   * ...
   * joint_groups:
   *   joint_group_name1:
   *     [joint_name1, joint_name2, ...]
   *   joint_group_name2:
   *     [joint_name1, joint_name3, ...]
   *   joint_group_name3:
   *
   *   joint_group_name4:
   *     joint_name4
   *   ...
   * ...
   * </pre>
   * body_namei and joint_namei need to match a link's name and joint's
   * name respectively in the RigidBodyTree provided at construction time.
   * body_namei and joint_namei can appear in multiple groups. Multiple body
   * groups with the same group name will be merged, and the same applies to
   * joint groups. body_groups or joint_groups can be absent or empty.
   * The above example shows all valid formats for the body names and joint
   * names.
   *
   * @param config YAML node specifying joint and body groups.
   *
   * @throws std::logic_error if body_namei or joint_namei cannot be found in
   * the RigidBodyTree provided at construction time.
   * @throws std::runtime_error if joint names or body names cannot be parsed
   * correctly.
   */
  void LoadFromYAMLFile(const YAML::Node& config);

  /**
   * Creates a body group named `group_name` whose elements have names from
   * `body_names`.
   * If a group named `group_name` already exists, `body_names` is appended to
   * the end of the existing group.
   * If `body_names` have repeated names, the resulting body group will contain
   * multiple instances of the same body.
   * @param group_name Name of the body group, can be arbitrary.
   * @param body_names Vector of body names, must be present in the
   * RigidBodyTree passed to the constructor.
   *
   * @throws std::logic_error if elements in `body_names` cannot be found in
   * the RigidBodyTree referenced at construction time.
   */
  void AddBodyGroup(const std::string& group_name,
                    const std::vector<std::string>& body_names);

  /**
   * Creates a joint group named `group_name` whose elements have names from
   * `joint_names`.
   * If a group named `group_name` already exists, `joint_names` is appended to
   * the end of the existing group.
   * If `joint_names` have repeated names, the resulting joint group will
   * contain multiple instances of the same joint.
   * @param group_name Name of the joint group, can be arbitrary.
   * @param joint_names Vector of joint names, must be present in the
   * RigidBodyTree passed to the constructor.
   *
   * @throws std::logic_error if elements in `joint_names` cannot be found in
   * the RigidBodyTree referenced at construction time.
   */
  void AddJointGroup(const std::string& group_name,
                     const std::vector<std::string>& joint_names);

  bool has_body_group(const std::string& group_name) const {
    return body_groups_.find(group_name) != body_groups_.end();
  }

  bool has_joint_group(const std::string& group_name) const {
    return joint_groups_.find(group_name) != joint_groups_.end();
  }

  bool has_position_group(const std::string& group_name) const {
    return position_groups_.find(group_name) !=
           position_groups_.end();
  }

  bool has_velocity_group(const std::string& group_name) const {
    return velocity_groups_.find(group_name) !=
           velocity_groups_.end();
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
  const std::vector<int>& get_position_group(
      const std::string& group_name) const {
    return position_groups_.at(group_name);
  }

  /**
   * Returns the generalized velocity indices associated with the joint group
   * identified by `group_name`.
   *
   * @throws std::out_of_range if `group_name` is not found.
   */
  const std::vector<int>& get_velocity_group(
      const std::string& group_name) const {
    return velocity_groups_.at(group_name);
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
  get_position_groups() const {
    return position_groups_;
  }

  const std::map<std::string, std::vector<int>>&
  get_velocity_groups() const {
    return velocity_groups_;
  }

  const RigidBodyTree<T>& get_tree() const { return *tree_; }

 private:
  const RigidBodyTree<T>* tree_;

  std::map<std::string, std::vector<const RigidBody<T>*>> body_groups_;
  std::map<std::string, std::vector<const DrakeJoint*>> joint_groups_;

  std::map<std::string, std::vector<int>> position_groups_;
  std::map<std::string, std::vector<int>> velocity_groups_;
};

}  // namespace parsers
}  // namespace drake
