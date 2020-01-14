#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_tree.h"

/**
 * This class provides a way to create aliases to groups of RigidBody or
 * DrakeJoint objects. The creation of these groups can be done
 * programmatically.
 *
 * For example, suppose we have a RigidBodyTree with 6 links named
 * [link0 ~ link5], and 6 joints [base, joint0 ~ joint4]. We can "rename"
 * `link0` to be `robot_base` by creating a body group named `robot_base`
 * with one entry `link0`, and accessing the first element in this body group.
 * Alias names for groups of joints can also be similarly created. Furthermore,
 * the corresponding indices to the generalized position and velocity are
 * computed and accessible through an instance of this class.
 *
 * Body groups and joint groups are independent, so a body group can have the
 * same name of a joint group. However, all body groups have unique group
 * names, and so do all joint groups. For each body or joint group, its
 * members are unique. A body or joint can belong to many groups. When adding
 * new members to an existing group, the new members will be appended to the
 * existing group.
 */
template <typename T>
class RigidBodyTreeAliasGroups {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidBodyTreeAliasGroups)

  static constexpr char kBodyGroupsKeyword[] = "body_groups";
  static constexpr char kJointGroupsKeyword[] = "joint_groups";

  /**
   * Constructor for RigidBodyTreeAliasGroups.
   * @param tree Pointer to the RigidBodyTree, which is aliased internally. So
   * @p tree needs to outlive this object.
   */
  explicit RigidBodyTreeAliasGroups(const RigidBodyTree<T>* tree)
      : tree_(*tree) {}

  /**
   * Creates a body group named @p group_name whose elements have names from
   * @p body_names. If a group named @p group_name already exists, @p body_names
   * is appended to the end of the existing group. The resulting group will not
   * have duplicated elements.
   * @param group_name Name of the body group, can be arbitrary.
   * @param body_names Vector of body names, must be present in the
   * RigidBodyTree passed to the constructor.
   *
   * @throws std::logic_error if elements in @p body_names cannot be found in
   * the RigidBodyTree referenced at construction time.
   */
  void AddBodyGroup(const std::string& group_name,
                    const std::vector<std::string>& body_names);

  /**
   * Creates a joint group named @p group_name whose elements have names from
   * @p joint_names. If a group named @p group_name already exists,
   * @p joint_names is appended to the end of the existing group. The resulting
   * group will not have duplicated elements.
   * @param group_name Name of the joint group, can be arbitrary.
   * @param joint_names Vector of joint names, must be present in the
   * RigidBodyTree passed to the constructor.
   *
   * @throws std::logic_error if elements in @p joint_names cannot be found in
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
   * Returns the body group identified by @p group_name.
   *
   * @throws std::out_of_range if @p group_name is not found.
   */
  const std::vector<const RigidBody<T>*>& get_body_group(
      const std::string& group_name) const {
    return body_groups_.at(group_name);
  }

  /**
   * Returns the body aliased by @p group_name. The body group referenced by
   * @p group_name must contain exactly one element.
   */
  const RigidBody<T>* get_body(const std::string& group_name) const {
    const auto& group = get_body_group(group_name);
    DRAKE_DEMAND(group.size() == 1);
    return group.front();
  }

  /**
   * Returns the joint group identified by @p group_name.
   *
   * @throws std::out_of_range if @p group_name is not found.
   */
  const std::vector<const DrakeJoint*>& get_joint_group(
      const std::string& group_name) const {
    return joint_groups_.at(group_name);
  }

  /**
   * Returns the generalized position indices associated with the joint group
   * identified by @p group_name.
   *
   * @throws std::out_of_range if @p group_name is not found.
   */
  const std::vector<int>& get_position_group(
      const std::string& group_name) const {
    return position_groups_.at(group_name);
  }

  /**
   * Returns the generalized velocity indices associated with the joint group
   * identified by @p group_name.
   *
   * @throws std::out_of_range if @p group_name is not found.
   */
  const std::vector<int>& get_velocity_group(
      const std::string& group_name) const {
    return velocity_groups_.at(group_name);
  }

  const std::unordered_map<std::string, std::vector<const RigidBody<T>*>>&
  get_body_groups() const {
    return body_groups_;
  }

  const std::unordered_map<std::string, std::vector<const DrakeJoint*>>&
  get_joint_groups() const {
    return joint_groups_;
  }

  const std::unordered_map<std::string, std::vector<int>>&
  get_position_groups() const {
    return position_groups_;
  }

  const std::unordered_map<std::string, std::vector<int>>&
  get_velocity_groups() const {
    return velocity_groups_;
  }

  const RigidBodyTree<T>& get_tree() const { return tree_; }

 private:
  const RigidBodyTree<T>& tree_;

  std::unordered_map<std::string, std::vector<const RigidBody<T>*>>
      body_groups_;
  std::unordered_map<std::string, std::vector<const DrakeJoint*>> joint_groups_;

  std::unordered_map<std::string, std::vector<int>> position_groups_;
  std::unordered_map<std::string, std::vector<int>> velocity_groups_;
};
