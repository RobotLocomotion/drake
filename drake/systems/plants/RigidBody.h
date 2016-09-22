#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/drakeRBM_export.h"
#include "drake/systems/plants/collision/DrakeCollision.h"
#include "drake/systems/plants/joints/DrakeJoint.h"

class DRAKERBM_EXPORT RigidBody {
 public:
  RigidBody();

  /**
   * Returns the name of this rigid body.
   */
  const std::string& get_name() const;

  /**
   * Sets the name of this rigid body.
   */
  void set_name(const std::string& name);

  /**
   * Returns the name of the model defining this rigid body.
   */
  const std::string& get_model_name() const;

  /**
   * Sets the name of the model defining this rigid body.
   */
  void set_model_name(const std::string& name);

  /**
   * Returns the ID of the model instance to which this rigid body belongs.
   */
  int get_model_instance_id() const;

  /**
   * Sets the ID of the model instance to which this rigid body belongs.
   */
  void set_model_instance_id(int model_instance_id);

  /**
   * Sets the parent joint through which this rigid body connects to its parent
   * rigid body.
   *
   * @param[in] joint The parent joint of this rigid body. Note that this
   * rigid body assumes ownership of this joint.
   */
  // TODO(liang.fok): Deprecate this method in favor of add_joint().
  // This requires, among potentially other things, updating the parsers.
  void setJoint(std::unique_ptr<DrakeJoint> joint);

  /**
   * Adds degrees of freedom to this body by connecting it to @p parent with
   * @p joint. The body takes ownership of the joint.
   *
   * This method aborts with an error message if the user attempts to assign a
   * joint to a body that already has one.
   *
   * Note that this is specifically a tree joint and that by "parent" we mean a
   * body that is closer to "world" in the tree topology.
   *
   * The @p parent pointer is copied and stored meaning its lifetime must
   * exceed the lifetime of this RigidBody.
   *
   * @param[in] parent The RigidBody this body gets connected to.
   * @param[in] joint The DrakeJoint connecting this body to @p parent and
   * adding degrees of freedom to this body.
   * @returns A pointer to the joint just added to the body.
   */
  template<typename JointType>
  JointType* add_joint(RigidBody* parent, std::unique_ptr<JointType> joint) {
    if (joint_ != nullptr) {
      DRAKE_ABORT_MSG(
          "Attempting to assign a new joint to a body that already has one");
    }
    set_parent(parent);
    setJoint(move(joint));
    return static_cast<JointType*>(joint_.get());
  }

  /**
   * An accessor to this rigid body's parent joint. By "parent joint" we
   * mean the joint through which this rigid body connects to its parent rigid
   * body in the rigid body tree.
   *
   * @return The parent joint of this rigid body.
   */
  const DrakeJoint& getJoint() const;

  /**
   * Sets the parent rigid body. This is the rigid body that is connected to
   * this rigid body's joint.
   *
   * @param[in] parent A pointer to this rigid body's parent rigid body.
   */
  void set_parent(RigidBody* parent);

  /**
   * Returns a const pointer to this rigid body's parent rigid body.
   */
  const RigidBody* get_parent() const;

  /**
   * Returns whether this RigidBody has a "parent", which is a RigidBody that is
   * connected to this RigidBody via a DrakeJoint and is closer to the root of
   * the RigidBodyTree relative to this RigidBody. In other words, the parent
   * RigidBody is part of a kinematic path from this RigidBody to the root of
   * the RigidBodyTree. Thus, by definition, all RigidBody objects should have a
   * parent RigidBody except for the RigidBodyTree's root, which is the world.
   */
  bool has_parent_body() const;

  // TODO(liang.fok): Remove this deprecated method prior to Release 1.0.
#ifndef SWIG
  DRAKE_DEPRECATED("Please use has_parent_body().")
#endif
  bool hasParent() const;

  /**
   * Checks if a particular rigid body is the parent of this rigid body.
   *
   * @param[in] other The potential parent of this rigid body.
   * @return true if the supplied rigid body parameter other is the parent of
   * this rigid body.
   */
  bool has_as_parent(const RigidBody& other) const {
    return parent_ == &other;
  }

  /**
   * Sets the "body index" of this `RigidBody`. The "body index" is the index of
   * this `RigidBody` within the vector of `RigidBody` objects within the
   * `RigidBodyTree`.
   */
  void set_body_index(int body_index);

  /**
   * Returns the "body index" of this `RigidBody`. This is the index within the
   * vector of `RigidBody` objects within the `RigidBodyTree`.
   */
  int get_body_index() const;

  /**
   * Sets the start index of this rigid body's mobilizer joint's contiguous
   * generalized coordinates `q` (joint position state variables) within the
   * full RigidBodyTree generalized coordinate vector.
   *
   * For more details about the semantics of @p position_start_index, see the
   * documentation for RigidBodyTree.
   */
  void set_position_start_index(int position_start_index);

  /**
   * Returns the start index of this body's parent jont's position states; see
   * RigidBody::set_position_start_index() for more information.
   */
  int get_position_start_index() const;

  /**
   * Sets the start index of this rigid body's mobilizer joint's contiguous
   * generalized velocity `v` (joint velocity state variables) within the full
   * RigidBodyTree generalized velocity vector.
   *
   * For more details about the semantics of @p velocity_start_index, see the
   * documentation for RigidBodyTree.
   */
  void set_velocity_start_index(int velocity_start_index);

  /**
   * Returns the start index of this body's parent jont's velocity states; see
   * RigidBody::set_velocity_start_index() for more information.
   */
  int get_velocity_start_index() const;

  void AddVisualElement(const DrakeShapes::VisualElement& elements);

  const DrakeShapes::VectorOfVisualElements& get_visual_elements() const;

  /**
   * Adds a collision element to this rigid body by collision element @p id.
   * This effectively defines the collision geometry of this rigid body. If more
   * than one collision element is added, the resulting collision geometry is
   * the union of the individual geometries of each collision element.
   */
  void AddCollisionElement(DrakeCollision::ElementId id);

  /**
   * Adds a collision element represented by its @p id to the collision group
   * @p group_name. Collision groups are just a convenient way to group a
   * collection of collision elements so that they can be referenced by the name
   * of the group they belong to. There is no implication on whether these
   * elements can collide between them or not.
   *
   * Note that the collision element @p id must have already been passed to
   * RigidBody::AddCollisionElement().
   */
  void AddCollisionElementToGroup(const std::string& group_name,
      DrakeCollision::ElementId id);

  /**
   * @returns A reference to an `std::vector` of collision elements that
   * represent the collision geometry of this rigid body.
   */
  const std::vector<DrakeCollision::ElementId>& get_collision_element_ids()
      const;

  /**
   * Returns a reference to an `std::vector` of collision elements that
   * represent the collision geometry of this rigid body.
   */
  std::vector<DrakeCollision::ElementId>& get_mutable_collision_element_ids();

  /**
   * Returns a map of collision element group names to vectors of collision
   * element IDs. These are the collision element groups created through calls
   * to RigidBody::AddCollisionElementToGroup().
   */
  const std::map<std::string, std::vector<DrakeCollision::ElementId>>&
      get_group_to_collision_ids_map() const;

  /**
   * Returns a map of collision element group names to vectors of collision
   * element IDs. These are the collision element groups created through calls
   * to RigidBody::AddCollisionElementToGroup().
   */
  std::map<std::string, std::vector<DrakeCollision::ElementId>>&
    get_mutable_group_to_collision_ids_map();


  void setCollisionFilter(const DrakeCollision::bitmask& group,
                          const DrakeCollision::bitmask& ignores);

  const DrakeCollision::bitmask& getCollisionFilterGroup() const;

  void setCollisionFilterGroup(const DrakeCollision::bitmask& group);

  const DrakeCollision::bitmask& getCollisionFilterIgnores() const;

  void setCollisionFilterIgnores(const DrakeCollision::bitmask& ignores);

  void addToCollisionFilterGroup(const DrakeCollision::bitmask& group);

  void ignoreCollisionFilterGroup(const DrakeCollision::bitmask& group);

  void collideWithCollisionFilterGroup(const DrakeCollision::bitmask& group);

  // TODO(amcastro-tri): Change to is_adjacent_to().
  bool adjacentTo(const RigidBody& other) const;

  bool CollidesWith(const RigidBody& other) const;

  bool appendCollisionElementIdsFromThisBody(
      const std::string& group_name,
      std::vector<DrakeCollision::ElementId>& ids) const;

  bool appendCollisionElementIdsFromThisBody(
      std::vector<DrakeCollision::ElementId>& ids) const;

  /**
   * Returns the points on this rigid body that should be checked for collision
   * with the environment. These are the contact points that were saved by
   * RigidBody::set_contact_points().
   */
  const Eigen::Matrix3Xd& get_contact_points() const;

  /**
   * Saves the points on this rigid body that should be checked for collision
   * between this rigid body and the environment. These contact points can be
   * obtained through RigidBody::get_contact_points().
   */
  void set_contact_points(const Eigen::Matrix3Xd& contact_points);

  /**
   * Sets the mass of this rigid body.
   */
  void set_mass(double mass);

  /**
   * Returns the mass of this rigid body.
   */
  double get_mass() const;

  /**
   * Sets the center of mass of this rigid body. The center of mass is expressed
   * in this body's frame.
   */
  void set_center_of_mass(const Eigen::Vector3d& center_of_mass);

  /**
   * Gets the center of mass of this rigid body. The center of mass is expressed
   * in this body's frame.
   */
  const Eigen::Vector3d& get_center_of_mass() const;

  /**
   * Sets the spatial inertia of this rigid body.
   */
  void set_spatial_inertia(const drake::SquareTwistMatrix<double>&
      inertia_matrix);

  /**
   * Returns the spatial inertia of this rigid body.
   */
  const drake::SquareTwistMatrix<double>& get_spatial_inertia()
      const;

  /**
   * Transforms all of the visual, collision, and inertial elements associated
   * with this body to the proper joint frame.  This is necessary, for instance,
   * to support SDF loading where the child frame can be specified independently
   * from the joint frame. In our RigidBodyTree classes, the body frame IS the
   * joint frame.
   *
   * @param transform_body_to_joint The transform from this body's frame to the
   * joint's frame.
   */
  void ApplyTransformToJointFrame(
      const Eigen::Isometry3d& transform_body_to_joint);

 public:
  friend std::ostream& operator<<(std::ostream& out, const RigidBody& b);

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

 private:
  // TODO(tkoolen): It's very ugly, but parent, dofnum, and pitch also exist
  // currently (independently) at the RigidBodyTree level to represent the
  // featherstone structure. This version is for the kinematics.

  // The "parent" joint of this rigid body. This is the joint through which this
  // rigid body connects to the rest of the rigid body tree.
  std::unique_ptr<DrakeJoint> joint_;

  // A bitmask that determines the collision groups that this rigid body is part
  // of. If the i-th bit is set this rigid body belongs to the i-th collision
  // group. A rigid body can belong to several collision groups.
  DrakeCollision::bitmask collision_filter_group_;

  // A bitmask that determines which collision groups this rigid body does not
  // collide with. Thus, if the i-th bit is set this rigid body is not checked
  // for collisions with bodies in the i-th group.
  DrakeCollision::bitmask collision_filter_ignores_;

  // The name of this rigid body.
  std::string name_;

  // TODO(liang.fok) Remove this member variable, see:
  // https://github.com/RobotLocomotion/drake/issues/3053
  // The name of the model that defined this rigid body.
  std::string model_name_;

  // A unique ID for the model instance to which this body belongs.
  int model_instance_id_{0};

  // The rigid body that's connected to this rigid body's joint.
  RigidBody* parent_{nullptr};

  // The index of this rigid body in the rigid body tree.
  int body_index_{0};

  // The starting index of this rigid body's joint's position value(s) within
  // the parent tree's state vector.
  int position_start_index_{0};

  // The starting index of this rigid body's joint's velocity value(s) within
  // the parent tree's state vector.
  int velocity_start_index_{0};

  // A list of visual elements for this RigidBody.
  DrakeShapes::VectorOfVisualElements visual_elements_;

  // A list of collision element IDs of collision elements that represent the
  // geometry of this rigid body.
  std::vector<DrakeCollision::ElementId> collision_element_ids_;

  // A map of groups of collision element IDs. This is just for conveniently
  // accessing particular groups of collision elements. The groups do not imply
  // anything in terms of how the collision elements relate to each other.
  std::map<std::string, std::vector<DrakeCollision::ElementId>>
      collision_element_groups_;

  // The contact points this rigid body has with its environment.
  Eigen::Matrix3Xd contact_points_;

  // The mass of this rigid body.
  double mass_{0};

  // The center of mass of this rigid body.
  Eigen::Vector3d center_of_mass_;

  // The spatial inertia of this rigid body.
  drake::SquareTwistMatrix<double> spatial_inertia_;
};
