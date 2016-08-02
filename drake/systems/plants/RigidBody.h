#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>

#include "drake/common/eigen_types.h"
#include "drake/drakeRBM_export.h"
#include "drake/systems/plants/collision/DrakeCollision.h"
#include "drake/systems/plants/joints/DrakeJoint.h"

class DRAKERBM_EXPORT RigidBody {
 private:
  std::unique_ptr<DrakeJoint> joint;
  DrakeCollision::bitmask collision_filter_group;
  DrakeCollision::bitmask collision_filter_ignores;

 public:
  RigidBody();

  /**
   * @brief Name of the body.
   *
   * An accessor for the name of the body that this rigid body represents.
   *
   * @return The name of the body that's modeled by this rigid body.
   */
  const std::string& get_name() const;

  /**
   * Sets the name of this rigid body.
   */
  void set_name(const std::string& name);

  /**
   * An accessor for the name of the model or robot that this rigid body is
   * a part of.
   *
   * @return The name of the model that this rigid body belongs to.
   */
  // TODO(amcastro-tri): Move concept of world out of here as per #2318.
  const std::string& get_model_name() const;

  /**
   * Sets the name of the model to which this rigid body belongs.
   */
  void set_model_name(const std::string& name);

  /**
   * Returns the ID of the model to which this rigid body belongs.
   */
  int get_model_id() const;

  /**
   * Sets the ID of the model to which this rigid body belongs.
   */
  void set_model_id(int model_id);

  /**
   * Sets the parent joint through which this rigid body connects to its parent
   * rigid body.
   *
   * @param[in] joint The parent joint of this rigid body. Note that this
   * rigid body assumes ownership of this joint.
   */
  void setJoint(std::unique_ptr<DrakeJoint> joint);

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
   * Sets the start index of this rigid body's position state within the
   * `RigidBodyTree`'s state vector.
   */
  void set_position_start_index(int position_start_index);

  /**
   * Returns the start index of this rigid body's position state within the
   * `RigidBodyTree`'s state vector.
   */
  int get_position_start_index() const;

  /**
   * Sets the start index of this rigid body's velocity state within the
   * `RigidBodyTree`'s state vector.
   */
  void set_velocity_start_index(int velocity_start_index);

  /**
   * Returns the start index of this rigid body's velocity state within the
   * `RigidBodyTree`'s state vector.
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
   * @returns A reference to an `std::vector` of collision elements that
   * represent the collision geometry of this rigid body.
   */
  std::vector<DrakeCollision::ElementId>& get_mutable_collision_element_ids();

  /**
   * @returns A map of collision element group names to vectors of collision
   * element IDs. These are the collision element groups created through calls
   * to RigidBody::AddCollisionElementToGroup().
   */
  const std::map<std::string, std::vector<DrakeCollision::ElementId>>&
      get_group_to_collision_ids_map() const;

  /**
   * @returns A map of collision element group names to vectors of collision
   * element IDs. These are the collision element groups created through calls
   * to RigidBody::AddCollisionElementToGroup().
   */
  std::map<std::string, std::vector<DrakeCollision::ElementId>>&
    get_mutable_group_to_collision_ids_map();


  void setCollisionFilter(const DrakeCollision::bitmask& group,
                          const DrakeCollision::bitmask& ignores);

  const DrakeCollision::bitmask& getCollisionFilterGroup() const {
    return collision_filter_group;
  }
  void setCollisionFilterGroup(const DrakeCollision::bitmask& group) {
    this->collision_filter_group = group;
  }

  const DrakeCollision::bitmask& getCollisionFilterIgnores() const {
    return collision_filter_ignores;
  }
  void setCollisionFilterIgnores(const DrakeCollision::bitmask& ignores) {
    this->collision_filter_ignores = ignores;
  }

  void addToCollisionFilterGroup(const DrakeCollision::bitmask& group) {
    this->collision_filter_group |= group;
  }
  void ignoreCollisionFilterGroup(const DrakeCollision::bitmask& group) {
    this->collision_filter_ignores |= group;
  }
  void collideWithCollisionFilterGroup(const DrakeCollision::bitmask& group) {
    this->collision_filter_ignores &= ~group;
  }

  // TODO(amcastro-tri): Change to is_adjacent_to().
  bool adjacentTo(const RigidBody& other) const;

  bool CollidesWith(const RigidBody& other) const {
    bool ignored =
        this == &other || adjacentTo(other) ||
        (collision_filter_group & other.getCollisionFilterIgnores()).any() ||
        (other.getCollisionFilterGroup() & collision_filter_ignores).any();
    return !ignored;
  }

  bool appendCollisionElementIdsFromThisBody(
      const std::string& group_name,
      std::vector<DrakeCollision::ElementId>& ids) const;

  bool appendCollisionElementIdsFromThisBody(
      std::vector<DrakeCollision::ElementId>& ids) const;

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
  // note: it's very ugly, but parent, dofnum, and pitch also exist currently
  // (independently) at the RigidBodyTree level to represent the featherstone
  // structure.  this version is for the kinematics.

  Eigen::Matrix3Xd contact_pts;

  /// The mass of this rigid body.
  double mass;

  /// The center of mass of this rigid body.
  Eigen::Vector3d com;

  /// The spatial rigid body inertia of this rigid body.
  drake::SquareTwistMatrix<double> I;

  friend std::ostream& operator<<(std::ostream& out, const RigidBody& b);

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

 private:
  // The name of this rigid body.
  std::string name_;

  // The name of the model to which this rigid body belongs.
  std::string model_name_;

  // A unique ID for each model. It uses 0-index, starts from 0.
  int model_id_{0};

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
};
