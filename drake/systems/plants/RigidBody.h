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

  /**
   * Returns a mutable pointer to this rigid body's parent rigid body.
   */
  RigidBody* get_mutable_parent();

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

  const DrakeShapes::VectorOfVisualElements& GetVisualElements() const;

  /**
   * Adds a collision element ID to this rigid body. This indicates that the
   * collision element with this ID may collide against this rigid body and thus
   * must be checked when computing collision reaction forces.
   */
  void AddCollisionElement(DrakeCollision::ElementId id);

  /**
   * Adds a collision element ID to a particular collision group. Collision
   * elements within a single collision group may collide with each other.
   * Those in different collision groups cannot collide with each other.
   */
  void AddCollisionElementToGroup(const std::string& group_name,
      DrakeCollision::ElementId id);

  /**
   * Returns a reference to a vector of collision element IDs belonging to the
   * collision elements that this rigid body can be in collision with.
   */
  const std::vector<DrakeCollision::ElementId>& get_collision_element_ids()
      const;

  /**
   * Returns a reference to a vector of collision element IDs belonging to the
   * collision elements that this rigid body can be in collision with.
   */
  std::vector<DrakeCollision::ElementId>& get_mutable_collision_element_ids();

  const std::map<std::string, std::vector<DrakeCollision::ElementId>>&
      get_collision_element_groups() const;

  std::map<std::string, std::vector<DrakeCollision::ElementId>>&
    get_mutable_collision_element_groups();


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
   * Returns a matrix of contact points that this rigid body has with its
   * environment.
   */
  const Eigen::Matrix3Xd& get_contact_points() const;

  /**
   * Saves the contact points that this rigid body has with its environment.
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
   * Sets the spatial rigid body inertia of this rigid body.
   */
  void set_inertia_matrix(const drake::SquareTwistMatrix<double>&
      inertia_matrix);

  /**
   * Returns the spatial rigid body inertia of this rigid body.
   */
  const drake::SquareTwistMatrix<double>& get_inertia_matrix()
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

  // TODO(amcastro-tri): move to a better place (h + cc files).
  class DRAKERBM_EXPORT CollisionElement : public DrakeCollision::Element {
   public:
    CollisionElement(const CollisionElement& other);
    // TODO(amcastro-tri): The RigidBody should be const?
    // TODO(amcastro-tri): It should not be possible to construct a
    // CollisionElement without specifying a geometry. Remove this constructor.
    CollisionElement(const Eigen::Isometry3d& T_element_to_link,
                     const RigidBody* const body);
    CollisionElement(const DrakeShapes::Geometry& geometry,
                     const Eigen::Isometry3d& T_element_to_link,
                     const RigidBody* const body);
    virtual ~CollisionElement() {}

    CollisionElement* clone() const override;

    bool CollidesWith(const DrakeCollision::Element* other) const override;

#ifndef SWIG
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
  };

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
  std::unique_ptr<DrakeJoint> joint;

  // A bitmask that determines the collision groups that this rigid body is part
  // of.
  DrakeCollision::bitmask collision_filter_group;

  // A bitmask that determines which collision groups should be ignored when
  // determinine the colliisons associated with this rigid body.
  DrakeCollision::bitmask collision_filter_ignores;

  // The name of this rigid body.
  std::string name_;

  // The name of the model to which this rigid body belongs.
  std::string model_name_;

  // A unique ID for each model. It uses 0-index, starts from 0.
  int model_id_;

  // The rigid body that's connected to this rigid body's joint.
  RigidBody* parent_;

  // The index of this rigid body in the rigid body tree.
  int body_index_;

  // The starting index of this rigid body's joint's position value(s) within
  // the parent tree's state vector.
  int position_start_index_;

  // The starting index of this rigid body's joint's velocity value(s) within
  // the parent tree's state vector.
  int velocity_start_index_;

  // A list of visual elements for this RigidBody.
  DrakeShapes::VectorOfVisualElements visual_elements_;

  // A list of collision element IDs of collision elements that may collide with
  // this rigid body.
  std::vector<DrakeCollision::ElementId> collision_element_ids_;

  // A map of groups of collision element IDs. Each group contains collision
  // elements that may collide with each other. Collision groups in different
  // groups do not collide with each other.
  std::map<std::string, std::vector<DrakeCollision::ElementId>>
      collision_element_groups_;

  // The contact points this rigid body has with its environment.
  Eigen::Matrix3Xd contact_points_;

  // The mass of this rigid body.
  double mass_;

  // The center of mass of this rigid body.
  Eigen::Vector3d center_of_mass_;

  // The spatial rigid body inertia of this rigid body.
  drake::SquareTwistMatrix<double> I_;
};
