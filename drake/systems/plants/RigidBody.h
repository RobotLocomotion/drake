#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>

#include "drake/drakeRBM_export.h"
#include "drake/systems/plants/collision/DrakeCollision.h"
#include "drake/systems/plants/joints/DrakeJoint.h"

class DRAKERBM_EXPORT RigidBody {
 private:
  std::unique_ptr<DrakeJoint> joint;

  // TODO(amcastro-tri): move this to CollisionElement.
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
  const std::string& name() const;

  /**
   * An accessor for the name of the model or robot that this rigid body is
   * a part of.
   *
   * @return The name of the model that this rigid body belongs to.
   */
  // TODO(amcastro-tri): Move concept of world out of here as per #2318.
  const std::string& model_name() const;

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

  bool hasParent() const;

  /**
   * Checks if a particular rigid body is the parent of this rigid body.
   *
   * @param[in] other The potential parent of this rigid body.
   * @return true if the supplied rigid body parameter other is the parent of
   * this rigid body.
   */
  bool has_as_parent(const RigidBody& other) const { return parent == &other; }

  void addVisualElement(const DrakeShapes::VisualElement& elements);

  const DrakeShapes::VectorOfVisualElements& getVisualElements() const;

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
  bool adjacentTo(const RigidBody& other) const {
    return ((has_as_parent(other) && !(joint && joint->isFloating())) ||
            (other.has_as_parent(*this) &&
             !(other.joint && other.joint->isFloating())));
  }

  bool CanCollideWith(const RigidBody& other) const {
    bool ignored =
        this == &other || adjacentTo(other) ||
        (collision_filter_group & other.getCollisionFilterIgnores()).any() ||
        (other.getCollisionFilterGroup() & collision_filter_ignores).any();
    return !ignored;
  }

  bool appendCollisionElementIdsFromThisBody(
      const std::string& group_name,
      std::vector<DrakeCollision::CollisionElementId>& ids) const;

  bool appendCollisionElementIdsFromThisBody(
      std::vector<DrakeCollision::CollisionElementId>& ids) const;

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
  /**
   * The name of the body that this rigid body represents.
   */
  std::string name_;

  /**
   * The name of the model to which this rigid body belongs.
   */
  std::string model_name_;

  /**
   * A unique ID for each model. It uses 0-index, starts from 0.
   */
  int robotnum;
  // note: it's very ugly, but parent, dofnum, and pitch also exist currently
  // (independently) at the RigidBodyTree level to represent the featherstone
  // structure.  this version is for the kinematics.

  // TODO(amcastro-tri): Make it private and change to parent_.
  RigidBody* parent;

  int body_index;
  int position_num_start;
  int velocity_num_start;

  DrakeShapes::VectorOfVisualElements visual_elements;

  std::vector<DrakeCollision::CollisionElementId> collision_element_ids;

  typedef std::vector<DrakeCollision::CollisionElement*>
      CollisionElementsVector;
  typedef typename CollisionElementsVector::iterator CollisionElementsIterator;
  typedef typename CollisionElementsVector::const_iterator
      CollisionElementsConstIterator;
  std::map<std::string, std::vector<DrakeCollision::CollisionElementId> >
      collision_element_groups;

  CollisionElementsIterator CollisionElementsBegin() {
    return collision_elements_.begin();
  }

  CollisionElementsIterator CollisionElementsEnd() {
    return collision_elements_.end();
  }

  /** Adds collision element `e` to this rigid body.
  @param e The collision element being added to this body. **/
  void AddCollisionElement(DrakeCollision::CollisionElement* e) {
    collision_elements_.push_back(e);
  }

  /** Adds body to a given collision clique by clique id.

  This call adds each of the collision elements in this body to the provided
  collision clique.

  @param[in] clique_id Collision clique id. Collision elements in this clique
  do not interact.

  @see CollisionElement::AddToCollisionClique. **/
  void AddToCollisionClique(int clique_id);

  Eigen::Matrix3Xd contact_pts;

  double mass;
  Eigen::Vector3d com;
  Eigen::Matrix<double, TWIST_SIZE, TWIST_SIZE> I;

  friend std::ostream& operator<<(std::ostream& out, const RigidBody& b);

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

 private:
  CollisionElementsVector collision_elements_;
};
