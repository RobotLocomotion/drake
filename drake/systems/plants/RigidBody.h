#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>

#include "drake/common/drake_deprecated.h"
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
  /**
   * This constructor.
   *
   * @param[in] model_name The name of the model to which this rigid body
   * belongs.
   *
   * @param[in] model_id The ID of the model to which this rigid body belongs.
   *
   * @param[in] name The name of the rigid body.
   */
  explicit RigidBody(const std::string& model_name = "", int model_id = 0,
    const std::string& name = "");

  /**
   * Returns the name of this rigid body.
   */
#ifndef SWIG
  DRAKE_DEPRECATED("Please use get_name().")
#endif
  const std::string& name() const;

  /**
   * Returns the name of this rigid body.
   */
  const std::string& get_name() const;

  /**
   * Returns the name of the model to which this rigid body belongs.
   */
#ifndef SWIG
  DRAKE_DEPRECATED("Please use get_model_name().")
#endif
  const std::string& model_name() const;

  // TODO(amcastro-tri): Move concept of world out of here as per #2318.
  /**
   * Returns the name of the model to which this rigid body belongs.
   */
  const std::string& get_model_name() const;

  /**
   * Returns the ID of the model to which this rigid body belongs.
   */
  int get_model_id() const;

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
  /// The name of this rigid body.
  const std::string name_;

  /// The name of the model to which this rigid body belongs.
  std::string model_name_;

  // TODO(liang.fok): Make the following variable private prior to Release 1.0.
  //                  Once private, rename it to be model_id_.
  /// A unique ID for each model. It uses 0-index, starts from 0.
#ifndef SWIG
  DRAKE_DEPRECATED("Please use get_model_id() and set_model_id().")
#endif
  const int robotnum;
  // note: it's very ugly, but parent, dofnum, and pitch also exist currently
  // (independently) at the RigidBodyTree level to represent the featherstone
  // structure.  this version is for the kinematics.

  // TODO(amcastro-tri): Make it private and change to parent_.
  /// The rigid body that's connected to this rigid body's joint.
  RigidBody* parent;

  /// The index of this rigid body in the rigid body tree.
  int body_index;

  /// The starting index of this rigid body's joint's position value(s) within
  /// the parent tree's state vector.
  int position_num_start;

  /// The starting index of this rigid body's joint's velocity value(s) within
  /// the parent tree's state vector.
  int velocity_num_start;

  /// A list of visual elements for this RigidBody
  DrakeShapes::VectorOfVisualElements visual_elements;

  std::vector<DrakeCollision::ElementId> collision_element_ids;
  std::map<std::string, std::vector<DrakeCollision::ElementId> >
      collision_element_groups;

  Eigen::Matrix3Xd contact_pts;

  /// The mass of this rigid body.
  double mass;

  /// The center of mass of this rigid body.
  Eigen::Vector3d com;

  /// The spatial rigid body inertia of this rigid body.
  drake::SquareTwistMatrix<double> I;

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
};
