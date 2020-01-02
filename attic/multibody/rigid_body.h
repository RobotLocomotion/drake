#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/collision/drake_collision.h"
#include "drake/multibody/joints/drake_joint.h"

#ifndef DRAKE_DOXYGEN_CXX
namespace drake {
namespace internal {

class RigidBodyAttorney;

}  // namespace internal
}  // namespace drake
#endif  // DRAKE_DOXYGEN_CXX

template <typename T>
class RigidBody {
 public:
  RigidBody();

  /**
   * Returns a clone of this RigidBody.
   *
   * @attention The following are not cloned:
   *
   *    - the joint
   *    - the parent %RigidBody
   *    - the visual elements
   *    - the collision elements
   *
   * The parent is *not* cloned because the reference to it can only be
   * determined by the RigidBodyTree (which owns both this body and the parent
   * body). Both the parent and the joint are expected to be set by calling
   * add_joint().
   *
   * The visual and collision elements will be cloned pending identified need.
   */
  std::unique_ptr<RigidBody<T>> Clone() const;

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
      throw std::logic_error(
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
  const DrakeJoint& getJoint() const {
    DRAKE_DEMAND(joint_ != nullptr);
    return *joint_;
  }

  /**
   * An accessor to this rigid body's mutable inboard joint. Also called "parent joint".
   *
   * @throws std::runtime_error if there is no joint (joint == nullptr)
   * @return The mutable inboard joint of this rigid body.
   */
  DrakeJoint& get_mutable_joint() {
    DRAKE_THROW_UNLESS(joint_ != nullptr);
    if (body_index_ == 0)
      throw std::runtime_error("This method cannot be called on world body");
    return *joint_;
  }

  /**
   * Reports if the body has a parent joint.
   */
  bool has_joint() const { return joint_ != nullptr; }

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
  const RigidBody* get_parent() const { return parent_; }

  /**
   * Returns whether this RigidBody has a "parent", which is a RigidBody that is
   * connected to this RigidBody via a DrakeJoint and is closer to the root of
   * the RigidBodyTree relative to this RigidBody. In other words, the parent
   * RigidBody is part of a kinematic path from this RigidBody to the root of
   * the RigidBodyTree. Thus, by definition, all RigidBody objects should have a
   * parent RigidBody except for the RigidBodyTree's root, which is the world.
   */
  bool has_parent_body() const { return parent_ != nullptr; }

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


  /// Sets the "body index" of this `RigidBody`. The "body index" is the
  /// index of this `RigidBody` within the vector of `RigidBody` objects
  /// within the `RigidBodyTree`.
  /// Users should NOT call this method. It is only here to be used
  /// internally by RigidBodyTree.
  void set_body_index(int body_index);

  /**
   * Returns the "body index" of this `RigidBody`. This is the index within the
   * vector of `RigidBody` objects within the `RigidBodyTree`.
   */
  int get_body_index() const { return body_index_; }

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
  int get_position_start_index() const { return position_start_index_; }

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
  int get_velocity_start_index() const { return velocity_start_index_; }

  void AddVisualElement(const DrakeShapes::VisualElement& elements);

  const DrakeShapes::VectorOfVisualElements& get_visual_elements() const;

  // TODO(SeanCurtis-TRI): This shouldn't be called publicly. Collision elements
  // have to be processed in the context of the rigid body tree.  Long term,
  // this will be displaced into SceneGraph.  Short term, just don't call
  // it. If you need to add a collision element to a body, add it through
  // RigidBodyTree::addCollisionElement.
  /**
   * Adds the given collision @p element to the body with the given group name.
   * @param[in] group_name The collision element's group name.
   * @param[in] element The element to associate with the rigid body.
   * @pre `element` has not already been added to this body.
   */
  void AddCollisionElement(const std::string& group_name,
                           drake::multibody::collision::Element* element);

  /**
   * @returns A reference to an `std::vector` of collision elements that
   * represent the collision geometry of this rigid body.
   */
  const std::vector<drake::multibody::collision::ElementId>&
  get_collision_element_ids() const { return collision_element_ids_; }

  /**
   * Returns a reference to an `std::vector` of collision elements that
   * represent the collision geometry of this rigid body.
   */
  std::vector<drake::multibody::collision::ElementId>&
  get_mutable_collision_element_ids();

  /**
   * Returns a map of collision element group names to vectors of collision
   * element IDs. These are the collision element groups created through calls
   * to RigidBody::AddCollisionElementToGroup().
   */
  const std::map<std::string,
                 std::vector<drake::multibody::collision::ElementId>>&
  get_group_to_collision_ids_map() const;

  /**
   * Returns a map of collision element group names to vectors of collision
   * element IDs. These are the collision element groups created through calls
   * to RigidBody::AddCollisionElementToGroup().
   */
  std::map<std::string, std::vector<drake::multibody::collision::ElementId>>&
  get_mutable_group_to_collision_ids_map();

  /**
   * Reports if there is a path in this tree from this body to the world where
   * all joints are *fixed*. This method throws an exception if the
   * RigidBodyTree is invalid in that:
   *    - This node is the descendant of a parentless node that is *not* the
   *      world node, or
   *    - This node does not have a valid DrakeJoint.
   */
  bool IsRigidlyFixedToWorld() const;

  /**
   * Reports `X_WBₙ`, the pose of this body, `Bₙ`, in the world frame based on
   * the *rigid* kinematic path from `Bₙ` to `W`.  As such,
   * the world-fixed pose is only defined for bodies that are rigidly fixed to
   * the world.
   *
   * For this body, with depth `n` in the tree, `Bₙ`, `X_WBₙ` is defined as:
   *
   * `X_WBₙ ≡ X_WB₁ * X_B₁B₂ * ... * X_Bₙ₋₂Bₙ₋₁ * X_Bₙ₋₁Bₙ`
   *
   * `X_Bₖ₋₁Bₖ` represents the transform from one body's frame
   * (`Bₖ`) to its parent's frame (Bₖ₋₁). By construction, body `Bₖ` has a
   * single inboard joint. This joint defines several frames, discussed in
   * @ref rigid_body_tree_frames, including its parent frame: `Pₖ ≡ Bₖ₋₁`. This
   * allows us to compute `X_Bₖ₋₁Bₖ` as follows:
   *
   * - `X_Bₖ₋₁Bₖ = X_PₖBₖ` because `Pₖ ≡ Bₖ₋₁`
   * - `X_PₖBₖ ≡ X_PₖFₖ * X_FₖMₖ(q) * X_MₖBₖ`, where:
   *    - `X_MₖBₖ = I` in Drake's implementation.
   *    - `X_FₖMₖ(q) = I` because we only follow FixedJoint instances.
   *
   *
   * If the body is not rigidly fixed to the world, an exception will be thrown.
   * @return `X_WBₙ`.
   * @see IsRigidlyFixedToWorld
   */
  Eigen::Isometry3d ComputeWorldFixedPose() const;

  // TODO(amcastro-tri): Change to is_adjacent_to().
  // TODO(SeanCurtis-TRI): This method is only used by the collision clique
  // calculation.  Maybe it would be better if the name reflected this: e.g.,
  // is_collision_adjacent(), or some such thing.
  /**
   * Reports if this body is considered "adjacent" to the given body.
   *
   * "Adjacency" refers to the idea that the bodies are connected to each other
   * in the rigid body tree by a non-floating joint. By this definition,
   * a rigid body is *not* adjacent to itself.
   *
   * In the degenerate case where one rigid body is a parent of the other, but
   * with no joint assigned, the rigid bodies will be considered adjacent.
   * Conversely, the degenerate case where a joint is assigned, but the parent
   * relationship is not set, the rigid bodies will *not* be considered
   * adjacent.
   * @param[in] other The body to test against this body.
   * @returns `true` if the bodies are "adjacent".
   */
  bool adjacentTo(const RigidBody& other) const;

  /**
   * Returns `true` if this body should be checked for collisions
   * with the @p other body.  CanCollideWith should be commutative: A can
   * collide with B implies B can collide with A.
   * @param[in] other The body to query against.
   * @returns `true` if collision between this and other should be tested.
   */
  bool CanCollideWith(const RigidBody& other) const;

  bool appendCollisionElementIdsFromThisBody(
      const std::string& group_name,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<drake::multibody::collision::ElementId>& ids) const;

  bool appendCollisionElementIdsFromThisBody(
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<drake::multibody::collision::ElementId>& ids) const;

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
  const drake::SquareTwistMatrix<double>& get_spatial_inertia() const {
    return spatial_inertia_;
  }

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
  friend std::ostream& operator<<(
      std::ostream& out, const RigidBody<double>& b);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::vector<drake::multibody::collision::Element*>
      CollisionElementsVector;
  typedef typename CollisionElementsVector::iterator CollisionElementsIterator;

  CollisionElementsIterator collision_elements_begin() {
    return collision_elements_.begin();
  }

  CollisionElementsIterator collision_elements_end() {
    return collision_elements_.end();
  }

  /**
   * Reports the total number of *registered* collision elements attached to
   * this body. See Model::AddElement() for definition of "registered".
   */
  int get_num_collision_elements() const {
    return static_cast<int>(collision_elements_.size());
  }

 private:
#ifndef DRAKE_DOXYGEN_CXX
  friend class drake::internal::RigidBodyAttorney;
#endif  // DRAKE_DOXYGEN_CXX

  // Assumes that elements have only been added via `AddCollisionElement`,
  // which should uphold the following:
  // - collision_element_ids_ has no duplicates
  // - collision_elements_ has exactly one entry per collision_element_ids item,
  // with a matching id
  // - collision_element_group_ ids have no duplicates, not even under
  // different group names
  void RemoveCollisionGroupAndElements(const std::string& group_name);

  // TODO(tkoolen): It's very ugly, but parent, dofnum, and pitch also exist
  // currently (independently) at the RigidBodyTree level to represent the
  // featherstone structure. This version is for the kinematics.

  // The "parent" joint of this rigid body. This is the joint through which this
  // rigid body connects to the rest of the rigid body tree.
  std::unique_ptr<DrakeJoint> joint_;

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
  std::vector<drake::multibody::collision::ElementId> collision_element_ids_;

  // A map of groups of collision element IDs. This is just for conveniently
  // accessing particular groups of collision elements. The groups do not imply
  // anything in terms of how the collision elements relate to each other.
  std::map<std::string, std::vector<drake::multibody::collision::ElementId>>
      collision_element_groups_;

  // The contact points this rigid body has with its environment.
  Eigen::Matrix3Xd contact_points_;

  // The mass of this rigid body.
  double mass_{0};

  // The center of mass of this rigid body.
  Eigen::Vector3d center_of_mass_;

  // The spatial inertia of this rigid body.
  drake::SquareTwistMatrix<double> spatial_inertia_;

  // TODO(SeanCurtis-TRI): This data is only used in the compilation of the
  // body.  As such, it should be moved into a factory so that the runtime
  // class only has runtime data.
  CollisionElementsVector collision_elements_;
};

// Forward declaration for attorney-client.
template <typename T>
class RigidBodyTree;

#ifndef DRAKE_DOXYGEN_CXX
namespace drake {
namespace internal {

class RigidBodyAttorney {
 private:
  template <typename T>
  friend class ::RigidBodyTree;

  // Removes collision element group, but does NOT make the
  // necessary calls to `collision::Model::RemoveElement`.
  // To be used by `RigidBodyTree::removeCollisionGroupsIf`.
  template <typename T>
  static void RemoveCollisionGroupAndElements(
      RigidBody<T>* body, const std::string& group_name) {
    body->RemoveCollisionGroupAndElements(group_name);
  }
};

}  // namespace internal
}  // namespace drake
#endif  // DRAKE_DOXYGEN_CXX
