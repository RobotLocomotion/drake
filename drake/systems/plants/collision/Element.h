#pragma once

#include <stdint.h>
#include <memory>
#include <utility>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "drake/drakeCollision_export.h"
#include "drake/systems/plants/shapes/DrakeShapes.h"

// Forward declaration.
// RigidBody's interface is never used by the collision engine.
// It is however useful the concept of having a CollisionElement attached to a
// Body.
class RigidBody;

namespace DrakeCollision {
typedef uintptr_t ElementId;

class DRAKECOLLISION_EXPORT CollisionElement : public DrakeShapes::Element {
 public:
  CollisionElement(const Eigen::Isometry3d& T_element_to_local =
                       Eigen::Isometry3d::Identity(),
                   const RigidBody* const body = nullptr);

  CollisionElement(const DrakeShapes::Geometry& geometry,
                   const Eigen::Isometry3d& T_element_to_local =
                       Eigen::Isometry3d::Identity(),
                   const RigidBody* const body = nullptr);

  virtual ~CollisionElement() {}

  virtual CollisionElement* clone() const;

  ElementId getId() const;

  virtual bool isStatic() const { return false; }

  /**
   * Returns true if this element should be checked for collisions
   * with the other object.  CollidesWith should be symmetric: if
   * A collides with B, B collides with A.
   */
  virtual bool CollidesWith(const CollisionElement* other) const;

  /**
   * @brief Returns a pointer to the const RigidBody to which this
   * CollisionElement is attached.
   *
   * @throws A std::runtime_error if the CollisionElement is not attached to a
   * body.
   */
  // TODO(amcastro-tri): getBody() -> get_body()
  const RigidBody* const getBody() const;

  /**
   * @brief Adds this collsion element to collision group group_id
   *
   * CollisionElement's within a group do not collide.
   * Calling this method to add an element to a group it already belongs to does
   * not have any effect.
   */
  void add_to_collision_group(int group_id);

  int number_of_groups() const;

  std::vector<int> collision_groups() const;

  /**
   * A toString method for this class.
   */
  friend DRAKECOLLISION_EXPORT std::ostream& operator<<(
      std::ostream&, const CollisionElement&);

 protected:
  CollisionElement(const CollisionElement& other);

 private:
  ElementId id;
  const RigidBody* const body_{};

  // Collision groups in Drake are represented simply by an integer.
  // Collision elements in the same group do not collide.
  // A collision element can belong to more than one group.
  // Conceptually it would seem like std::set is the right fit for
  // CollisionElement::collision_groups_. However, std::set is really good for
  // dynamically adding elements to a set that needs to change.
  // Once you are done adding elements to your set, access time is poor when
  // compared to a simple std::vector (nothing faster than scanning a vector of
  // adjacent entries in memory).
  // Here adding elements to the groups vector only happens during problem
  // setup by the user or from a URDF/SDF file. What we really want is that once
  // this vector is setup, we can query it very fast during simulation.
  // This is done in CollisionElement::CollidesWith which to be Order(N)
  // requires the entries in CollisionElement::collision_groups_ to be sorted.
  std::vector<int> collision_groups_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace DrakeCollision
