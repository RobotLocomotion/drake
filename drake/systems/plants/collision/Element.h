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
// It is however useful for the concept of having a CollisionElement attached
// to a Body.
class RigidBody;

namespace DrakeCollision {
typedef uintptr_t ElementId;

class DRAKECOLLISION_EXPORT CollisionElement : public DrakeShapes::Element {
 public:
  CollisionElement(const Eigen::Isometry3d& T_element_to_local =
                       Eigen::Isometry3d::Identity());

  CollisionElement(const DrakeShapes::Geometry& geometry,
                   const Eigen::Isometry3d& T_element_to_local =
                       Eigen::Isometry3d::Identity());

  ~CollisionElement() override {}

  CollisionElement* clone() const override;

  ElementId getId() const;

  virtual bool isStatic() const { return false; }

  /** Returns `true` if this element should be checked for collisions
  with the other object.  CanCollideWith should be symmetric: if
  A collides with B, B collides with A. **/
  virtual bool CanCollideWith(const CollisionElement *other) const;

  /** Returns a pointer to the const RigidBody to which this CollisionElement
  is attached. **/
  // TODO(amcastro-tri): getBody() -> get_body()
  const RigidBody* const getBody() const;

  void set_rigid_body(const RigidBody* body);

  /** Adds this collision element to collision clique clique_id.

  CollisionElement's within a clique do not collide.
  Calling this method to add an element to a clique it already belongs to
  does not have any effect. **/
  void AddToCollisionClique(int clique_id);

  int number_of_cliques() const;

  const std::vector<int>& collision_cliques() const;

  /**
   * A toString method for this class.
   */
  friend DRAKECOLLISION_EXPORT std::ostream& operator<<(
      std::ostream&, const CollisionElement&);

 protected:
  CollisionElement(const CollisionElement& other);

 private:
  ElementId id;
  const RigidBody* body_{};

  // Collision cliques are defined as a group of collision elements that do not
  // collide.
  // Collision cliques in Drake are represented simply by an integer.
  // A collision element can belong to more than one clique.
  // Conceptually it would seem like std::set is the right fit for
  // CollisionElement::collision_cliques_. However, std::set is really good for
  // dynamically adding elements to a set that needs to change.
  // Once you are done adding elements to your set, access time is poor when
  // compared to a simple std::vector (nothing faster than scanning a vector of
  // adjacent entries in memory).
  // Here adding elements to the cliques vector only happens during problem
  // setup by the user or from a URDF/SDF file. What we really want is that once
  // this vector is setup, we can query it very fast during simulation.
  // This is done in CollisionElement::CanCollideWith which to be Order(N)
  // requires the entries in CollisionElement::collision_cliques_ to be sorted.
  std::vector<int> collision_cliques_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace DrakeCollision
