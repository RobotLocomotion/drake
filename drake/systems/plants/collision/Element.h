#pragma once

#include <cstdint>
#include <memory>
#include <utility>

#include <Eigen/Dense>

#include "drake/drakeCollision_export.h"
#include "drake/systems/plants/shapes/DrakeShapes.h"

// Forward declaration.
// This forward declaration is made in order to be able to add a reference
// to the parent body without the collision element ever using the RigidBody or
// any of its methods.
// This is particularly useful when the physics engine (at the RigidBody or
// RigidBodyTree scope) needs to retrieve the parent body (for instance to
// query its world transform).
class RigidBody;

namespace DrakeCollision {
typedef uintptr_t ElementId;

class DRAKECOLLISION_EXPORT Element : public DrakeShapes::Element {
 public:
  Element(const Eigen::Isometry3d& T_element_to_local =
              Eigen::Isometry3d::Identity());

  Element(const DrakeShapes::Geometry& geometry,
          const Eigen::Isometry3d& T_element_to_local =
              Eigen::Isometry3d::Identity());

  virtual ~Element() {}

  virtual Element* clone() const;

  ElementId getId() const;

  virtual bool isStatic() const { return false; }

  /** Flags this collision element to be static, i.e. does not move.
  @see Element::is_static(). **/
  void set_static() { is_static_ = true;}

  /** Returns `true` if the shape does not move, `false` otherwise.

  For instance, a terrain commonly is a static collision element. This property
  could allow the collision dispatcher to perform certain optimizations not
  generally possible for moving objects. **/
  bool is_static() { return is_static_;}

  /**
   * Returns true if this element should be checked for collisions
   * with the other object.  CollidesWith should be symmetric: if
   * A collides with B, B collides with A.
   */
  virtual bool CollidesWith(const Element* other) const { return true; }

  /** Returns a pointer to the const RigidBody to which this CollisionElement
  is attached. **/
  const RigidBody* get_body() const;

  /** Sets the rigid body this collision element is attached to. **/
  void set_body(const RigidBody *body);

  /**
   * A toString method for this class.
   */
  friend DRAKECOLLISION_EXPORT std::ostream& operator<<(std::ostream&,
                                                        const Element&);

 protected:
  Element(const Element& other);

 private:
  ElementId id;
  bool is_static_{false};
  const RigidBody* body_{};

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace DrakeCollision
