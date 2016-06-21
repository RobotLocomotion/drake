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

  /**
   * Returns true if this element should be checked for collisions
   * with the other object.  CollidesWith should be symmetric: if
   * A collides with B, B collides with A.
   */
  virtual bool CollidesWith(const Element* other) const { return true; }

  /** Returns a pointer to the const RigidBody to which this CollisionElement
  is attached. **/
  const RigidBody* const get_body() const;

  /** Set the rigid body this collision element is attached to. **/
  void set_rigid_body(const RigidBody* body);

  /**
   * A toString method for this class.
   */
  friend DRAKECOLLISION_EXPORT std::ostream& operator<<(std::ostream&,
                                                        const Element&);

 protected:
  Element(const Element& other);

 private:
  ElementId id;
  const RigidBody* body_{};

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace DrakeCollision
