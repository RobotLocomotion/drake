#pragma once

#include <stdint.h>
#include <memory>
#include <utility>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "drake/drakeCollision_export.h"
#include "drake/systems/plants/shapes/DrakeShapes.h"

template <class InputIterator1, class InputIterator2>
bool have_intersection (InputIterator1 first1, InputIterator1 last1,
                        InputIterator2 first2, InputIterator2 last2)
{
  while (first1!=last1 && first2!=last2)
  {
    if (*first1<*first2) ++first1;
    else if (*first2<*first1) ++first2;
    else {
      return true;
    }
  }
  return false;
}

// Forward declaration.
// RigidBody's interface is never used by the collision engine.
// It is however useful the concept of having a CollisionElement attached to a
// Body.
class RigidBody;

namespace DrakeCollision {
typedef uintptr_t ElementId;

class DRAKECOLLISION_EXPORT Element : public DrakeShapes::Element {
 public:
  Element(const Eigen::Isometry3d& T_element_to_local =
              Eigen::Isometry3d::Identity(), const RigidBody* const body = nullptr);

  Element(const DrakeShapes::Geometry& geometry,
          const Eigen::Isometry3d& T_element_to_local =
              Eigen::Isometry3d::Identity(), const RigidBody* const body = nullptr);

  virtual ~Element() {}

  virtual Element* clone() const;

  ElementId getId() const;

  virtual bool isStatic() const { return false; }

  /**
   * Returns true if this element should be checked for collisions
   * with the other object.  CollidesWith should be symmetric: if
   * A collides with B, B collides with A.
   */
  virtual bool CollidesWith(const Element* other) const;

  /**
   * @brief Returns a const reference to the body to which this
   * CollisionElement is attached.
   *
   * @throws A std::runtime_error if the CollisionElement is not attached to a
   * body.
   */
  // TODO(amcastro-tri): getBody() -> get_body()
  const RigidBody& getBody() const;

  bool is_attached_to_body() const { return body_!= nullptr; }

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
  friend DRAKECOLLISION_EXPORT std::ostream& operator<<(std::ostream&,
                                                        const Element&);

 protected:
  Element(const Element& other);

 private:
  ElementId id;
  const RigidBody* const body_{};

  // Collision groups in Drake are represented simply by an integer.
  // Collision elements in the same group do not collide.
  // A collision element can belong to more than one group.
  // Vector collision_groups_ is kept sorted.
  std::vector<int> collision_groups_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace DrakeCollision
