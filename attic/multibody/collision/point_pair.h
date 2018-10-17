#pragma once

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/multibody/collision/element.h"

namespace drake {
namespace multibody {
namespace collision {

/** Structure containing the results of a collision query. */
template <typename T>
struct PointPair {
  PointPair() {}

  PointPair(const Element* elementA_in, const Element* elementB_in,
            const Vector3<T>& ptA_in, const Vector3<T>& ptB_in,
            const Vector3<T>& normal_in, const T& distance_in)
      : elementA(elementA_in), elementB(elementB_in),
        idA(elementA_in->getId()), idB(elementB_in->getId()),
        ptA(ptA_in), ptB(ptB_in),
        normal(normal_in),
        distance(distance_in) {}

  /** Scalar-converting copy constructor. */
  template <typename U>
  explicit PointPair(const PointPair<U>& other)
      : PointPair<T>(other.elementA, other.elementB, other.ptA, other.ptB,
                     other.normal, other.distance) {}

  /** Element A in the pair participating in the collision. */
  const Element* elementA{nullptr};

  /** Element B in the pair participating in the collision. */
  const Element* elementB{nullptr};

  /** Id of element A participating in the collision. */
  ElementId idA{0};

  /** Id of element B participating in the collision. */
  ElementId idB{0};

  /** Collision point on the surface of body A. */
  Vector3<T> ptA;

  /** Collision point on the surface of body B. */
  Vector3<T> ptB;

  /** Outwards normal on body B. On body A it points in the opposite
  direction. */
  Vector3<T> normal;

  /** Distance between the point on body A and the point on body B. */
  T distance{};
};

}  // namespace collision
}  // namespace multibody
}  // namespace drake
