#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "Element.h"
#include "drake/drakeCollision_export.h"

namespace DrakeCollision {

/** Structure containing the results of a collision query. **/
struct DRAKECOLLISION_EXPORT PointPair {
  PointPair() {}

  PointPair(const Element* _elementA, const Element* _elementB,
            const Eigen::Vector3d& _ptA, const Eigen::Vector3d& _ptB,
            const Eigen::Vector3d& _normal, double _distance)
      : elementA(_elementA), elementB(_elementB),
        idA(_elementA->getId()), idB(_elementB->getId()),
        ptA(_ptA), ptB(_ptB),
        normal(_normal),
        distance(_distance) {}

  /** Element A in the pair participating in the collision. **/
  const Element* elementA{nullptr};

  /** Element B in the pair participating in the collision. **/
  const Element* elementB{nullptr};

  /** Id of element A participating in the collision. **/
  ElementId idA{0};

  /** Id of element B participating in the collision. **/
  ElementId idB{0};

  /** Collision point on the surface of body A. **/
  Eigen::Vector3d ptA;

  /** Collision point on the surface of body B. **/
  Eigen::Vector3d ptB;

  /** Outwards normal on body B. On body A it points in the opposite
  direction. **/
  Eigen::Vector3d normal;

  /** Distance between the point on body A and the point on body B. **/
  double distance{};
};

}  // namespace DrakeCollision
