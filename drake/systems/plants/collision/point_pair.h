#pragma once

#include <Eigen/Dense>

#include "drake/common/drake_export.h"
#include "drake/systems/plants/collision/Element.h"

namespace DrakeCollision {

/** Structure containing the results of a collision query. **/
struct DRAKE_EXPORT PointPair {
  PointPair() {}

  PointPair(const Element* elementA_in, const Element* elementB_in,
            const Eigen::Vector3d& ptA_in, const Eigen::Vector3d& ptB_in,
            const Eigen::Vector3d& normal_in, double distance_in)
      : elementA(elementA_in), elementB(elementB_in),
        idA(elementA_in->getId()), idB(elementB_in->getId()),
        ptA(ptA_in), ptB(ptB_in),
        normal(normal_in),
        distance(distance_in) {}

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
