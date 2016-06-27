#include "drake/systems/plants/collision/point_pair.h"

using Eigen::Vector3d;

namespace DrakeCollision {

const Element* PointPair::get_elementA() const { return elementA_; }

const Element* PointPair::get_elementB() const { return elementB_; }

Vector3d PointPair::getNormal() const { return normal_; }

double PointPair::getDistance() const { return distance_; }

Vector3d PointPair::getPtA() const { return ptA_; }

Vector3d PointPair::getPtB() const { return ptB_; }

}  // namespace DrakeCollision
