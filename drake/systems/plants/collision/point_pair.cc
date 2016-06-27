#include "drake/systems/plants/collision/point_pair.h"

using Eigen::Vector3d;

namespace DrakeCollision {

double PointPair::getDistance() const { return distance_; }

Vector3d PointPair::getPtA() const { return ptA_; }

Vector3d PointPair::getPtB() const { return ptB_; }

}  // namespace DrakeCollision
