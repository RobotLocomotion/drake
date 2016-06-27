#include "drake/systems/plants/collision/point_pair.h"

using Eigen::Vector3d;

namespace DrakeCollision {

double PointPair::getDistance() const { return distance_; }

}  // namespace DrakeCollision
