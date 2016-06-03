#include "drake/systems/plants/collision/point_pair.h"

using Eigen::Vector3d;

namespace DrakeCollision {

void PointPair::getResults(Vector3d* ptA, Vector3d* ptB,
                           Vector3d* normal) const {
  *ptA = ptA_;
  *ptB = ptB_;
  *normal = normal_;
}

void PointPair::getResults(Vector3d* ptA, Vector3d* ptB, Vector3d* normal,
                           double* distance) const {
  getResults(ptA, ptB, normal);
  *distance = distance_;
}

CollisionElementId PointPair::getIdA() const { return idA_; }

CollisionElementId PointPair::getIdB() const { return idB_; }

Vector3d PointPair::getNormal() const { return normal_; }

double PointPair::getDistance() const { return distance_; }

Vector3d PointPair::getPtA() const { return ptA_; }

Vector3d PointPair::getPtB() const { return ptB_; }

}  // namespace DrakeCollision
