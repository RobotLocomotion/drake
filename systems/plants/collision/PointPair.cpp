#include "PointPair.h"

namespace DrakeCollision
{
  void PointPair::getResults(Vector3d& ptA, Vector3d& ptB, Vector3d& normal)
  {
    ptA = this->ptA;
    ptB = this->ptB;
    normal = this->normal;
  }

  void PointPair::getResults(Vector3d& ptA, Vector3d& ptB, Vector3d& normal, double& distance)
  {
    getResults(ptA,ptB,normal);
    distance = this->distance;
  }
}
