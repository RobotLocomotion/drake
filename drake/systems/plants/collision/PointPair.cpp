#include "PointPair.h"

using namespace std;
using namespace Eigen;

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

  ElementId PointPair::getIdA()
  {
    return this->idA;
  }

  ElementId PointPair::getIdB()
  {
    return this->idB;
  }

  Vector3d PointPair::getNormal()
  {
    return this->normal;
  }

  double PointPair::getDistance()
  {
    return this->distance;
  }

  Vector3d PointPair::getPtA()
  {
    return this->ptA;
  }

  Vector3d PointPair::getPtB()
  {
    return this->ptB;
  }
}
