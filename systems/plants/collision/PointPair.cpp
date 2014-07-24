
#include "DrakeCollision.h"
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
}
