#ifndef __PointPair_H__
#define __PointPair_H__

#include "DrakeCollision.h"

namespace DrakeCollision
{
	Vector3d toVector3d(const Vector3d& vec);

  class PointPair
  {
    public:
      PointPair(const int bodyA_idx, const int bodyB_idx,
                const Vector3d ptA, const Vector3d ptB,
                const Vector3d normal, double distance)
      : bodyA_idx(bodyA_idx), bodyB_idx(bodyB_idx), ptA(ptA),
        ptB(ptB), normal(normal),
        distance(distance)
      {}

      int bodyA_idx;
      int bodyB_idx;
      Vector3d ptA;
      Vector3d ptB;
      Vector3d normal;
      double distance;

      void getResults(Vector3d& ptA, Vector3d& ptB, Vector3d& normal);
      void getResults(Vector3d& ptA, Vector3d& ptB, Vector3d& normal, double& distance);

      bool operator == (const PointPair& pt) const
      {
        return (distance == pt.distance);
      }

      bool operator != (const PointPair& pt) const
      {
        return (distance != pt.distance);
      }

      bool operator < (const PointPair& pt) const
      {
        return (distance < pt.distance);
      }

      bool operator <= (const PointPair& pt) const
      {
        return (distance <= pt.distance);
      }

      bool operator > (const PointPair& pt) const
      {
        return (distance > pt.distance);
      }

      bool operator >= (const PointPair& pt) const
      {
        return (distance >= pt.distance);
      }
  };

}
#endif
