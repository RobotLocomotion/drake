#ifndef __PointPair_H__
#define __PointPair_H__

#include "DrakeCollision.h"

namespace DrakeCollision
{
  class PointPair
  {
    public:
      template<typename inputVector3T=Vector3d, typename inputScalarT=double>
      PointPair(const int bodyA_idx, const int bodyB_idx,
                const inputVector3T ptA, const inputVector3T ptB, 
                const inputVector3T normal, inputScalarT distance)
      : bodyA_idx(bodyA_idx), bodyB_idx(bodyB_idx), ptA(toVector3d(ptA)), 
        ptB(toVector3d(ptB)), normal(toVector3d(normal)), 
        distance((double) distance)
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

  Vector3d toVector3d(const Vector3d& vec);
}
#endif
