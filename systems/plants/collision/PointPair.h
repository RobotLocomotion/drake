#ifndef __PointPair_H__
#define __PointPair_H__
#include "DrakeCollision.h"

namespace DrakeCollision
{
  class PointPair
  {
    public:
      PointPair(const ElementId elemA, const ElementId elemB,
                const Eigen::Vector3d ptA, const Eigen::Vector3d ptB,
                const Eigen::Vector3d normal, double distance)
      : elemA(elemA), elemB(elemB), ptA(ptA),
        ptB(ptB), normal(normal),
        distance(distance)
      {}

      const ElementId elemA;
      const ElementId elemB;
      const Eigen::Vector3d ptA;
      const Eigen::Vector3d ptB;
      const Eigen::Vector3d normal;
      double distance;

      void getResults(Eigen::Vector3d& ptA, Eigen::Vector3d& ptB, Eigen::Vector3d& normal);
      void getResults(Eigen::Vector3d& ptA, Eigen::Vector3d& ptB, Eigen::Vector3d& normal, double& distance);

      bool operator < (const PointPair& pt) const
      {
        return (distance < pt.distance);
      }

      bool operator == (const PointPair& pt) const
      {
        return (distance == pt.distance);
      }

      bool operator != (const PointPair& pt) const
      {
        return (distance != pt.distance);
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
