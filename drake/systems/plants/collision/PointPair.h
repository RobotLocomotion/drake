#ifndef __DrakeCollisionPointPair_H__
#define __DrakeCollisionPointPair_H__

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "Element.h"

#include "drakeCollisionMacros.h"

namespace DrakeCollision
{
  class DLLEXPORT_drakeCollision PointPair {
    public:
      PointPair(const ElementId idA, const ElementId idB,
                const Eigen::Vector3d ptA, const Eigen::Vector3d ptB,
                const Eigen::Vector3d normal, double distance)
      : idA(idA), idB(idB), ptA(ptA),
        ptB(ptB), normal(normal),
        distance(distance)
      {}

      void getResults(Eigen::Vector3d& ptA, Eigen::Vector3d& ptB, Eigen::Vector3d& normal);
      void getResults(Eigen::Vector3d& ptA, Eigen::Vector3d& ptB, Eigen::Vector3d& normal, double& distance);
      ElementId getIdA();
      ElementId getIdB();
      Eigen::Vector3d getNormal();
      Eigen::Vector3d getPtA();
      Eigen::Vector3d getPtB();
      double getDistance();

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

    protected:
      ElementId idA;
      ElementId idB;
      Eigen::Vector3d ptA;
      Eigen::Vector3d ptB;
      Eigen::Vector3d normal;
      double distance;
  };
}
#endif
