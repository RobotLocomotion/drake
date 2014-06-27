#ifndef __PointPair_H__
#define __PointPair_H__

namespace DrakeCollision
{
  class PointPair
  {
    public:
      PointPair(const int bodyA_idx, const int bodyB_idx,
                const Eigen::Vector3d ptA, const Eigen::Vector3d ptB,
                const Eigen::Vector3d normal, double distance)
      : bodyA_idx(bodyA_idx), bodyB_idx(bodyB_idx), ptA(ptA),
        ptB(ptB), normal(normal),
        distance(distance)
      {}

      int bodyA_idx;
      int bodyB_idx;
      Eigen::Vector3d ptA;
      Eigen::Vector3d ptB;
      Eigen::Vector3d normal;
      double distance;

      void getResults(Eigen::Vector3d& ptA, Eigen::Vector3d& ptB, Eigen::Vector3d& normal);
      void getResults(Eigen::Vector3d& ptA, Eigen::Vector3d& ptB, Eigen::Vector3d& normal, double& distance);

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
