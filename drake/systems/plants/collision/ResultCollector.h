#ifndef DRAKE_COLLISION_RESULT_COLLECTOR
#define DRAKE_COLLISION_RESULT_COLLECTOR

#include "drake/systems/plants/collision/DrakeCollision.h"

namespace DrakeCollision
{
  Eigen::Vector3d toVector3d(const Eigen::Vector3d& vec);

  class ResultCollector {
    public:
      virtual ~ResultCollector(){};

      virtual void addPointPairResult(const PointPair& result);
        
      inline void addSingleResult(const ElementId idA,
                                  const ElementId idB,
                                  const Eigen::Vector3d& ptA,
                                  const Eigen::Vector3d& ptB,
                                  const Eigen::Vector3d& normal,
                                  double distance)
      {
        addPointPairResult(PointPair(idA, idB, ptA, ptB, normal, 
                                      distance));
      }

      std::vector<PointPair> getResults() const
      {
        return pts;
      }

      PointPair minDistPoint();

      std::vector<PointPair> pts;
  };
  
  typedef std::shared_ptr< ResultCollector > ResultCollShPtr;
}

#endif
