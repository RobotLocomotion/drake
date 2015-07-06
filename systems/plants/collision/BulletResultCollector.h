#ifndef __DrakeCollisionBullletResultCollector_H__
#define __DrakeCollisionBullletResultCollector_H__

#include "DrakeCollision.h"

namespace DrakeCollision
{
	Eigen::Vector3d toVector3d(const Eigen::Vector3d& vec);
  Eigen::Vector3d toVector3d(const btVector3& bt_vec);

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
  
  class BulletResultCollector : public ResultCollector, public btCollisionWorld::ContactResultCallback
  {
    public:
      virtual ~BulletResultCollector(){};

      using ResultCollector::addSingleResult;

      BulletResultCollector() : curr_bodyA_idx(-1), curr_bodyB_idx(-1) {};

      virtual	btScalar	addSingleResult(btManifoldPoint& cp,	
                                        const btCollisionObjectWrapper* colObj0Wrap,
                                        int partId0,int index0,
                                        const btCollisionObjectWrapper* colObj1Wrap,
                                        int partId1,int index1);

      btCollisionWorld::ContactResultCallback* getBtPtr();

      void setElements(const Element* elemA, const Element* elemB);

    protected:
      int curr_bodyA_idx;
      int curr_bodyB_idx;
  };
  
  typedef std::shared_ptr< ResultCollector > ResultCollShPtr;
}

#endif
