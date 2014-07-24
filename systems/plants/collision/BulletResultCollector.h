#ifndef __DrakeCollisionBullletResultCollector_H__
#define __DrakeCollisionBullletResultCollector_H__

#include "PointPair.h"

namespace DrakeCollision
{
	Eigen::Vector3d toVector3d(const Eigen::Vector3d& vec);
  Eigen::Vector3d toVector3d(const btVector3& bt_vec);

  class ResultCollector {
    public:
      virtual void addPointPairResult(const PointPair& result);
        
      inline void addSingleResult(const int bodyA_idx, const int bodyB_idx,
                                  const Eigen::Vector3d& ptA,
                                  const Eigen::Vector3d& ptB,
                                  const Eigen::Vector3d& normal,
                                  double distance)
      {
        addPointPairResult(PointPair(bodyA_idx, bodyB_idx, ptA, ptB, normal, 
                                      distance));
      }

      template<typename Derived>
      void getResults(Eigen::MatrixBase<Derived> &ptsA, Eigen::MatrixBase<Derived> &ptsB, 
                      Eigen::MatrixBase<Derived> &normals);

      template<typename DerivedA, typename DerivedB>
      void getResults(std::vector<int>& bodyA_idx, 
          std::vector<int>& bodyB_idx,
          Eigen::MatrixBase<DerivedA> &ptsA, 
          Eigen::MatrixBase<DerivedA> &ptsB, 
          Eigen::MatrixBase<DerivedA> &normals,
          Eigen::MatrixBase<DerivedB>& distance);

      template<typename Derived>
      void getResults(std::vector<int>& bodyA_idx, std::vector<int>& bodyB_idx,
                      Eigen::MatrixBase<Derived> &ptsA, Eigen::MatrixBase<Derived> &ptsB, 
                      Eigen::MatrixBase<Derived> &normals,std::vector<double>& distance);

      PointPair minDistPoint();

      std::vector<PointPair> pts;
  };
  
  class BulletResultCollector : public ResultCollector, public btCollisionWorld::ContactResultCallback
  {
    public:
      using ResultCollector::addSingleResult;

      BulletResultCollector() : curr_bodyA_idx(-1), curr_bodyB_idx(-1) {};

      virtual	btScalar	addSingleResult(btManifoldPoint& cp,	
                                        const btCollisionObjectWrapper* colObj0Wrap,
                                        int partId0,int index0,
                                        const btCollisionObjectWrapper* colObj1Wrap,
                                        int partId1,int index1);

      btCollisionWorld::ContactResultCallback* getBtPtr();

      void setBodyIdx(const int bodyA_idx, const int bodyB_idx);

    protected:
      int curr_bodyA_idx;
      int curr_bodyB_idx;
  };
  
  class MinDistResultCollector : public ResultCollector {
    public:
      virtual void addPointPairResult(const PointPair& result);
  };
  
  typedef std::shared_ptr< ResultCollector > ResultCollShPtr;
}

#endif
