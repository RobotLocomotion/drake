#ifndef __DrakeCollisionResultCollector_H__
#define __DrakeCollisionResultCollector_H__

#include "DrakeCollision.h"
#include "PointPair.h"

namespace DrakeCollision
{
  class ResultCollector {
    public:
      virtual void addPointPairResult(const PointPair& result);
        
      inline void addSingleResult(const int bodyA_idx, const int bodyB_idx,
                                  const Vector3d& ptA,
                                  const Vector3d& ptB,
                                  const Vector3d& normal,
                                  double distance)
      {
        addPointPairResult(PointPair(bodyA_idx, bodyB_idx, ptA, ptB, normal, 
                                      distance));
      }

      template<typename Derived>
      void getResults(MatrixBase<Derived> &ptsA, MatrixBase<Derived> &ptsB, 
                      MatrixBase<Derived> &normals);

      template<typename DerivedA, typename DerivedB>
      void getResults(std::vector<int>& bodyA_idx, 
          std::vector<int>& bodyB_idx,
          MatrixBase<DerivedA> &ptsA, 
          MatrixBase<DerivedA> &ptsB, 
          MatrixBase<DerivedA> &normals,
          MatrixBase<DerivedB>& distance);

      template<typename Derived>
      void getResults(std::vector<int>& bodyA_idx, std::vector<int>& bodyB_idx,
                      MatrixBase<Derived> &ptsA, MatrixBase<Derived> &ptsB, 
                      MatrixBase<Derived> &normals,std::vector<double>& distance);

      PointPair minDistPoint();

      std::vector<PointPair> pts;
  };

}
#endif
