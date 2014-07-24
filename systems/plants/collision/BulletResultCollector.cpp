#include <iostream>
#include <btBulletCollisionCommon.h>

#include "DrakeCollision.h"
#include "BulletModel.h"

using namespace std;
using namespace Eigen;

namespace DrakeCollision
{
  Vector3d toVector3d(const Vector3d& vec)
  {
    return vec;
  };

  Vector3d toVector3d(const btVector3& bt_vec)
  {
    Vector3d vec;
    vec(0) = (double) bt_vec.getX();
    vec(1) = (double) bt_vec.getY();
    vec(2) = (double) bt_vec.getZ();
    //DEBUG
    //cout << "BulletResultCollector:toVector3d:" << endl;
    //cout << "btVector3:" << endl;
    //cout << bt_vec.getX() << ' ' << bt_vec.getY() << ' ' << bt_vec.getZ() << endl;
    //cout << "Vector3d:" << endl;
    //cout << vec.transpose() << endl;
    //END_DEBUG
    return vec;
  }
  
  void ResultCollector::addPointPairResult(const PointPair& result)
  {
    pts.push_back(result);
  }

  template<typename Derived>
  void ResultCollector::getResults(MatrixBase<Derived> &ptsA, 
                                    MatrixBase<Derived> &ptsB, 
                                    MatrixBase<Derived> &normals)
  {
    ptsA = MatrixXd::Zero(3,pts.size());
    ptsB = MatrixXd::Zero(3,pts.size());
    normals = MatrixXd::Zero(3,pts.size());

    for (int i=0; i<pts.size(); ++i) {
      ptsA.col(i) = pts[i].ptA;
      ptsB.col(i) = pts[i].ptB;
      normals.col(i) = pts[i].normal;
    }
  }

  template<typename DerivedA, typename DerivedB>
  void ResultCollector::getResults(vector<int>& bodyA_idx, 
                                   vector<int>& bodyB_idx,
                                   MatrixBase<DerivedA> &ptsA, 
                                   MatrixBase<DerivedA> &ptsB, 
                                   MatrixBase<DerivedA> &normals,
                                   MatrixBase<DerivedB>& distance)
  {
    ptsA = MatrixXd::Zero(3,pts.size());
    ptsB = MatrixXd::Zero(3,pts.size());
    normals = MatrixXd::Zero(3,pts.size());
    distance = VectorXd::Zero(pts.size());

    for (int i=0; i<pts.size(); ++i) {
      ptsA.col(i) = pts[i].ptA;
      ptsB.col(i) = pts[i].ptB;
      normals.col(i) = pts[i].normal;
      distance[i] = pts[i].distance;
      bodyA_idx.push_back(pts[i].bodyA_idx);
      bodyB_idx.push_back(pts[i].bodyB_idx);
    }
  }
  
  template<typename Derived>
  void ResultCollector::getResults(vector<int>& bodyA_idx, 
                                   vector<int>& bodyB_idx,
                                   MatrixBase<Derived> &ptsA, 
                                   MatrixBase<Derived> &ptsB, 
                                   MatrixBase<Derived> &normals,
                                   vector<double>& distance)
  {
    getResults(ptsA, ptsB, normals);
    bodyA_idx.clear();
    bodyB_idx.clear();
    for (auto it = pts.begin(); it != pts.end(); ++it) {
      bodyA_idx.push_back(it->bodyA_idx);
      bodyB_idx.push_back(it->bodyB_idx);
      distance.push_back(it->distance);
    }
  }

  PointPair ResultCollector::minDistPoint()
  {
    return *min_element(pts.begin(), pts.end());
  }
  
  
  btScalar BulletResultCollector::addSingleResult(btManifoldPoint& cp,	const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
  {
    addSingleResult(curr_bodyA_idx, curr_bodyB_idx, toVector3d(cp.getPositionWorldOnA()),
                    toVector3d(cp.getPositionWorldOnB()), toVector3d(cp.m_normalWorldOnB),
                    (double) cp.m_distance1);

    return 0;
  }

  btCollisionWorld::ContactResultCallback* BulletResultCollector::getBtPtr()
  {
    return static_cast<btCollisionWorld::ContactResultCallback*>(this);
  }

  void BulletResultCollector::setBodyIdx(const int bodyA_idx, const int bodyB_idx)
  {
    curr_bodyA_idx = bodyA_idx;
    curr_bodyB_idx = bodyB_idx;
  }

  void MinDistResultCollector::addPointPairResult(const PointPair& result)
  {
    if (pts.size() > 0) { 
      if (result >= pts[0]) {
        return;
      } else {
        pts.pop_back();
      }
    } 
    ResultCollector::addPointPairResult(result);
  };

  
  // explicit instantiations (required for linking):
  template void ResultCollector::getResults(MatrixBase<MatrixXd>&,
                                            MatrixBase<MatrixXd>&,
                                            MatrixBase<MatrixXd>&);

  template void ResultCollector::getResults(MatrixBase< Map<MatrixXd> >&,
                                            MatrixBase< Map<MatrixXd> >&,
                                            MatrixBase< Map<MatrixXd> >&);

  template void ResultCollector::getResults(MatrixBase<Vector3d>&,
                                            MatrixBase<Vector3d>&,
                                            MatrixBase<Vector3d>&);

  template void ResultCollector::getResults(MatrixBase< Map<Vector3d> >&,
                                            MatrixBase< Map<Vector3d> >&,
                                            MatrixBase< Map<Vector3d> >&);

  template void ResultCollector::getResults(vector<int>&, vector<int>&,
                                            MatrixBase<MatrixXd>&,
                                            MatrixBase<MatrixXd>&,
                                            MatrixBase<MatrixXd>&,
                                            vector<double>&);

  template void ResultCollector::getResults(vector<int>&, vector<int>&,
                                            MatrixBase< Map<MatrixXd> >&,
                                            MatrixBase< Map<MatrixXd> >&,
                                            MatrixBase< Map<MatrixXd> >&,
                                            vector<double>&);

  template void ResultCollector::getResults(vector<int>&, vector<int>&,
                                            MatrixBase<Vector3d>&,
                                            MatrixBase<Vector3d>&,
                                            MatrixBase<Vector3d>&,
                                            vector<double>&);

  template void ResultCollector::getResults(vector<int>&, vector<int>&,
                                            MatrixBase< Map<Vector3d> >&,
                                            MatrixBase< Map<Vector3d> >&,
                                            MatrixBase< Map<Vector3d> >&,
                                            MatrixBase< Map<VectorXd> >&);

  template void ResultCollector::getResults(vector<int>&, vector<int>&,
                                            MatrixBase<MatrixXd>&,
                                            MatrixBase<MatrixXd>&,
                                            MatrixBase<MatrixXd>&,
                                            MatrixBase<VectorXd>&);

  template void ResultCollector::getResults(vector<int>&, vector<int>&,
                                            MatrixBase< Map<MatrixXd> >&,
                                            MatrixBase< Map<MatrixXd> >&,
                                            MatrixBase< Map<MatrixXd> >&,
                                            MatrixBase< Map<VectorXd> >&);

  template void ResultCollector::getResults(vector<int>&, vector<int>&,
                                            MatrixBase<Vector3d>&,
                                            MatrixBase<Vector3d>&,
                                            MatrixBase<Vector3d>&,
                                            MatrixBase<VectorXd>&);

}
