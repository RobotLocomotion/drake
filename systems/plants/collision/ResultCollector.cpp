#include "ResultCollector.h"
#include "DrakeCollision.h"

using namespace std;

namespace DrakeCollision
{
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

