#include "GenericModel.h"
#include "Element.h"
#include "ResultCollector.h"
#include "MinDistResultCollector.h"

using namespace std;

namespace DrakeCollision
{
  bool GenericModel::findClosestPointsBtwElements(const int bodyA_idx,
                                                  const int bodyB_idx,
                                                  const Element& elemA, 
                                                  const Element& elemB, 
                                                  const ResultCollShPtr& c) 
  {
    cout << "DrakeCollision::GenericModel::findClosestPointsBtwElements:" << endl;
    cout << "\tRe-compile with Bullet to use this function." << endl;
    return false;
  };

  bool GenericModel::findCollisionPointsBtwElements(const int bodyA_idx,
                                                    const int bodyB_idx,
                                                    const Element& elemA, 
                                                    const Element& elemB, 
                                                    const ResultCollShPtr& c)
  {
    cout << "DrakeCollision::GenericModel::findCollisionPointsBtwElements:" << endl;
    cout << "\tRe-compile with Bullet to use this function." << endl;
    return false;
  };

  bool GenericModel::getPointCollision(const int body_idx, 
                                const int body_collision_idx, 
                                Vector3d& ptA, Vector3d& ptB, Vector3d& normal)
  {
    cout << "DrakeCollision::GenericModel::getPointCollision:" << endl;
    cout << "\tRe-compile with Bullet to use this function." << endl;
    return false;
  };

  bool GenericModel::allCollisions(vector<int>& bodyA_idx, 
      vector<int>& bodyB_idx, 
      MatrixXd& ptsA, MatrixXd& ptsB)
  {
    cout << "DrakeCollision::GenericModel::allCollisions:" << endl;
    cout << "\tRe-compile with Bullet to use this function." << endl;
    return false;
  };
  
  bool GenericModel::collisionRaycast(const Matrix3Xd &origins, const Matrix3Xd &ray_endpoints, VectorXd &distances)
  {
    cout << "DrakeCollision::GenericModel::collisionRaycast:" << endl;
    cout << "\tRe-compile with Bullet to use this function." << endl;
    return false;
  };
  
};
