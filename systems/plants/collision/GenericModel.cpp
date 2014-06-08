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
    if (print_warning) {
      cout << "DrakeCollision::GenericModel::findClosestPointsBtwElements:" << endl;
      cout << "\tRe-compile with Bullet to use this function." << endl;
      print_warning = false;
    }
    return false;
  };

  bool GenericModel::findCollisionPointsBtwElements(const int bodyA_idx,
                                                    const int bodyB_idx,
                                                    const Element& elemA, 
                                                    const Element& elemB, 
                                                    const ResultCollShPtr& c)
  {
    if (print_warning) {
      cout << "DrakeCollision::GenericModel::findCollisionPointsBtwElements:" << endl;
      cout << "\tRe-compile with Bullet to use this function." << endl;
      print_warning = false;
    }
    return false;
  };

  bool GenericModel::getPointCollision(const int body_idx, 
                                const int body_collision_idx, 
                                Vector3d& ptA, Vector3d& ptB, Vector3d& normal)
  {
    if (print_warning) {
      cout << "DrakeCollision::GenericModel::getPointCollision:" << endl;
      cout << "\tRe-compile with Bullet to use this function." << endl;
      print_warning = false;
    }
    return false;
  };

  bool GenericModel::allCollisions(vector<int>& bodyA_idx, 
      vector<int>& bodyB_idx, 
      MatrixXd& ptsA, MatrixXd& ptsB)
  {
    if (print_warning) {
      cout << "DrakeCollision::GenericModel::allCollisions:" << endl;
      cout << "\tRe-compile with Bullet to use this function." << endl;
      print_warning = false;
    }
    return false;
  };
  
  bool GenericModel::collisionRaycast(const Matrix3Xd &origins, const Matrix3Xd &ray_endpoints, VectorXd &distances)
  {
    if (print_warning) {
      cout << "DrakeCollision::GenericModel::collisionRaycast:" << endl;
      cout << "\tRe-compile with Bullet to use this function." << endl;
      print_warning = false;
    }
    return false;
  };
  
};
