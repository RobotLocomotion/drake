#include "Model.h"
#include "Element.h"

using namespace std;

namespace DrakeCollision
{
  void Model::resize(int num_bodies)
  {
    element_pool.resize(num_bodies);
  };

  void Model::addElement(const int body_ind, Matrix4d T_element_to_link, Shape shape, vector<double> params, bool is_static)
  {
    ElementShPtr new_element( new Element(T_element_to_link, shape, params) );
    addElement(body_ind, new_element);
  };

  void Model::addElement(const int body_ind, ElementShPtr new_element)
  {
    element_pool.at(body_ind).push_back(new_element);
  };

  void Model::updateElementsForBody(const int body_ind, Matrix4d T_link_to_world)
  {
    for (ElementShPtr elem : element_pool[body_ind]) {
      updateElement(elem,T_link_to_world);
    }
  };

  void Model::updateElement(ElementShPtr elem, Matrix4d T_link_to_world)
  {
    elem->updateWorldTransform(T_link_to_world);
  };

  bool Model::getPairwiseCollision(const int body_indA, const int body_indB, MatrixXd &ptsA, MatrixXd &ptsB, MatrixXd &normals)
  {
    cout << "DrakeCollision::Model::getClosestPoints:" << endl;
    cout << "\tRe-compile with Bullet to use this function." << endl;
    return false;
  };
  bool Model::getPairwisePointCollision(const int body_indA, const int body_indB, const int body_collision_indA, Vector3d &ptA, Vector3d &ptB, Vector3d &normal)
  {
    cout << "DrakeCollision::Model::getClosestPoints:" << endl;
    cout << "\tRe-compile with Bullet to use this function." << endl;
    return false;
  };
  bool Model::getPointCollision(const int body_ind, const int body_collision_ind, Vector3d &ptA, Vector3d &ptB, Vector3d &normal)
  {
    cout << "DrakeCollision::Model::getClosestPoints:" << endl;
    cout << "\tRe-compile with Bullet to use this function." << endl;
    return false;
  };
  bool Model::getClosestPoints(const int body_indA,const int body_indB,Vector3d& ptA,Vector3d& ptB,Vector3d& normal,double& distance)
  {
    cout << "DrakeCollision::Model::getClosestPoints:" << endl;
    cout << "\tRe-compile with Bullet to use this function." << endl;
    return false;
  };
};
