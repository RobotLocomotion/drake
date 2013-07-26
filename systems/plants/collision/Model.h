#ifndef __DrakeCollisionModel_H__
#define __DrakeCollisionModel_H__

#include "DrakeCollision.h"

namespace DrakeCollision
{
  class Model
  {
    public:
      void resize(int num_bodies);
      virtual void addElement(const int body_ind, Matrix4d T_element_to_link, Shape shape, std::vector<double> params,bool is_static);
      void addElement(const int body_ind,ElementShPtr new_element);
      void updateElementsForBody(const int body_ind, Matrix4d T_link_to_world);
      virtual void updateElement(ElementShPtr elem, Matrix4d T_link_to_world);
      virtual bool getPairwiseCollision(const int body_indA, const int body_indB, MatrixXd &ptsA, MatrixXd &ptsB, MatrixXd &normals);
      virtual bool getPairwisePointCollision(const int body_indA, const int body_indB, const int body_collision_indA, Vector3d &ptA, Vector3d &ptB, Vector3d &normal);
      virtual bool getPointCollision(const int body_ind, const int body_collision_ind, Vector3d &ptA, Vector3d &ptB, Vector3d &normal);
      virtual bool getClosestPoints(const int body_indA,const int body_indB,Vector3d& ptA,Vector3d& ptB,Vector3d& normal,double& distance);

      friend class Element;

    protected:
      std::vector< std::vector< std::shared_ptr<Element> > > element_pool;
  };
}
#endif
