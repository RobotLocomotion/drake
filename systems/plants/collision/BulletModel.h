#ifndef __DrakeCollisionBulletModel_H__
#define __DrakeCollisionBulletModel_H__

#include <btBulletCollisionCommon.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>

#include "DrakeCollision.h"
#include "Model.h"

namespace DrakeCollision
{
  class BulletModel 
    : public Model, public std::enable_shared_from_this<BulletModel>
  {
    public:
      BulletModel();
      ~BulletModel();
      virtual void addElement(const int body_ind, Matrix4d T_element_to_link, Shape shape, std::vector<double> params, bool is_static);
      virtual void updateElement(ElementShPtr elem, Matrix4d T_link_to_world);
      virtual bool getPairwiseCollision(const int body_indA, const int body_indB, MatrixXd &ptsA, MatrixXd &ptsB, MatrixXd &normals);
      virtual bool getPairwisePointCollision(const int body_indA, const int body_indB, const int body_collision_indA, Vector3d &ptA, Vector3d &ptB, Vector3d &normal);
      virtual bool getPointCollision(const int body_ind, const int body_collision_ind, Vector3d &ptA, Vector3d &ptB, Vector3d &normal);
      virtual bool getClosestPoints(const int body_indA,const int body_indB,Vector3d& ptA,Vector3d& ptB,Vector3d& normal,double& distance);

      btCollisionWorld* bt_collision_world;
    protected:

      btDefaultCollisionConfiguration bt_collision_configuration;
      btCollisionDispatcher* bt_collision_dispatcher;
      btDbvtBroadphase bt_collision_broadphase;

      // For getClosestPoints
      btGjkEpaPenetrationDepthSolver epa;
      btVoronoiSimplexSolver sGjkSimplexSolver;
      //TestbtCollisionWorld test;
  };
}
#endif
