#ifndef __DrakeCollisionBulletModel_H__
#define __DrakeCollisionBulletModel_H__

#include <btBulletCollisionCommon.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>

#include "DrakeCollision.h"
#include "ModelTemplate.h"
#include "PointPair.h"
#include "ResultCollector.h"

namespace DrakeCollision
{
  struct OverlapFilterCallback : public btOverlapFilterCallback
  {
    // return true when pairs need collision
    virtual bool  needBroadphaseCollision(btBroadphaseProxy* proxy0,
        btBroadphaseProxy* proxy1) const;

    BulletModel* parent_model;
  };

  struct ElementData 
  {
    int body_idx;
    Shape shape;
    
    ElementData(int body_idx, Shape shape) : body_idx(body_idx), shape(shape) {};
  };

  class BulletModel 
    : public ModelTemplate<BulletElement>
  {
    public:
      BulletModel();

      ~BulletModel();

      virtual void addElement(const int body_ind, const int parent_idx, 
                              const Matrix4d& T_elem_to_link, Shape shape, 
                              const std::vector<double>& params, 
                              bool is_static);

      virtual bool updateElementsForBody(const int body_idx,
                                  const Matrix4d& T_link_to_world);

      //virtual void setCollisionFilter(Body<BulletElement>& body,
                                       //const bitmask& group,
                                       //const bitmask&  mask);

      btCollisionWorld* bt_collision_world;
      
    protected:

      virtual bool findClosestPointsBtwElements(const int bodyA_idx,
                                                const int bodyB_idx, 
                                                const BulletElement& elemA, 
                                                const BulletElement& elemB, 
                                                const ResultCollShPtr& c);

      virtual bool findCollisionPointsBtwElements(const int bodyA_idx,
                                                  const int bodyB_idx, 
                                                  const BulletElement& elemA, 
                                                  const BulletElement& elemB, 
                                                  const ResultCollShPtr& c);

      virtual bool getPointCollision(const int body_idx, 
                                      const int body_collision_idx, 
                                      Vector3d &ptA, Vector3d &ptB, 
                                      Vector3d &normal);

      virtual bool allCollisions(std::vector<int>& bodyA_idx, 
          std::vector<int>& bodyB_idx, 
          MatrixXd& ptsA, MatrixXd& ptsB);
          
      virtual bool collisionRaycast(const Matrix3Xd &origins, const Matrix3Xd &ray_endpoints, VectorXd &distances);

      btDefaultCollisionConfiguration bt_collision_configuration;
      btCollisionDispatcher* bt_collision_dispatcher;
      btDbvtBroadphase bt_collision_broadphase;
      OverlapFilterCallback filter_callback;
      std::vector< std::unique_ptr< ElementData > > element_data;

      // For getClosestPoints
      //btGjkEpaPenetrationDepthSolver epa;
      //btVoronoiSimplexSolver sGjkSimplexSolver;
      //TestbtCollisionWorld test;
  };
}
#endif
