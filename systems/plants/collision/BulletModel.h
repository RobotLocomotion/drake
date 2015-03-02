#ifndef __DrakeCollisionBulletModel_H__
#define __DrakeCollisionBulletModel_H__

#include <btBulletCollisionCommon.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>

#include "DrakeCollision.h"
#include "BulletResultCollector.h"

namespace DrakeCollision
{
  class BulletModel;    // forward declaration
  
  typedef std::map< ElementId, std::unique_ptr<btCollisionObject> > ElementToBtObjMap;

  struct OverlapFilterCallback : public btOverlapFilterCallback
  {
    // return true when pairs need collision
    virtual bool  needBroadphaseCollision(btBroadphaseProxy* proxy0,
        btBroadphaseProxy* proxy1) const;

    BulletModel* parent_model;
  };

  struct BulletCollisionWorldWrapper
  {
    BulletCollisionWorldWrapper();

    btDefaultCollisionConfiguration bt_collision_configuration;
    btDbvtBroadphase bt_collision_broadphase;
    OverlapFilterCallback filter_callback;

    std::unique_ptr<btCollisionDispatcher> bt_collision_dispatcher;
    std::unique_ptr<btCollisionWorld> bt_collision_world;

    ElementToBtObjMap bt_collision_objects;
  };

  class BulletModel : public Model
  {
    public:
      //Required member functions for Model interface
      virtual void updateModel();

      virtual void resize(int num_bodies) {};

      virtual ElementId addElement(std::unique_ptr<Element> element);

      virtual bool updateElementWorldTransform(const ElementId, 
                                               const Eigen::Matrix4d& T_local_to_world);

      virtual bool closestPointsAllToAll(const std::vector<ElementId>& ids_to_check, 
                                         const bool use_margins,
                                         std::vector<PointPair>& closest_points);

      virtual bool collisionPointsAllToAll(const bool use_margins,
                                           std::vector<PointPair>& points);

      virtual bool closestPointsPairwise(const std::vector<ElementIdPair>& id_pairs, 
                                         const bool use_margins,
                                         std::vector<PointPair>& closest_points);

      virtual bool findClosestPointsBtwElements(const ElementId elemA,
                                                const ElementId elemB,
                                                const bool use_margins,
                                                std::unique_ptr<ResultCollector>& c);

      virtual bool collisionRaycast(const Eigen::Matrix3Xd &origins, 
              const Eigen::Matrix3Xd &ray_endpoints, bool use_margins, 
              Eigen::VectorXd &distances);
      // END Required member functions
      
    protected:

      BulletCollisionWorldWrapper& getBulletWorld(bool use_margins);

      BulletCollisionWorldWrapper bullet_world;
      BulletCollisionWorldWrapper bullet_world_no_margin;
      std::vector< std::unique_ptr<btCollisionShape> > bt_collision_shapes;

    protected:
      static std::unique_ptr<btCollisionShape> newBulletBoxShape(const Box* geometry, bool use_margins);
      static std::unique_ptr<btCollisionShape> newBulletSphereShape(const Sphere* geometry, bool use_margins);
      static std::unique_ptr<btCollisionShape> newBulletCylinderShape(const Cylinder* geometry, bool use_margins);
      static std::unique_ptr<btCollisionShape> newBulletCapsuleShape(const Capsule* geometry, bool use_margins);
      static std::unique_ptr<btCollisionShape> newBulletMeshShape(const Mesh* geometry, bool use_margins);

      static constexpr double small_margin = 1e-9;
      static constexpr double large_margin = 0.05;
  };
}
#endif
