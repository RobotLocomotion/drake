#include <iostream>

#include "DrakeCollision.h"
#include "BulletModel.h"

using namespace std;
using namespace Eigen;

namespace DrakeCollision
{
  bool OverlapFilterCallback::needBroadphaseCollision(btBroadphaseProxy* proxy0,
      btBroadphaseProxy* proxy1) const
  {
    bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
    collides = collides && (proxy1->m_collisionFilterGroup &
        proxy0->m_collisionFilterMask);

    //add some additional logic here that modified 'collides'
    if (collides) {
      btCollisionObject* bt_collision_object0 = (btCollisionObject*) proxy0->m_clientObject;
      btCollisionObject* bt_collision_object1 = (btCollisionObject*) proxy1->m_clientObject;
      if ((bt_collision_object0->getUserPointer() == NULL) || 
          (bt_collision_object1->getUserPointer() == NULL)) {
        return false;
      }
      auto element0 = static_cast< Element* >(bt_collision_object0->getUserPointer());
      auto element1 = static_cast< Element* >(bt_collision_object1->getUserPointer());
      collides = collides && element0->collidesWith(element1);
    }
    return collides;
  }

  BulletCollisionWorldWrapper::BulletCollisionWorldWrapper()
    : bt_collision_configuration(), bt_collision_broadphase(), filter_callback()
  {
    bt_collision_dispatcher = unique_ptr<btCollisionDispatcher>(new btCollisionDispatcher(&bt_collision_configuration));
    bt_collision_world = unique_ptr<btCollisionWorld>(new btCollisionWorld(bt_collision_dispatcher.get(), &bt_collision_broadphase, &bt_collision_configuration));

    bt_collision_world->getPairCache()->setOverlapFilterCallback(&filter_callback);
  }

  unique_ptr<btCollisionShape> BulletModel::newBulletBoxShape(const Box* geometry, bool use_margins)
  {
    unique_ptr<btCollisionShape> bt_shape(new btConvexHullShape());
    btBoxShape bt_box( btVector3(geometry->size(0)/2,geometry->size(1)/2,geometry->size(2)/2) );
    /* Strange things happen to the collision-normals when we use the
     * convex interface to the btBoxShape. Instead, we'll explicitly create
     * a btConvexHullShape.
     */
    if (use_margins)
      bt_shape->setMargin(BulletModel::large_margin);
    else
      bt_shape->setMargin(BulletModel::small_margin);
    for (int i=0; i<8; ++i){
      btVector3 vtx;
      bt_box.getVertex(i,vtx);
      dynamic_cast<btConvexHullShape*>(bt_shape.get())->addPoint(vtx);
    }

    return bt_shape;
  }

  unique_ptr<btCollisionShape> BulletModel::newBulletSphereShape(const Sphere* geometry, bool use_margins)
  {
    unique_ptr<btCollisionShape> bt_shape(new btSphereShape(geometry->radius));
    return bt_shape;
  }

  unique_ptr<btCollisionShape> BulletModel::newBulletCylinderShape(const Cylinder* geometry, bool use_margins)
  {
    unique_ptr<btCollisionShape> bt_shape(new btCylinderShapeZ( btVector3(geometry->radius,geometry->radius,geometry->length/2) ));
    return bt_shape;
  }

  unique_ptr<btCollisionShape> BulletModel::newBulletCapsuleShape(const Capsule* geometry, bool use_margins)
  {
    unique_ptr<btCollisionShape> bt_shape(new btConvexHullShape());
    dynamic_cast<btConvexHullShape*>(bt_shape.get())->addPoint(btVector3(0,0,-geometry->length/2));
    dynamic_cast<btConvexHullShape*>(bt_shape.get())->addPoint(btVector3(0,0,geometry->length/2));
    bt_shape->setMargin(geometry->radius);
    return bt_shape;
  }

  unique_ptr<btCollisionShape> BulletModel::newBulletMeshShape(const Mesh* geometry, bool use_margins)
  {
    unique_ptr<btCollisionShape> bt_shape(new btConvexHullShape());
    if (use_margins)
      bt_shape->setMargin(BulletModel::large_margin);
    else
      bt_shape->setMargin(BulletModel::small_margin);
    for (int i=0; i<geometry->points.cols(); i++){
      dynamic_cast<btConvexHullShape*>(bt_shape.get())->addPoint(btVector3(geometry->points(0,i),geometry->points(1,i),geometry->points(2,i)));
    }
    return bt_shape;
  }

  ElementId BulletModel::addElement(std::unique_ptr<Element> element)
  {
    ElementId id =  Model::addElement(move(element));

    if (id != 0) {
      unique_ptr<btCollisionShape> bt_shape;
      unique_ptr<btCollisionShape> bt_shape_no_margin;
      switch (elements[id]->getShape()) {
        case BOX:
          {
            const auto box = static_cast<const Box*>(elements[id]->getGeometry());
            bt_shape = newBulletBoxShape(box, true);          
            bt_shape_no_margin = newBulletBoxShape(box, false);
          }
          break;
        case SPHERE:
          {
            const auto sphere = static_cast<const Sphere*>(elements[id]->getGeometry());
            bt_shape = newBulletSphereShape(sphere, true);
            bt_shape_no_margin = newBulletSphereShape(sphere, false);
          }
          break;
        case CYLINDER:
          {
            const auto cylinder = static_cast<const Cylinder*>(elements[id]->getGeometry());
            bt_shape = newBulletCylinderShape(cylinder, true);
            bt_shape_no_margin = newBulletCylinderShape(cylinder, false);
          }
          break;
        case MESH:
          {
            const auto mesh = static_cast<const Mesh*>(elements[id]->getGeometry());
            bt_shape = newBulletMeshShape(mesh, true);
            bt_shape_no_margin = newBulletMeshShape(mesh, false);
          }
          break;
        case CAPSULE:
          {
            const auto capsule = static_cast<const Capsule*>(elements[id]->getGeometry());
            bt_shape = newBulletCapsuleShape(capsule, true);
            bt_shape_no_margin = newBulletCapsuleShape(capsule, false);
          }
          break;
        default:
          cerr << "Warning: Collision elements[id] has an unknown type " << elements[id]->getShape() << endl;
          throw unknownShapeException(elements[id]->getShape());
          break;
      }
      // Create the collision objects
      unique_ptr<btCollisionObject> bt_obj(new btCollisionObject());
      unique_ptr<btCollisionObject> bt_obj_no_margin(new btCollisionObject());
      bt_obj->setCollisionShape(bt_shape.get());
      bt_obj_no_margin->setCollisionShape(bt_shape_no_margin.get());
      bt_obj->setUserPointer(elements[id].get());
      bt_obj_no_margin->setUserPointer(elements[id].get());

      // Add the collision objects to the collision worlds
      bullet_world.bt_collision_world->addCollisionObject(bt_obj.get());
      bullet_world_no_margin.bt_collision_world->addCollisionObject(bt_obj_no_margin.get());

      if (elements[id]->isStatic()) {
        bt_obj->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
        bt_obj->activate();
        bt_obj_no_margin->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
        bt_obj_no_margin->activate();
      } else {
        bt_obj->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);
        bt_obj_no_margin->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);
      }

      // Store the Bullet collision objects
      bullet_world.bt_collision_objects.insert(make_pair(id, move(bt_obj)));
      bullet_world_no_margin.bt_collision_objects.insert(make_pair(id, move(bt_obj_no_margin)));

      // Store the Bullet collision shapes too, because Bullet does no cleanup
      bt_collision_shapes.push_back(move(bt_shape));
      bt_collision_shapes.push_back(move(bt_shape_no_margin));
    }
    return id;
  }

  bool BulletModel::updateElementWorldTransform(const ElementId id, 
                                                const Matrix4d& T_local_to_world)
  {
    const bool element_exists(Model::updateElementWorldTransform(id, T_local_to_world));
    if (element_exists) {
      const Matrix4d& T = elements[id]->getWorldTransform();
      btMatrix3x3 rot;
      btVector3 pos;
      btTransform btT;

      rot.setValue( T(0,0), T(0,1), T(0,2),
                    T(1,0), T(1,1), T(1,2),
                    T(2,0), T(2,1), T(2,2) );
      btT.setBasis(rot);
      pos.setValue( T(0,3), T(1,3), T(2,3) );
      btT.setOrigin(pos);

      bullet_world.bt_collision_objects.at(id)->setWorldTransform(btT);
      bullet_world_no_margin.bt_collision_objects.at(id)->setWorldTransform(btT);
      bullet_world.bt_collision_world->updateAabbs();
      bullet_world_no_margin.bt_collision_world->updateAabbs();
    }
    return element_exists;
  }

  void BulletModel::updateModel()
  {
    bullet_world.bt_collision_world->updateAabbs();
    bullet_world_no_margin.bt_collision_world->updateAabbs();
  }

  bool BulletModel::findClosestPointsBtwElements(const ElementId idA,
                                                 const ElementId idB,
                                                 const bool use_margins,
                                                 std::unique_ptr<ResultCollector>& c)
  {
    btConvexShape* shapeA;
    btConvexShape* shapeB;
    btGjkPairDetector::ClosestPointInput input;
    btPointCollector gjkOutput;

    BulletCollisionWorldWrapper& bt_world = getBulletWorld(use_margins);

    auto bt_objA_iter = bt_world.bt_collision_objects.find(idA);
    if (bt_objA_iter == bt_world.bt_collision_objects.end())
      return false;

    auto bt_objB_iter = bt_world.bt_collision_objects.find(idB);
    if (bt_objB_iter == bt_world.bt_collision_objects.end())
      return false;

    unique_ptr<btCollisionObject>& bt_objA = bt_objA_iter->second;
    unique_ptr<btCollisionObject>& bt_objB = bt_objB_iter->second;

    shapeA = (btConvexShape*) bt_objA->getCollisionShape();
    shapeB = (btConvexShape*) bt_objB->getCollisionShape();

    btGjkEpaPenetrationDepthSolver epa;
    btVoronoiSimplexSolver sGjkSimplexSolver;
    sGjkSimplexSolver.setEqualVertexThreshold(0.f);

    btGjkPairDetector	convexConvex(shapeA,shapeB,&sGjkSimplexSolver,&epa);

    input.m_transformA = bt_objA->getWorldTransform();
    input.m_transformB = bt_objB->getWorldTransform();

    convexConvex.getClosestPoints(input ,gjkOutput,0);

    btVector3 pointOnAinWorld;
    btVector3 pointOnBinWorld;
    if (elements[idA]->getShape() == MESH || elements[idA]->getShape() == BOX) {
      pointOnAinWorld = gjkOutput.m_pointInWorld +
        gjkOutput.m_normalOnBInWorld*(gjkOutput.m_distance+shapeA->getMargin());
    } else {
      pointOnAinWorld = gjkOutput.m_pointInWorld +
        gjkOutput.m_normalOnBInWorld*gjkOutput.m_distance;
    }
    if (elements[idB]->getShape() == MESH || elements[idB]->getShape() == BOX) {
      pointOnBinWorld = gjkOutput.m_pointInWorld - gjkOutput.m_normalOnBInWorld*shapeB->getMargin();
    } else {
      pointOnBinWorld = gjkOutput.m_pointInWorld;
    }

    btVector3 pointOnElemA = input.m_transformA.invXform(pointOnAinWorld);
    btVector3 pointOnElemB = input.m_transformB.invXform(pointOnBinWorld);

    VectorXd pointOnA_1(4);
    VectorXd pointOnB_1(4);
    pointOnA_1 << toVector3d(pointOnElemA), 1;
    pointOnB_1 << toVector3d(pointOnElemB), 1;

    Vector3d pointOnA;
    Vector3d pointOnB;

    pointOnA << elements[idA]->getLocalTransform().topRows(3)*pointOnA_1;
    pointOnB << elements[idB]->getLocalTransform().topRows(3)*pointOnB_1;

    btScalar distance = gjkOutput.m_normalOnBInWorld.dot(pointOnAinWorld-pointOnBinWorld);
    

    if (gjkOutput.m_hasResult) {
      c->addSingleResult(idA,idB,pointOnA,pointOnB, toVector3d(gjkOutput.m_normalOnBInWorld),(double) distance);
    } else {
      c->addSingleResult(idA, 
            idB, 
            Vector3d::Zero(), 
            Vector3d::Zero(),
            Vector3d::Zero(),1.0);
      cerr << "In BulletModel::findClosestPointsBtwElements: No closest point found!" << endl;
    }

    return (c->pts.size() > 0);
  }

  
  bool BulletModel::collisionRaycast(const Matrix3Xd &origins, const Matrix3Xd &ray_endpoints, bool use_margins, VectorXd &distances)
  {
    
    distances.resize(origins.cols());
    
    for (int i = 0; i < origins.cols(); i ++)
    {
    
        btVector3 ray_from_world(origins(0,i), origins(1,i), origins(2,i));
        btVector3 ray_to_world(ray_endpoints(0,i), ray_endpoints(1,i), ray_endpoints(2,i));
        
        btCollisionWorld::ClosestRayResultCallback ray_callback(ray_from_world, ray_to_world);
        
        bullet_world.bt_collision_world->rayTest(ray_from_world, ray_to_world, ray_callback);
        
        if (ray_callback.hasHit()) {
            
            // compute distance to hit
            
            btVector3 end = ray_callback.m_hitPointWorld;
            
            Vector3d end_eigen(end.getX(), end.getY(), end.getZ());
            
            distances(i) = (end_eigen - origins.col(i)).norm();
          
        } else {
            distances(i) = -1;
        }
    }
    
    return true;
  } 

  bool BulletModel::closestPointsAllToAll(const vector<ElementId>& ids_to_check, 
      const bool use_margins,
      vector<PointPair>& closest_points)
  {
    vector<ElementIdPair> id_pairs;
    const auto elements_end = elements.end();
    for (auto idA_iter = ids_to_check.begin();
         idA_iter != ids_to_check.end();
         ++idA_iter) {
      auto elementA_iter = elements.find(*idA_iter);
      if (elementA_iter != elements_end) {
        for (auto idB_iter = idA_iter+1;
            idB_iter != ids_to_check.end();
            ++idB_iter) {
          auto elementB_iter = elements.find(*idB_iter);
          if (elementB_iter != elements_end) {
            if (elements[*idA_iter]->collidesWith(elements[*idB_iter].get())) {
              id_pairs.push_back(make_pair(*idA_iter, *idB_iter));
            }
          }
        }
      } 
    }
    return closestPointsPairwise(id_pairs, use_margins, closest_points);
  }

  bool BulletModel::closestPointsPairwise(const vector<ElementIdPair>& id_pairs, 
                                          const bool use_margins,
                                          vector<PointPair>& closest_points)
  {
    unique_ptr<ResultCollector> c(new ResultCollector());
    for (auto id_pair_iter = id_pairs.begin();
        id_pair_iter != id_pairs.end();
        ++id_pair_iter) {
      findClosestPointsBtwElements(id_pair_iter->first, id_pair_iter->second, 
                                   use_margins, c);
    }

    closest_points = c->getResults();
    return closest_points.size() > 0;
  }
  
  bool BulletModel::collisionPointsAllToAll(const bool use_margins,
                                            vector<PointPair>& collision_points) 
  {
    BulletResultCollector c;
    MatrixXd normals;
    vector<double> distance;
    bullet_world.bt_collision_world->performDiscreteCollisionDetection();
    int numManifolds = bullet_world.bt_collision_world->getDispatcher()->getNumManifolds();
    for (int i=0;i<numManifolds;i++)
    {
      btPersistentManifold* contactManifold =  bullet_world.bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
      const btCollisionObject* obA = contactManifold->getBody0();
      const btCollisionObject* obB = contactManifold->getBody1();
      auto elementA = static_cast< Element*  >(obA->getUserPointer());
      auto elementB = static_cast< Element*  >(obB->getUserPointer());
      Shape shapeA = elementA->getShape();
      Shape shapeB = elementB->getShape();
      double marginA = 0;
      double marginB = 0;
      if (shapeA == MESH || shapeA == BOX) { 
        marginA = obA->getCollisionShape()->getMargin();
      }
      if (shapeB == MESH || shapeB == BOX) { 
        marginB = obB->getCollisionShape()->getMargin();
      }
      int numContacts = contactManifold->getNumContacts();
      for (int j=0;j<numContacts;j++)
      {        btManifoldPoint& pt = contactManifold->getContactPoint(j);
        if (pt.getDistance()+marginA+marginB<0.f)
        {
          const btVector3& normalOnB = pt.m_normalWorldOnB;
          const btVector3& ptA = pt.getPositionWorldOnA() + normalOnB*marginA;
          const btVector3& ptB = pt.getPositionWorldOnB() - normalOnB*marginB;
          c.addSingleResult(elementA->getId(),elementB->getId(),toVector3d(ptA),toVector3d(ptB),
                            toVector3d(normalOnB),(double) pt.getDistance());
        }
      }
    }   
    collision_points = c.getResults();
    return c.pts.size() > 0;
  }

  BulletCollisionWorldWrapper& BulletModel::getBulletWorld(bool use_margins)
  {
    if (use_margins) {
      return bullet_world;
    } else {
      return bullet_world_no_margin;
    }
  }

  BulletModel::unknownShapeException::unknownShapeException(Shape shape)
  { 
    std::ostringstream ostr; 
    ostr << shape; 
    this->shape_str = ostr.str(); 
  }
 
  const char* BulletModel::unknownShapeException::what() const throw()
  {
    return ("Unknown collision shape: " + shape_str + ". Ignoring this collision element").c_str();
  }

}
