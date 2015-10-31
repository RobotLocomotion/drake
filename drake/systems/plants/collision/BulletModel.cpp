#include <iostream>

#include "DrakeCollision.h"
#include "BulletModel.h"

using namespace std;
using namespace Eigen;

namespace DrakeCollision
{

  struct BinaryContactResultCallback : public btCollisionWorld::ContactResultCallback
  {
    public:
      BinaryContactResultCallback()
      {
        in_collision = false;
      }

      bool isInCollision()
      {
        return in_collision;
      }

      virtual	btScalar	addSingleResult(
          btManifoldPoint& cp,	
          const btCollisionObjectWrapper* colObj0Wrap,
          int partId0,int index0,
          const btCollisionObjectWrapper* colObj1Wrap,
          int partId1,int index1) 
      {
        in_collision = true;
        return 0;
      }
    private:
        bool in_collision;
  };

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
      if ((bt_collision_object0->getUserPointer() != NULL) &&
          (bt_collision_object1->getUserPointer() != NULL)) {
        auto element0 = static_cast< Element* >(bt_collision_object0->getUserPointer());
        auto element1 = static_cast< Element* >(bt_collision_object1->getUserPointer());
        collides = collides && element0->collidesWith(element1);
      }
    }
    return collides;
  }

  BulletCollisionWorldWrapper::BulletCollisionWorldWrapper()
    : bt_collision_configuration(), bt_collision_broadphase(), filter_callback()
  {
    bt_collision_configuration.setConvexConvexMultipointIterations(0, 0);
    bt_collision_configuration.setPlaneConvexMultipointIterations(0, 0);
    bt_collision_dispatcher = unique_ptr<btCollisionDispatcher>(new btCollisionDispatcher(&bt_collision_configuration));
    bt_collision_world = unique_ptr<btCollisionWorld>(new btCollisionWorld(bt_collision_dispatcher.get(), &bt_collision_broadphase, &bt_collision_configuration));

    bt_collision_world->getPairCache()->setOverlapFilterCallback(&filter_callback);
  }

  unique_ptr<btCollisionShape> BulletModel::newBulletBoxShape(const DrakeShapes::Box& geometry, bool use_margins)
  {
    unique_ptr<btCollisionShape> bt_shape(new btConvexHullShape());
    btBoxShape bt_box( btVector3(geometry.size(0)/2,geometry.size(1)/2,geometry.size(2)/2) );
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

  unique_ptr<btCollisionShape> BulletModel::newBulletSphereShape(const DrakeShapes::Sphere& geometry, bool use_margins)
  {
    unique_ptr<btCollisionShape> bt_shape(new btSphereShape(geometry.radius));
    return bt_shape;
  }

  unique_ptr<btCollisionShape> BulletModel::newBulletCylinderShape(const DrakeShapes::Cylinder& geometry, bool use_margins)
  {
    unique_ptr<btCollisionShape> bt_shape(new btCylinderShapeZ( btVector3(geometry.radius,geometry.radius,geometry.length/2) ));
    return bt_shape;
  }

  unique_ptr<btCollisionShape> BulletModel::newBulletCapsuleShape(const DrakeShapes::Capsule& geometry, bool use_margins)
  {
    unique_ptr<btCollisionShape> bt_shape(new btConvexHullShape());
    dynamic_cast<btConvexHullShape*>(bt_shape.get())->addPoint(btVector3(0,0,-geometry.length/2));
    dynamic_cast<btConvexHullShape*>(bt_shape.get())->addPoint(btVector3(0,0,geometry.length/2));
    bt_shape->setMargin(geometry.radius);
    return bt_shape;
  }

  unique_ptr<btCollisionShape> BulletModel::newBulletMeshShape(const DrakeShapes::Mesh& geometry, bool use_margins)
  {
    unique_ptr<btCollisionShape> bt_shape(new btConvexHullShape());
    Matrix3Xd vertices;
    if(geometry.extractMeshVertices(vertices)) {
      if (use_margins)
        bt_shape->setMargin(BulletModel::large_margin);
      else
        bt_shape->setMargin(BulletModel::small_margin);
      auto bt_convex_hull_shape = dynamic_cast<btConvexHullShape*>(bt_shape.get());
      for (int i = 0; i < vertices.cols(); i++) {
        bt_convex_hull_shape->addPoint(btVector3(vertices(0, i), vertices(1, i), vertices(2, i)), false);
      }
      bt_convex_hull_shape->recalcLocalAabb();

      return bt_shape;
    } else {
      return nullptr;
    }
  }

  unique_ptr<btCollisionShape> BulletModel::newBulletMeshPointsShape(const DrakeShapes::MeshPoints& geometry, bool use_margins)
  {
    unique_ptr<btCollisionShape> bt_shape(new btConvexHullShape());
    if (use_margins)
      bt_shape->setMargin(BulletModel::large_margin);
    else
      bt_shape->setMargin(BulletModel::small_margin);
    auto bt_convex_hull_shape = dynamic_cast<btConvexHullShape*>(bt_shape.get());
    for (int i = 0; i < geometry.points.cols(); i++){
      bt_convex_hull_shape->addPoint(btVector3(geometry.points(0, i), geometry.points(1, i), geometry.points(2, i)), false);
    }
    bt_convex_hull_shape->recalcLocalAabb();
    return bt_shape;
  }

  ElementId BulletModel::addElement(const Element& element)
  {
    ElementId id =  Model::addElement(element);

    if (id != 0) {
      unique_ptr<btCollisionShape> bt_shape;
      unique_ptr<btCollisionShape> bt_shape_no_margin;
      switch (elements[id]->getShape()) {
        case DrakeShapes::BOX:
          {
            const auto box = static_cast<const DrakeShapes::Box&>(elements[id]->getGeometry());
            bt_shape = newBulletBoxShape(box, true);          
            bt_shape_no_margin = newBulletBoxShape(box, false);
          }
          break;
        case DrakeShapes::SPHERE:
          {
            const auto sphere = static_cast<const DrakeShapes::Sphere&>(elements[id]->getGeometry());
            bt_shape = newBulletSphereShape(sphere, true);
            bt_shape_no_margin = newBulletSphereShape(sphere, false);
          }
          break;
        case DrakeShapes::CYLINDER:
          {
            const auto cylinder = static_cast<const DrakeShapes::Cylinder&>(elements[id]->getGeometry());
            bt_shape = newBulletCylinderShape(cylinder, true);
            bt_shape_no_margin = newBulletCylinderShape(cylinder, false);
          }
          break;
        case DrakeShapes::MESH:
          {
            const auto mesh = static_cast<const DrakeShapes::Mesh&>(elements[id]->getGeometry());
            bt_shape = newBulletMeshShape(mesh, true);
            bt_shape_no_margin = newBulletMeshShape(mesh, false);
          }
          break;
        case DrakeShapes::MESH_POINTS:
          {
            const auto mesh = static_cast<const DrakeShapes::MeshPoints&>(elements[id]->getGeometry());
            bt_shape = newBulletMeshPointsShape(mesh, true);
            bt_shape_no_margin = newBulletMeshPointsShape(mesh, false);
          }
          break;
        case DrakeShapes::CAPSULE:
          {
            const auto capsule = static_cast<const DrakeShapes::Capsule&>(elements[id]->getGeometry());
            bt_shape = newBulletCapsuleShape(capsule, true);
            bt_shape_no_margin = newBulletCapsuleShape(capsule, false);
          }
          break;
        default:
          cerr << "Warning: Collision elements[id] has an unknown type " << elements[id]->getShape() << endl;
          throw unknownShapeException(elements[id]->getShape());
          break;
      }
      if (bt_shape) {
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
    }
    return id;
  }

  vector<PointPair> BulletModel::potentialCollisionPoints(bool use_margins)
  {
    BulletCollisionWorldWrapper& bt_world = getBulletWorld(use_margins);
    bt_world.bt_collision_configuration.setConvexConvexMultipointIterations(PERTURBATION_ITERATIONS, MINIMUM_POINTS_PERTURBATION_THRESHOLD);
    bt_world.bt_collision_configuration.setPlaneConvexMultipointIterations(PERTURBATION_ITERATIONS, MINIMUM_POINTS_PERTURBATION_THRESHOLD);
    BulletResultCollector c;
    bt_world.bt_collision_world->performDiscreteCollisionDetection();
    size_t numManifolds = bt_world.bt_collision_world->getDispatcher()->getNumManifolds();
    
    for (size_t i = 0; i < numManifolds; i++)
    {
      btPersistentManifold* contact_manifold =  bt_world.bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
      const btCollisionObject* obA = contact_manifold->getBody0();
      const btCollisionObject* obB = contact_manifold->getBody1();

      auto elementA = static_cast< Element*  >(obA->getUserPointer());
      auto elementB = static_cast< Element*  >(obB->getUserPointer());

      DrakeShapes::Shape shapeA = elementA->getShape();
      DrakeShapes::Shape shapeB = elementB->getShape();

      ElementId idA = elementA->getId();
      ElementId idB = elementB->getId();

      double marginA = 0;
      double marginB = 0;

      if (shapeA ==  DrakeShapes::MESH || shapeA ==  DrakeShapes::BOX) { 
        marginA = obA->getCollisionShape()->getMargin();
      }
      if (shapeB == DrakeShapes::MESH || shapeB == DrakeShapes::BOX) { 
        marginB = obB->getCollisionShape()->getMargin();
      }
      size_t num_contacts = contact_manifold->getNumContacts();

      for (size_t j = 0; j < num_contacts; j++)
      {        
        btManifoldPoint& pt = contact_manifold->getContactPoint(j);
        const btVector3& normal_on_B = pt.m_normalWorldOnB;
        const btVector3& point_on_A_in_world = pt.getPositionWorldOnA() + normal_on_B * marginA;
        const btVector3& point_on_B_in_world = pt.getPositionWorldOnB() - normal_on_B * marginB;
        const btVector3 point_on_elemA = obA->getWorldTransform().invXform(point_on_A_in_world);
        const btVector3 point_on_elemB = obB->getWorldTransform().invXform(point_on_B_in_world);

        Vector4d point_on_A_1, point_on_B_1;

        point_on_A_1 << toVector3d(point_on_elemA), 1.0;
        point_on_B_1 << toVector3d(point_on_elemB), 1.0;

        Vector3d point_on_A, point_on_B;

        point_on_A << elements[idA]->getLocalTransform().topRows(3) * point_on_A_1;
        point_on_B << elements[idB]->getLocalTransform().topRows(3) * point_on_B_1;

        c.addSingleResult(idA, idB, point_on_A, point_on_B, toVector3d(normal_on_B), static_cast<double>(pt.getDistance()) + marginA + marginB);
      }
    }   

    bt_world.bt_collision_configuration.setConvexConvexMultipointIterations(0, 0);
    bt_world.bt_collision_configuration.setPlaneConvexMultipointIterations(0, 0);
    return c.getResults();
  }

  vector<size_t> BulletModel::collidingPoints(const vector<Vector3d>& points, 
                                              double collision_threshold)
  {
    // Create sphere geometry
    btSphereShape bt_shape(collision_threshold);

    // Create Bullet collision object
    btCollisionObject bt_obj;
    btCollisionObject* bt_obj_ptr = static_cast<btCollisionObject*>(&bt_obj);
    bt_obj.setCollisionShape(static_cast<btCollisionShape*>(&bt_shape));

    btTransform btT;
    btT.setIdentity();
    BulletCollisionWorldWrapper& bt_world = getBulletWorld(false);
    vector<size_t> in_collision_indices;

    for (size_t i = 0; i < points.size(); ++i) {
      BinaryContactResultCallback c;

      btVector3 pos(static_cast<btScalar>(points[i](0)), 
                    static_cast<btScalar>(points[i](1)),
                    static_cast<btScalar>(points[i](2)));
      btT.setOrigin(pos);
      bt_obj.setWorldTransform(btT);

      bt_world.bt_collision_world->contactTest(bt_obj_ptr, c);

      if (c.isInCollision()) {
        in_collision_indices.push_back(i);
      }
    }

    return in_collision_indices;
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

      auto bt_obj_iter = bullet_world.bt_collision_objects.find(id);
      auto bt_obj_no_margin_iter = bullet_world_no_margin.bt_collision_objects.find(id);
      if (bt_obj_iter != bullet_world.bt_collision_objects.end()) {
        bullet_world.bt_collision_objects.at(id)->setWorldTransform(btT);
      }

      if (bt_obj_no_margin_iter != bullet_world_no_margin.bt_collision_objects.end()) {
        bullet_world_no_margin.bt_collision_objects.at(id)->setWorldTransform(btT);
      }
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
    if (elements[idA]->getShape() == DrakeShapes::MESH || 
        elements[idA]->getShape() == DrakeShapes::MESH_POINTS || 
        elements[idA]->getShape() == DrakeShapes::BOX) {
      pointOnAinWorld = gjkOutput.m_pointInWorld +
        gjkOutput.m_normalOnBInWorld*(gjkOutput.m_distance+shapeA->getMargin());
    } else {
      pointOnAinWorld = gjkOutput.m_pointInWorld +
        gjkOutput.m_normalOnBInWorld*gjkOutput.m_distance;
    }
    if (elements[idB]->getShape() == DrakeShapes::MESH || 
        elements[idB]->getShape() == DrakeShapes::MESH_POINTS || 
        elements[idB]->getShape() == DrakeShapes::BOX) {
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
      cerr << "In BulletModel::findClosestPointsBtwElements: No closest point found between " << idA << " and " << idB << endl;
    }

    return (c->pts.size() > 0);
  }

  
  bool BulletModel::collisionRaycast(const Matrix3Xd &origins, const Matrix3Xd &ray_endpoints, bool use_margins, VectorXd &distances)
  {
    
    distances.resize(origins.cols());
    BulletCollisionWorldWrapper& bt_world = getBulletWorld(use_margins);
    
    for (int i = 0; i < origins.cols(); i ++)
    {
    
        btVector3 ray_from_world(origins(0,i), origins(1,i), origins(2,i));
        btVector3 ray_to_world(ray_endpoints(0,i), ray_endpoints(1,i), ray_endpoints(2,i));
        
        btCollisionWorld::ClosestRayResultCallback ray_callback(ray_from_world, ray_to_world);
        
        bt_world.bt_collision_world->rayTest(ray_from_world, ray_to_world, ray_callback);
        
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
    BulletCollisionWorldWrapper& bt_world = getBulletWorld(use_margins);
    bt_world.bt_collision_world->performDiscreteCollisionDetection();
    int numManifolds = bt_world.bt_collision_world->getDispatcher()->getNumManifolds();
    for (int i=0;i<numManifolds;i++)
    {
      btPersistentManifold* contactManifold =  bt_world.bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
      const btCollisionObject* obA = contactManifold->getBody0();
      const btCollisionObject* obB = contactManifold->getBody1();
      auto elementA = static_cast< Element*  >(obA->getUserPointer());
      auto elementB = static_cast< Element*  >(obB->getUserPointer());
      DrakeShapes::Shape shapeA = elementA->getShape();
      DrakeShapes::Shape shapeB = elementB->getShape();
      double marginA = 0;
      double marginB = 0;
      if (shapeA == DrakeShapes::MESH || 
          shapeA == DrakeShapes::MESH_POINTS || 
          shapeA == DrakeShapes::BOX) { 
        marginA = obA->getCollisionShape()->getMargin();
      }
      if (shapeB == DrakeShapes::MESH || 
          shapeB == DrakeShapes::MESH_POINTS || 
          shapeB == DrakeShapes::BOX) { 
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

  BulletModel::unknownShapeException::unknownShapeException(DrakeShapes::Shape shape)
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
