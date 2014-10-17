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
      auto element_data0 = static_cast< ElementData* >(bt_collision_object0->getUserPointer());
      auto element_data1 = static_cast< ElementData* >(bt_collision_object1->getUserPointer());
      const Body& body0 = parent_model->getBody(element_data0->body_idx);
      const Body& body1 = parent_model->getBody(element_data1->body_idx);
      collides = (body0.getBodyIdx() != body1.getBodyIdx());
      collides = collides && !body0.adjacentTo(body1);
      collides = collides && body0.collidesWith(body1);
    }
    return collides;
  }

  BulletModel::BulletModel()
    : bt_collision_configuration(),
      bt_collision_broadphase(),
      filter_callback()
  {
    bt_collision_dispatcher =  new btCollisionDispatcher( &bt_collision_configuration );
    bt_collision_world = new btCollisionWorld(bt_collision_dispatcher,
        &bt_collision_broadphase, &bt_collision_configuration);
    filter_callback.parent_model = this;
    bt_collision_world->getPairCache()->setOverlapFilterCallback(&filter_callback);
  }
  BulletModel::~BulletModel() {
    delete bt_collision_world;
    delete bt_collision_dispatcher;
  }

  void BulletModel::addElement(const int body_idx, const int parent_idx, 
                                const Matrix4d& T_element_to_link, Shape shape, 
                                const vector<double>& params, 
                                const string& group_name, bool is_static,
                                bool use_margins)
  {
    //DEBUG
    //cout << "BulletModel::addElement: START" << endl;
    //END_DEBUG
    try {
      bodies[body_idx].addElement(body_idx, parent_idx, T_element_to_link, shape, params, group_name, use_margins );
      
      const BulletElement& elem = bodies.at(body_idx).back();
      element_data.push_back(unique_ptr<ElementData>(new ElementData(body_idx,elem.getShape())));
      elem.bt_obj->setUserPointer(element_data.back().get());
      bt_collision_world->addCollisionObject(elem.bt_obj.get());
      if (is_static) {
        elem.bt_obj->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
        elem.bt_obj->activate();
      } else {
        elem.bt_obj->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);
      }
    } catch (badShapeException& e) {
      cerr << "BulletModel::addElement: On body " << body_idx << ": " 
           << e.what() << endl;
      return;
    }
    //DEBUG
    //cout << "BulletModel::addElement: END" << endl;
    //END_DEBUG
  }

  bool BulletModel::updateElementsForBody(const int body_idx,
                                  const Matrix4d& T_link_to_world)
  {
    auto iter_for_body( bodies.find(body_idx) );
    if ( iter_for_body!=bodies.end() ) { // Then bodies[body_idx] exists
      iter_for_body->second.updateElements(T_link_to_world);
      for (const BulletElement& elem : bodies.at(body_idx).getElements()) {
        bt_collision_world->updateSingleAabb(elem.bt_obj.get());
      }
      return true;
    } else {
      return false;
    }
  }

  bool BulletModel::setCollisionFilter(const int body_idx,
                                      const uint16_t group, 
                                      const uint16_t mask)
  {
    auto iter_for_body( bodies.find(body_idx) );
    if ( iter_for_body!=bodies.end() ) { // Then bodies[body_idx] exists
      setCollisionFilter(iter_for_body->second,group,mask);
      return true;
    } else {
      return false;
    }
  }


  bool BulletModel::findClosestPointsBtwElements(const int bodyA_idx,
                                                 const int bodyB_idx,
                                                 const BulletElement& elemA, 
                                                 const BulletElement& elemB, 
                                                 const set<string>& active_element_groups,
                                                 const ResultCollShPtr& c) 
  {
    //DEBUG
    //if ((bodyA_idx==5) && (bodyB_idx==19)) {
      //cout << "BulletModel::findClosestPointsBtwElements: BAD!!!" << endl;
    //}
    //END_DEBUG
    btConvexShape* shapeA;
    btConvexShape* shapeB;
    btGjkPairDetector::ClosestPointInput input;
    btPointCollector gjkOutput;

    shapeA = (btConvexShape*) elemA.bt_obj->getCollisionShape();
    shapeB = (btConvexShape*) elemB.bt_obj->getCollisionShape();

    btGjkEpaPenetrationDepthSolver epa;
    btVoronoiSimplexSolver sGjkSimplexSolver;
    sGjkSimplexSolver.setEqualVertexThreshold(0.f);

    btGjkPairDetector	convexConvex(shapeA,shapeB,&sGjkSimplexSolver,&epa);

    input.m_transformA = elemA.bt_obj->getWorldTransform();
    input.m_transformB = elemB.bt_obj->getWorldTransform();

    convexConvex.getClosestPoints(input ,gjkOutput,0);

    //DEBUG
    //cout << "Margin A: " << shapeA->getMargin() << endl;
    //cout << "Margin B: " << shapeB->getMargin() << endl;
    //END_DEBUG
    btVector3 pointOnAinWorld;
    btVector3 pointOnBinWorld;
    if (elemA.getShape() == MESH || elemA.getShape() == BOX) {
      pointOnAinWorld = gjkOutput.m_pointInWorld +
        gjkOutput.m_normalOnBInWorld*(gjkOutput.m_distance+shapeA->getMargin());
    } else {
      pointOnAinWorld = gjkOutput.m_pointInWorld +
        gjkOutput.m_normalOnBInWorld*gjkOutput.m_distance;
    }
    if (elemB.getShape() == MESH || elemB.getShape() == BOX) {
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
    pointOnA << elemA.getLinkTransform().topRows(3)*pointOnA_1;
    pointOnB << elemB.getLinkTransform().topRows(3)*pointOnB_1;

    btScalar distance = gjkOutput.m_normalOnBInWorld.dot(pointOnAinWorld-pointOnBinWorld);
    
    //DEBUG
    //cerr << "In BulletModel::findClosestPointsBtwElements:" << endl;
    //std::cout << "ptsA:" << std::endl;
    //cout << pointOnA.getX() << ' ' 
         //<< pointOnA.getY() << ' ' 
         //<< pointOnA.getZ() << endl;
    //std::cout << "ptsB:" << std::endl;
    //cout << pointOnB.getX() << ' ' 
         //<< pointOnB.getY() << ' ' 
         //<< pointOnB.getZ() << endl;
    //std::cout << "normal:" << std::endl;
    //cout << gjkOutput.m_normalOnBInWorld.getX() << ' ' 
         //<< gjkOutput.m_normalOnBInWorld.getY() << ' ' 
         //<< gjkOutput.m_normalOnBInWorld.getZ() << endl;
    //END_DEBUG

    if (gjkOutput.m_hasResult) {
      static_pointer_cast<MinDistResultCollector>(c)->addSingleResult(bodyA_idx,bodyB_idx,toVector3d(pointOnA),toVector3d(pointOnB),
          toVector3d(gjkOutput.m_normalOnBInWorld),(double) distance);
    } else {
      c->addPointPairResult(PointPair(bodyA_idx, 
            bodyB_idx, 
            Vector3d::Zero(), 
            Vector3d::Zero(),
            Vector3d::Zero(),1.0));
      cerr << "In BulletModel::findClosestPointsBtwElements: No closest point found!" << endl;
      //DEBUG
      //btVector3 originA(input.m_transformA.getOrigin());
      //btVector3 originB(input.m_transformB.getOrigin());
      //cerr << "body " << bodyA_idx << " origin: [" << originA.x() << ", " << originA.y() << ", " << originA.z() << "]" << endl;
      //cerr <<" << bodyA_idx << "  "body " << bodyB_idx << " origin: [" << originB.x() << ", " << originB.y() << ", " << originB.z() << "]" << endl;
      //END_DEBUG
      //throw exception();
    }

    return (c->pts.size() > 0);
  }

  bool BulletModel::findCollisionPointsBtwElements(const int bodyA_idx,
                                                   const int bodyB_idx,
                                                   const BulletElement& elemA, 
                                                   const BulletElement& elemB, 
                                                   const ResultCollShPtr& c)
  {
    auto bt_c = static_pointer_cast<BulletResultCollector>(c);
    bt_c->setBodyIdx(bodyA_idx, bodyB_idx);
    bt_collision_world->contactPairTest(elemA.bt_obj.get(),elemB.bt_obj.get(),
        *bt_c->getBtPtr());

    return (c->pts.size() > 0);
  }

  bool BulletModel::getPointCollision(const int body_idx, 
                                      const int body_collision_idx, 
                                      Vector3d& ptA, Vector3d& ptB, 
                                      Vector3d& normal)
  {
    ResultCollShPtr c = ResultCollShPtr(new BulletResultCollector());
    const BulletElement& elem = bodies.at(body_idx).at(body_collision_idx);

    bt_collision_world->contactTest(elem.bt_obj.get(),
            *static_pointer_cast<BulletResultCollector>(c)->getBtPtr());

    c->getResults(ptA,ptB,normal);

    return (c->pts.size() > 0);
  };
  
  bool BulletModel::getPairwiseCollision(const int bodyA_idx, 
          const int bodyB_idx,
          MatrixXd& ptsA, MatrixXd& ptsB,
          MatrixXd& normals)
  {
    //DEBUG
    //try {
    //END_DEBUG
    ResultCollShPtr c = ResultCollShPtr(new BulletResultCollector());
    findCollisionPointsBtwBodies(bodyA_idx,bodyB_idx,c);
    c->getResults(ptsA,ptsB,normals);
    
    return (c->pts.size() > 0);
    //DEBUG
    //} catch (const std::out_of_range& oor) {
    //std::string msg("In ModelTemplate::getPairwiseCollision:\n");
    //throw std::out_of_range(msg + oor.what());
    //}
    //END_DEBUG
  };

  bool BulletModel::getPairwisePointCollision(const int bodyA_idx,
          const int bodyB_idx,
          const int bodyA_collision_idx,
          Vector3d &ptA, Vector3d &ptB,
          Vector3d &normal)
  {
    //DEBUG
    //try {
    //END_DEBUG
    ResultCollShPtr c = ResultCollShPtr(new BulletResultCollector());
    const BulletElement& elemA = bodies.at(bodyA_idx).at(bodyA_collision_idx);
    findCollisionPointsBtwElements(bodyA_idx,bodyB_idx,elemA,bodies[bodyB_idx].getElements(),c);
    c->getResults(ptA,ptB,normal);
    
    return (c->pts.size() > 0);
    //DEBUG
    //} catch (const std::out_of_range& oor) {
    //std::string msg("In ModelTemplate::getPairwisePointCollision:\n");
    //throw std::out_of_range(msg + oor.what());
    //}
    //END_DEBUG
  };
  
  bool BulletModel::collisionRaycast(const Matrix3Xd &origins, const Matrix3Xd &ray_endpoints, VectorXd &distances)
  {
    
    distances.resize(origins.cols());
    
    for (int i = 0; i < origins.cols(); i ++)
    {
    
        btVector3 ray_from_world(origins(0,i), origins(1,i), origins(2,i));
        btVector3 ray_to_world(ray_endpoints(0,i), ray_endpoints(1,i), ray_endpoints(2,i));
        
        btCollisionWorld::ClosestRayResultCallback ray_callback(ray_from_world, ray_to_world);
        
        bt_collision_world->rayTest(ray_from_world, ray_to_world, ray_callback);
        
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

  bool BulletModel::getClosestPoints(const int bodyA_idx, const int bodyB_idx,
          Vector3d& ptA, Vector3d& ptB, Vector3d& normal,
          double& distance)
  {
    //DEBUG
    //try {
    //END_DEBUG
    ResultCollShPtr c(new MinDistResultCollector());
    findClosestPointsBtwBodies(bodyA_idx,bodyB_idx,elementGroupNames(),c);
    c->pts.at(0).getResults(ptA, ptB, normal, distance);
    return (c->pts.size() > 0);
    //DEBUG
    //} catch (const std::out_of_range& oor) {
    //std::string msg("In ModelTemplate::getClosestPoints:\n");
    //throw std::out_of_range(msg + oor.what());
    //}
    //END_DEBUG
  }
     
  bool BulletModel::closestPointsAllBodies(std::vector<int>& bodyA_idx,
          std::vector<int>& bodyB_idx,
          MatrixXd& ptsA, MatrixXd& ptsB,
          MatrixXd& normal,
          VectorXd& distance,
          const std::vector<int>& bodies_idx,
          const std::set<std::string>& active_element_groups)
  {
    bool has_result=false;
    //DEBUG
    //try {
    //END_DEBUG
    //ResultCollector c;
    ResultCollShPtr c = std::make_shared<ResultCollector>();
    //DEBUG
    //std::cout << "ModelTemplate::closestPointsAllBodies: " << std::endl;
    //std::cout << "Num active bodies: " << bodies_idx.size() << std::endl;
    //END_DEBUG
    //for (auto itA=bodies.begin(); itA!=bodies.end(); ++itA) {
    for (typename std::vector<int>::const_iterator itA=bodies_idx.begin(); itA!=bodies_idx.end(); ++itA) {
      if (bodies.count(*itA) > 0) {
        //DEBUG
        //std::cout << "ModelTemplate::closestPointsAllBodies: " << std::endl;
        //std::cout << "Body A found" << std::endl;
        //END_DEBUG
        //for (typename std::map<int,Body<ElementT>>::iterator itB=itA; itB!=bodies.end(); ++itB) {
        for (typename std::vector<int>::const_iterator itB=itA; itB!=bodies_idx.end(); ++itB) {
          //for (auto itB=bodies.begin(); itB!=bodies.end(); ++itB) {
          if (bodies.count(*itB) > 0) {
            //DEBUG
            //std::cout << "ModelTemplate::closestPointsAllBodies: " << std::endl;
            //std::cout << "Body B found" << std::endl;
            //END_DEBUG
            Body& bodyA(bodies[*itA]);
            Body& bodyB(bodies[*itB]);
            //DEBUG
            //std::cout << "ModelTemplate::closestPointsAllBodies: " << std::endl;
            //std::cout << "Body A idx:" << bodyA.getBodyIdx() << std::endl;
            //std::cout << "Body B idx:" << bodyB.getBodyIdx() << std::endl;
            //END_DEBUG
            //ResultCollShPtr c_min_dist = std::make_shared<MinDistResultCollector>();
            if ( bodyA.collidesWith(bodyB) ) {
              //DEBUG
              //std::cout << "ModelTemplate::closestPointsAllBodies: Body A: " << bodyA.getBodyIdx() << std::endl;
              //std::cout << "ModelTemplate::closestPointsAllBodies: Body B: " << bodyB.getBodyIdx() << std::endl;
              //END_DEBUG
              has_result = findClosestPointsBtwBodies(bodyA.getBodyIdx(),
                      bodyB.getBodyIdx(), active_element_groups,
                      c);
            }
          }
        }
      }
    }
    c->getResults(bodyA_idx, bodyB_idx, ptsA, ptsB,normal,distance);
    
    //DEBUG
    //std::cout << "ModelTemplate:closestPointsAllBodies:" << std::endl;
    //std::cout << "ptsA:" << std::endl;
    //std::cout << ptsA.transpose() << std::endl;
    //std::cout << "ptsB:" << std::endl;
    //std::cout << ptsB.transpose() << std::endl;
    //std::cout << "normal:" << std::endl;
    //std::cout << normal.transpose() << std::endl;
    //END_DEBUG
    //DEBUG
    //} catch (const std::out_of_range& oor) {
    //std::string msg("In ModelTemplate::closestPointsAllBodies:\n");
    //throw std::out_of_range(msg + oor.what());
    //}
    //END_DEBUG
    return has_result;
  };
  
  bool BulletModel::allCollisions(vector<int>& bodyA_idx,
          vector<int>& bodyB_idx,
          MatrixXd& ptsA, MatrixXd& ptsB)
  {
    BulletResultCollector c;
    MatrixXd normals;
    vector<double> distance;
    bt_collision_world->performDiscreteCollisionDetection();
    int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
    for (int i=0;i<numManifolds;i++)
    {
      btPersistentManifold* contactManifold =  bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
      const btCollisionObject* obA = contactManifold->getBody0();
      const btCollisionObject* obB = contactManifold->getBody1();
      auto element_dataA = static_cast< ElementData*  >(obA->getUserPointer());
      auto element_dataB = static_cast< ElementData*  >(obB->getUserPointer());
      int bodyA_idx_i = element_dataA->body_idx;
      int bodyB_idx_i = element_dataB->body_idx;
      Shape shapeA = element_dataA->shape;
      Shape shapeB = element_dataB->shape;
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
          c.addSingleResult(bodyA_idx_i,bodyB_idx_i,toVector3d(ptA),toVector3d(ptB),
                            toVector3d(normalOnB),(double) pt.getDistance());
        }
      }
    }   
    c.getResults(bodyA_idx,bodyB_idx,ptsA,ptsB,normals,distance);
    return c.pts.size() > 0;
  }

  const vector<int> BulletModel::bodyIndices() const
  {
    std::vector<int> bodies_idx;
    for (auto it = bodies.begin(); it != bodies.end(); ++it) {
      bodies_idx.push_back(it->first);
    }
    return bodies_idx;
  }
}
