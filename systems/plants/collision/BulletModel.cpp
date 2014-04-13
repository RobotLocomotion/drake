#include "BulletModel.h"
#include "BulletElement.h"
#include "BulletResultCollector.h"

using namespace std;

namespace DrakeCollision
{
  BulletModel::BulletModel()
    : bt_collision_configuration(),
      bt_collision_broadphase()
  {
    bt_collision_dispatcher =  new btCollisionDispatcher( &bt_collision_configuration );
    bt_collision_world = new btCollisionWorld(bt_collision_dispatcher,
        &bt_collision_broadphase, &bt_collision_configuration);
  }
  BulletModel::~BulletModel() {
    delete bt_collision_world;
    delete bt_collision_dispatcher;
  }

  void BulletModel::addElement(const int body_idx, const int parent_idx, 
                                const Matrix4d& T_element_to_link, Shape shape, 
                                const vector<double>& params, bool is_static)
  {
    //DEBUG
    //cout << "BulletModel::addElement: START" << endl;
    //END_DEBUG
    try {
      ModelTemplate::addElement(body_idx,parent_idx, T_element_to_link, shape, 
                                params, is_static);
      const BulletElement& elem = bodies.at(body_idx).back();
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
    bool idx_valid = ModelTemplate::updateElementsForBody(body_idx, T_link_to_world);
    if (idx_valid) {
      for (const BulletElement& elem : bodies.at(body_idx).getElements()) {
        bt_collision_world->updateSingleAabb(elem.bt_obj.get());
      }
    }
    return idx_valid;
  }

  //void BulletModel::setCollisionFilter(Body<BulletElement>& body,
                                       //const bitmask& group,
                                       //const bitmask&  mask)
  //{
    // Bullet uses the 6 lowest bits for its internal collision groups
    // Don't change those bits. 
    //
    // Disregard. They only use those in their dynamics code
    //const bitmask bt_internal = bitmask(string(6,'1')); 
    //const bitmask bt_group = bitmask(group<<6) | (body.getGroup() & bt_internal); 
    //const bitmask bt_mask = bitmask(mask<<6) | (body.getMask() & bt_internal);
    //DEBUG
    //cout << "BulletModel::setCollisionFilter: Group: " << bt_group << endl;
    //cout << "BulletModel::setCollisionFilter: Mask: " << bt_mask << endl;
    //END_DEBUG

    //ModelTemplate<BulletElement>::setCollisionFilter(body,bt_group,bt_mask);
  //}

  bool BulletModel::findClosestPointsBtwElements(const int bodyA_idx,
                                                 const int bodyB_idx,
                                                 const BulletElement& elemA, 
                                                 const BulletElement& elemB, 
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
    if (elemA.getShape() == MESH) {
      pointOnAinWorld = gjkOutput.m_pointInWorld +
        gjkOutput.m_normalOnBInWorld*(gjkOutput.m_distance+shapeA->getMargin());
    } else {
      pointOnAinWorld = gjkOutput.m_pointInWorld +
        gjkOutput.m_normalOnBInWorld*gjkOutput.m_distance;
    }
    if (elemB.getShape() == MESH) {
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
    ResultCollShPtr c = newResultCollector();
    const BulletElement& elem = bodies.at(body_idx).at(body_collision_idx);

    bt_collision_world->contactTest(elem.bt_obj.get(),
            *static_pointer_cast<BulletResultCollector>(c)->getBtPtr());

    c->getResults(ptA,ptB,normal);

    return (c->pts.size() > 0);
  };

}
