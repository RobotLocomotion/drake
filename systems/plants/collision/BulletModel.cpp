#include "BulletModel.h"
#include "BulletElement.h"

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

  void BulletModel::addElement(const int body_ind, Matrix4d T_element_to_link, Shape shape, vector<double> params, bool is_static)
  {
    try {
      BulletElementShPtr new_element( new BulletElement(T_element_to_link, shape, params) );
      bt_collision_world->addCollisionObject(new_element->bt_obj);
      if (is_static) {
        new_element->bt_obj->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
        new_element->bt_obj->activate();
      } else {
        new_element->bt_obj->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);
      }
      Model::addElement(body_ind, new_element);
    } catch (badShapeException& e) {
      cerr << e.what() << endl;
      return;
    }
  }

  void BulletModel::updateElement(ElementShPtr elem, Matrix4d T_link_to_world)
  {
    Model::updateElement(elem, T_link_to_world);
    BulletElementShPtr bullet_elem = static_pointer_cast<BulletElement>(elem);
    bt_collision_world->updateSingleAabb(bullet_elem->bt_obj);
  }

  class BulletResultCollector : public btCollisionWorld::ContactResultCallback
  {
    public:
      vector<Vector3d> ptsA, ptsB, normals;
      double distance;

      virtual	btScalar	addSingleResult(btManifoldPoint& cp,	const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
      {
        Vector3d pt;
        btVector3 bt_pt = cp.getPositionWorldOnA();
        pt(0) = (double) bt_pt.getX();
        pt(1) = (double) bt_pt.getY();
        pt(2) = (double) bt_pt.getZ();
        ptsA.push_back(pt);

        bt_pt = cp.getPositionWorldOnB();
        pt(0) = (double) bt_pt.getX();
        pt(1) = (double) bt_pt.getY();
        pt(2) = (double) bt_pt.getZ();
        ptsB.push_back(pt);

        bt_pt = cp.m_normalWorldOnB;
        pt(0) = (double) bt_pt.getX();
        pt(1) = (double) bt_pt.getY();
        pt(2) = (double) bt_pt.getZ();
        normals.push_back(pt);

        distance = cp.m_distance1;

        return 0;
      }
  };

  bool BulletModel::getPairwiseCollision(const int body_indA, const int body_indB, MatrixXd &ptsA, MatrixXd &ptsB, MatrixXd &normals)
  {
    BulletResultCollector c;
    btCollisionObject* bt_objA;
    btCollisionObject* bt_objB;
    for (ElementShPtr elemA : element_pool[body_indA]){
      bt_objA = static_pointer_cast<BulletElement>(elemA)->bt_obj;
      for (ElementShPtr elemB : element_pool[body_indB]){
        bt_objB = static_pointer_cast<BulletElement>(elemB)->bt_obj;
        bt_collision_world->contactPairTest(bt_objA,bt_objB,c);
      }
    }

    ptsA.resize(3,c.ptsA.size());
    ptsB.resize(3,c.ptsB.size());
    normals.resize(3,c.normals.size());

    for (int i=0; i<c.ptsA.size(); i++) {
      ptsA.col(i) = c.ptsA[i];
      ptsB.col(i) = c.ptsB[i];
      normals.col(i) = c.normals[i];
    }

    return (c.ptsA.size() > 0);
  };

  bool BulletModel::getPairwisePointCollision(const int body_indA, const int body_indB, const int body_collision_indA, Vector3d &ptA, Vector3d &ptB, Vector3d &normal)
  {
    BulletResultCollector c;
    btCollisionObject* bt_objA;
    btCollisionObject* bt_objB;

    ElementShPtr elemA = element_pool[body_indA][body_collision_indA];
    bt_objA = static_pointer_cast<BulletElement>(elemA)->bt_obj;
    for (ElementShPtr elemB : element_pool[body_indB]){
      bt_objB = static_pointer_cast<BulletElement>(elemB)->bt_obj;
      bt_collision_world->contactPairTest(bt_objA,bt_objB,c);
    }

    if (c.ptsA.size() > 0) {
      ptA = c.ptsA[0];
      ptB = c.ptsB[0];
      normal = c.normals[0];
    }
    else {
      ptA << 1,1,1;
      ptB << -1,-1,-1;
      normal << 0,0,1;
    }
    return (c.ptsA.size() > 0);
  };
  bool BulletModel::getPointCollision(const int body_ind, const int body_collision_ind, Vector3d &ptA, Vector3d &ptB, Vector3d &normal)
  {
    BulletResultCollector c;
    ElementShPtr elem = element_pool[body_ind][body_collision_ind];
    btCollisionObject* bt_obj = static_pointer_cast<BulletElement>(elem)->bt_obj;

    bt_collision_world->contactTest(bt_obj,c);

    if (c.ptsA.size() > 0) {
      ptA = c.ptsA[0];
      ptB = c.ptsB[0];
      normal = c.normals[0];
    }
    else {
      ptA << 1,1,1;
      ptB << -1,-1,-1;
      normal << 0,0,1;
    }
    return (c.ptsA.size() > 0);
  };
  bool BulletModel::getClosestPoints(const int body_indA,const int body_indB,Vector3d& ptA,Vector3d& ptB,Vector3d& normal,double& distance)
  {
    btConvexShape* shapeA;
    btConvexShape* shapeB;
    btCollisionObject* bt_objA;
    btCollisionObject* bt_objB;
    btGjkPairDetector::ClosestPointInput input;
    btPointCollector gjkOutput;
    btPointCollector closestOutput;
    closestOutput.m_distance = -1;

    for (ElementShPtr elemA : element_pool[body_indA]){
      bt_objA = static_pointer_cast<BulletElement>(elemA)->bt_obj;
      shapeA = (btConvexShape*) bt_objA->getCollisionShape();
      for (ElementShPtr elemB : element_pool[body_indB]){
        bt_objB = static_pointer_cast<BulletElement>(elemB)->bt_obj;
        shapeB = (btConvexShape*) bt_objB->getCollisionShape();

        btGjkPairDetector	convexConvex(shapeA,shapeB,&sGjkSimplexSolver,&epa);

        input.m_transformA = bt_objA->getWorldTransform();
        input.m_transformB = bt_objB->getWorldTransform();

        convexConvex.getClosestPoints(input ,gjkOutput,0);
        if (gjkOutput.m_distance < closestOutput.m_distance || 
            closestOutput.m_distance == -1){
          closestOutput = gjkOutput;
        }
      }
    }
    btVector3 pointOnAinWorld = closestOutput.m_pointInWorld +
      closestOutput.m_normalOnBInWorld*closestOutput.m_distance;
    ptA(0) = (double) pointOnAinWorld.x();
    ptA(1) = (double) pointOnAinWorld.y();
    ptA(2) = (double) pointOnAinWorld.z();

    ptB(0) = (double) closestOutput.m_pointInWorld.x();
    ptB(1) = (double) closestOutput.m_pointInWorld.y();
    ptB(2) = (double) closestOutput.m_pointInWorld.z();

    normal(0) = (double) closestOutput.m_normalOnBInWorld.x();
    normal(1) = (double) closestOutput.m_normalOnBInWorld.y();
    normal(2) = (double) closestOutput.m_normalOnBInWorld.z();

    distance = (double) closestOutput.m_distance;

    return (closestOutput.m_hasResult);
  }
}
