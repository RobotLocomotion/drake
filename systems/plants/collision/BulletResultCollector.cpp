#include <btBulletCollisionCommon.h>

#include "BulletResultCollector.h"
#include "ResultCollector.h"
#include <iostream>


using namespace std;

namespace DrakeCollision
{
  btScalar BulletResultCollector::addSingleResult(btManifoldPoint& cp,	const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
  {
    addSingleResult(curr_bodyA_idx, curr_bodyB_idx, toVector3d(cp.getPositionWorldOnA()),
                    toVector3d(cp.getPositionWorldOnB()), toVector3d(cp.m_normalWorldOnB),
                    (double) cp.m_distance1);

    return 0;
  }

  btCollisionWorld::ContactResultCallback* BulletResultCollector::getBtPtr()
  {
    return static_cast<btCollisionWorld::ContactResultCallback*>(this);
  }

  void BulletResultCollector::setBodyIdx(const int bodyA_idx, const int bodyB_idx)
  {
    curr_bodyA_idx = bodyA_idx;
    curr_bodyB_idx = bodyB_idx;
  }

  Vector3d toVector3d(const btVector3& bt_vec)
  {
    Vector3d vec;
    vec(0) = (double) bt_vec.getX();
    vec(1) = (double) bt_vec.getY();
    vec(2) = (double) bt_vec.getZ();
    //DEBUG
    //cout << "BulletResultCollector:toVector3d:" << endl;
    //cout << "btVector3:" << endl;
    //cout << bt_vec.getX() << ' ' << bt_vec.getY() << ' ' << bt_vec.getZ() << endl;
    //cout << "Vector3d:" << endl;
    //cout << vec.transpose() << endl;
    //END_DEBUG
    return vec;
  }

}
