#include <btBulletCollisionCommon.h>

#include "BulletResultCollector.h"
#include "ResultCollector.h"


namespace DrakeCollision
{
  btScalar BulletResultCollector::addSingleResult(btManifoldPoint& cp,	const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
  {
    addSingleResult(curr_bodyA_idx, curr_bodyB_idx, cp.getPositionWorldOnA(),
                    cp.getPositionWorldOnB(), cp.m_normalWorldOnB,
                    cp.m_distance1);

    return 0;
  }

  btCollisionWorld::ContactResultCallback& BulletResultCollector::getBtPtr()
  {
    static_cast<btCollisionWorld::ContactResultCallback*>(this);
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
    return vec;
  }

  // explicit instantiations (required for linking):
  template void ResultCollector::addSingleResult(const int, const int, 
                                                 const btVector3&, 
                                                 const btVector3&,
                                                 const btVector3&, btScalar);
}
