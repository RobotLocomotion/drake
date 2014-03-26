#ifndef __DrakeCollisionBullletResultCollector_H__
#define __DrakeCollisionBullletResultCollector_H__

#define BT_USE_DOUBLE_PRECISION

#include "DrakeCollision.h"
#include "ResultCollector.h"

namespace DrakeCollision
{
  Vector3d toVector3d(const btVector3& bt_vec);

  class BulletResultCollector : public ResultCollector, public btCollisionWorld::ContactResultCallback
  {
    public:
      using ResultCollector::addSingleResult;

      BulletResultCollector() : curr_bodyA_idx(-1), curr_bodyB_idx(-1) {};

      virtual	btScalar	addSingleResult(btManifoldPoint& cp,	
                                        const btCollisionObjectWrapper* colObj0Wrap,
                                        int partId0,int index0,
                                        const btCollisionObjectWrapper* colObj1Wrap,
                                        int partId1,int index1);

      btCollisionWorld::ContactResultCallback* getBtPtr();

      void setBodyIdx(const int bodyA_idx, const int bodyB_idx);

    protected:
      int curr_bodyA_idx;
      int curr_bodyB_idx;
  };
}

#endif
