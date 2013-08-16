#ifndef __DrakeCollisionMinDistResultCollector_H__
#define __DrakeCollisionMinDistResultCollector_H__

#include "DrakeCollision.h"
#include "ResultCollector.h"

namespace DrakeCollision
{
  class MinDistResultCollector : public ResultCollector {
    public:
      virtual void addPointPairResult(const PointPair& result);
  };
}
#endif
